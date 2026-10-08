#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/twai.h"
#include "driver/gpio.h"
#include "driver/rmt_tx.h"
#include "driver/rmt_encoder.h"
#include "hal/twai_ll.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "nvs.h"

#define ENROLL_CALLS      3
#define ENROLL_WINDOW_MS  180000
#define FACTORY_HOLD_MS   5000
#define SMS_MIN_GAP_MS    8000
#define SMS_MAX_PER_BOOT  100
#define EVENT_SETTLE_MS   1000
#define GSM_QUIET_MS      120000

#define SIM800_UART       UART_NUM_1
#define SIM800_TX_PIN     1
#define SIM800_RX_PIN     2
#define CAN_TX_PIN        GPIO_NUM_3
#define CAN_RX_PIN        GPIO_NUM_4
#define BOOT_BTN_PIN      GPIO_NUM_0
#define LED_PIN           21

#define NO_TEMP           (-999)

static const char *TAG = "OBD_SMS";

static uint32_t now_ms(void) { return (uint32_t)(esp_timer_get_time() / 1000); }

/* ---------- LED RGB de stare (WS2812, ordine RGB) ---------- */

typedef struct { uint8_t r, g, b; } rgb_t;
static const rgb_t C_YELLOW = { 24, 14, 0 }, C_MAGENTA = { 20, 0, 20 }, C_GREEN = { 0, 24, 0 },
                   C_ORANGE = { 32, 8, 0 }, C_BLUE = { 0, 0, 40 }, C_RED = { 40, 0, 0 },
                   C_WHITE = { 20, 20, 20 }, C_CYAN = { 0, 20, 20 };

static rmt_channel_handle_t s_led_chan;
static rmt_encoder_handle_t s_led_enc;
static rgb_t s_led_cur, s_led_flash_color;
static uint32_t s_led_last_ms, s_led_flash_until;
static uint32_t g_last_can_ms;                 /* ultimul mesaj CAN primit, 0 = niciunul */

static void led_write(rgb_t c)
{
    if (!s_led_chan) return;
    uint8_t px[3] = { c.r, c.g, c.b };
    rmt_transmit_config_t tc = { .loop_count = 0 };
    if (rmt_transmit(s_led_chan, s_led_enc, px, sizeof(px), &tc) == ESP_OK) rmt_tx_wait_all_done(s_led_chan, 20);
    s_led_cur = c;
    s_led_last_ms = now_ms();
}

static void led_init(void)
{
    rmt_tx_channel_config_t cc = { .gpio_num = LED_PIN, .clk_src = RMT_CLK_SRC_DEFAULT,
                                   .resolution_hz = 10 * 1000 * 1000, .mem_block_symbols = 64, .trans_queue_depth = 2 };
    rmt_bytes_encoder_config_t ec = {
        .bit0 = { .level0 = 1, .duration0 = 3, .level1 = 0, .duration1 = 9 },
        .bit1 = { .level0 = 1, .duration0 = 9, .level1 = 0, .duration1 = 3 },
        .flags.msb_first = 1,
    };
    if (rmt_new_tx_channel(&cc, &s_led_chan) != ESP_OK || rmt_new_bytes_encoder(&ec, &s_led_enc) != ESP_OK ||
        rmt_enable(s_led_chan) != ESP_OK) {
        s_led_chan = NULL;
        return;
    }
    led_write(C_YELLOW);
}

static void led_flash(rgb_t c, uint32_t ms)
{
    s_led_flash_color = c;
    s_led_flash_until = now_ms() + ms;
    led_write(c);
}

/* Culoarea de baza se retrimite in fiecare secunda, ca un zgomot pe linie sa nu ramana afisat. */
static void led_tick(uint32_t t, bool registered, bool enrolled)
{
    rgb_t want;
    if ((int32_t)(s_led_flash_until - t) > 0)                   want = s_led_flash_color;
    else if (!registered)                                       want = C_YELLOW;
    else if (!enrolled)                                         want = C_MAGENTA;
    else if (g_last_can_ms && (t - g_last_can_ms) < 3000)       want = C_GREEN;
    else                                                        want = C_ORANGE;
    bool changed = want.r != s_led_cur.r || want.g != s_led_cur.g || want.b != s_led_cur.b;
    if (changed || (t - s_led_last_ms) >= 1000) led_write(want);
}

/* ---------- Modem GSM ---------- */

static bool sim800_wait(const char *expect, int timeout_ms, char *out, size_t out_sz)
{
    size_t len = 0;
    out[0] = '\0';
    uint32_t start = now_ms();
    while ((int)(now_ms() - start) < timeout_ms) {
        int n = uart_read_bytes(SIM800_UART, (uint8_t *)out + len, out_sz - 1 - len, pdMS_TO_TICKS(50));
        if (n > 0) {
            len += n;
            out[len] = '\0';
            if (strstr(out, expect)) return true;
            if (strstr(out, "ERROR")) return false;
        }
        if (len >= out_sz - 1) break;
    }
    return false;
}

static bool sim800_cmd(const char *cmd, const char *expect, int timeout_ms, char *out, size_t out_sz)
{
    uart_flush_input(SIM800_UART);
    uart_write_bytes(SIM800_UART, cmd, strlen(cmd));
    return sim800_wait(expect, timeout_ms, out, out_sz);
}

static bool gsm_registered(void)
{
    char buf[128];
    if (!sim800_cmd("AT+CREG?\r\n", "OK", 2000, buf, sizeof(buf))) return false;
    return strstr(buf, "+CREG: 0,1") || strstr(buf, "+CREG: 0,5");
}

static bool sim800_send_sms(const char *number, const char *text)
{
    char buf[320], cmd[64];
    const char ctrl_z = 0x1A, esc = 0x1B;

    if (!sim800_cmd("AT+CMGF=1\r\n", "OK", 2000, buf, sizeof(buf))) return false;
    snprintf(cmd, sizeof(cmd), "AT+CMGS=\"%s\"\r\n", number);
    if (!sim800_cmd(cmd, ">", 5000, buf, sizeof(buf))) {
        uart_write_bytes(SIM800_UART, &esc, 1);
        return false;
    }
    uart_write_bytes(SIM800_UART, text, strlen(text));
    uart_write_bytes(SIM800_UART, &ctrl_z, 1);
    bool ok = sim800_wait("+CMGS:", 30000, buf, sizeof(buf));
    if (!ok) ESP_LOGE(TAG, "Trimitere SMS esuata: %s", buf);
    return ok;
}

/* ---------- CAN / OBD-II ---------- */

/* Emisia GSM produce erori pe CAN; fiecare eroare ar declansa o intrerupere "bus error", iar o avalansa de astfel de
 * intreruperi a dus la reset de watchdog. Se dezactiveaza doar aceasta intrerupere. Registrul se scrie numai in modul
 * reset: dupa twai_driver_install si dupa twai_stop. */
static void can_mask_bus_error_intr(void)
{
    twai_ll_set_enabled_intrs(TWAI_LL_GET_HW(0), 0xE7 & ~TWAI_LL_INTR_BEI);
}

static bool can_start(void)
{
    twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN, CAN_RX_PIN, TWAI_MODE_NORMAL);
    twai_timing_config_t t = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t f = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    g.rx_queue_len = 32;
    if (twai_driver_install(&g, &t, &f) != ESP_OK) return false;
    can_mask_bus_error_intr();
    return twai_start() == ESP_OK;
}

static void can_check_health(void)
{
    twai_status_info_t st;
    if (twai_get_status_info(&st) != ESP_OK) return;
    if (st.state == TWAI_STATE_BUS_OFF) twai_initiate_recovery();
    else if (st.state == TWAI_STATE_STOPPED) twai_start();
}

static void body_on_frame(const twai_message_t *m);

/* Goleste coada de receptie; mesajele de caroserie ajung la body_on_frame. */
static void can_drain(void)
{
    twai_message_t m;
    while (twai_receive(&m, 0) == ESP_OK) body_on_frame(&m);
}

static bool obd_query_once(uint8_t mode, uint8_t pid, uint8_t *resp)
{
    twai_message_t req = { .identifier = 0x7DF, .data_length_code = 8 };
    req.data[0] = (mode == 0x03 || mode == 0x04) ? 0x01 : 0x02;
    req.data[1] = mode;
    req.data[2] = pid;
    memset(&req.data[3], 0x55, 5);

    can_drain();
    if (twai_transmit(&req, pdMS_TO_TICKS(20)) != ESP_OK) return false;

    uint32_t start = now_ms();
    while ((now_ms() - start) < 150) {
        twai_message_t rx;
        if (twai_receive(&rx, pdMS_TO_TICKS(20)) != ESP_OK) continue;
        if (rx.identifier != 0x7E8) { body_on_frame(&rx); continue; }
        if (rx.data_length_code < 3 || rx.data[1] != (uint8_t)(mode + 0x40)) continue;
        if (mode == 0x01 && rx.data[2] != pid) continue;
        memcpy(resp, rx.data, 8);
        return true;
    }
    return false;
}

static bool obd_query(uint8_t mode, uint8_t pid, uint8_t *resp)
{
    return obd_query_once(mode, pid, resp) || obd_query_once(mode, pid, resp);
}

typedef struct {
    int  rpm, speed, ect, iat, load, vbat_mv;      /* -1 / NO_TEMP = necitit */
    bool dtc_valid;
    char dtc[40];                                  /* "P0335 P0101" sau "" */
} snapshot_t;

/* Raspuns Mode 03 -> "P0335 P0101" (maxim 2 coduri intr-un cadru). */
static void dtc_list(const uint8_t *r, char *out, size_t sz)
{
    static const char letters[4] = { 'P', 'C', 'B', 'U' };
    out[0] = '\0';
    for (int i = 0; i < r[2] && i < 2; i++) {
        uint8_t hi = r[3 + i * 2], lo = r[4 + i * 2];
        size_t l = strlen(out);
        snprintf(out + l, sz - l, "%s%c%d%X%02X", l ? " " : "", letters[hi >> 6], (hi >> 4) & 3, hi & 0x0F, lo);
    }
}

static void obd_sweep(snapshot_t *s)
{
    uint8_t r[8];
    s->rpm     = obd_query(0x01, 0x0C, r) ? (r[3] * 256 + r[4]) / 4 : -1;
    s->speed   = obd_query(0x01, 0x0D, r) ? r[3] : -1;
    s->ect     = obd_query(0x01, 0x05, r) ? r[3] - 40 : NO_TEMP;
    s->iat     = obd_query(0x01, 0x0F, r) ? r[3] - 40 : NO_TEMP;
    s->load    = obd_query(0x01, 0x04, r) ? r[3] * 100 / 255 : -1;
    s->vbat_mv = obd_query(0x01, 0x42, r) ? r[3] * 256 + r[4] : -1;
    s->dtc_valid = obd_query(0x03, 0x00, r);
    s->dtc[0] = '\0';
    if (s->dtc_valid) dtc_list(r, s->dtc, sizeof(s->dtc));
}

/* ---------- Evenimente ---------- */

static bool     g_pending;
static uint32_t g_pending_since;
static char     g_reason[96];

static int  g_eng = -1;                        /* -1 necunoscut, 0 oprit, 1 pornit */
static int  g_hi, g_lo;
static bool g_dtc_init;
static char g_last_dtc[40];

static void queue_event(const char *reason, bool immediate)
{
    if (!g_pending) {
        g_reason[0] = '\0';
        g_pending_since = now_ms();
    }
    if (!strstr(g_reason, reason)) {
        size_t l = strlen(g_reason);
        if (l + strlen(reason) + 3 < sizeof(g_reason))
            snprintf(g_reason + l, sizeof(g_reason) - l, "%s%s", l ? ", " : "", reason);
    }
    if (immediate) g_pending_since = now_ms() - EVENT_SETTLE_MS;
    g_pending = true;
    ESP_LOGI(TAG, "Eveniment: %s", reason);
}

/* Motorul: pornit la >= 400 rpm, oprit la < 100 rpm, in ambele cazuri la 3 citiri consecutive. */
static void detect_events(const snapshot_t *s)
{
    if (s->rpm >= 0) {
        if (s->rpm >= 400)     { g_hi++; g_lo = 0; }
        else if (s->rpm < 100) { g_lo++; g_hi = 0; }
        if (g_hi >= 3) {
            if (g_eng == 0) queue_event("MOTOR PORNIT", false);
            g_eng = 1; g_hi = 3;
        }
        if (g_lo >= 3) {
            if (g_eng == 1) queue_event("MOTOR OPRIT", false);
            g_eng = 0; g_lo = 3;
        }
    }
    if (s->dtc_valid) {
        if (g_dtc_init && strcmp(g_last_dtc, s->dtc) != 0) queue_event(s->dtc[0] ? "DTC NOU" : "DTC STERS", false);
        snprintf(g_last_dtc, sizeof(g_last_dtc), "%s", s->dtc);
        g_dtc_init = true;
    }
}

/* ---------- Caroserie: mesaje CAN ----------
 * ID-uri VAG din opendbc (vw_pq.dbc) si OpenVehicles (VW e-Up); 0x4F0 si 0x4F1 sunt simulate. */

#define ID_ZAS_1      0x572     /* bit 1 = contact pus */
#define ID_BSG_KOMBI  0x470     /* octet 1: usi, capota, portbagaj */
#define ID_BSG_LAST   0x570     /* octet 2: baterie, V = 5 + 0.05 * valoare */
#define ID_EPB_1      0x5C0     /* octet 0 bit 6 = frana de parcare trasa */
#define ID_ZV_STATUS  0x381     /* octet 0 = 0x02 incuiat */
#define ID_WINDOWS    0x4F0     /* simulat: octet 0 biti 0..3 = geamuri deschise */
#define ID_ZV_CMD     0x4F1     /* simulat: octet 0 = 1 incuie, 2 descuie */
#define BAT_LOW_MV    11800
#define BAT_OK_MV     12200

enum { SEEN_ZAS = 1, SEEN_DOORS = 2, SEEN_EPB = 8, SEEN_ZV = 16, SEEN_WIN = 32 };

static struct {
    uint8_t seen;                              /* mesaje primite cel putin o data; prima citire nu da eveniment */
    bool    kl15, locked, handbrake, batt_low;
    uint8_t doors, wins;
    int     vbat_mv;
} g_body = { .vbat_mv = -1 };

static const char *const k_door_names[6] = { "USA FS", "USA FD", "USA SS", "USA SD", "CAPOTA", "PORTBAGAJ" };
static const char *const k_win_names[4]  = { "GEAM FS", "GEAM FD", "GEAM SS", "GEAM SD" };

/* Un eveniment pentru fiecare bit schimbat; daca se schimba toti, in aceeasi directie, un singur eveniment. */
static void flag_events(uint8_t before, uint8_t after, int n, const char *const *names, int fem_count,
                        const char *all_open, const char *all_closed)
{
    uint8_t all = (1u << n) - 1, diff = (before ^ after) & all;
    if (!diff) return;
    if (all_open && diff == all && (after == all || after == 0)) {
        queue_event(after ? all_open : all_closed, false);
        return;
    }
    for (int i = 0; i < n; i++) {
        if (!(diff & (1u << i))) continue;
        bool open = after & (1u << i), fem = i < fem_count;
        char b[40];
        snprintf(b, sizeof(b), "%s %s", names[i], open ? (fem ? "DESCHISA" : "DESCHIS") : (fem ? "INCHISA" : "INCHIS"));
        queue_event(b, false);
    }
}

static void body_on_frame(const twai_message_t *m)
{
    if (m->extd || m->rtr || m->data_length_code < 1) return;
    g_last_can_ms = now_ms() ? now_ms() : 1;
    const uint8_t *d = m->data;
    bool v;

    switch (m->identifier) {
    case ID_ZAS_1:
        v = d[0] & 0x02;
        if ((g_body.seen & SEEN_ZAS) && v != g_body.kl15) queue_event(v ? "CONTACT PORNIT" : "CONTACT OPRIT", false);
        g_body.kl15 = v;
        g_body.seen |= SEEN_ZAS;
        break;
    case ID_BSG_KOMBI:
        if (m->data_length_code < 2) return;
        if (g_body.seen & SEEN_DOORS) flag_events(g_body.doors, d[1] & 0x3F, 6, k_door_names, 5, NULL, NULL);
        g_body.doors = d[1] & 0x3F;
        g_body.seen |= SEEN_DOORS;
        break;
    case ID_BSG_LAST:
        if (m->data_length_code < 3) return;
        g_body.vbat_mv = 5000 + d[2] * 50;
        if (g_body.vbat_mv < BAT_LOW_MV && !g_body.batt_low) {
            g_body.batt_low = true;
            queue_event("BATERIE DESCARCATA", false);
        } else if (g_body.vbat_mv >= BAT_OK_MV) {
            g_body.batt_low = false;
        }
        break;
    case ID_EPB_1:
        v = d[0] & 0x40;
        if ((g_body.seen & SEEN_EPB) && v != g_body.handbrake)
            queue_event(v ? "FRANA DE MANA TRASA" : "FRANA DE MANA LASATA", false);
        g_body.handbrake = v;
        g_body.seen |= SEEN_EPB;
        break;
    case ID_ZV_STATUS:
        v = d[0] & 0x02;
        if ((g_body.seen & SEEN_ZV) && v != g_body.locked) queue_event(v ? "INCUIAT" : "DESCUIAT", false);
        g_body.locked = v;
        g_body.seen |= SEEN_ZV;
        break;
    case ID_WINDOWS:
        if (g_body.seen & SEEN_WIN)
            flag_events(g_body.wins, d[0] & 0x0F, 4, k_win_names, 0, "TOATE GEAMURILE DESCHISE", "TOATE GEAMURILE INCHISE");
        g_body.wins = d[0] & 0x0F;
        g_body.seen |= SEEN_WIN;
        break;
    }
}

/* ---------- Textul SMS-urilor: un rand pe informatie ---------- */

static void sms_append(char *out, size_t sz, const char *fmt, ...) __attribute__((format(printf, 3, 4)));
static void sms_append(char *out, size_t sz, const char *fmt, ...)
{
    size_t len = strlen(out);
    if (len >= sz - 1) return;
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(out + len, sz - len, fmt, ap);
    va_end(ap);
}

static void list_bits(char *out, size_t sz, uint8_t mask, const char *const *names, int n)
{
    for (int i = 0; i < n; i++) if (mask & (1u << i)) sms_append(out, sz, " %s", names[i]);
}

/* Stare completa. La activarea incuierii, randurile problematice (frana netrasa, in miscare, usi sau geamuri
 * deschise) primesc "! " in fata, iar titlul "ATENTIE". */
static void build_sms_text(char *out, size_t sz, const char *reason, const snapshot_t *s)
{
    static const char *const dn[6] = { "FS", "FD", "SS", "SD", "CAP", "PB" };
    static const char *const wn[4] = { "FS", "FD", "SS", "SD" };
    bool lock_on = strstr(reason, "INCUIAT") != NULL;
    bool bad_brake = (g_body.seen & SEEN_EPB) && !g_body.handbrake;
    bool moving = s->speed > 0;
    bool bad_doors = (g_body.seen & SEEN_DOORS) && g_body.doors;
    bool bad_wins = (g_body.seen & SEEN_WIN) && g_body.wins;
    bool warn = lock_on && (bad_brake || moving || bad_doors || bad_wins);

    out[0] = '\0';
    sms_append(out, sz, "%s%s", reason, warn ? " - ATENTIE" : "");
    if (!g_body.seen && s->rpm < 0 && s->speed < 0 && s->vbat_mv < 0) {
        sms_append(out, sz, "\nFara date CAN.");
        return;
    }
    if (g_body.seen & SEEN_ZAS) sms_append(out, sz, "\nContact: %s", g_body.kl15 ? "PUS" : "OPRIT");
    if (s->rpm >= 400)      sms_append(out, sz, "\nMotor: PORNIT %d rpm", s->rpm);
    else if (s->rpm >= 0)   sms_append(out, sz, "\nMotor: OPRIT");
    if (moving) sms_append(out, sz, "\n%sViteza: %d km/h", lock_on ? "! " : "", s->speed);
    int mv = s->vbat_mv >= 0 ? s->vbat_mv : g_body.vbat_mv;
    if (mv >= 0) sms_append(out, sz, "\nBaterie: %d.%d V", mv / 1000, (mv % 1000) / 100);
    if (g_body.seen & SEEN_ZV) sms_append(out, sz, "\nIncuiere: %s", g_body.locked ? "DA" : "NU");
    if (g_body.seen & SEEN_EPB)
        sms_append(out, sz, "\n%sFrana: %s", (lock_on && bad_brake) ? "! " : "", g_body.handbrake ? "TRASA" : "NETRASA");
    if (g_body.seen & SEEN_DOORS) {
        sms_append(out, sz, "\n%sUsi:", (lock_on && bad_doors) ? "! " : "");
        if (g_body.doors) list_bits(out, sz, g_body.doors, dn, 6);
        else              sms_append(out, sz, " inchise");
    }
    if (g_body.seen & SEEN_WIN) {
        sms_append(out, sz, "\n%sGeamuri:", (lock_on && bad_wins) ? "! " : "");
        if (g_body.wins) list_bits(out, sz, g_body.wins, wn, 4);
        else             sms_append(out, sz, " inchise");
    }
    if (s->dtc_valid && s->dtc[0]) sms_append(out, sz, "\nDefecte: %s", s->dtc);
}

/* Raspunsul la comanda DIAG. */
static void build_diag_text(char *out, size_t sz, const snapshot_t *s)
{
    out[0] = '\0';
    sms_append(out, sz, "DIAGNOZA");
    if (s->rpm >= 0)       sms_append(out, sz, "\nTuratie: %d rpm", s->rpm);
    if (s->speed >= 0)     sms_append(out, sz, "\nViteza: %d km/h", s->speed);
    if (s->ect != NO_TEMP) sms_append(out, sz, "\nTemp motor: %d C", s->ect);
    if (s->iat != NO_TEMP) sms_append(out, sz, "\nTemp aer: %d C", s->iat);
    if (s->load >= 0)      sms_append(out, sz, "\nSarcina: %d%%", s->load);
    if (s->vbat_mv >= 0)   sms_append(out, sz, "\nBaterie: %d.%d V", s->vbat_mv / 1000, (s->vbat_mv % 1000) / 100);
    sms_append(out, sz, "\nDefecte: %s", s->dtc_valid ? (s->dtc[0] ? s->dtc : "nimic") : "?");
}

/* ---------- Numar inrolat (NVS) ---------- */

static char     g_owner[24];                   /* gol = starea din fabrica */
static bool     g_enrolled;
static char     g_enr_num[24];
static uint32_t g_enr_t[ENROLL_CALLS];
static int      g_enr_n;
static int      g_conf_tries;                  /* >0: mai trebuie trimis SMS-ul de confirmare */

/* "0757..." / "0040757..." / "40757..." -> "+40757..." */
static void normalize_number(const char *in, char *out, size_t sz)
{
    char d[24];
    size_t n = 0;
    for (const char *p = in; *p && n < sizeof(d) - 1; p++)
        if ((*p >= '0' && *p <= '9') || (*p == '+' && n == 0)) d[n++] = *p;
    d[n] = '\0';
    if (d[0] == '+')                         snprintf(out, sz, "%s", d);
    else if (!strncmp(d, "00", 2))           snprintf(out, sz, "+%s", d + 2);
    else if (d[0] == '0')                    snprintf(out, sz, "+40%s", d + 1);
    else if (!strncmp(d, "40", 2) && n > 9)  snprintf(out, sz, "+%s", d);
    else                                     snprintf(out, sz, "%s", d);
}

static void owner_store(const char *num)
{
    nvs_handle_t h;
    if (nvs_open("cfg", NVS_READWRITE, &h) != ESP_OK) return;
    if (num && num[0]) nvs_set_str(h, "owner", num);
    else nvs_erase_key(h, "owner");
    nvs_commit(h);
    nvs_close(h);
}

static void owner_load(void)
{
    nvs_handle_t h;
    char raw[24] = "";
    size_t len = sizeof(raw);
    if (nvs_open("cfg", NVS_READONLY, &h) != ESP_OK) return;
    if (nvs_get_str(h, "owner", raw, &len) == ESP_OK && raw[0]) {
        normalize_number(raw, g_owner, sizeof(g_owner));
        g_enrolled = g_owner[0] != '\0';
    }
    nvs_close(h);
    if (g_enrolled && strcmp(raw, g_owner) != 0) owner_store(g_owner);
}

static void factory_reset(void)
{
    owner_store(NULL);
    g_owner[0] = '\0';
    g_enrolled = false;
    g_enr_n = g_conf_tries = 0;
    g_pending = false;
    g_eng = -1;
    g_hi = g_lo = 0;
    g_dtc_init = false;
    memset(&g_body, 0, sizeof(g_body));
    g_body.vbat_mv = -1;
    led_flash(C_WHITE, 1500);
    ESP_LOGW(TAG, "SETARI DIN FABRICA: numar sters. Sun de %d ori in %d min de pe telefon ca sa inrolezi.",
             ENROLL_CALLS, ENROLL_WINDOW_MS / 60000);
}

/* Al 3-lea apel de la acelasi numar, in cel mult 3 minute, inroleaza numarul. */
static void enroll_on_call(const char *num, uint32_t t)
{
    if (!num[0]) return;
    if (g_enr_n > 0 && strcmp(num, g_enr_num) != 0) g_enr_n = 0;
    if (g_enr_n > 0 && (t - g_enr_t[g_enr_n - 1]) < 2000) return;       /* acelasi apel (RING repetat) */
    snprintf(g_enr_num, sizeof(g_enr_num), "%s", num);
    if (g_enr_n == ENROLL_CALLS) {
        memmove(g_enr_t, g_enr_t + 1, (ENROLL_CALLS - 1) * sizeof(g_enr_t[0]));
        g_enr_n--;
    }
    g_enr_t[g_enr_n++] = t;
    ESP_LOGI(TAG, "[INROLARE] apel %d/%d de la %s", g_enr_n, ENROLL_CALLS, num);
    led_flash(C_CYAN, 600);
    if (g_enr_n == ENROLL_CALLS && (t - g_enr_t[0]) <= ENROLL_WINDOW_MS) {
        normalize_number(num, g_owner, sizeof(g_owner));
        owner_store(g_owner);
        g_enrolled = true;
        g_enr_n = 0;
        g_conf_tries = 3;
        ESP_LOGW(TAG, "[INROLARE] numar inrolat: %s", g_owner);
    }
}

/* ---------- Linii nesolicitate de la modem: apeluri (+CLIP) si SMS primit (+CMTI) ---------- */

static volatile bool g_sms_check;

static void gsm_handle_line(const char *line, uint32_t t)
{
    if (!strncmp(line, "+CMTI:", 6)) g_sms_check = true;
    if (strncmp(line, "+CLIP:", 6) != 0) return;

    char num[24] = "";
    const char *q1 = strchr(line, '"');
    const char *q2 = q1 ? strchr(q1 + 1, '"') : NULL;
    if (q2 && (size_t)(q2 - q1 - 1) < sizeof(num)) {
        memcpy(num, q1 + 1, q2 - q1 - 1);
        num[q2 - q1 - 1] = '\0';
    }
    uart_write_bytes(SIM800_UART, "ATH\r\n", 5);
    if (!g_enrolled) enroll_on_call(num, t);
}

static void gsm_poll(uint32_t t)
{
    static char line[128];
    static size_t len;
    uint8_t b[64];
    int n;
    while ((n = uart_read_bytes(SIM800_UART, b, sizeof(b), 0)) > 0) {
        for (int i = 0; i < n; i++) {
            if (b[i] == '\n' || b[i] == '\r') {
                if (len) {
                    line[len] = '\0';
                    gsm_handle_line(line, t);
                    len = 0;
                }
            } else if (len < sizeof(line) - 1) {
                line[len++] = b[i];
            } else {
                len = 0;
            }
        }
    }
}

/* ---------- Trimiterea SMS-urilor ---------- */

static uint32_t g_sms_done_ms;
static bool     g_sms_done;

/* Pe durata transmisiei controlerul CAN e oprit si consola e redusa la erori (perturbatii radio). */
static bool send_sms_safe(bool can_ok, const char *text)
{
    uint32_t t0 = now_ms();
    led_write(C_BLUE);
    vTaskDelay(pdMS_TO_TICKS(20));
    if (can_ok) { twai_stop(); can_mask_bus_error_intr(); }
    esp_log_level_set("*", ESP_LOG_ERROR);
    bool ok = sim800_send_sms(g_owner, text);
    esp_log_level_set("*", ESP_LOG_INFO);
    if (can_ok) twai_start();
    if (!ok) led_flash(C_RED, 3000);
    ESP_LOGI(TAG, "SMS %s in %u ms", ok ? "trimis" : "ESUAT", (unsigned)(now_ms() - t0));
    g_sms_done_ms = now_ms();
    g_sms_done = true;
    return ok;
}

/* ---------- Comenzi primite prin SMS (numai de la numarul inrolat) ---------- */

static void sms_reply(bool can_ok, const char *text)
{
    ESP_LOGI(TAG, "[SMS-CMD] raspuns: %s", text);
    send_sms_safe(can_ok, text);
}

static void sms_dtc(bool can_ok)
{
    uint8_t r[8];
    char list[40], out[60];
    if (!can_ok || !obd_query(0x03, 0x00, r)) { sms_reply(can_ok, "Fara raspuns de la ECU."); return; }
    dtc_list(r, list, sizeof(list));
    snprintf(out, sizeof(out), "DTC: %s", list[0] ? list : "nimic");
    sms_reply(can_ok, out);
}

static void sms_clear_dtc(bool can_ok, const snapshot_t *s)
{
    uint8_t r[8];
    if (!can_ok) { sms_reply(can_ok, "CAN indisponibil."); return; }
    if (s->dtc_valid && !s->dtc[0]) { sms_reply(can_ok, "Nu exista defecte."); return; }
    if (!obd_query(0x04, 0x00, r)) sms_reply(can_ok, "Fara raspuns de la ECU.");
    /* confirmarea pleaca ca SMS de eveniment ("DTC STERS") */
}

static void sms_lock_cmd(bool can_ok, bool lock)
{
    if (!can_ok) { sms_reply(can_ok, "CAN indisponibil."); return; }
    if ((g_body.seen & SEEN_ZV) && g_body.locked == lock) {
        sms_reply(can_ok, lock ? "Deja incuiat." : "Deja descuiat.");
        return;
    }
    twai_message_t m = { .identifier = ID_ZV_CMD, .data_length_code = 1 };
    m.data[0] = lock ? 1 : 2;
    twai_transmit(&m, pdMS_TO_TICKS(20));
    for (uint32_t t0 = now_ms(); (now_ms() - t0) < 2000; ) {
        vTaskDelay(pdMS_TO_TICKS(20));
        can_drain();
        if ((g_body.seen & SEEN_ZV) && g_body.locked == lock) return;   /* confirmarea pleaca ca SMS de eveniment */
    }
    sms_reply(can_ok, "Comanda trimisa, fara confirmare de la masina.");
}

static void sms_command(bool can_ok, const snapshot_t *snap, const char *from, const char *text)
{
    char from_n[24], c[40];
    size_t n = 0;
    normalize_number(from, from_n, sizeof(from_n));
    if (!g_enrolled || strcmp(from_n, g_owner) != 0) {
        ESP_LOGW(TAG, "[SMS-CMD] SMS de la '%s' ignorat", from);
        return;
    }
    for (const char *p = text; *p && n < sizeof(c) - 1; p++) {          /* majuscule, fara spatii in plus */
        char ch = (*p >= 'a' && *p <= 'z') ? *p - 32 : *p;
        if (ch == ' ' && (n == 0 || c[n - 1] == ' ')) continue;
        c[n++] = ch;
    }
    while (n && c[n - 1] == ' ') n--;
    c[n] = '\0';
    ESP_LOGI(TAG, "[SMS-CMD] comanda: '%s'", c);
    led_flash(C_CYAN, 500);

    if (!strcmp(c, "STARE"))           queue_event("STARE", true);
    else if (!strcmp(c, "DIAG"))       { char d[161]; build_diag_text(d, sizeof(d), snap); sms_reply(can_ok, d); }
    else if (!strcmp(c, "DTC"))        sms_dtc(can_ok);
    else if (!strcmp(c, "STERGE DTC")) sms_clear_dtc(can_ok, snap);
    else if (!strcmp(c, "INCUIE"))     sms_lock_cmd(can_ok, true);
    else if (!strcmp(c, "DESCUIE"))    sms_lock_cmd(can_ok, false);
    else if (!strcmp(c, "AJUTOR"))     sms_reply(can_ok, "Comenzi:\nSTARE\nDIAG\nDTC\nSTERGE DTC\nINCUIE\nDESCUIE");
    else                               sms_reply(can_ok, "Comanda necunoscuta. Trimite AJUTOR.");
}

/* Citeste SMS-urile din memoria modemului, le sterge si executa comenzile (maxim 4 odata). */
static void sms_fetch(bool can_ok, const snapshot_t *snap)
{
    static char buf[1200];
    struct { int idx; char from[24]; char text[48]; } in[4];
    int n = 0;

    if (!sim800_cmd("AT+CMGL=\"ALL\"\r\n", "OK", 5000, buf, sizeof(buf))) return;
    for (char *p = buf; n < 4 && (p = strstr(p, "+CMGL: ")) != NULL; n++) {
        p += 7;
        in[n].idx = atoi(p);
        char *q = strchr(p, '"');                                       /* "REC UNREAD" */
        for (int k = 0; k < 2 && q; k++) q = strchr(q + 1, '"');       /* "+40..." */
        char *q4 = q ? strchr(q + 1, '"') : NULL;
        char *eol = q4 ? strstr(q4, "\r\n") : NULL;
        if (!eol) break;
        snprintf(in[n].from, sizeof(in[n].from), "%.*s", (int)(q4 - q - 1), q + 1);
        char *text = eol + 2;
        size_t tl = strcspn(text, "\r\n");
        snprintf(in[n].text, sizeof(in[n].text), "%.*s", (int)tl, text);
        p = text + tl;
    }
    for (int i = 0; i < n; i++) {                                       /* intai stergerea, ca sa nu se reciteasca */
        char cmd[24], b[48];
        snprintf(cmd, sizeof(cmd), "AT+CMGD=%d\r\n", in[i].idx);
        sim800_cmd(cmd, "OK", 3000, b, sizeof(b));
    }
    for (int i = 0; i < n; i++) sms_command(can_ok, snap, in[i].from, in[i].text);
}

/* ---------- Bucla principala ---------- */

static const char *reset_reason_str(esp_reset_reason_t r)
{
    switch (r) {
    case ESP_RST_POWERON:  return "power-on";
    case ESP_RST_EXT:      return "pin RESET extern";
    case ESP_RST_SW:       return "software";
    case ESP_RST_PANIC:    return "PANIC";
    case ESP_RST_INT_WDT:  return "watchdog intreruperi";
    case ESP_RST_TASK_WDT: return "watchdog task";
    case ESP_RST_WDT:      return "alt watchdog";
    case ESP_RST_BROWNOUT: return "BROWNOUT";
    case ESP_RST_USB:      return "resetat de USB";
    default:               return "altul";
    }
}

static esp_reset_reason_t g_rr;
static uint32_t g_boot_ms, g_last_creg_ms, g_last_cmgl_ms;
static bool g_gsm_ready, g_clip_set, g_sms_ready;

/* Inregistrarea in retea, setarile modemului si citirea SMS-urilor primite. */
static void gsm_service(uint32_t t, bool can_ok, const snapshot_t *snap)
{
    static bool boot_checked;
    char b[64];

    if (!boot_checked && (t - g_boot_ms) >= 5000) {              /* daca doar ESP32 a repornit, modemul e deja inregistrat */
        boot_checked = true;
        ESP_LOGW(TAG, "Motiv ultimul reset: %s (%d)", reset_reason_str(g_rr), (int)g_rr);
        g_gsm_ready = gsm_registered();
        ESP_LOGI(TAG, "[GSM] la pornire inregistrat: %s", g_gsm_ready ? "DA" : "nu (astept)");
    }
    if (!g_gsm_ready && (t - g_boot_ms) >= GSM_QUIET_MS && (t - g_last_creg_ms) >= 10000) {
        g_last_creg_ms = t;
        g_gsm_ready = gsm_registered();
        ESP_LOGI(TAG, "[GSM] inregistrat: %s", g_gsm_ready ? "DA" : "inca nu");
    }
    if (g_gsm_ready && (t - g_last_creg_ms) >= 30000) {          /* verific ca ramane inregistrat (SIM scos, retea pierduta) */
        g_last_creg_ms = t;
        if (!gsm_registered()) {
            g_gsm_ready = g_clip_set = g_sms_ready = false;
            ESP_LOGW(TAG, "[GSM] inregistrarea in retea a fost pierduta");
        }
    }
    if (g_gsm_ready && !g_clip_set) {                            /* afisarea numarului apelantului (inrolare) */
        g_clip_set = sim800_cmd("AT+CLIP=1\r\n", "OK", 2000, b, sizeof(b));
        ESP_LOGI(TAG, "[GSM] AT+CLIP=1: %s", g_clip_set ? "ok" : "esuat");
    }
    if (g_gsm_ready && g_clip_set && !g_sms_ready) {             /* SMS in mod text, notificare la sosire, memorie goala */
        bool ok1 = sim800_cmd("AT+CMGF=1\r\n", "OK", 2000, b, sizeof(b));
        bool ok2 = sim800_cmd("AT+CNMI=2,1,0,0,0\r\n", "OK", 2000, b, sizeof(b));
        sim800_cmd("AT+CMGD=1,4\r\n", "OK", 8000, b, sizeof(b));
        g_sms_ready = ok1 && ok2;
        g_last_cmgl_ms = t;
        ESP_LOGI(TAG, "[GSM] primire SMS (CNMI): %s", g_sms_ready ? "ok" : "esuat");
    }
    if (g_sms_ready && (g_sms_check || (t - g_last_cmgl_ms) >= 30000)) {   /* la notificare si, de siguranta, la 30 s */
        g_sms_check = false;
        g_last_cmgl_ms = t;
        sms_fetch(can_ok, snap);
    }
}

/* Confirmarea de inrolare si SMS-ul de eveniment, cu pauza minima intre mesaje. */
static void sms_service(uint32_t t, bool can_ok, const snapshot_t *snap)
{
    static int sms_count, fail_count;
    bool gap_ok = !g_sms_done || (t - g_sms_done_ms) >= SMS_MIN_GAP_MS;

    if (!g_gsm_ready || !g_enrolled) return;

    if (g_conf_tries > 0) {
        bool ok = send_sms_safe(can_ok, "Numar inrolat. Sistemul de semnalizare este activ.");
        g_conf_tries = ok ? 0 : g_conf_tries - 1;
        return;
    }
    if (!g_pending || (int32_t)(t - g_pending_since) < EVENT_SETTLE_MS || !gap_ok) return;

    if (sms_count >= SMS_MAX_PER_BOOT) {
        ESP_LOGW(TAG, "Plafon SMS atins (%d), ignor evenimentul: %s", SMS_MAX_PER_BOOT, g_reason);
        g_pending = false;
        return;
    }
    char text[161];
    build_sms_text(text, sizeof(text), g_reason, snap);
    ESP_LOGI(TAG, "Trimit SMS (%d chr): %s", (int)strlen(text), text);
    if (send_sms_safe(can_ok, text)) {
        sms_count++;
        fail_count = 0;
        g_pending = false;
    } else if (++fail_count >= 3) {
        ESP_LOGE(TAG, "3 esecuri consecutive, renunt la eveniment");
        fail_count = 0;
        g_pending = false;
    }
}

/* BOOT: apasare scurta = SMS de stare; apasare lunga = setari din fabrica. */
static void button_service(uint32_t t)
{
    static bool prev = true, long_done;
    static uint32_t down_ms;
    bool now = gpio_get_level(BOOT_BTN_PIN);

    if (prev && !now) { down_ms = t; long_done = false; }
    if (!now && !long_done && (t - down_ms) >= FACTORY_HOLD_MS) {
        long_done = true;
        factory_reset();
    }
    if (!prev && now && !long_done && (t - down_ms) >= 30 && g_enrolled) queue_event("MANUAL", true);
    prev = now;
}

void app_main(void)
{
    led_init();
    esp_err_t e = nvs_flash_init();
    if (e == ESP_ERR_NVS_NO_FREE_PAGES || e == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }
    owner_load();
    g_rr = esp_reset_reason();

    uart_config_t uc = { .baud_rate = 9600, .data_bits = UART_DATA_8_BITS, .parity = UART_PARITY_DISABLE,
                         .stop_bits = UART_STOP_BITS_1, .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
                         .source_clk = UART_SCLK_DEFAULT };
    ESP_ERROR_CHECK(uart_driver_install(SIM800_UART, 1024, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(SIM800_UART, &uc));
    ESP_ERROR_CHECK(uart_set_pin(SIM800_UART, SIM800_TX_PIN, SIM800_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    gpio_config_t btn = { .pin_bit_mask = 1ULL << BOOT_BTN_PIN, .mode = GPIO_MODE_INPUT, .pull_up_en = GPIO_PULLUP_ENABLE };
    gpio_config(&btn);

    bool can_ok = can_start();
    snapshot_t snap = { .rpm = -1, .speed = -1, .ect = NO_TEMP, .iat = NO_TEMP, .load = -1, .vbat_mv = -1 };
    g_boot_ms = now_ms();
    uint32_t last_sweep_ms = 0;

    if (g_enrolled) ESP_LOGI(TAG, "Pornit. Numar inrolat: %s", g_owner);
    else ESP_LOGW(TAG, "Stare din fabrica: sun de %d ori in %d min de pe telefon ca sa inrolezi numarul.",
                  ENROLL_CALLS, ENROLL_WINDOW_MS / 60000);

    while (1) {
        uint32_t t = now_ms();
        gsm_poll(t);
        led_tick(t, g_gsm_ready, g_enrolled);
        button_service(t);

        if (can_ok && g_enrolled) {
            can_drain();
            if ((t - last_sweep_ms) >= 1000) {
                last_sweep_ms = t;
                can_check_health();
                obd_sweep(&snap);
                if (snap.rpm < 0 && snap.speed < 0 && snap.vbat_mv < 0) {
                    ESP_LOGW(TAG, "[OBD] fara raspuns de la ECU (cablaj, terminatii 120R, simulator pornit?)");
                } else {
                    g_last_can_ms = now_ms() ? now_ms() : 1;
                    ESP_LOGI(TAG, "[OBD] RPM=%d SPD=%d ECT=%d IAT=%d LOAD=%d BAT=%dmV DTC=%s", snap.rpm, snap.speed,
                             snap.ect, snap.iat, snap.load, snap.vbat_mv,
                             snap.dtc_valid ? (snap.dtc[0] ? snap.dtc : "-") : "?");
                }
                detect_events(&snap);
            }
        }

        t = now_ms();                                  /* sweep-ul OBD dureaza ~1 s */
        gsm_service(t, can_ok, &snap);
        sms_service(t, can_ok, &snap);
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}
