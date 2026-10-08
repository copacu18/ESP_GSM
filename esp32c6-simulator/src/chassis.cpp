#include "chassis.h"
#include <Arduino.h>
#include <driver/twai.h>
#include "can_bus.h"

#define BOOT_BTN_PIN  9
#define TFT_CS       14
#define SD_CS         4
#define BABY_BLUE  0x8E79
#define DARK_TEXT  0x2104
#define RED_TEXT   0xF800
#define GREEN_TEXT 0x03E0
#define ACTION_COL 0xA800

/* Mesaje CAN VAG, din opendbc (vw_pq.dbc) si OpenVehicles (VW e-Up); 0x4F0 si 0x4F1 sunt simulate. */
#define ID_ZAS_1      0x572     /* bit 1 = contact pus, bit 3 = demaror */
#define ID_BSG_KOMBI  0x470     /* octet 1: usi FS FD SS SD, capota, portbagaj */
#define ID_BSG_LAST   0x570     /* octet 2: baterie, V = 5 + 0.05 * valoare */
#define ID_EPB_1      0x5C0     /* octet 0 bit 6 = frana de parcare trasa */
#define ID_ZV_STATUS  0x381     /* octet 0 = 0x02 incuiat */
#define ID_WINDOWS    0x4F0     /* octet 0 biti 0..3 = geamuri deschise */
#define ID_ZV_CMD     0x4F1     /* comanda de la S3: octet 0 = 1 incuie, 2 descuie */

#define N_FRAMES       6
#define FRAME_STEP_MS  16       /* un mesaj la 16 ms, deci fiecare se repeta la ~100 ms */
#define CRANK_MS       1000     /* durata demarorului la pornirea motorului */

static struct {
    uint8_t ign;                /* 0 oprit, 1 contact, 2 motor pornit */
    bool door[4], hood, trunk, locked, win[4], handbrake, batt_low, dtc, moving;
} g;

static Adafruit_ST7789* g_tft;
static uint32_t g_crank_until, g_frame_ms, g_health_ms, g_cmd_count, g_serial_dead_until;
static uint8_t  g_frame_idx;
static bool     g_btn_prev = true, g_dirty = true, g_was_cranking;
static char     g_action[40] = "PORNIT";           /* ultima comanda, afisata pe ecran */

static bool cranking()    { return (int32_t)(g_crank_until - millis()) > 0; }
static int  engine_rpm()  { return (g.ign == 2 && !cranking() && !g.dtc) ? (g.moving ? 2200 : 850) : 0; }
static int  speed_kmh()   { return g.moving ? 30 : 0; }
static int  battery_mv()  { return g.batt_low ? 11400 : ((g.ign == 2 && !cranking()) ? 14100 : 12600); }
static const char* ign_str() { return g.ign == 0 ? "OPRIT" : (g.ign == 1 ? "PUS" : "MOTOR"); }
static const char* oc(bool open) { return open ? "DESCHIS" : "inchis"; }

static uint8_t bits(const bool* v, int n) {
    uint8_t m = 0;
    for (int i = 0; i < n; i++) if (v[i]) m |= 1 << i;
    return m;
}

/* ---------- CAN ---------- */

static void send_frame(uint8_t which) {
    can_frame_t f = {};
    switch (which) {
        case 0:
            f.id = ID_ZAS_1; f.dlc = 2;
            if (g.ign >= 1) f.data[0] = 0x07;                           /* cheie + contact + Kl.X */
            if (cranking()) f.data[0] = (f.data[0] & ~0x04) | 0x08;     /* la start: demaror activ */
            break;
        case 1:
            f.id = ID_BSG_KOMBI; f.dlc = 5;
            f.data[1] = bits(g.door, 4) | (g.hood ? 0x10 : 0) | (g.trunk ? 0x20 : 0);
            break;
        case 2:
            f.id = ID_BSG_LAST; f.dlc = 4;
            f.data[2] = (uint8_t)((battery_mv() / 1000.0f - 5.0f) / 0.05f + 0.5f);
            break;
        case 3:
            f.id = ID_EPB_1; f.dlc = 8;
            f.data[0] = g.handbrake ? 0x40 : 0;
            break;
        case 4:
            f.id = ID_ZV_STATUS; f.dlc = 8;
            f.data[0] = g.locked ? 0x02 : 0;
            break;
        default:
            f.id = ID_WINDOWS; f.dlc = 2;
            f.data[0] = bits(g.win, 4);
            break;
    }
    can_send(&f);
}

static void send_all_now() {
    for (uint8_t i = 0; i < N_FRAMES; i++) send_frame(i);
}

static void done(const char* label, const char* name, const char* state, const char* can);

/* Raspunsuri OBD-II: mod 01 (parametri), 03 (defecte), 04 (stergere defecte). */
static void handle_obd_request(const can_frame_t& req) {
    if (req.id != 0x7DF) return;
    can_frame_t r = {};
    r.id = 0x7E8; r.dlc = 8;

    if (req.data[1] == 0x04) {
        g.dtc = false;
        r.data[0] = 1; r.data[1] = 0x44;
        can_send(&r);
        done("SMS", "DEFECT P0335", "STERS (OBD Mode 04)", "0x7E8");
        return;
    }
    if (req.data[1] == 0x03) {
        r.data[1] = 0x43;
        if (g.dtc) { r.data[0] = 4; r.data[2] = 1; r.data[3] = 0x03; r.data[4] = 0x35; }    /* P0335 */
        else       { r.data[0] = 2; r.data[2] = 0; }
        can_send(&r);
        return;
    }
    if (req.data[1] != 0x01) return;

    uint8_t pid = req.data[2];
    bool eng = engine_rpm() > 0;
    r.data[0] = 3; r.data[1] = 0x41; r.data[2] = pid;
    switch (pid) {
        case 0x0C: { uint16_t e = engine_rpm() * 4; r.data[0] = 4; r.data[3] = e >> 8; r.data[4] = e & 0xFF; break; }
        case 0x0D: r.data[3] = speed_kmh(); break;
        case 0x05: r.data[3] = eng ? 130 : 65; break;           /* 90 C / 25 C */
        case 0x0F: r.data[3] = 65; break;                       /* 25 C */
        case 0x04: r.data[3] = eng ? 51 : 0; break;             /* 20 % */
        case 0x42: { uint16_t mv = battery_mv(); r.data[0] = 4; r.data[3] = mv >> 8; r.data[4] = mv & 0xFF; break; }
        default: return;
    }
    can_send(&r);
}

/* ---------- Ecran (acelasi stil ca modul SIMULATOR) ---------- */

static void tft_cell(int x, int y, uint16_t fg, const char* fmt, ...) {
    char s[32];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(s, sizeof(s), fmt, ap);
    va_end(ap);
    g_tft->setTextSize(1);
    g_tft->setTextColor(fg, BABY_BLUE);
    g_tft->setCursor(x, y);
    g_tft->printf("%-17.17s", s);
}

static void tft_update() {
    const uint16_t OK = GREEN_TEXT, BAD = RED_TEXT, N = DARK_TEXT;

    g_tft->setTextSize(1);
    g_tft->setTextColor(DARK_TEXT, BABY_BLUE);
    g_tft->setCursor(8, 2);
    g_tft->printf("SASIU VAG   CAN 500kbps   CMD:%-5u       ", (unsigned)g_cmd_count);
    g_tft->drawFastHLine(0, 12, 320, DARK_TEXT);
    g_tft->setCursor(139, 14);
    g_tft->print("CONTACT");

    g_tft->setTextSize(4);
    g_tft->setTextColor(g.ign == 0 ? DARK_TEXT : GREEN_TEXT, BABY_BLUE);
    g_tft->setCursor(100, 26);
    g_tft->print(cranking() ? "START" : (g.ign == 0 ? "OPRIT" : (g.ign == 1 ? " PUS " : "MOTOR")));
    g_tft->drawFastHLine(0, 62, 320, DARK_TEXT);

    tft_cell(  8, 66, g.locked ? OK : BAD,         "INCUIERE:%s", g.locked ? "INCUIAT" : "DESCUIAT");
    tft_cell(120, 66, g.handbrake ? OK : BAD,      "FRANA:%s", g.handbrake ? "TRASA" : "LASATA");
    tft_cell(232, 66, g.batt_low ? BAD : N,        "BAT  :%.1fV", battery_mv() / 1000.0f);
    static const char* dn[4] = { "FS", "FD", "SS", "SD" };
    for (int i = 0; i < 4; i++) {
        int row = 80 + (i / 3) * 14, col = (i % 3) * 112 + 8;
        tft_cell(col, row, g.door[i] ? BAD : N, "USA %s:%s", dn[i], g.door[i] ? "DESCH" : "inch");
    }
    tft_cell(120, 94, g.trunk ? BAD : N,           "PORTB.:%s", g.trunk ? "DESCH" : "inch");
    tft_cell(232, 94, g.hood ? BAD : N,            "CAPOTA:%s", g.hood ? "DESCH" : "inch");
    for (int i = 0; i < 4; i++) {
        int row = 108 + (i / 3) * 14, col = (i % 3) * 112 + 8;
        tft_cell(col, row, g.win[i] ? BAD : N, "GEAM %s:%s", dn[i], g.win[i] ? "DESCH" : "inch");
    }
    tft_cell(120, 122, g.dtc ? BAD : N,            "DTC  :%s", g.dtc ? "P0335" : "nimic");
    tft_cell(232, 122, g.moving ? BAD : N,         "RPM:%d VIT:%d", engine_rpm(), speed_kmh());
    g_tft->drawFastHLine(0, 136, 320, DARK_TEXT);

    g_tft->setTextSize(2);
    g_tft->setTextColor(ACTION_COL, BABY_BLUE);
    g_tft->setCursor(8, 140);
    g_tft->printf("%-26.26s", g_action);
    g_tft->setTextSize(1);
    g_tft->setTextColor(DARK_TEXT, BABY_BLUE);
    g_tft->setCursor(8, 162);
    g_tft->print("BOOT: contact   |   serial: numar + Enter    ");
    digitalWrite(TFT_CS, HIGH);
}

/* ---------- Consola serial ---------- */

/* Scrie in bucati cat permite bufferul USB; daca nu citeste nimeni (150 ms) renunta pentru 2 s. */
static void say(const char* fmt, ...) {
    static char b[512];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(b, sizeof(b), fmt, ap);
    va_end(ap);
    if (n <= 0 || (int32_t)(g_serial_dead_until - millis()) > 0) return;
    if (n > (int)sizeof(b) - 1) n = sizeof(b) - 1;
    int off = 0;
    for (uint32_t t0 = millis(); off < n && millis() - t0 < 150; ) {
        int space = Serial.availableForWrite();
        if (space <= 0) { delay(1); continue; }
        int k = min(n - off, space);
        Serial.write((const uint8_t*)b + off, k);
        off += k;
    }
    if (off < n) g_serial_dead_until = millis() + 2000;
}

static void print_menu() {
    say("\r\n=== SIMULARE SASIU VAG + OBD ===   (tasteaza numarul si apasa Enter)\r\n");
    say(" 0 - Contact (oprit > pus > motor)  [%s]   CAN 0x572   (BOOT la fel)\r\n", ign_str());
    say(" 1 - Incuiere centralizata            [%s]   CAN 0x381\r\n", g.locked ? "INCUIAT" : "descuiat");
    say(" 2 - Frana de mana                    [%s]   CAN 0x5C0\r\n", g.handbrake ? "TRASA" : "lasata");
    say(" 3 - Baterie slaba                    [%s]   CAN 0x570 (%.1f V)\r\n", g.batt_low ? "SLABA" : "ok", battery_mv() / 1000.0f);
    say(" 4 - Defect P0335 (motor oprit)       [%s]   OBD\r\n", g.dtc ? "ACTIV" : "nu");
    say(" 5 - Usa FS (sofer)                   [%s]   CAN 0x470\r\n", oc(g.door[0]));
    say(" 6 - Usa FD (pasager)                 [%s]\r\n", oc(g.door[1]));
    say(" 7 - Usa SS (spate stanga)            [%s]\r\n", oc(g.door[2]));
    say(" 8 - Usa SD (spate dreapta)           [%s]\r\n", oc(g.door[3]));
    say(" 9 - Portbagaj                        [%s]\r\n", oc(g.trunk));
    say("10 - Capota                           [%s]\r\n", oc(g.hood));
    say("11 - Geam FS                          [%s]   CAN 0x4F0 (simulat)\r\n", oc(g.win[0]));
    say("12 - Geam FD                          [%s]\r\n", oc(g.win[1]));
    say("13 - Geam SS                          [%s]\r\n", oc(g.win[2]));
    say("14 - Geam SD                          [%s]\r\n", oc(g.win[3]));
    say("15 - Toate geamurile (deschide / inchide)\r\n");
    say("16 - Viteza (0 / 30 km/h)             [%d km/h]   OBD 0x0D\r\nNumar: ", speed_kmh());
}

/* O schimbare: se afiseaza in consola, se trimite imediat pe CAN si se actualizeaza ecranul. */
static void done(const char* label, const char* name, const char* state, const char* can) {
    snprintf(g_action, sizeof(g_action), "[%s] %s %s", label, name, state);
    g_cmd_count++;
    say("  -> %s: %s   (trimis pe CAN: %s)\r\n", name, state, can);
    send_all_now();
    g_dirty = true;
}

static void do_ignition(const char* label) {
    g.ign = (g.ign + 1) % 3;
    g_crank_until = (g.ign == 2) ? millis() + CRANK_MS : 0;
    done(label, "CONTACT", ign_str(), "0x572");
}

static void toggle(bool& v, const char* label, const char* name, const char* on, const char* off, const char* can) {
    v = !v;
    done(label, name, v ? on : off, can);
}

static void run_cmd(int n) {
    static const char* dn[4] = { "FS", "FD", "SS", "SD" };
    char lab[8], nm[16];
    snprintf(lab, sizeof(lab), "%d", n);
    say("\r\n");
    if (n == 0)       do_ignition(lab);
    else if (n == 1)  toggle(g.locked, lab, "INCUIERE", "INCUIAT", "DESCUIAT", "0x381");
    else if (n == 2)  toggle(g.handbrake, lab, "FRANA DE MANA", "TRASA", "LASATA", "0x5C0");
    else if (n == 3)  toggle(g.batt_low, lab, "BATERIE", "SLABA (11.4 V)", "OK", "0x570");
    else if (n == 4)  toggle(g.dtc, lab, "DEFECT P0335", "ACTIV", "STERS", "OBD 0x7E8");
    else if (n <= 8) {
        snprintf(nm, sizeof(nm), "USA %s", dn[n - 5]);
        toggle(g.door[n - 5], lab, nm, "DESCHISA", "INCHISA", "0x470");
    }
    else if (n == 9)  toggle(g.trunk, lab, "PORTBAGAJ", "DESCHIS", "INCHIS", "0x470");
    else if (n == 10) toggle(g.hood, lab, "CAPOTA", "DESCHISA", "INCHISA", "0x470");
    else if (n <= 14) {
        snprintf(nm, sizeof(nm), "GEAM %s", dn[n - 11]);
        toggle(g.win[n - 11], lab, nm, "DESCHIS", "INCHIS", "0x4F0");
    }
    else if (n == 15) {
        bool open = bits(g.win, 4) != 0x0F;           /* daca vreunul e inchis, le deschide pe toate */
        for (bool& w : g.win) w = open;
        done(lab, "GEAMURI", open ? "DESCHISE" : "INCHISE", "0x4F0");
    }
    else {
        g.moving = !g.moving;
        if (g.moving && g.ign != 2) { g.ign = 2; g_crank_until = 0; }      /* se circula numai cu motorul pornit */
        done(lab, "VITEZA", g.moving ? "30 km/h" : "0 km/h", "OBD 0x0D");
    }
    print_menu();
}

/* Cifrele tastate se aduna si se vad in consola; comanda se executa la Enter. */
static void input_char(char c) {
    static char in[3];
    static int len;
    static bool last_cr;

    if (c == '\r' || c == '\n') {
        if (c == '\n' && last_cr) { last_cr = false; return; }          /* perechea \r\n = un singur Enter */
        last_cr = (c == '\r');
        int n = len ? atoi(in) : -1;
        len = 0;
        in[0] = '\0';
        if (n >= 0 && n <= 16) { run_cmd(n); return; }
        say(n > 16 ? "\r\n  Numar necunoscut (0-16).\r\n" : "\r\n");
        print_menu();
        return;
    }
    last_cr = false;
    if ((c == 0x08 || c == 0x7F) && len > 0) {
        in[--len] = '\0';
        say("\b \b");
    } else if (c >= '0' && c <= '9' && len < 2) {
        in[len++] = c;
        in[len] = '\0';
        say("%c", c);
    } else if (c == '?') {
        say("\r\n");
        print_menu();
    }
}

/* ---------- Mod SIMULARE SASIU ---------- */

void chassis_init(Adafruit_ST7789& display) {
    g_tft = &display;
    pinMode(BOOT_BTN_PIN, INPUT_PULLUP);
    digitalWrite(SD_CS, HIGH);
    g.locked = g.handbrake = true;                      /* masina parcata: incuiata, frana trasa, totul inchis */
    Serial.setTxTimeoutMs(0);

    g_tft->fillScreen(BABY_BLUE);
    tft_update();
    g_dirty = false;
    if (!can_init_500kbps()) say("[SASIU] AVERTISMENT: CAN init esuat\r\n");
    print_menu();
}

void chassis_loop() {
    uint32_t now = millis();

    bool btn = digitalRead(BOOT_BTN_PIN);
    if (g_btn_prev && !btn) {
        say("BOOT\r\n");
        do_ignition("BOOT");
        print_menu();
        delay(60);
    }
    g_btn_prev = btn;

    while (Serial.available()) input_char((char)Serial.read());

    can_frame_t req;
    while (can_recv(&req)) {
        if (req.id == ID_ZV_CMD && req.dlc >= 1 && (req.data[0] == 1 || req.data[0] == 2)) {
            g.locked = req.data[0] == 1;
            done("SMS", "INCUIERE", g.locked ? "INCUIAT" : "DESCUIAT", "0x381");
        } else {
            handle_obd_request(req);
        }
    }

    if (now - g_frame_ms >= FRAME_STEP_MS) {
        g_frame_ms = now;
        send_frame(g_frame_idx);
        g_frame_idx = (g_frame_idx + 1) % N_FRAMES;
    }

    bool c = cranking();
    if (g_was_cranking && !c) {
        send_all_now();
        g_dirty = true;
        say("  -> Motorul a pornit (demaror terminat, RPM %d)\r\n", engine_rpm());
        print_menu();
    }
    g_was_cranking = c;

    if (now - g_health_ms >= 1000) {
        g_health_ms = now;
        twai_status_info_t st;
        if (twai_get_status_info(&st) == ESP_OK) {
            if (st.state == TWAI_STATE_BUS_OFF) twai_initiate_recovery();
            else if (st.state == TWAI_STATE_STOPPED) twai_start();
        }
    }

    if (g_dirty) {
        g_dirty = false;
        tft_update();
    }
    delayMicroseconds(200);
}
