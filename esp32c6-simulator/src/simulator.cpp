#include "simulator.h"
#include <Arduino.h>
#include "csv_parser.h"
#include "defect_injector.h"
#include "playback.h"
#include "can_bus.h"
#include "serial_menu.h"

// ─── Pinuri ───────────────────────────────────────────────────────────────────
#define TFT_CS    14
#define SD_CS      4   // tinut HIGH in mod simulator (nu e folosit)

// ─── Culori ───────────────────────────────────────────────────────────────────
#define BABY_BLUE  0x8E79
#define DARK_TEXT  0x2104
#define SIM_GREEN  0x07E0

// ─── Globals ─────────────────────────────────────────────────────────────────
static Adafruit_ST7789* g_tft    = nullptr;
static uint32_t g_stats_last_ms  = 0;
static uint32_t g_tft_last_ms    = 0;
static CarParameter g_last_param = {};   // ultima linie trimisa, pentru display

// ─── Buton BOOT cycle defecte ────────────────────────────────────────────────
#define BOOT_BTN_PIN  9
static bool     g_btn_prev       = true;   // active-low
static uint8_t  g_defect_cycle   = 0;      // 0=NONE 1=P0335 2=P0101 3=P0128

static void cycle_defect() {
    g_defect_cycle = (g_defect_cycle + 1) % 4;
    clearAllDefects();
    switch (g_defect_cycle) {
        case 1: enableDefect(DEFECT_P0335); Serial.println("[SIM] Defect: P0335"); break;
        case 2: enableDefect(DEFECT_P0101); Serial.println("[SIM] Defect: P0101"); break;
        case 3: enableDefect(DEFECT_P0128); Serial.println("[SIM] Defect: P0128"); break;
        default: Serial.println("[SIM] Defect: NONE"); break;
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Raspunde la o cerere OBD2 Mode 01 venita de la P4 (ID=0x7DF).
// Trimite un singur frame 0x7E8 cu datele curente din g_last_param.
// ─────────────────────────────────────────────────────────────────────────────
// ── Mode 03: raspunde cu DTC-urile active ────────────────────────────────────
static void handle_mode03_request(const can_frame_t& req) {
    (void)req;
    uint16_t dtc[3];
    uint8_t cnt = getActiveDefects(dtc, 3);

    can_frame_t resp = {};
    resp.id      = 0x7E8;
    resp.dlc     = 8;
    // Format ISO 15031-5 Mode 03 response:
    // byte0 = lungime (1 + 2*cnt), byte1 = 0x43 (mode 03 response)
    // byte2+3 = DTC1, byte4+5 = DTC2, byte6+7 = DTC3
    // Format: data[0]=len, data[1]=0x43, data[2]=cnt, data[3..]=DTC bytes
    resp.data[0] = 2 + 2 * cnt;  // mode + count + 2 bytes per DTC
    resp.data[1] = 0x43;
    resp.data[2] = cnt;           // numarul de DTC-uri — P4 citeste asta ca n
    for (uint8_t i = 0; i < cnt && i < 3; i++) {
        resp.data[3 + i * 2] = (dtc[i] >> 8) & 0xFF;
        resp.data[4 + i * 2] = dtc[i] & 0xFF;
    }
    can_send(&resp);
}

static void handle_obd_request(const can_frame_t& req) {
    if (req.id != 0x7DF) return;

    // Mode 03: citire DTC
    if (req.data[1] == 0x03) {
        handle_mode03_request(req);
        return;
    }

    // Acceptam doar Mode 01 pentru restul
    if (req.data[1] != 0x01) return;

    uint8_t pid = req.data[2];
    const CarParameter& p = g_last_param;

    can_frame_t resp = {};
    resp.id      = 0x7E8;
    resp.dlc     = 8;
    resp.data[0] = 0x03;   // lungime default: 3 bytes (mode + PID + A)
    resp.data[1] = 0x41;   // mode 01 response
    resp.data[2] = pid;
    // data[3..7] = 0 implicit

    uint16_t rpm_enc = (uint16_t)p.rpm * 4;
    bool known = true;

    switch (pid) {
        case 0x0C:  // RPM: (A*256 + B) / 4
            resp.data[0] = 0x04;
            resp.data[3] = rpm_enc >> 8;
            resp.data[4] = rpm_enc & 0xFF;
            break;
        case 0x0D:  // Viteza km/h
            resp.data[3] = p.speed;
            break;
        case 0x04:  // Engine load %
            resp.data[3] = p.load_percent;
            break;
        case 0x05:  // Coolant temp (°C + 40)
            resp.data[3] = p.coolant;
            break;
        case 0x0F:  // IAT (°C + 40)
            resp.data[3] = p.iat;
            break;
        case 0x10: {  // MAF: (A*256 + B) / 100 g/s
            // p.maf e cuantizat din [0,655.35 g/s] → [0,255]
            // Decodificam inapoi la g/s, apoi re-encodam ca doua bytes OBD2
            float actual_maf = (float)p.maf / 255.0f * 655.35f;
            uint16_t maf_enc = (uint16_t)(actual_maf * 100.0f + 0.5f);
            resp.data[0] = 0x04;
            resp.data[3] = maf_enc >> 8;
            resp.data[4] = maf_enc & 0xFF;
            break;
        }
        case 0x0E:  // Timing advance
            // Formula OBD: byte = (grade_reale + 64) * 2
            // p.timing e stocat in CSV ca grade reale (ex: -22 grade)
            resp.data[3] = (uint8_t)((int8_t)p.timing + 64) * 2;
            break;
        case 0x11:  // Throttle position (TPS%)
            resp.data[3] = p.tps_percent;
            break;
        case 0x14:  // O2 sensor 1 voltage
            resp.data[3] = p.o2v1;
            break;
        case 0x15:  // O2 sensor 2 voltage
            resp.data[3] = p.o2v2;
            break;
        case 0x06:  // Short fuel trim
            // p.stft e stocat ca two's complement al valorii in %
            // OBD2 foloseste encodare 128-offset: byte = pct*128/100 + 128
            resp.data[3] = (uint8_t)((float)(int8_t)p.stft * 1.28f + 128.5f);
            break;
        case 0x07:  // Long fuel trim
            resp.data[3] = (uint8_t)((float)(int8_t)p.ltft * 1.28f + 128.5f);
            break;
        case 0x43:  // Absolute engine load: (A*256 + B) / 2.55 %
            // p.abs_load e deja byte-ul OBD2 raw (stocat ca quantize [0,255] = identity)
            // CSV-ul contine valoarea byte OBD2 directa, nu procentul
            resp.data[0] = 0x04;
            resp.data[3] = 0;
            resp.data[4] = p.abs_load;
            break;
        case 0x49:  // Accelerator pedal D
            resp.data[3] = p.pedal_d;
            break;
        case 0x4A:  // Accelerator pedal E
            resp.data[3] = p.pedal_e;
            break;
        case 0x4C:  // Commanded throttle
            resp.data[3] = p.cmd_throt;
            break;
        case 0x30:  // Warmups since DTC clear
            resp.data[3] = p.warmups;
            break;
        case 0x31:  // Distance since DTC clear (km): A*256 + B
            resp.data[0] = 0x04;
            resp.data[3] = p.dist_clear >> 8;
            resp.data[4] = p.dist_clear & 0xFF;
            break;
        case 0x42: {  // Control module voltage: (A*256 + B) / 1000 V
            resp.data[0] = 0x04;
            uint16_t mv = (uint16_t)((p.vbat + 5) * 1000);
            resp.data[3] = mv >> 8;
            resp.data[4] = mv & 0xFF;
            break;
        }
        default:
            known = false;
            break;
    }

    if (known) {
        can_send(&resp);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Redeseneaza ecranul cu datele curente ale simulatorului
// Layout full 320x172px:
//   y=2  : bara status (Row, PLAYING/PAUSED)
//   y=12 : linie
//   y=14 : eticheta RPM
//   y=20 : valoare RPM (size 5, 40px) → pana la y=60
//   y=62 : linie
//   y=66 : rand 1: SPD / ECT / IAT
//   y=80 : rand 2: LOAD / TPS / THR
//   y=94 : rand 3: MAF / STFT / LTFT
//   y=108: rand 4: VBAT / TMG / BRK
//   y=122: rand 5: O2V1 / O2V2 / PEDAL
//   y=136: linie
//   y=140: DTC status
// ─────────────────────────────────────────────────────────────────────────────
static void tft_update_sim(const CarParameter& p) {
    if (!g_tft) return;

    PlaybackState* st = playback_get_state();

    // ── Bara status ──────────────────────────────────────────────────────────
    g_tft->setTextColor(DARK_TEXT, BABY_BLUE);
    g_tft->setTextSize(1);
    g_tft->setCursor(8, 2);
    g_tft->printf("SIM  Row:%3u/%-3u   %-7s",
        st->current_row, st->total_rows,
        st->paused ? "PAUSED" : "PLAYING");

    g_tft->drawFastHLine(0, 12, 320, DARK_TEXT);

    // ── RPM (centrat, font mare) ─────────────────────────────────────────────
    g_tft->setTextSize(1);
    g_tft->setCursor(130, 14);
    g_tft->print("RPM");

    g_tft->setTextSize(5);   // 30px/char latime, 40px inaltime
    g_tft->setCursor(100, 20);
    g_tft->printf("%4u", p.rpm);

    g_tft->drawFastHLine(0, 62, 320, DARK_TEXT);

    // ── 5 randuri de date (size 1 = 6px/char, 8px inaltime, pas 14px) ───────
    // Coloane: x=8, x=120, x=232
    g_tft->setTextSize(1);

    int ect_c   = (int)p.coolant - 40;
    int iat_c   = (int)p.iat     - 40;
    int load_p  = (int)p.load_percent * 100 / 255;
    int tps_p   = (int)p.tps_percent  * 100 / 255;
    int thr_p   = (int)p.cmd_throt    * 100 / 255;
    int stft_p  = (int)(int8_t)p.stft;   // two's complement → valoare % directa
    int ltft_p  = (int)(int8_t)p.ltft;
    int pdl_p   = (int)p.pedal_d      * 100 / 255;
    float vbat  = (float)(p.vbat) + 5.0f;
    float o2v1  = p.o2v1 / 200.0f;
    float o2v2  = p.o2v2 / 200.0f;
    int   tmg   = (int)(int8_t)p.timing;

    // Rand 1: viteza / temperatura apa / temperatura aer admisie
    g_tft->setCursor(  8, 66); g_tft->printf("SPD:%3u km/h", p.speed);
    g_tft->setCursor(120, 66); g_tft->printf("ECT:%4d C  ", ect_c);
    g_tft->setCursor(232, 66); g_tft->printf("IAT:%4d C  ", iat_c);

    // Rand 2: sarcina / pozitie acceleratie / comanda acceleratie
    g_tft->setCursor(  8, 80); g_tft->printf("LOAD:%3u%%   ", load_p);
    g_tft->setCursor(120, 80); g_tft->printf("TPS :%3u%%   ", tps_p);
    g_tft->setCursor(232, 80); g_tft->printf("THR :%3u%%   ", thr_p);

    // Rand 3: debit aer / corectii carburant
    g_tft->setCursor(  8, 94); g_tft->printf("MAF :%4u    ", p.maf);
    g_tft->setCursor(120, 94); g_tft->printf("STFT:%+4d%%  ", stft_p);
    g_tft->setCursor(232, 94); g_tft->printf("LTFT:%+4d%%  ", ltft_p);

    // Rand 4: baterie / avans aprindere / frana
    g_tft->setCursor(  8, 108); g_tft->printf("VBAT:%4.1fV  ", vbat);
    g_tft->setCursor(120, 108); g_tft->printf("TMG :%+4d d  ", tmg);
    g_tft->setCursor(232, 108); g_tft->printf("BRK :%-3s    ", p.brake ? "ON" : "--");

    // Rand 5: senzori O2 / pedala
    g_tft->setCursor(  8, 122); g_tft->printf("O2V1:%5.3fV ", o2v1);
    g_tft->setCursor(120, 122); g_tft->printf("O2V2:%5.3fV ", o2v2);
    g_tft->setCursor(232, 122); g_tft->printf("PDL :%3u%%   ", pdl_p);

    g_tft->drawFastHLine(0, 136, 320, DARK_TEXT);

    // ── Indicator DTC (size 2, vizibil) ─────────────────────────────────────
    g_tft->setTextSize(2);
    uint16_t dc[3]; uint8_t dc_cnt = getActiveDefects(dc, 3);
    if (dc_cnt > 0) {
        g_tft->setTextColor(0xF800, BABY_BLUE);  // rosu
        g_tft->setCursor(8, 140);
        char dtc_buf[32] = {};
        for (uint8_t i = 0; i < dc_cnt; i++) snprintf(dtc_buf + strlen(dtc_buf), sizeof(dtc_buf) - strlen(dtc_buf), "P%04X ", dc[i]);
        g_tft->printf("%-26s", dtc_buf);
    } else {
        g_tft->setTextColor(SIM_GREEN, BABY_BLUE);  // verde
        g_tft->setCursor(8, 140);
        g_tft->print("OK - NO DTC       ");
    }
    // hint buton
    g_tft->setTextSize(1);
    g_tft->setTextColor(DARK_TEXT, BABY_BLUE);
    g_tft->setCursor(8, 162);
    g_tft->printf("BOOT: cycle DTC [%u/3]  ", g_defect_cycle);
    digitalWrite(TFT_CS, HIGH);
}

// ─────────────────────────────────────────────────────────────────────────────
void simulator_init(Adafruit_ST7789& display) {
    g_tft = &display;

    // Buton BOOT pentru cycle defecte
    pinMode(BOOT_BTN_PIN, INPUT_PULLUP);
    g_btn_prev     = true;
    g_defect_cycle = 0;

    // SD CS mentinut HIGH - nu e folosit in mod simulator
    digitalWrite(SD_CS, HIGH);

    // Mesaj initializare pe ecran
    g_tft->fillScreen(BABY_BLUE);
    g_tft->setTextColor(DARK_TEXT, BABY_BLUE);
    g_tft->setTextSize(2);
    g_tft->setCursor(55, 78);
    g_tft->print("SIM INIT...");
    digitalWrite(TFT_CS, HIGH);

    // Incarca datele CSV embedded
    if (!playback_init(true)) {
        g_tft->fillScreen(BABY_BLUE);
        g_tft->setTextColor(0xF800, BABY_BLUE);
        g_tft->setTextSize(2);
        g_tft->setCursor(20, 78);
        g_tft->print("SIM DATA ERROR!");
        while (true) delay(1000);
    }

    // Init CAN bus (TX=20, RX=19 pe C6)
    if (!can_init_500kbps()) {
        Serial.println("[SIM] AVERTISMENT: CAN init esuat - simulatorul ruleaza fara CAN");
    }

    // Init defect injector + meniu serial
    clearAllDefects();
    serial_menu_init();

    g_stats_last_ms = millis();
    g_tft_last_ms   = millis();

    // Ecran ready
    g_tft->fillScreen(BABY_BLUE);
    g_tft->setTextColor(DARK_TEXT, BABY_BLUE);
    g_tft->setTextSize(2);
    g_tft->setCursor(65, 58);
    g_tft->print("SIMULATOR READY");
    g_tft->setTextSize(1);
    g_tft->setCursor(30, 90);
    g_tft->printf("Date: %u randuri   CAN: 500kbps", playback_get_state()->total_rows);
    g_tft->setCursor(30, 106);
    g_tft->print("Serial: meniu defecte activ");
    g_tft->setCursor(30, 122);
    g_tft->print("Pinuri CAN: TX=20  RX=19");
    digitalWrite(TFT_CS, HIGH);
    delay(2000);
    g_tft->fillScreen(BABY_BLUE);
    digitalWrite(TFT_CS, HIGH);
}

// ─────────────────────────────────────────────────────────────────────────────
void simulator_loop() {
    PlaybackState* state = playback_get_state();

    // ── Buton BOOT: falling edge → cycle defect ──────────────────────────────
    bool btn_now = (bool)digitalRead(BOOT_BTN_PIN);
    if (g_btn_prev && !btn_now) {   // falling edge (active-low)
        cycle_defect();
        // Forteaza refresh TFT imediat
        g_tft_last_ms = 0;
    }
    g_btn_prev = btn_now;

    // Actualizeaza meniul serial (non-blocking)
    const CarParameter* all_data = get_embedded_sample_data();
    size_t cur_idx = (state->current_row > 0) ? (state->current_row - 1) : 0;
    serial_menu_update(&all_data[cur_idx]);

    // ── Avanseaza randul CSV la intervalul corespunzator ─────────────────────
    if (playback_is_time_for_next_row()) {
        CarParameter param = playback_get_next_row();

        // Aplica defectele active
        if (isDefectActive(DEFECT_P0335)) {
            DefectMode dm = {DEFECT_P0335, true, SEVERITY_CRITICAL};
            applyDefect(&param, dm);
        }
        if (isDefectActive(DEFECT_P0101)) {
            DefectMode dm = {DEFECT_P0101, true, SEVERITY_ERROR};
            applyDefect(&param, dm);
        }
        if (isDefectActive(DEFECT_P0128)) {
            DefectMode dm = {DEFECT_P0128, true, SEVERITY_WARNING};
            applyDefect(&param, dm);
        }

        g_last_param = param;   // retine pentru display si raspunsuri OBD

        playback_record_success();
        playback_update_timestamp();
    }

    // ── Raspunde la cererile OBD2 venite de la P4 (request/response ECU) ─────
    // P4 trimite cereri ID=0x7DF la fiecare ~20ms; raspundem cu ID=0x7E8
    {
        can_frame_t req;
        while (can_recv(&req)) {
            handle_obd_request(req);
        }
    }

    uint32_t now = millis();

    // ── Update TFT la ~1s ────────────────────────────────────────────────────
    if (now - g_tft_last_ms > 900) {
        g_tft_last_ms = now;
        tft_update_sim(g_last_param);
    }

    // ── Statistici seriale la 5s ─────────────────────────────────────────────
    if (now - g_stats_last_ms >= 5000) {
        g_stats_last_ms = now;
        char buf[128];
        playback_get_stats(buf, sizeof(buf));
        Serial.printf("[SIM] %s\n", buf);
    }

    delayMicroseconds(100);
}
