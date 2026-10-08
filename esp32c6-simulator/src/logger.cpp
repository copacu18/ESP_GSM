#include "logger.h"
#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
#include <WiFi.h>
#include <time.h>
#include "driver/twai.h"

// ─── Pinuri hardware ──────────────────────────────────────────────────────────
#define CAN_TX_PIN  20
#define CAN_RX_PIN  19
#define SD_CS        4
#define TFT_CS      14

// ─── Culori ───────────────────────────────────────────────────────────────────
#define BABY_BLUE  0x8E79
#define DARK_TEXT  0x2104
#define REC_RED    0xF800

// ─── WiFi / NTP ───────────────────────────────────────────────────────────────
static const char* WIFI_SSID = "copacu";
static const char* WIFI_PASS = "?12345678#";
static const char* TZ_INFO   = "EET-2EEST,M3.5.0/3,M10.5.0/4";

// ─── TFT pointer (init in main.cpp, primit prin logger_init) ─────────────────
static Adafruit_ST7789* g_tft = nullptr;

// ─── Starea datelor logger ────────────────────────────────────────────────────
struct LoggerData {
    uint16_t rpm = 0;   uint8_t speed = 0;
    float load = 0;     float absLoad = 0;
    int8_t coolant = 0; int8_t iat = 0;   float vbat = 0;
    float tps = 0;      float tpsVolts = 0;
    float pedalD = 0;   float pedalE = 0; float cmdThrottle = 0;
    float maf = 0;      float timing = 0;
    float o2v1 = 0;     float o2v2 = 0;
    float stft = 0;     float ltft = 0;
    float baro = 0;     float catTemp = 0;
    uint32_t warmups = 0; uint32_t distSinceClear = 0;
    int16_t steer = 0;  bool brake = false;
    char timeStr[32] = ""; char dateStr[16] = "";
    char logFileName[24] = "";
    unsigned long lastRx = 0;
    bool sdOk = false; bool isLogging = false; bool prevConn = false;
} ld;

// PIDs OBD-II ceruti secvential
static uint8_t pids[] = {
    0x0C, 0x0D, 0x11, 0x04, 0x43, 0x05, 0x0F,
    0x49, 0x4A, 0x4C, 0x10, 0x0E, 0x14, 0x15,
    0x06, 0x07, 0x33, 0x3C, 0x30, 0x31
};
static int pIdx = 0;
static File logFile;

// ─── CS management (SD si TFT pe acelasi bus SPI) ────────────────────────────
static inline void sel_sd()  { digitalWrite(TFT_CS, HIGH); digitalWrite(SD_CS, LOW);  }
static inline void sel_tft() { digitalWrite(SD_CS,  HIGH); digitalWrite(TFT_CS, LOW); }

// ─────────────────────────────────────────────────────────────────────────────
void logger_init(Adafruit_ST7789& display) {
    g_tft = &display;

    // ── SD Card ──────────────────────────────────────────────────────────────
    Serial.println("[SD] Initializare...");
    Serial.printf("[SD] CS=GPIO%d  SPI: SCK=7 MISO=5 MOSI=6\n", SD_CS);
    sel_sd();
    if (SD.begin(SD_CS, SPI, 4000000)) {
        Serial.printf("[SD] Montat OK  tip=%u  marime=%llu MB\n",
            SD.cardType(), SD.cardSize() / (1024ULL * 1024ULL));
        char fn[32];
        for (int i = 0; i < 999; i++) {
            snprintf(fn, sizeof(fn), "/predict_%d.csv", i);
            if (!SD.exists(fn)) break;
        }
        Serial.printf("[SD] Fisier log: %s\n", fn);
        logFile = SD.open(fn, FILE_WRITE);
        if (logFile) {
            logFile.println(
                "Time,RPM,Speed,TPS%,TPS_V,Load%,AbsLoad%,Coolant,IAT,MAF,"
                "Timing,PedalD%,PedalE%,CmdThrot%,O2V1,O2V2,STFT,LTFT,"
                "VBAT,Steer,Brake,Warmups,DistClear");
            Serial.println("[SD] Fisier deschis OK, logging activ");
            strncpy(ld.logFileName, fn + 1, sizeof(ld.logFileName) - 1); // sare peste '/'
            ld.sdOk = true;
        } else {
            Serial.printf("[SD] EROARE: nu s-a putut deschide %s pentru scriere\n", fn);
        }
    } else {
        Serial.println("[SD] EROARE: SD.begin() a esuat");
        Serial.println("[SD]  - verifica daca cardul e introdus");
        Serial.println("[SD]  - verifica tensiunea (3.3V)");
        Serial.println("[SD]  - verifica pinii SPI si CS");
        uint8_t ct = SD.cardType();
        Serial.printf("[SD]  - cardType() returneaza: %u (0=NONE)\n", ct);
    }

    // ── CAN / TWAI ───────────────────────────────────────────────────────────
    twai_general_config_t g_cfg = TWAI_GENERAL_CONFIG_DEFAULT(
        (gpio_num_t)CAN_TX_PIN, (gpio_num_t)CAN_RX_PIN, TWAI_MODE_NORMAL);
    twai_timing_config_t  t_cfg = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t  f_cfg = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    twai_driver_install(&g_cfg, &t_cfg, &f_cfg);
    twai_start();

    // ── WiFi + NTP (timeout 10s, nu bloca la nesfarsit) ──────────────────────
    sel_tft();
    g_tft->fillScreen(BABY_BLUE);
    g_tft->setTextColor(DARK_TEXT, BABY_BLUE);
    g_tft->setTextSize(2);
    g_tft->setCursor(20, 62);
    g_tft->print("SEARCHING HOTSPOT...");
    g_tft->setTextSize(1);
    g_tft->setCursor(20, 94);
    g_tft->printf("SSID: %s", WIFI_SSID);
    digitalWrite(TFT_CS, HIGH);

    WiFi.begin(WIFI_SSID, WIFI_PASS);
    unsigned long wt = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - wt < 10000) {
        delay(500);
        sel_tft();
        g_tft->setTextSize(2);
        g_tft->print(".");
        digitalWrite(TFT_CS, HIGH);
    }

    sel_tft();
    g_tft->fillScreen(BABY_BLUE);
    g_tft->setTextColor(DARK_TEXT, BABY_BLUE);
    g_tft->setTextSize(2);
    if (WiFi.status() == WL_CONNECTED) {
        configTzTime(TZ_INFO, "pool.ntp.org");
        g_tft->setCursor(20, 62);
        g_tft->print("WIFI OK - TIME SYNC");
        struct tm ti;
        if (getLocalTime(&ti)) {
            char buf[32]; strftime(buf, 32, "%H:%M:%S", &ti);
            g_tft->setCursor(20, 94); g_tft->printf("TIME: %s", buf);
        }
    } else {
        g_tft->setCursor(20, 62);
        g_tft->print("NO WIFI - NO NTP");
        g_tft->setCursor(20, 94);
        g_tft->print("Logging continua fara ora");
    }
    digitalWrite(TFT_CS, HIGH);
    delay(1800);

    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);

    sel_tft();
    g_tft->fillScreen(BABY_BLUE);
    digitalWrite(TFT_CS, HIGH);
}

// ─────────────────────────────────────────────────────────────────────────────
void logger_loop() {
    unsigned long now = millis();
    static unsigned long lReq = 0, lLog = 0, lUI = 0;

    // ── Cerere OBD la fiecare 20ms ───────────────────────────────────────────
    if (now - lReq > 20) {
        lReq = now;
        twai_message_t m = {};
        m.identifier = 0x7DF;
        m.data_length_code = 8;
        m.data[0] = 0x02; m.data[1] = 0x01; m.data[2] = pids[pIdx];
        twai_transmit(&m, pdMS_TO_TICKS(5));
        pIdx = (pIdx + 1) % (int)(sizeof(pids));
    }

    // ── Receptie CAN ─────────────────────────────────────────────────────────
    twai_message_t rx;
    while (twai_receive(&rx, 0) == ESP_OK) {
        if (rx.identifier == 0x7E8) {
            ld.lastRx = now;
            uint8_t p = rx.data[2], A = rx.data[3], B = rx.data[4];
            switch (p) {
                case 0x0C: ld.rpm = ((A*256u)+B)/4; break;
                case 0x0D: ld.speed = A; break;
                case 0x11: ld.tps = A*100.0f/255.0f; ld.tpsVolts = A*5.0f/255.0f; break;
                case 0x04: ld.load = A*100.0f/255.0f; break;
                case 0x43: ld.absLoad = ((A*256u)+B)*100.0f/255.0f; break;
                case 0x05: ld.coolant = (int8_t)(A-40); break;
                case 0x0F: ld.iat    = (int8_t)(A-40); break;
                case 0x10: ld.maf    = ((A*256.0f)+B)/100.0f; break;
                case 0x0E: ld.timing = (A-128)/2.0f; break;
                case 0x49: ld.pedalD = A*100.0f/255.0f; break;
                case 0x4A: ld.pedalE = A*100.0f/255.0f; break;
                case 0x14: ld.o2v1   = A/200.0f; break;
                case 0x15: ld.o2v2   = A/200.0f; break;
                case 0x06: ld.stft   = (A-128)*100.0f/128.0f; break;
                case 0x07: ld.ltft   = (A-128)*100.0f/128.0f; break;
                case 0x33: ld.baro   = A; break;
                case 0x3C: ld.catTemp= (((A*256u)+B)/10.0f)-40.0f; break;
                case 0x30: ld.warmups= A; break;
                case 0x31: ld.distSinceClear = (A*256u)+B; break;
                case 0x42: ld.vbat   = (A*256u+B)/1000.0f; break;
            }
        } else if (rx.identifier == 0x120) {
            ld.steer = (int16_t)((rx.data[1]<<8)|rx.data[2]);
        } else if (rx.identifier == 0x114) {
            ld.brake = (rx.data[3] & 0x01);
        }
    }

    bool ecuOnline  = (now - ld.lastRx < 2000);
    ld.isLogging    = (ld.sdOk && ecuOnline);

    // ── Logging SD la fiecare 50ms (20 Hz) ───────────────────────────────────
    if (ld.isLogging && now - lLog > 50) {
        lLog = now;
        char fTS[32]; time_t t_n; time(&t_n);
        strftime(fTS, 32, "%Y-%m-%d %H:%M:%S", localtime(&t_n));
        sel_sd();
        logFile.printf(
            "%s,%u,%u,%.1f,%.2f,%.1f,%.1f,%d,%d,%.2f,%.1f,"
            "%.1f,%.1f,%.1f,%.2f,%.2f,%.1f,%.1f,%.2f,%d,%d,%u,%u\n",
            fTS, ld.rpm, ld.speed, ld.tps, ld.tpsVolts,
            ld.load, ld.absLoad, ld.coolant, ld.iat,
            ld.maf, ld.timing, ld.pedalD, ld.pedalE, ld.cmdThrottle,
            ld.o2v1, ld.o2v2, ld.stft, ld.ltft, ld.vbat,
            ld.steer, (int)ld.brake, ld.warmups, ld.distSinceClear);
        logFile.flush();
    }

    // ── Update TFT la fiecare 200ms ───────────────────────────────────────────
    if (g_tft && now - lUI > 200) {
        lUI = now;
        time_t t_n; time(&t_n);
        struct tm* ti = localtime(&t_n);
        strftime(ld.timeStr, sizeof(ld.timeStr), "%H:%M:%S", ti);
        strftime(ld.dateStr, sizeof(ld.dateStr), "%d/%m/%Y", ti);

        sel_tft();
        if (ecuOnline != ld.prevConn) {
            g_tft->fillScreen(BABY_BLUE);
            ld.prevConn = ecuOnline;
        }

        if (ecuOnline) {
            g_tft->setTextSize(1);

            // Indicator SD/REC + nume fisier (sus, centrat, y=5)
            g_tft->fillRect(80, 2, 200, 14, BABY_BLUE);   // sterge zona
            if (ld.sdOk && ld.isLogging) {
                g_tft->fillCircle(123, 9, 5, REC_RED);
                g_tft->setTextColor(REC_RED, BABY_BLUE);
                g_tft->setCursor(132, 5);
                g_tft->printf("REC  %s", ld.logFileName);
            } else if (ld.sdOk) {
                g_tft->setTextColor(0x07E0, BABY_BLUE);
                g_tft->setCursor(105, 5);
                g_tft->printf("SD:OK  %s", ld.logFileName);
            } else {
                g_tft->setTextColor(0xFBE0, BABY_BLUE);
                g_tft->setCursor(118, 5); g_tft->print("SD: ERR");
            }

            // Data si ora (linia de sus)
            g_tft->setTextColor(DARK_TEXT, BABY_BLUE);
            g_tft->setCursor(10,  30);  g_tft->print(ld.dateStr);
            g_tft->setCursor(220, 30);  g_tft->print(ld.timeStr);
            g_tft->setTextSize(2); g_tft->setCursor(135, 44); g_tft->print("RPM");
            g_tft->setTextSize(5); g_tft->setCursor(100, 62);
            g_tft->printf("%4u", ld.rpm);
            g_tft->drawFastHLine(10, 108, 300, DARK_TEXT);
            g_tft->setTextSize(2);
            g_tft->setCursor(8,   114); g_tft->printf("ECT:%-3dC",   ld.coolant);
            g_tft->setCursor(120, 114); g_tft->printf("SPD:%-3u   ", ld.speed);
            g_tft->setCursor(235, 114); g_tft->printf("IAT:%-3dC",   ld.iat);
            g_tft->setTextSize(1);
            g_tft->setCursor(8,   138); g_tft->printf("O2V1:%.2fV  STFT:%+.1f%%  LTFT:%+.1f%%  VBAT:%.1fV",
                ld.o2v1, ld.stft, ld.ltft, ld.vbat);
        } else {
            g_tft->setTextSize(3);
            g_tft->setTextColor(REC_RED, BABY_BLUE);
            g_tft->setCursor(45, 74); g_tft->print("WAITING ECU");
        }
        digitalWrite(TFT_CS, HIGH);
    }
}
