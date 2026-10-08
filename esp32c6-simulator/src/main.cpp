#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <Preferences.h>
#include "logger.h"
#include "simulator.h"
#include "chassis.h"

// ─── Hardware pins ────────────────────────────────────────────────────────────
#define BOOT_BTN   9   // GPIO9 = butonul BOOT pe ESP32-C6-DevKitC-1 (active LOW)
#define SD_CS      4
#define TFT_CS    14
#define TFT_DC    15
#define TFT_RST   21
#define TFT_BL    22
// SPI: SCK=7, MISO=5, MOSI=6

// ─── Culori TFT ───────────────────────────────────────────────────────────────
#define BABY_BLUE  0x8E79
#define DARK_TEXT  0x2104
#define SEL_YELLOW 0xFFC0
#define SEL_RED    0xF800


// ─── Mod de operare ──────────────────────────────────────────────────────────
typedef enum { MODE_LOGGER = 0, MODE_SIMULATOR = 1, MODE_CHASSIS = 2 } AppMode;
#define MODE_COUNT 3

static const char* mode_name(AppMode m) {
    switch (m) {
        case MODE_LOGGER:    return "LOGGER";
        case MODE_SIMULATOR: return "SIMULATOR";
        default:             return "SASIU";
    }
}

// ─── Global TFT (extern-uit in logger.h si simulator.h) ──────────────────────
Adafruit_ST7789 tft = Adafruit_ST7789(&SPI, TFT_CS, TFT_DC, TFT_RST);

static AppMode selected_mode = MODE_LOGGER;

// ─────────────────────────────────────────────────────────────────────────────
// Deseneaza meniul de boot pe ecran
// ─────────────────────────────────────────────────────────────────────────────
// Deseneaza elementele statice (o singura data, inainte de loop).
static void draw_boot_menu_static() {
    tft.setTextColor(DARK_TEXT, BABY_BLUE);
    tft.setTextSize(2);
    tft.setCursor(75, 5);
    tft.print("SELECT MODE");
}

// Deseneaza / actualizeaza elementele dinamice (casete + countdown).
// Fara fillScreen → fara flicker.
static void draw_boot_menu(AppMode mode, int countdown_s) {
    // Trei casete: LOGGER, SIMULATOR, SASIU
    static const char* labels[MODE_COUNT]   = { "LOGGER", "SIMULATOR", "SIMULARE SASIU" };
    static const char* labels_s[MODE_COUNT] = { "> LOGGER <", "> SIMULATOR <", "> SIMULARE SASIU <" };
    for (int i = 0; i < MODE_COUNT; i++) {
        int y = 26 + i * 36;
        bool sel = ((int)mode == i);
        uint16_t border = sel ? SEL_RED : DARK_TEXT;
        uint16_t bg     = sel ? SEL_YELLOW : BABY_BLUE;
        tft.drawRect(25, y, 270, 32, border);
        tft.fillRect(26, y + 1, 268, 30, bg);
        tft.setTextColor(DARK_TEXT, bg);
        tft.setTextSize(2);
        const char* s = sel ? labels_s[i] : labels[i];
        tft.setCursor(160 - (int)strlen(s) * 6, y + 9);
        tft.print(s);
    }

    // Linia de countdown — intreaga, o singura bucata, fara split
    tft.setTextColor(DARK_TEXT, BABY_BLUE);
    tft.setTextSize(1);
    tft.setCursor(18, 138);
    tft.printf("BOOT btn = schimba mod    pornire in %2d secunde", countdown_s);
}

// ─────────────────────────────────────────────────────────────────────────────
// SETUP: init TFT, citeste NVS, meniu 5s, porneste modul ales
// ─────────────────────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);

    // SD CS HIGH cat mai devreme, ca sa nu interfereze cu SPI la init TFT
    pinMode(SD_CS, OUTPUT);  digitalWrite(SD_CS,  HIGH);

    // TFT backlight + CS
    pinMode(TFT_BL, OUTPUT); digitalWrite(TFT_BL, HIGH);
    pinMode(TFT_CS, OUTPUT); digitalWrite(TFT_CS, HIGH);

    // SPI: SCK=7, MISO=5, MOSI=6
    SPI.begin(7, 5, 6, -1);
    tft.init(172, 320);
    tft.setRotation(1);
    tft.fillScreen(BABY_BLUE);

    // Buton BOOT (GPIO9, active LOW, pull-up intern)
    pinMode(BOOT_BTN, INPUT_PULLUP);

    // Citeste ultimul mod ales din NVS (implicit: LOGGER)
    Preferences prefs;
    prefs.begin("c6app", true);
    int saved_mode = prefs.getInt("mode", (int)MODE_LOGGER);
    selected_mode = (saved_mode >= 0 && saved_mode < MODE_COUNT) ? (AppMode)saved_mode : MODE_LOGGER;
    prefs.end();

    // ── Meniu de boot: 15 secunde countdown ─────────────────────────────────
    const unsigned long BOOT_MS = 15000;
    unsigned long t_start = millis();
    int  last_drawn_sec  = -1;
    bool btn_prev        = HIGH;
    bool mode_changed    = true;   // forteaza prima desenare a casetelor

    draw_boot_menu_static();       // titlu + legenda → o singura data, fara flicker

    while (millis() - t_start < BOOT_MS) {
        bool btn_now = (bool)digitalRead(BOOT_BTN);

        if (btn_prev == HIGH && btn_now == LOW) {
            selected_mode = (AppMode)(((int)selected_mode + 1) % MODE_COUNT);
            mode_changed  = true;
            delay(60);   // debounce
        }
        btn_prev = btn_now;

        int remaining = (int)((BOOT_MS - (millis() - t_start)) / 1000);
        if (mode_changed || remaining != last_drawn_sec) {
            draw_boot_menu(selected_mode, remaining);
            last_drawn_sec = remaining;
            mode_changed   = false;
        }
        delay(20);
    }

    // Salveaza modul ales in NVS pentru urmatorul boot
    prefs.begin("c6app", false);
    prefs.putInt("mode", (int)selected_mode);
    prefs.end();

    // Confirmare vizuala scurta
    tft.fillScreen(BABY_BLUE);
    tft.setTextColor(DARK_TEXT, BABY_BLUE);
    tft.setTextSize(3);
    tft.setCursor(55, 58);
    tft.print(mode_name(selected_mode));
    tft.setTextSize(2);
    tft.setCursor(85, 92);
    tft.print("starting...");
    delay(900);

    // ── Pornire mod ales ─────────────────────────────────────────────────────
    if (selected_mode == MODE_LOGGER) {
        logger_init(tft);
    } else if (selected_mode == MODE_SIMULATOR) {
        simulator_init(tft);
    } else {
        chassis_init(tft);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// LOOP: delega la modul activ
// ─────────────────────────────────────────────────────────────────────────────
void loop() {
    if (selected_mode == MODE_LOGGER) {
        logger_loop();
    } else if (selected_mode == MODE_SIMULATOR) {
        simulator_loop();
    } else {
        chassis_loop();
    }
}
