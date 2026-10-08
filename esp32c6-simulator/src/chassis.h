#pragma once
#include <Adafruit_ST7789.h>

// Mod "SIMULARE SASIU": trimite ciclic pe CAN mesaje de caroserie in stil VAG
// (contact, usi, incuiere, frana de mana, baterie, geamuri) si raspunde la cererile OBD-II de baza.
void chassis_init(Adafruit_ST7789& display);
void chassis_loop();
