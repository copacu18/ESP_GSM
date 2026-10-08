#include "serial_menu.h"
#include <Arduino.h>
#include <stdio.h>
#include <string.h>

// Global menu context
static SerialMenu menu_ctx = {
    .current_state = MENU_STATE_MAIN,
    .previous_state = MENU_STATE_MAIN,
    .playback_speed = PLAYBACK_SPEED_1X,
    .last_input_time = 0,
    .display_refresh_time = 0,
    .input_index = 0,
    .needs_redraw = true
};

// Defect database
static const struct {
    uint16_t code;
    const char* name;
    const char* description;
    const char* effect;
} defect_database[] = {
    { DEFECT_P0335, "P0335", "Crankshaft Position Sensor",
      "RPM/Timing = 0, engine won't run" },
    { DEFECT_P0101, "P0101", "Mass Air Flow Sensor",
      "MAF/Load = 0, lean condition" },
    { DEFECT_P0128, "P0128", "Coolant Thermostat",
      "Cold start issue, improper warming" }
};

#define DEFECT_COUNT (sizeof(defect_database) / sizeof(defect_database[0]))

// Forward declarations
static void serial_menu_draw_ui(void);
static void serial_menu_handle_main_selection(char selection);
static void serial_menu_handle_parameters_selection(char selection);
static void serial_menu_handle_manage_defects_selection(char selection);
static void serial_menu_handle_defect_list_selection(char selection);
static void serial_menu_handle_adjust_speed_selection(char selection);

// ============================================================================
// UTILITY FUNCTIONS
// ============================================================================

void serial_menu_print_line(uint8_t width) {
    for (uint8_t i = 0; i < width; i++) {
        Serial.write('=');
    }
    Serial.write('\n');
}

const char* serial_menu_get_defect_name(uint16_t code) {
    for (uint8_t i = 0; i < DEFECT_COUNT; i++) {
        if (defect_database[i].code == code) {
            return defect_database[i].name;
        }
    }
    return "UNKNOWN";
}

const char* serial_menu_get_defect_description(uint16_t code) {
    for (uint8_t i = 0; i < DEFECT_COUNT; i++) {
        if (defect_database[i].code == code) {
            return defect_database[i].description;
        }
    }
    return "Unknown defect";
}

const char* serial_menu_get_defect_effect(uint16_t code) {
    for (uint8_t i = 0; i < DEFECT_COUNT; i++) {
        if (defect_database[i].code == code) {
            return defect_database[i].effect;
        }
    }
    return "No effect data";
}

const char* serial_menu_get_title(MenuState state) {
    switch (state) {
        case MENU_STATE_MAIN:
            return "ESP32 OBD-II CAR SIMULATOR";
        case MENU_STATE_VIEW_PARAMETERS:
            return "CURRENT PARAMETERS";
        case MENU_STATE_MANAGE_DEFECTS:
            return "MANAGE DEFECTS";
        case MENU_STATE_DEFECT_LIST:
            return "VIEW DEFECT LIST";
        case MENU_STATE_ADJUST_SPEED:
            return "ADJUST PLAYBACK SPEED";
        case MENU_STATE_DEFECT_DETAILS:
            return "DEFECT DETAILS";
        default:
            return "MENU";
    }
}

float serial_menu_get_speed_multiplier(void) {
    const float speeds[] = { 0.5f, 1.0f, 2.0f, 4.0f };
    return speeds[menu_ctx.playback_speed];
}

PlaybackSpeed serial_menu_get_playback_speed(void) {
    return menu_ctx.playback_speed;
}

void serial_menu_set_playback_speed(PlaybackSpeed speed) {
    if (speed < 4) {
        menu_ctx.playback_speed = speed;
        menu_ctx.needs_redraw = true;
    }
}

void serial_menu_clear_input(void) {
    memset(menu_ctx.input_buffer, 0, sizeof(menu_ctx.input_buffer));
    menu_ctx.input_index = 0;
}

SerialMenu* serial_menu_get_context(void) {
    return &menu_ctx;
}

// ============================================================================
// MENU DISPLAY FUNCTIONS
// ============================================================================

void serial_menu_display_main(void) {
    Serial.print("\n\n");
    serial_menu_print_line(50);
    Serial.printf("  %s\n", serial_menu_get_title(MENU_STATE_MAIN));
    serial_menu_print_line(50);
    
    // Display active defects
    uint16_t active_codes[3] = {0};
    uint8_t active_count = getActiveDefects(active_codes, 3);
    
    Serial.print("Current Defects: ");
    if (active_count == 0) {
        Serial.println("NONE");
    } else {
        for (uint8_t i = 0; i < active_count; i++) {
            if (i > 0) Serial.print(", ");
            Serial.print(serial_menu_get_defect_name(active_codes[i]));
        }
        Serial.write('\n');
    }
    
    // Display speed - bounds check to prevent out-of-bounds access
    const char* speed_str[] = { "0.5x", "1x", "2x", "4x" };
    uint16_t rows_per_sec[] = { 30, 60, 120, 240 };
    uint8_t speed_idx = menu_ctx.playback_speed;
    if (speed_idx >= 4) {
        speed_idx = 1;  // Default to 1x if invalid
    }
    Serial.printf("Current Speed: %s (%u rows/sec)\n\n",
        speed_str[speed_idx],
        rows_per_sec[speed_idx]);
    
    Serial.println("MAIN MENU:");
    Serial.println("[1] View Parameters");
    Serial.println("[2] Manage Defects");
    Serial.println("[3] View Defect List");
    Serial.println("[4] Adjust Speed");
    Serial.println("[5] Reset All");
    Serial.println("[0] Exit Menu");
    Serial.print("\nSelect: ");
}

void serial_menu_display_parameters(void) {
    Serial.print("\n\n");
    serial_menu_print_line(50);
    Serial.printf("  %s\n", serial_menu_get_title(MENU_STATE_VIEW_PARAMETERS));
    serial_menu_print_line(50);
    Serial.write('\n');
    
    // Convert temperature (stored with offset of 40)
    int16_t coolant_c = (int16_t)menu_ctx.current_params.coolant - 40;
    int16_t iat_c = (int16_t)menu_ctx.current_params.iat - 40;
    
    // Convert MAF and O2 from quantized values
    float maf_gs = (menu_ctx.current_params.maf * 655.35f) / 255.0f;
    float o2_1_v = (menu_ctx.current_params.o2v1 * 1.275f) / 255.0f;
    float o2_2_v = (menu_ctx.current_params.o2v2 * 1.275f) / 255.0f;
    float tps_pct = (menu_ctx.current_params.tps_percent * 100.0f) / 255.0f;
    
    Serial.println("Parameter                Value");
    Serial.println("----------------------------------------");
    Serial.printf("RPM                      %u rpm\n", menu_ctx.current_params.rpm);
    Serial.printf("Speed                    %u km/h\n", menu_ctx.current_params.speed);
    Serial.printf("Coolant                  %d°C\n", coolant_c);
    Serial.printf("IAT                      %d°C\n", iat_c);
    Serial.printf("MAF                      %.1f g/s\n", maf_gs);
    Serial.printf("Load                     %u%%\n", menu_ctx.current_params.load_percent);
    Serial.printf("TPS                      %.1f%%\n", tps_pct);
    Serial.printf("Timing                   %d°\n", menu_ctx.current_params.timing);
    Serial.printf("O2 Sensor 1              %.2f V\n", o2_1_v);
    Serial.printf("O2 Sensor 2              %.2f V\n", o2_2_v);
    Serial.printf("STFT                     %d%%\n", menu_ctx.current_params.stft);
    Serial.printf("LTFT                     %d%%\n", menu_ctx.current_params.ltft);
    Serial.printf("Steer (raw)              %u\n", menu_ctx.current_params.steer);
    Serial.printf("Brake                    %s\n", 
        menu_ctx.current_params.brake ? "ON" : "OFF");
    
    Serial.println("\n[1] Refresh");
    Serial.println("[0] Back");
    Serial.print("Select: ");
}

void serial_menu_display_manage_defects(void) {
    Serial.print("\n\n");
    serial_menu_print_line(50);
    Serial.printf("  %s\n", serial_menu_get_title(MENU_STATE_MANAGE_DEFECTS));
    serial_menu_print_line(50);
    
    // Display active defects - defensive check for null
    uint16_t active_codes[3] = {0};
    uint8_t active_count = 0;
    
    // Safely call getActiveDefects with bounds checking
    if (active_codes != NULL) {
        active_count = getActiveDefects(active_codes, 3);
        if (active_count > 3) {
            active_count = 3;  // Clamp to array size
        }
    }
    
    Serial.print("Active defects: ");
    if (active_count == 0) {
        Serial.println("NONE");
    } else {
        for (uint8_t i = 0; i < active_count; i++) {
            if (i > 0) Serial.print(", ");
            Serial.print(serial_menu_get_defect_name(active_codes[i]));
        }
        Serial.write('\n');
    }
    
    Serial.println("\n[1] Inject Defect:");
    for (uint8_t i = 0; i < DEFECT_COUNT; i++) {
        Serial.printf("    [1.%u] %s (%s)\n",
            i + 1,
            serial_menu_get_defect_name(defect_database[i].code),
            serial_menu_get_defect_description(defect_database[i].code));
    }
    
    Serial.println("[2] Clear All Active Defects");
    Serial.println("[3] View Defect Details");
    Serial.println("[0] Back to Main Menu");
    Serial.print("\nSelect: ");
}

void serial_menu_display_defect_list(void) {
    Serial.print("\n\n");
    serial_menu_print_line(50);
    Serial.printf("  %s\n", serial_menu_get_title(MENU_STATE_DEFECT_LIST));
    serial_menu_print_line(50);
    Serial.write('\n');
    
    for (uint8_t i = 0; i < DEFECT_COUNT; i++) {
        Serial.printf("[%u] %s - %s\n",
            i + 1,
            serial_menu_get_defect_name(defect_database[i].code),
            serial_menu_get_defect_description(defect_database[i].code));
        Serial.printf("    Effect: %s\n",
            serial_menu_get_defect_effect(defect_database[i].code));
    }
    
    Serial.println("\n[0] Back");
    Serial.print("Select: ");
}

void serial_menu_display_adjust_speed(void) {
    Serial.print("\n\n");
    serial_menu_print_line(50);
    Serial.printf("  %s\n", serial_menu_get_title(MENU_STATE_ADJUST_SPEED));
    serial_menu_print_line(50);
    Serial.write('\n');
    
    const char* speed_str[] = { "0.5x", "1x", "2x", "4x" };
    uint16_t rows_per_sec[] = { 30, 60, 120, 240 };
    
    Serial.println("Available playback speeds:");
    for (uint8_t i = 0; i < 4; i++) {
        char marker = (i == menu_ctx.playback_speed) ? '*' : ' ';
        Serial.printf("[%u] %s (%u rows/sec) %c\n",
            i + 1, speed_str[i], rows_per_sec[i], marker);
    }
    
    Serial.println("\n[0] Back");
    Serial.print("Select: ");
}

void serial_menu_display_defect_details(void) {
    Serial.print("\n\n");
    serial_menu_print_line(50);
    Serial.printf("  %s\n", serial_menu_get_title(MENU_STATE_DEFECT_DETAILS));
    serial_menu_print_line(50);
    Serial.write('\n');
    
    Serial.println("Active Defects Detail:");
    
    uint16_t active_codes[3] = {0};
    uint8_t active_count = getActiveDefects(active_codes, 3);
    
    if (active_count == 0) {
        Serial.println("\nNo active defects.\n");
    } else {
        for (uint8_t i = 0; i < active_count; i++) {
            Serial.printf("\n--- %s ---\n", 
                serial_menu_get_defect_name(active_codes[i]));
            Serial.printf("Description: %s\n",
                serial_menu_get_defect_description(active_codes[i]));
            Serial.printf("Effect: %s\n",
                serial_menu_get_defect_effect(active_codes[i]));
        }
        Serial.write('\n');
    }
    
    Serial.println("[0] Back");
    Serial.print("Select: ");
}

// ============================================================================
// INPUT HANDLING
// ============================================================================

void serial_menu_handle_main_selection(char selection) {
    switch (selection) {
        case '1':
            menu_ctx.current_state = MENU_STATE_VIEW_PARAMETERS;
            menu_ctx.needs_redraw = true;
            break;
        case '2':
            menu_ctx.current_state = MENU_STATE_MANAGE_DEFECTS;
            menu_ctx.needs_redraw = true;
            break;
        case '3':
            menu_ctx.current_state = MENU_STATE_DEFECT_LIST;
            menu_ctx.needs_redraw = true;
            break;
        case '4':
            menu_ctx.current_state = MENU_STATE_ADJUST_SPEED;
            menu_ctx.needs_redraw = true;
            break;
        case '5':
            clearAllDefects();
            menu_ctx.playback_speed = PLAYBACK_SPEED_1X;
            Serial.println("\n\n*** All parameters reset ***\n");
            menu_ctx.needs_redraw = true;
            break;
        case '0':
            menu_ctx.current_state = MENU_STATE_MAIN;
            menu_ctx.needs_redraw = true;
            break;
    }
}

void serial_menu_handle_parameters_selection(char selection) {
    switch (selection) {
        case '1':
            menu_ctx.needs_redraw = true;
            break;
        case '0':
            menu_ctx.current_state = MENU_STATE_MAIN;
            menu_ctx.needs_redraw = true;
            break;
    }
}

void serial_menu_handle_manage_defects_selection(char selection) {
    switch (selection) {
        case '1':
            // Inject defect submenu - parse next input as 1.X
            // For simplicity, we'll inject all defects on first press
            for (uint8_t i = 0; i < DEFECT_COUNT; i++) {
                enableDefect(defect_database[i].code);
            }
            Serial.println("\n\n*** All defects injected ***\n");
            menu_ctx.needs_redraw = true;
            break;
        case '2':
            clearAllDefects();
            Serial.println("\n\n*** All defects cleared ***\n");
            menu_ctx.needs_redraw = true;
            break;
        case '3':
            menu_ctx.current_state = MENU_STATE_DEFECT_DETAILS;
            menu_ctx.needs_redraw = true;
            break;
        case '0':
            menu_ctx.current_state = MENU_STATE_MAIN;
            menu_ctx.needs_redraw = true;
            break;
    }
}

void serial_menu_handle_defect_list_selection(char selection) {
    switch (selection) {
        case '0':
            menu_ctx.current_state = MENU_STATE_MAIN;
            menu_ctx.needs_redraw = true;
            break;
    }
}

void serial_menu_handle_adjust_speed_selection(char selection) {
    switch (selection) {
        case '1':
            menu_ctx.playback_speed = PLAYBACK_SPEED_HALF;
            menu_ctx.needs_redraw = true;
            break;
        case '2':
            menu_ctx.playback_speed = PLAYBACK_SPEED_1X;
            menu_ctx.needs_redraw = true;
            break;
        case '3':
            menu_ctx.playback_speed = PLAYBACK_SPEED_2X;
            menu_ctx.needs_redraw = true;
            break;
        case '4':
            menu_ctx.playback_speed = PLAYBACK_SPEED_4X;
            menu_ctx.needs_redraw = true;
            break;
        case '0':
            menu_ctx.current_state = MENU_STATE_MAIN;
            menu_ctx.needs_redraw = true;
            break;
    }
}

void serial_menu_handle_selection(char selection) {
    switch (menu_ctx.current_state) {
        case MENU_STATE_MAIN:
            serial_menu_handle_main_selection(selection);
            break;
        case MENU_STATE_VIEW_PARAMETERS:
            serial_menu_handle_parameters_selection(selection);
            break;
        case MENU_STATE_MANAGE_DEFECTS:
            serial_menu_handle_manage_defects_selection(selection);
            break;
        case MENU_STATE_DEFECT_LIST:
            serial_menu_handle_defect_list_selection(selection);
            break;
        case MENU_STATE_ADJUST_SPEED:
            serial_menu_handle_adjust_speed_selection(selection);
            break;
        case MENU_STATE_DEFECT_DETAILS:
            if (selection == '0') {
                menu_ctx.current_state = MENU_STATE_MANAGE_DEFECTS;
                menu_ctx.needs_redraw = true;
            }
            break;
    }
}

// ============================================================================
// INPUT PROCESSING
// ============================================================================

void serial_menu_process_input(void) {
    while (Serial.available()) {
        char c = Serial.read();
        menu_ctx.last_input_time = millis();
        
        // Echo back to user
        if (c == '\r' || c == '\n') {
            Serial.write('\n');
            
            if (menu_ctx.input_index > 0) {
                serial_menu_handle_selection(menu_ctx.input_buffer[0]);
                serial_menu_clear_input();
            }
        } else if (c == '\b' || c == 0x7F) {
            // Backspace
            if (menu_ctx.input_index > 0) {
                menu_ctx.input_index--;
                menu_ctx.input_buffer[menu_ctx.input_index] = 0;
                Serial.write('\b');
                Serial.write(' ');
                Serial.write('\b');
            }
        } else if (c >= 32 && c <= 126) {
            // Printable character
            if (menu_ctx.input_index < sizeof(menu_ctx.input_buffer) - 1) {
                menu_ctx.input_buffer[menu_ctx.input_index++] = c;
                Serial.write(c);
            }
        }
    }
}

static void serial_menu_draw_ui(void) {
    switch (menu_ctx.current_state) {
        case MENU_STATE_MAIN:
            serial_menu_display_main();
            break;
        case MENU_STATE_VIEW_PARAMETERS:
            serial_menu_display_parameters();
            break;
        case MENU_STATE_MANAGE_DEFECTS:
            serial_menu_display_manage_defects();
            break;
        case MENU_STATE_DEFECT_LIST:
            serial_menu_display_defect_list();
            break;
        case MENU_STATE_ADJUST_SPEED:
            serial_menu_display_adjust_speed();
            break;
        case MENU_STATE_DEFECT_DETAILS:
            serial_menu_display_defect_details();
            break;
    }
}

// ============================================================================
// PUBLIC API
// ============================================================================

void serial_menu_init(void) {
    Serial.begin(115200);
    delay(100);
    
    menu_ctx.current_state = MENU_STATE_MAIN;
    menu_ctx.playback_speed = PLAYBACK_SPEED_1X;
    menu_ctx.needs_redraw = true;
    menu_ctx.display_refresh_time = millis();
    
    // Initialize current_params to safe defaults to prevent null pointer crashes
    memset(&menu_ctx.current_params, 0, sizeof(CarParameter));
    menu_ctx.current_params.rpm = 0;
    menu_ctx.current_params.speed = 0;
    menu_ctx.current_params.tps_percent = 0;
    menu_ctx.current_params.tps_voltage = 0;
    menu_ctx.current_params.load_percent = 0;
    menu_ctx.current_params.abs_load = 0;
    menu_ctx.current_params.coolant = 95;      // ~55°C (95 - 40 offset)
    menu_ctx.current_params.iat = 50;          // ~10°C (50 - 40 offset)
    menu_ctx.current_params.maf = 5;
    menu_ctx.current_params.timing = 5;
    menu_ctx.current_params.pedal_d = 0;
    menu_ctx.current_params.pedal_e = 0;
    menu_ctx.current_params.cmd_throt = 10;
    menu_ctx.current_params.o2v1 = 50;
    menu_ctx.current_params.o2v2 = 50;
    menu_ctx.current_params.stft = 0;
    menu_ctx.current_params.ltft = 0;
    menu_ctx.current_params.vbat = 10;         // ~15V (10 + 5 offset)
    menu_ctx.current_params.steer = 900;
    menu_ctx.current_params.brake = 0;
    menu_ctx.current_params.warmups = 2;
    menu_ctx.current_params.dist_clear = 100;
    
    Serial.println("\n\n========================================");
    Serial.println("  ESP32 OBD-II CAR SIMULATOR MENU");
    Serial.println("========================================\n");
}

void serial_menu_update(const CarParameter* current_params) {
    // Always update current_params, use defaults if NULL pointer passed
    if (current_params != NULL) {
        memcpy(&menu_ctx.current_params, current_params, sizeof(CarParameter));
    } else {
        // If NULL is passed, ensure structure isn't left uninitialized
        memset(&menu_ctx.current_params, 0, sizeof(CarParameter));
    }
    
    serial_menu_process_input();
    
    // ONLY redraw when explicitly needed (user input or state change)
    // NOT on timer - this prevents constant menu spam
    if (menu_ctx.needs_redraw) {
        serial_menu_draw_ui();
        menu_ctx.needs_redraw = false;
        menu_ctx.display_refresh_time = millis();
    }
}
