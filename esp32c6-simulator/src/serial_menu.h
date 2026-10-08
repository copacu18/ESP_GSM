#ifndef SERIAL_MENU_H
#define SERIAL_MENU_H

#include <stdint.h>
#include <stdbool.h>
#include "csv_parser.h"
#include "defect_injector.h"
#include "playback.h"  // Get PlaybackSpeed definition

// Menu state machine enum
typedef enum {
    MENU_STATE_MAIN = 0,
    MENU_STATE_VIEW_PARAMETERS = 1,
    MENU_STATE_MANAGE_DEFECTS = 2,
    MENU_STATE_DEFECT_LIST = 3,
    MENU_STATE_ADJUST_SPEED = 4,
    MENU_STATE_DEFECT_DETAILS = 5
} MenuState;

// Serial menu context structure
typedef struct {
    MenuState current_state;
    MenuState previous_state;
    PlaybackSpeed playback_speed;
    uint32_t last_input_time;
    uint32_t display_refresh_time;
    char input_buffer[16];
    uint8_t input_index;
    bool needs_redraw;
    CarParameter current_params;
} SerialMenu;

// Function declarations

/**
 * Initialize the serial menu
 * Should be called from setup()
 * Initializes Serial at 115200 baud if not already initialized
 */
void serial_menu_init(void);

/**
 * Update the serial menu (call from loop)
 * Handles input processing, display refresh, and state management
 * Non-blocking - uses millis() for timing
 * @param current_params Current car parameters to display
 */
void serial_menu_update(const CarParameter* current_params);

/**
 * Process serial input from user
 * Called by serial_menu_update()
 * Buffers input and processes complete commands
 */
void serial_menu_process_input(void);

/**
 * Display the main menu
 */
void serial_menu_display_main(void);

/**
 * Display view parameters submenu
 */
void serial_menu_display_parameters(void);

/**
 * Display manage defects submenu
 */
void serial_menu_display_manage_defects(void);

/**
 * Display defect list submenu
 */
void serial_menu_display_defect_list(void);

/**
 * Display adjust speed submenu
 */
void serial_menu_display_adjust_speed(void);

/**
 * Display defect details submenu
 */
void serial_menu_display_defect_details(void);

/**
 * Handle user selection in current menu state
 * @param selection Single character/digit representing user choice
 */
void serial_menu_handle_selection(char selection);

/**
 * Get current playback speed multiplier
 * @return Speed multiplier (0.5, 1.0, 2.0, or 4.0)
 */
float serial_menu_get_speed_multiplier(void);

/**
 * Get current playback speed level
 * @return PlaybackSpeed enum value
 */
PlaybackSpeed serial_menu_get_playback_speed(void);

/**
 * Set playback speed
 * @param speed PlaybackSpeed enum value
 */
void serial_menu_set_playback_speed(PlaybackSpeed speed);

/**
 * Clear input buffer
 */
void serial_menu_clear_input(void);

/**
 * Get formatted defect name string
 * @param code OBD-II defect code
 * @return Pointer to static string with defect name
 */
const char* serial_menu_get_defect_name(uint16_t code);

/**
 * Get formatted defect description
 * @param code OBD-II defect code
 * @return Pointer to static string with defect description
 */
const char* serial_menu_get_defect_description(uint16_t code);

/**
 * Get formatted defect effect string
 * @param code OBD-II defect code
 * @return Pointer to static string describing parameter effect
 */
const char* serial_menu_get_defect_effect(uint16_t code);

/**
 * Display formatted horizontal line for menu borders
 * @param width Width of line in characters
 */
void serial_menu_print_line(uint8_t width);

/**
 * Get menu title
 * @return Title string for current menu state
 */
const char* serial_menu_get_title(MenuState state);

/**
 * Get reference to internal menu context (advanced use)
 * @return Pointer to SerialMenu context structure
 */
SerialMenu* serial_menu_get_context(void);

#endif // SERIAL_MENU_H
