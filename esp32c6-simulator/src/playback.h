#ifndef PLAYBACK_H
#define PLAYBACK_H

#include <cstdint>
#include <cstddef>
#include "csv_parser.h"

// ============================================================================
// Playback Configuration Constants
// ============================================================================

// Default timing interval (1 row per second at 1x speed)
#define PLAYBACK_DEFAULT_INTERVAL_MS 1000

// Speed multiplier values (0.5x = 2000ms, 1x = 1000ms, 2x = 500ms, 4x = 250ms)
typedef enum {
    PLAYBACK_SPEED_HALF = 0x05,    // 0.5x (2000ms per row)
    PLAYBACK_SPEED_1X = 0x10,      // 1x (1000ms per row) - default
    PLAYBACK_SPEED_2X = 0x20,      // 2x (500ms per row)
    PLAYBACK_SPEED_4X = 0x40       // 4x (250ms per row)
} PlaybackSpeed;

// ============================================================================
// Playback State Structure
// ============================================================================

typedef struct {
    uint32_t current_row;           // Current row index in CSV data
    uint32_t total_rows;            // Total number of rows available
    PlaybackSpeed speed;            // Current playback speed (0.5x to 4x)
    bool paused;                    // True if playback is paused
    uint32_t last_update_ms;        // Timestamp of last row transmitted
    uint32_t rows_transmitted;      // Cumulative rows sent (for monitoring)
    uint32_t frames_failed;         // CAN transmission failures
    uint32_t parse_errors;          // CSV parsing errors
} PlaybackState;

// ============================================================================
// Public API Functions
// ============================================================================

/**
 * Initialize playback system with embedded or SPIFFS data
 * @param use_embedded If true, use embedded sample data; if false, try SPIFFS
 * @return true on success, false on failure
 */
bool playback_init(bool use_embedded);

/**
 * Get current playback state
 * @return Pointer to PlaybackState struct
 */
PlaybackState* playback_get_state(void);

/**
 * Get next row from CSV data (wraps around at end)
 * @return Next CarParameter row, or invalid row if error
 */
CarParameter playback_get_next_row(void);

/**
 * Set playback speed
 * @param speed PlaybackSpeed value (PLAYBACK_SPEED_HALF, 1X, 2X, or 4X)
 */
void playback_set_speed(PlaybackSpeed speed);

/**
 * Get time interval (in ms) between rows based on current speed
 * @return Milliseconds to wait between transmitting rows
 */
uint32_t playback_get_interval_ms(void);

/**
 * Toggle pause/resume state
 * @return New paused state (true = paused, false = playing)
 */
bool playback_toggle_pause(void);

/**
 * Check if it's time to transmit the next row
 * @return true if interval has elapsed, false otherwise
 */
bool playback_is_time_for_next_row(void);

/**
 * Update last transmission timestamp (call after sending a row)
 */
void playback_update_timestamp(void);

/**
 * Record a successful transmission
 */
void playback_record_success(void);

/**
 * Record a CAN transmission failure
 */
void playback_record_can_failure(void);

/**
 * Record a CSV parsing error
 */
void playback_record_parse_error(void);

/**
 * Get performance statistics string
 * @param buf Output buffer for statistics string
 * @param buf_size Maximum buffer size
 */
void playback_get_stats(char* buf, size_t buf_size);

/**
 * Reset playback to beginning
 */
void playback_reset(void);

#endif // PLAYBACK_H
