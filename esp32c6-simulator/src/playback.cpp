#include "playback.h"
#include "csv_parser.h"
#include <cstring>
#include <Arduino.h>

// ============================================================================
// Static State
// ============================================================================

static PlaybackState g_playback_state = {
    .current_row = 0,
    .total_rows = 0,
    .speed = PLAYBACK_SPEED_1X,
    .paused = false,
    .last_update_ms = 0,
    .rows_transmitted = 0,
    .frames_failed = 0,
    .parse_errors = 0
};

static const CarParameter* g_csv_data = nullptr;
static bool g_data_initialized = false;

// ============================================================================
// Initialization
// ============================================================================

bool playback_init(bool use_embedded) {
    // Initialize CSV parser infrastructure
    if (!csv_parser_init()) {
        Serial.println("[PLAYBACK] Warning: CSV parser init failed");
    }

    if (use_embedded) {
        // Use embedded sample data
        g_csv_data = get_embedded_sample_data();
        g_playback_state.total_rows = get_embedded_sample_data_count();
        
        if (g_playback_state.total_rows == 0) {
            Serial.println("[PLAYBACK] ERROR: Embedded data unavailable");
            return false;
        }
        
        Serial.printf("[PLAYBACK] Using embedded CSV data: %u rows\n", g_playback_state.total_rows);
    } else {
        // Try to load from SPIFFS (future enhancement)
        Serial.println("[PLAYBACK] SPIFFS loading not yet implemented, using embedded data");
        g_csv_data = get_embedded_sample_data();
        g_playback_state.total_rows = get_embedded_sample_data_count();
        
        if (g_playback_state.total_rows == 0) {
            Serial.println("[PLAYBACK] ERROR: No data available");
            return false;
        }
    }

    // Initialize timing
    g_playback_state.current_row = 0;
    g_playback_state.last_update_ms = millis();
    g_playback_state.rows_transmitted = 0;
    g_playback_state.frames_failed = 0;
    g_playback_state.parse_errors = 0;

    g_data_initialized = true;
    return true;
}

// ============================================================================
// State Access
// ============================================================================

PlaybackState* playback_get_state(void) {
    return &g_playback_state;
}

// ============================================================================
// Row Management
// ============================================================================

CarParameter playback_get_next_row(void) {
    static CarParameter invalid_param = {};
    
    if (!g_data_initialized || g_csv_data == nullptr || g_playback_state.total_rows == 0) {
        playback_record_parse_error();
        return invalid_param;
    }

    // Get current row and wrap around if at end
    if (g_playback_state.current_row >= g_playback_state.total_rows) {
        g_playback_state.current_row = 0;  // Loop back to start
        Serial.printf("[PLAYBACK] Looping: Reset to row 0 (total=%u)\n", g_playback_state.total_rows);
    }

    // Return the current row and increment
    CarParameter param = g_csv_data[g_playback_state.current_row];
    g_playback_state.current_row++;

    return param;
}

// ============================================================================
// Speed Control
// ============================================================================

void playback_set_speed(PlaybackSpeed speed) {
    g_playback_state.speed = speed;
    
    const char* speed_name = "Unknown";
    switch (speed) {
        case PLAYBACK_SPEED_HALF: speed_name = "0.5x"; break;
        case PLAYBACK_SPEED_1X: speed_name = "1x"; break;
        case PLAYBACK_SPEED_2X: speed_name = "2x"; break;
        case PLAYBACK_SPEED_4X: speed_name = "4x"; break;
    }
    
    Serial.printf("[PLAYBACK] Speed set to %s\n", speed_name);
}

uint32_t playback_get_interval_ms(void) {
    // Use REAL timing delays from CSV data to preserve authentic playback speed
    // This ensures the simulator runs at the same speed as the original recording
    uint16_t base_delay_ms = get_timing_delay(g_playback_state.current_row);
    
    // Apply speed multiplier (as float for precision)
    float speed_multiplier = 1.0f;
    switch (g_playback_state.speed) {
        case PLAYBACK_SPEED_HALF:
            speed_multiplier = 2.0f;    // 0.5x = 2x slower (90 min for 45min data)
            break;
        case PLAYBACK_SPEED_1X:
            speed_multiplier = 1.0f;    // 1x = real speed (45min for 45min data)
            break;
        case PLAYBACK_SPEED_2X:
            speed_multiplier = 0.5f;    // 2x = 2x faster (22.5 min)
            break;
        case PLAYBACK_SPEED_4X:
            speed_multiplier = 0.25f;   // 4x = 4x faster (11 min)
            break;
        default:
            speed_multiplier = 1.0f;
    }
    
    // Return adjusted interval (min 50ms to prevent busyloop)
    uint32_t adjusted = (uint32_t)(base_delay_ms * speed_multiplier);
    return (adjusted < 50) ? 50 : adjusted;
}

// ============================================================================
// Pause/Resume Control
// ============================================================================

bool playback_toggle_pause(void) {
    g_playback_state.paused = !g_playback_state.paused;
    
    if (g_playback_state.paused) {
        Serial.println("[PLAYBACK] Paused");
    } else {
        Serial.println("[PLAYBACK] Resumed");
        g_playback_state.last_update_ms = millis();  // Reset timing when resuming
    }
    
    return g_playback_state.paused;
}

// ============================================================================
// Timing Control
// ============================================================================

bool playback_is_time_for_next_row(void) {
    if (g_playback_state.paused) {
        return false;  // Don't send rows when paused
    }

    uint32_t elapsed_ms = millis() - g_playback_state.last_update_ms;
    uint32_t interval_ms = playback_get_interval_ms();

    return elapsed_ms >= interval_ms;
}

void playback_update_timestamp(void) {
    g_playback_state.last_update_ms = millis();
}

// ============================================================================
// Statistics & Monitoring
// ============================================================================

void playback_record_success(void) {
    g_playback_state.rows_transmitted++;
}

void playback_record_can_failure(void) {
    g_playback_state.frames_failed++;
}

void playback_record_parse_error(void) {
    g_playback_state.parse_errors++;
}

void playback_get_stats(char* buf, size_t buf_size) {
    if (!buf || buf_size == 0) return;

    uint32_t rows_total = g_playback_state.total_rows;
    uint32_t rows_sent = g_playback_state.rows_transmitted;
    uint32_t errors = g_playback_state.frames_failed + g_playback_state.parse_errors;

    const char* speed_name = "?";
    switch (g_playback_state.speed) {
        case PLAYBACK_SPEED_HALF: speed_name = "0.5x"; break;
        case PLAYBACK_SPEED_1X: speed_name = "1x"; break;
        case PLAYBACK_SPEED_2X: speed_name = "2x"; break;
        case PLAYBACK_SPEED_4X: speed_name = "4x"; break;
    }

    const char* state_name = g_playback_state.paused ? "PAUSED" : "PLAYING";

    snprintf(buf, buf_size, 
        "Row: %u/%u | Sent: %u | Speed: %s | State: %s | Errors: %u",
        g_playback_state.current_row, rows_total, rows_sent, speed_name, state_name, errors);
}

// ============================================================================
// Reset
// ============================================================================

void playback_reset(void) {
    g_playback_state.current_row = 0;
    g_playback_state.rows_transmitted = 0;
    g_playback_state.frames_failed = 0;
    g_playback_state.parse_errors = 0;
    g_playback_state.last_update_ms = millis();
    g_playback_state.paused = false;
    
    Serial.println("[PLAYBACK] Reset to beginning");
}
