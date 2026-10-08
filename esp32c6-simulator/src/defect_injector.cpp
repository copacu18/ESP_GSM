#include "defect_injector.h"
#include <string.h>
#include <math.h>

// Global defect state tracker
static DefectState g_defect_state = {0};

// Helper function to convert DefectCode to bitmask position
static uint8_t defect_code_to_bit(uint16_t code) {
    switch (code) {
        case DEFECT_P0335:
            return 0;
        case DEFECT_P0101:
            return 1;
        case DEFECT_P0128:
            return 2;
        default:
            return 0xFF;  // Invalid code
    }
}

// Helper function to validate parameter ranges after applying defect
static void validate_parameters(CarParameter* param) {
    // Clamp RPM to valid range
    if (param->rpm > 8000) param->rpm = 8000;

    // Clamp timing to valid range (-20 to +50 BTDC)
    if (param->timing < -20) param->timing = -20;
    if (param->timing > 50) param->timing = 50;

    // Clamp MAF to valid range (0-255 quantized)
    if (param->maf > 255) param->maf = 255;

    // Clamp load percentages
    if (param->load_percent > 100) param->load_percent = 100;
    if (param->abs_load > 255) param->abs_load = 255;

    // Clamp coolant temperature (offset by 40, so 0-130°C maps to 40-170)
    if (param->coolant > 170) param->coolant = 170;

    // Clamp intake air temperature (offset by 40, so -20 to 80°C maps to 20-120)
    if (param->iat > 120) param->iat = 120;

    // Clamp LTFT to reasonable range (0-255 but represents -30 to +30%)
    // Represented as: value = (ltft + 100) / 1.28, so valid range is 0-255
}

// P0335 - Crankshaft Position Sensor malfunction
// Behavior: Motor will not start or stalls immediately
static void apply_P0335(CarParameter* param) {
    param->rpm = 0.0f;      // Sensor detects no signal
    param->timing = 0.0f;   // Cannot synchronize without crankshaft signal
}

// P0101 - Mass Air Flow / Air Intake Flow sensor malfunction
// Behavior: Engine runs lean, rough idle, poor acceleration
static void apply_P0101(CarParameter* param) {
    param->maf = 0.0f;              // Sensor reads zero air flow
    param->load_percent = 0.0f;     // Depends on MAF
    param->abs_load = 0.0f;         // Absolute load zeros out
}

// P0128 - Coolant Thermostat malfunction (stuck open)
// Behavior: Poor fuel economy, slow engine warm-up
static void apply_P0128(CarParameter* param) {
    param->coolant = 35.0f;         // Stuck at 35°C (too cold)
    
    // IAT raised by +18°C relative to actual (sensor compensation fails)
    // Cap the increase to prevent unrealistic values
    param->iat = param->iat + 18.0f;
    
    // LTFT increases by +25% to compensate for cold start conditions
    param->ltft = param->ltft + 25.0f;
}

void applyDefect(CarParameter* param, DefectMode mode) {
    if (param == NULL) {
        return;  // Invalid parameter pointer
    }

    // Apply defect based on code
    switch (mode.code) {
        case DEFECT_P0335:
            apply_P0335(param);
            break;
        case DEFECT_P0101:
            apply_P0101(param);
            break;
        case DEFECT_P0128:
            apply_P0128(param);
            break;
        default:
            return;  // Unknown defect code
    }

    // Validate all parameters after applying defect
    validate_parameters(param);
}

void enableDefect(uint16_t code) {
    uint8_t bit = defect_code_to_bit(code);
    if (bit != 0xFF) {
        g_defect_state.active_defects |= (1 << bit);
    }
}

void disableDefect(uint16_t code) {
    uint8_t bit = defect_code_to_bit(code);
    if (bit != 0xFF) {
        g_defect_state.active_defects &= ~(1 << bit);
    }
}

bool isDefectActive(uint16_t code) {
    uint8_t bit = defect_code_to_bit(code);
    if (bit == 0xFF) {
        return false;
    }
    return (g_defect_state.active_defects & (1 << bit)) != 0;
}

void clearAllDefects(void) {
    g_defect_state.active_defects = 0;
}

DefectState getDefectState(void) {
    return g_defect_state;
}

uint8_t getActiveDefects(uint16_t* codes, uint8_t max_count) {
    if (codes == NULL || max_count == 0) {
        return 0;
    }

    uint8_t count = 0;

    // Check each possible defect
    if (isDefectActive(DEFECT_P0335) && count < max_count) {
        codes[count++] = DEFECT_P0335;
    }
    if (isDefectActive(DEFECT_P0101) && count < max_count) {
        codes[count++] = DEFECT_P0101;
    }
    if (isDefectActive(DEFECT_P0128) && count < max_count) {
        codes[count++] = DEFECT_P0128;
    }

    return count;
}
