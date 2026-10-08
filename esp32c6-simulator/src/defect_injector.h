#ifndef DEFECT_INJECTOR_H
#define DEFECT_INJECTOR_H

#include <stdint.h>
#include <stdbool.h>
#include "csv_parser.h"  // Get CarParameter definition

// OBD-II Defect Code Enum
typedef enum {
    DEFECT_P0335 = 0x0335,  // Crankshaft Position Sensor malfunction
    DEFECT_P0101 = 0x0101,  // Mass Air Flow / Air Intake Flow sensor
    DEFECT_P0128 = 0x0128   // Coolant Thermostat malfunction
} DefectCode;

// Defect severity levels
typedef enum {
    SEVERITY_WARNING = 1,   // Warning - affects performance
    SEVERITY_ERROR = 2,     // Error - significant impact
    SEVERITY_CRITICAL = 3   // Critical - engine won't start
} SeverityLevel;

// Defect mode structure - tracks state of a single defect
typedef struct {
    DefectCode code;        // OBD-II defect code
    bool active;            // Whether defect is currently active
    SeverityLevel severity; // Severity of the defect
} DefectMode;

// Active defects tracker (supports up to 3 simultaneous defects using bitmask)
typedef struct {
    uint8_t active_defects;  // Bitmask: bit 0=P0335, bit 1=P0101, bit 2=P0128
} DefectState;

// Function Declarations

/**
 * Apply defect transformations to car parameters
 * @param param Pointer to CarParameter structure to modify
 * @param mode DefectMode specifying which defect to apply
 */
void applyDefect(CarParameter* param, DefectMode mode);

/**
 * Enable a specific defect by its OBD-II code
 * @param code Defect code (DEFECT_P0335, DEFECT_P0101, or DEFECT_P0128)
 */
void enableDefect(uint16_t code);

/**
 * Disable a specific defect by its OBD-II code
 * @param code Defect code to disable
 */
void disableDefect(uint16_t code);

/**
 * Check if a defect is currently active
 * @param code Defect code to check
 * @return true if defect is active, false otherwise
 */
bool isDefectActive(uint16_t code);

/**
 * Clear all active defects
 */
void clearAllDefects(void);

/**
 * Get the current defect state
 * @return DefectState structure showing which defects are active
 */
DefectState getDefectState(void);

/**
 * Get the active defect codes as an array
 * @param codes Array to store active defect codes
 * @param max_count Maximum number of codes to return
 * @return Number of active defects
 */
uint8_t getActiveDefects(uint16_t* codes, uint8_t max_count);

#endif // DEFECT_INJECTOR_H
