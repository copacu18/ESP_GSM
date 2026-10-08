#ifndef CSV_PARSER_H
#define CSV_PARSER_H

#include <cstdint>
#include <cstddef>

// Constants
constexpr size_t CSV_COLUMN_COUNT = 23;
constexpr size_t CSV_MAX_LINE_LENGTH = 512;
constexpr size_t CSV_BUFFER_SIZE = 4096;
constexpr size_t CAR_PARAM_BINARY_SIZE = 25;  // Optimized binary size per row

/**
 * @struct CarParameter
 * @brief Binary-packed OBD-II car parameter data structure
 * 
 * Size: 29 bytes per record
 * Encoding:
 *   - RPM: uint16_t (0-8000)
 *   - Speed: uint8_t (0-255 km/h)
 *   - TPS%: uint8_t (0-100% quantized to 0-255)
 *   - TPS_V: uint8_t (0-5V quantized to 0-255)
 *   - Load%: uint8_t (0-100% quantized to 0-255)
 *   - AbsLoad%: uint8_t (0-255)
 *   - Coolant: uint8_t (-40 to 215°C, offset 40)
 *   - IAT: uint8_t (-40 to 215°C, offset 40)
 *   - MAF: uint8_t (0-655.35 g/s quantized to 0-255)
 *   - Timing: int8_t (-30 to +47 degrees)
 *   - PedalD%: uint8_t (0-100% quantized to 0-255)
 *   - PedalE%: uint8_t (0-100% quantized to 0-255)
 *   - CmdThrot%: uint8_t (0-100% quantized to 0-255)
 *   - O2V1: uint8_t (0-1.275V quantized to 0-255)
 *   - O2V2: uint8_t (0-1.275V quantized to 0-255)
 *   - Timing: uint8_t (stored as two's complement, interpret as int8_t)
 *   - STFT: uint8_t (stored as two's complement, interpret as int8_t)
 *   - LTFT: uint8_t (stored as two's complement, interpret as int8_t)
 *   - VBAT: uint8_t (5V to 15V, offset 5)
 *   - Steer: uint16_t (651 to 1241 ADC counts)
 *   - Brake: uint8_t (0-255, binary pedal state)
 *   - Warmups: uint8_t (0-255 startup cycles)
 *   - DistClear: uint16_t (0-65535 km since clear)
 */
struct CarParameter {
    uint16_t rpm;           // Offset 0: Engine RPM (0-8000)
    uint8_t speed;          // Offset 2: Vehicle speed km/h
    uint8_t tps_percent;    // Offset 3: Throttle position %
    uint8_t tps_voltage;    // Offset 4: Throttle voltage (0-5V as 0-255)
    uint8_t load_percent;   // Offset 5: Engine load %
    uint8_t abs_load;       // Offset 6: Absolute load
    uint8_t coolant;        // Offset 7: Coolant temp (°C + 40 offset)
    uint8_t iat;            // Offset 8: Intake air temp (°C + 40 offset)
    uint8_t maf;            // Offset 9: Mass air flow
    uint8_t timing;         // Offset 10: Ignition timing (two's complement)
    uint8_t pedal_d;        // Offset 11: Driver pedal demand %
    uint8_t pedal_e;        // Offset 12: ECU pedal expected %
    uint8_t cmd_throt;      // Offset 13: Commanded throttle %
    uint8_t o2v1;           // Offset 14: Oxygen sensor 1 voltage
    uint8_t o2v2;           // Offset 15: Oxygen sensor 2 voltage
    uint8_t stft;           // Offset 16: Short term fuel trim (two's complement)
    uint8_t ltft;           // Offset 17: Long term fuel trim (two's complement)
    uint8_t vbat;           // Offset 18: Battery voltage (volts - 5)
    uint16_t steer;         // Offset 19-20: Steering angle (ADC)
    uint8_t brake;          // Offset 21: Brake pedal state
    uint8_t warmups;        // Offset 22: Engine warmup cycles
    uint16_t dist_clear;    // Offset 23-24: Distance since DTC clear
} __attribute__((packed));

static_assert(sizeof(CarParameter) == CAR_PARAM_BINARY_SIZE, "CarParameter must be 29 bytes");

/**
 * @brief Parse CSV line into CarParameter structure
 * @param line CSV line string (comma-separated values)
 * @param param Output parameter structure
 * @return true on success, false on parse error
 */
bool parse_csv_line(const char* line, CarParameter& param);

/**
 * @brief Convert float value to 0-255 range based on min/max bounds
 * @param value Input float value
 * @param min Minimum expected value
 * @param max Maximum expected value
 * @return Quantized 0-255 value
 */
uint8_t quantize_value(float value, float min_val, float max_val);

/**
 * @brief Convert signed float value to 0-255 range
 * @param value Input float value (can be negative)
 * @param min Minimum expected value
 * @param max Maximum expected value
 * @return Quantized 0-255 value
 */
uint8_t quantize_signed_value(float value, float min_val, float max_val);

/**
 * @brief Initialize CSV parser (mounts SPIFFS)
 * @return true on success
 */
bool csv_parser_init();

/**
 * @brief Read CSV file from SPIFFS with streaming buffer
 * @param filename File path (e.g., "/predict_0.csv")
 * @param callback Function called for each parsed row: bool callback(const CarParameter& param)
 * @return true if file read successfully
 */
bool csv_read_file_streaming(const char* filename, bool (*callback)(const CarParameter& param));

/**
 * @brief Get embedded sample data (first 100 rows from predict_0.csv)
 * @return Pointer to array of CarParameter structs
 */
const CarParameter* get_embedded_sample_data();

/**
 * @brief Get number of embedded sample data rows
 * @return Number of rows in embedded data
 */
size_t get_embedded_sample_data_count();

/**
 * @brief Get real-world timing delay (ms) before next row
 * Preserves actual data collection rate from predict_1.csv
 * @param row_index Index of current row (0-based)
 * @return Delay in milliseconds to next row, or 75ms if out of bounds
 */
uint16_t get_timing_delay(size_t row_index);

#endif // CSV_PARSER_H
