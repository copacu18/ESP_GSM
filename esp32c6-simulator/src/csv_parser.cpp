#include "csv_parser.h"
#include <cstdlib>
#include <cstring>
#include <cmath>

/**
 * @brief Extract a CSV field (handles quoted and unquoted values)
 * @param line Pointer to current position in CSV line
 * @param field Output buffer
 * @param max_len Maximum field length
 * @return Pointer to next field start, or nullptr at end
 */
static const char* extract_csv_field(const char* line, char* field, size_t max_len)
{
    if (!line || *line == '\0') {
        return nullptr;
    }

    // Handle quoted fields
    if (*line == '"') {
        line++;  // Skip opening quote
        size_t i = 0;
        while (*line != '\0' && i < max_len - 1) {
            if (*line == '"') {
                if (*(line + 1) == '"') {
                    // Escaped quote
                    field[i++] = '"';
                    line += 2;
                } else {
                    // End of quoted field
                    line++;
                    break;
                }
            } else {
                field[i++] = *line++;
            }
        }
        field[i] = '\0';
        // Skip comma if present
        if (*line == ',') line++;
    } else {
        // Unquoted field
        size_t i = 0;
        while (*line != '\0' && *line != ',' && i < max_len - 1) {
            field[i++] = *line++;
        }
        field[i] = '\0';
        // Skip comma if present
        if (*line == ',') line++;
    }

    return line;
}

/**
 * @brief Quantize float value to 0-255 range
 */
uint8_t quantize_value(float value, float min_val, float max_val)
{
    if (value <= min_val) return 0;
    if (value >= max_val) return 255;
    return (uint8_t)((value - min_val) / (max_val - min_val) * 255.0f + 0.5f);
}

/**
 * @brief Quantize signed float value to 0-255 range
 */
uint8_t quantize_signed_value(float value, float min_val, float max_val)
{
    return quantize_value(value, min_val, max_val);
}

/**
 * @brief Parse single CSV line into CarParameter
 */
bool parse_csv_line(const char* line, CarParameter& param)
{
    if (!line) return false;

    // Skip header line
    if (strstr(line, "Time") != nullptr && strstr(line, "RPM") != nullptr) {
        return false;
    }

    char field[64];
    const char* pos = line;
    int field_index = 0;

    // Parse all 23 fields
    while (pos && field_index < CSV_COLUMN_COUNT) {
        pos = extract_csv_field(pos, field, sizeof(field));
        
        if (field[0] == '\0' && field_index > 0) {
            break;  // End of line
        }

        float fval;
        int ival;

        switch (field_index) {
            case 0:  // Time - skip
                break;
            case 1:  // RPM (0-8000)
                param.rpm = (uint16_t)atoi(field);
                break;
            case 2:  // Speed (0-255)
                param.speed = (uint8_t)atoi(field);
                break;
            case 3:  // TPS% (0-100)
                fval = atof(field);
                param.tps_percent = quantize_value(fval, 0.0f, 100.0f);
                break;
            case 4:  // TPS_V (0-5V)
                fval = atof(field);
                param.tps_voltage = quantize_value(fval, 0.0f, 5.0f);
                break;
            case 5:  // Load% (0-100)
                fval = atof(field);
                param.load_percent = quantize_value(fval, 0.0f, 100.0f);
                break;
            case 6:  // AbsLoad% (0-255)
                fval = atof(field);
                param.abs_load = quantize_value(fval, 0.0f, 255.0f);
                break;
            case 7:  // Coolant (-40 to 215°C, offset by 40)
                fval = atof(field);
                param.coolant = (uint8_t)(fval + 40.0f);
                break;
            case 8:  // IAT (-40 to 215°C, offset by 40)
                fval = atof(field);
                param.iat = (uint8_t)(fval + 40.0f);
                break;
            case 9:  // MAF (0-655.35 g/s, quantize to 0-255)
                fval = atof(field);
                param.maf = quantize_value(fval, 0.0f, 655.35f);
                break;
            case 10:  // Timing (-30 to +47 degrees)
                ival = atoi(field);
                param.timing = (uint8_t)(int8_t)ival;  // Store as two's complement
                break;
            case 11:  // PedalD% (0-100)
                fval = atof(field);
                param.pedal_d = quantize_value(fval, 0.0f, 100.0f);
                break;
            case 12:  // PedalE% (0-100)
                fval = atof(field);
                param.pedal_e = quantize_value(fval, 0.0f, 100.0f);
                break;
            case 13:  // CmdThrot% (0-100)
                fval = atof(field);
                param.cmd_throt = quantize_value(fval, 0.0f, 100.0f);
                break;
            case 14:  // O2V1 (0-1.275V)
                fval = atof(field);
                param.o2v1 = quantize_value(fval, 0.0f, 1.275f);
                break;
            case 15:  // O2V2 (0-1.275V)
                fval = atof(field);
                param.o2v2 = quantize_value(fval, 0.0f, 1.275f);
                break;
            case 16:  // STFT (-16 to +11 %)
                ival = atoi(field);
                param.stft = (uint8_t)(int8_t)ival;  // Store as two's complement
                break;
            case 17:  // LTFT (-4 to +5 %)
                ival = atoi(field);
                param.ltft = (uint8_t)(int8_t)ival;  // Store as two's complement
                break;
            case 18:  // VBAT (5V to 15V, store as volts - 5)
                fval = atof(field);
                param.vbat = (uint8_t)(fval - 5.0f);
                break;
            case 19:  // Steer (651 to 1241 ADC)
                param.steer = (uint16_t)atoi(field);
                break;
            case 20:  // Brake (0-255)
                param.brake = (uint8_t)atoi(field);
                break;
            case 21:  // Warmups (0-255)
                param.warmups = (uint8_t)atoi(field);
                break;
            case 22:  // DistClear (0-65535 km)
                param.dist_clear = (uint16_t)atoi(field);
                break;
        }

        field_index++;
    }

    return field_index >= CSV_COLUMN_COUNT;
}

/**
 * @brief Initialize CSV parser (mount SPIFFS if available)
 */
bool csv_parser_init()
{
    // For ESP32, this would mount SPIFFS/LittleFS
    // Implementation depends on the specific setup
    // For now, we return true since embedded data is always available
    
    // If using real SPIFFS:
    // if (!LittleFS.begin(true, "/lfs", 10, "spiffs")) {
    //     return false;
    // }
    
    return true;
}

/**
 * @brief Read CSV file from SPIFFS with streaming buffer
 */
bool csv_read_file_streaming(const char* filename, bool (*callback)(const CarParameter& param))
{
    if (!filename || !callback) {
        return false;
    }

    // This is a placeholder for SPIFFS file reading
    // In a real ESP32 implementation:
    // FILE* file = fopen(filename, "r");
    // if (!file) return false;
    // 
    // char buffer[CSV_BUFFER_SIZE];
    // char line[CSV_MAX_LINE_LENGTH];
    // size_t line_pos = 0;
    // bool first_line = true;
    // 
    // while (fgets(buffer, sizeof(buffer), file)) {
    //     for (const char* p = buffer; *p; p++) {
    //         if (*p == '\n') {
    //             line[line_pos] = '\0';
    //             if (line_pos > 0 && !first_line) {
    //                 CarParameter param;
    //                 if (parse_csv_line(line, param)) {
    //                     if (!callback(param)) {
    //                         fclose(file);
    //                         return false;
    //                     }
    //                 }
    //             }
    //             first_line = false;
    //             line_pos = 0;
    //         } else if (line_pos < sizeof(line) - 1) {
    //             line[line_pos++] = *p;
    //         }
    //     }
    // }
    // fclose(file);
    // return true;

    // For now, return success (fallback to embedded data)
    return true;
}
