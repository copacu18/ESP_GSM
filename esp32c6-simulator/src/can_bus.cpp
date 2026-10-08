#include "can_bus.h"
#include <Arduino.h>
#include <driver/twai.h>

// ─── Pinuri CAN pe ESP32-C6 ──────────────────────────────────────────────────
// (diferite de WROOM-ul original care folosea TX=5, RX=4)
#define CAN_TX_GPIO  GPIO_NUM_20
#define CAN_RX_GPIO  GPIO_NUM_19

static bool g_initialized = false;

static const twai_general_config_t g_twai_cfg =
    TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_GPIO, CAN_RX_GPIO, TWAI_MODE_NORMAL);
static const twai_timing_config_t  g_timing_cfg = TWAI_TIMING_CONFIG_500KBITS();
static const twai_filter_config_t  g_filter_cfg = TWAI_FILTER_CONFIG_ACCEPT_ALL();

bool can_init_500kbps(void) {
    if (g_initialized) return true;

    if (twai_driver_install(&g_twai_cfg, &g_timing_cfg, &g_filter_cfg) != ESP_OK) {
        Serial.println("[CAN] Eroare: nu s-a putut instala driverul TWAI");
        return false;
    }
    if (twai_start() != ESP_OK) {
        Serial.println("[CAN] Eroare: nu s-a putut porni TWAI");
        twai_driver_uninstall();
        return false;
    }

    g_initialized = true;
    Serial.printf("[CAN] TWAI 500kbps initializat  TX=GPIO%d  RX=GPIO%d\n",
                  (int)CAN_TX_GPIO, (int)CAN_RX_GPIO);
    return true;
}

bool can_send(const can_frame_t* frame) {
    if (!g_initialized || !frame) return false;

    twai_message_t msg = {};
    msg.identifier       = frame->id;
    msg.data_length_code = frame->dlc;
    memcpy(msg.data, frame->data, frame->dlc);

    // Timeout 10ms - daca nu e nimeni pe bus, nu blocam
    return (twai_transmit(&msg, pdMS_TO_TICKS(10)) == ESP_OK);
}

bool can_recv(can_frame_t* frame) {
    if (!g_initialized || !frame) return false;

    twai_message_t msg;
    if (twai_receive(&msg, 0) != ESP_OK) return false;

    frame->id  = msg.identifier;
    frame->dlc = msg.data_length_code;
    memcpy(frame->data, msg.data, msg.data_length_code);
    return true;
}

bool can_is_active(void) {
    if (!g_initialized) return false;
    twai_status_info_t info;
    twai_get_status_info(&info);
    return (info.state == TWAI_STATE_RUNNING);
}
