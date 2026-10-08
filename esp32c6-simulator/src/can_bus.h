#pragma once
#include <stdint.h>

typedef struct {
    uint32_t id;
    uint8_t  dlc;
    uint8_t  data[8];
} can_frame_t;

bool can_init_500kbps(void);
bool can_send(const can_frame_t* frame);
bool can_recv(can_frame_t* frame);
bool can_is_active(void);
