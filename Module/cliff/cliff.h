#pragma once

#include "stdlib.h"
#include "bsp_adc.h"

typedef struct {
    float left_front_voltage;
    float left_back_voltage;
    float right_front_voltage;
    float right_back_voltage;
    uint8_t lf_detect;
    uint8_t lb_detect;
    uint8_t rf_detect;
    uint8_t rb_detect;
    uint8_t cliff_detect;
} Cliff_Instance;

Cliff_Instance *Cliff_Init(void);
void Cliff_Update(void);