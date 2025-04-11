#pragma once

#include "tim.h"
#include "stdint.h"
#include "stm32f4xx_hal_rcc.h"

#define MAX_ENCODER_NUM 4

typedef struct
{
    TIM_HandleTypeDef *htim; // TIM句柄
    uint32_t channel1;       // 通道
    uint32_t channel2;       // 通道
} TIM_Encoder_Instance;

typedef struct
{
    TIM_HandleTypeDef *htim; // TIM句柄
    uint32_t channel1;       // 通道
    uint32_t channel2;
} TIM_Encoder_Config;

TIM_Encoder_Instance *TIM_Encoder_Register(TIM_Encoder_Config *config);
