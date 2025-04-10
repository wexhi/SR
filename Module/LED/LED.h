#pragma once

#include "main.h"

#define LED_LEFT_ON     HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_RESET)
#define LED_LEFT_OFF    HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_SET)
#define LED_RIGHT_ON     HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET)
#define LED_RIGHT_OFF    HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET)
#define LED_LEFT_TOGGLE HAL_GPIO_TogglePin(LED0_GPIO_Port, LED0_Pin)
#define LED_RIGHT_TOGGLE HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin)