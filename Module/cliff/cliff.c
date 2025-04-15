#include "cliff.h"
#include "adc.h"
#include "string.h"
#include "bsp_adc.h"

#define SAFE_DISTANCE 0.1f // Safe distance between cliff and ground

static ADC_Instance *left_back_adc, *left_front_adc;
static ADC_Instance *right_back_adc, *right_front_adc;

static Cliff_Instance cliff_data; // 不再使用 malloc，直接静态结构体

Cliff_Instance *Cliff_Init(void)
{
    // 注册 ADC 通道
    left_back_adc   = ADCRegister(ADC_CHANNEL_0);
    left_front_adc  = ADCRegister(ADC_CHANNEL_1);
    right_front_adc = ADCRegister(ADC_CHANNEL_2);
    right_back_adc  = ADCRegister(ADC_CHANNEL_3);

    // 初始化为 0
    memset(&cliff_data, 0, sizeof(Cliff_Instance));
    return &cliff_data;
}

void Cliff_Update(void)
{
    if (left_back_adc)
        cliff_data.left_back_voltage = ADC_GetVoltage(left_back_adc);
    if (left_front_adc)
        cliff_data.left_front_voltage = ADC_GetVoltage(left_front_adc);
    if (right_back_adc)
        cliff_data.right_back_voltage = ADC_GetVoltage(right_back_adc);
    if (right_front_adc)
        cliff_data.right_front_voltage = ADC_GetVoltage(right_front_adc);

    cliff_data.lb_detect    = cliff_data.left_back_voltage < SAFE_DISTANCE ? 1 : 0;
    cliff_data.lf_detect    = cliff_data.left_front_voltage < SAFE_DISTANCE ? 1 : 0;
    cliff_data.rf_detect    = cliff_data.right_front_voltage < SAFE_DISTANCE ? 1 : 0;
    cliff_data.rb_detect    = cliff_data.right_back_voltage < SAFE_DISTANCE ? 1 : 0;
    cliff_data.cliff_detect = cliff_data.lb_detect || cliff_data.lf_detect || cliff_data.rf_detect || cliff_data.rb_detect;
}
