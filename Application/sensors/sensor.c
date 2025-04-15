#include "sensor.h"

static IR_Instance *ir_l, *ir_r;
static int a = 0;
static void Infrared_Callback(IR_Instance *ir)
{
    a++;
}
void SensorInit(void)
{
    IR_Config_s ir_cofig = {
        .gpio_config = {
            .GPIOx     = GPIOE,
            .GPIO_Pin  = GPIO_PIN_5,
            .exti_mode = GPIO_EXTI_MODE_FALLING,
            .pin_state = GPIO_PIN_SET,
        },
        .on_detect = Infrared_Callback, // 注册回调函数
    };
    ir_l = IRRegister(&ir_cofig);

    ir_cofig.gpio_config.GPIOx    = GPIOB;
    ir_cofig.gpio_config.GPIO_Pin = GPIO_PIN_8;
    ir_r                          = IRRegister(&ir_cofig);
}

void SensorTask(void)
{
}