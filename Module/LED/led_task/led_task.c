#include "led_task.h"
#include "led.h"

#include "message_center.h"

static LED_Instance *led_r, *led_l;

void LEDInit(void)
{
    // Initialize the LED instances here if needed
    // For example, you can register the LEDs and set their initial states
    LED_Config_s led_config = {
        .gpio_config = {
            .GPIOx     = LED0_GPIO_Port,
            .GPIO_Pin  = LED0_Pin,
            .pin_state = GPIO_PIN_SET,
            .exti_mode = GPIO_EXTI_MODE_NONE,
        },
        .initial_state = LED_OFF,
        .on_change     = NULL, // No callback for now
    };
    led_r = LEDRegister(&led_config);

    led_config.gpio_config.GPIOx    = LED1_GPIO_Port;
    led_config.gpio_config.GPIO_Pin = LED1_Pin;

    led_l = LEDRegister(&led_config);
}

void LEDTask(void)
{
}