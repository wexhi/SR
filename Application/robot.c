#include "robot.h"
#include "robot_task.h"
#include "robot_def.h"
#include "robot_cmd.h"
#include "chassis.h"

#include "bsp_init.h"
#include "LED.h"

#include "adc.h"

#define ADC_CHANNEL_NUM 4
uint16_t adc_buf[ADC_CHANNEL_NUM];

void ADC_Start()
{
    HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_buf, ADC_CHANNEL_NUM);
}

/**
 * @brief Initialize the robot's hardware and peripherals.
 *
 */
void Robot_Init(void)
{
    // Initialize the robot's hardware and peripherals here
    // For example, initialize motors, sensors, etc.
    // This function should be called before starting the robot task
    // to ensure that all hardware is ready for use.

    __disable_irq(); // Disable interrupts to ensure atomic operation

    // Initialize the BSP layer
    BSPInit();
    ADC_Start(); // Start the ADC in DMA mode

    // Initialize the Application layer
    RobotCMDInit();
    ChassisInit(); // Initialize the chassis application

    // Initialize the FreeRTOS tasks
    OSTaskInit();

    __enable_irq(); // Enable interrupts after initialization
}
float v0, v1, v2, v3; // Voltage values for ADC channels
void ReadVoltages()
{
    v0 = adc_buf[0] * 3.3f / 4095.0f;
    v1 = adc_buf[1] * 3.3f / 4095.0f;
    v2 = adc_buf[2] * 3.3f / 4095.0f;
    v3 = adc_buf[3] * 3.3f / 4095.0f;

    // printf 或上传串口调试
}

/**
 * @brief The task entry point for the robot.
 *
 */
void RobotTask(void)
{
    RobotCMDTask(); // Call the robot command task function here
    ChassisTask();  // Call the chassis task function here
    ReadVoltages();
}