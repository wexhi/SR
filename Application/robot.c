#include "robot.h"
#include "robot_task.h"
#include "robot_def.h"
#include "robot_cmd.h"
#include "chassis.h"

#include "bsp_init.h"
#include "LED.h"

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

    // Initialize the Application layer
    RobotCMDInit();
    // ChassisInit(); // Initialize the chassis application
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL); // Start the encoder timer
    HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_ALL); // Start the encoder timer

    // Initialize the FreeRTOS tasks
    OSTaskInit();

    __enable_irq(); // Enable interrupts after initialization
}

static int32_t test2, test5;
/**
 * @brief The task entry point for the robot.
 *
 */
void RobotTask(void)
{
    RobotCMDTask(); // Call the robot command task function here
    // ChassisTask();  // Call the chassis task function here
    test2 = __HAL_TIM_GET_COUNTER(&htim2);
    test5 = __HAL_TIM_GET_COUNTER(&htim5);
}