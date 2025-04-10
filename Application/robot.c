#include "robot.h"
#include "robot_task.h"

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

    // Initialize the FreeRTOS tasks
    OSTaskInit();

    __enable_irq(); // Enable interrupts after initialization
}

static int count = 0; 

/**
 * @brief The task entry point for the robot.
 *
 */
void RobotTask(void)
{
    count++;
    if (count > 1000)
    {
        count = 0;
        LED_LEFT_TOGGLE; // Toggle LED0 every 1000 iterations
    }
    LED_RIGHT_ON;
}