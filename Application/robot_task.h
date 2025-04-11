/**
 * @file robot_task.h
 * @author Bi Kaixiang (wexhicy@gmail.com)
 * @brief The Task for the robot's freeRTOS
 * @version 0.1
 * @date 2025-04-04
 *
 * @copyright Copyright (c) 2025
 *
 */

#pragma once

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

#include "robot.h"
#include "led_task.h"

osThreadId_t StartRobotTaskHandle;
osThreadId_t StartLEDTaskHandle;

const osThreadAttr_t StartRobotTask_attributes = {
    .name       = "StartRobotTask",
    .stack_size = 1024 * 4, // Multiply by 4 to convert to bytes
    .priority   = (osPriority_t)osPriorityAboveNormal,
};
const osThreadAttr_t StartLEDTask_attributes = {
    .name       = "StartLEDTask",
    .stack_size = 128 * 4, // Multiply by 4 to convert to bytes
    .priority   = (osPriority_t)osPriorityNormal,
};

void StartRobotTask(void *argument);
void StartLEDTask(void *argument);

/**
 * @brief Intialize the FreeRTOS tasks, all tasks should be created here.
 *
 */
void OSTaskInit(void)
{
    StartRobotTaskHandle = osThreadNew(StartRobotTask, NULL, &StartRobotTask_attributes);
    StartLEDTaskHandle   = osThreadNew(StartLEDTask, NULL, &StartLEDTask_attributes);
}

/**
 * @brief The main task for the robot,
 * this task will be used to control the robot and handle the messages from PC.
 *
 * @param argument
 */
void StartRobotTask(void *argument)
{
    /* USER CODE BEGIN StartRobotTask */
    /* Infinite loop */
    for (;;) {
        RobotTask(); // Call the robot task function here
        osDelay(5);
    }
    /* USER CODE END StartRobotTask */
}

/**
 * @brief The task for the LED,
 * this task will be used to control the LED and handle the messages from cmd.
 *
 * @param argument
 */
void StartLEDTask(void *argument)
{
    /* USER CODE BEGIN StartLEDTask */
    LEDInit(); // Initialize the LED instances here if needed
    /* Infinite loop */
    for (;;)
    {
        LEDTask(); // Call the LED task function here
        osDelay(10);
    }
    /* USER CODE END StartLEDTask */
}
