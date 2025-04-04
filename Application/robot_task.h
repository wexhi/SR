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

osThreadId_t StartRobotTaskHandle;
const osThreadAttr_t StartRobotTask_attributes = {
    .name       = "StartRobotTask",
    .stack_size = 1024 * 4, // Multiply by 4 to convert to bytes
    .priority   = (osPriority_t)osPriorityNormal,
};

void StartRobotTask(void *argument);

/**
 * @brief Intialize the FreeRTOS tasks, all tasks should be created here.
 *
 */
void OSTaskInit(void)
{
    StartRobotTaskHandle = osThreadNew(StartRobotTask, NULL, &StartRobotTask_attributes);
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
        osDelay(1);
    }
    /* USER CODE END StartRobotTask */
}
