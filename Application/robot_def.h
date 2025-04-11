/**
 * @file robot_def.h
 * @author Bi Kaixiang (wexhicy@gmail.com)
 * @brief The definition of the robot,
 *          which contains the parameters and functionalities of the robot.
 * @version 0.1
 * @date 2025-04-10
 *
 * @copyright Copyright (c) 2025
 *
 */

#pragma once

#include "stdint.h"

// #define VISION_USE_VCP // 是否使用虚拟串口

#define VISION_USE_UART // 是否使用硬件串口

#pragma pack(1) // Code alignment for structure packing

// The state of the robot
typedef enum {
    ROBOT_STOP = 0,
    ROBOT_READY,
} Robot_Status_e;

// The mode of the chassis
typedef enum {
    CHASSIS_ZERO_FORCE = 0, // stop mode
    CHASSIS_NORMAL          // the normal mode of the robot
} chassis_mode_e;

// the mode of the cleaning machine
typedef enum {
    CLEANING_IDLE = 0, // idle mode
    CLEANING_ACTIVE    // active cleaning mode
} cleaning_mode_e;

/* ----------------CMD Application public the control data---------------- */
typedef struct {
    // control part
    float vx; // the forward speed of the robot m/s
    float wz; // the angular speed of the robot rad/s
    chassis_mode_e chassis_mode;
} Chassis_Ctrl_Cmd_s;

/* ----------------The feedback data---------------- */
typedef struct
{
    // the real speed of the robot
    float real_vx;       // the real speed of the robot m/s
    float real_wz;       // the real angular speed of the robot rad/s
    float battery_level; // the battery level of the robot in percentage
} Chassis_Upload_Data_s;

#pragma pack() // 取消压缩