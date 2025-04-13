#pragma once

#include "REG.h"
#include "wit_c_sdk.h"
#include "stdint.h"
#include "bsp_usart.h"

typedef struct {
    float Gyro[3];  // 角速度
    float Accel[3]; // 加速度
    // 还需要增加角速度数据
    float Mag[3];        // 磁场
    float Quaternion[4]; // 四元数
    float Roll_Angle;
    float Pitch_Angle;
    float Yaw_Angle;
    float YawTotalAngle;

} JY901S_attitude_t;

typedef struct {
    uint8_t is_init;
    USART_Instance *usart_instance;
    JY901S_attitude_t attitude;
} JY901S_Instance;

JY901S_attitude_t *INS_Init(void);