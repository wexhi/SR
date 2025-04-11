#pragma once

#include "controller.h"
#include "motor_def.h"
#include "stdint.h"
#include "daemon.h"
#include "bsp_pwm.h"
#include "bsp_encoder.h"

#define WHEEL_MOTOR_MAX_NUM 2 // 最大支持的轮子电机数量

/* 滤波系数设置为1的时候即关闭滤波 */
#define SPEED_SMOOTH_COEF   0.85f // 最好大于0.85
#define CURRENT_SMOOTH_COEF 0.9f  // 必须大于0.9
#define REDUCTION_RATIAO    63.0f // 减速比

/* The feedback data for wheel motor */
typedef struct
{
    volatile int32_t encoder; // 编码器值
    volatile float speed_aps; // 角速度,单位为:度/秒

    float total_angle;   // 绝对角度,单位为:度
    int32_t total_round; // 绝对圈数,单位为:圈
} WheelMotor_Measurement_s;

typedef struct
{
    WheelMotor_Measurement_s measurement;   // 反馈数据
    Motor_Control_Setting_s motor_settings; // 电机设置
    Motor_Controller_s motor_controller;    // 电机控制器
    Motor_Type_e motor_type;                // 电机类型
    PWM_Instance *pwm;                      // PWM实例
    TIM_Encoder_Instance *encoder;         // 编码器实例

    Motor_Working_Type_e stop_flag; // 启停标志

    Daemon_Instance *daemon;
    uint32_t feed_cnt;
    float dt;
} WheelMotor_Instance;

WheelMotor_Instance *WheelMotorInit(Motor_Init_Config_s *config);
void WheelMotorEnable(WheelMotor_Instance *motor);
void WheelMotorStop(WheelMotor_Instance *motor);
void WheelMotorSetRef(WheelMotor_Instance *motor, float ref);
void WheelMotorControl(void);
