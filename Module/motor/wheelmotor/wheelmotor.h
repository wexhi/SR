#pragma once

#include "controller.h"
#include "motor_def.h"
#include "stdint.h"
#include "daemon.h"
#include "bsp_pwm.h"
#include "bsp_encoder.h"
#include "bsp_gpio.h"
#include "filter_utils.h"

// MAX SPEED 0.44f m/s

// 编码器及轮子参数
#define ENCODER_PULSES_PER_REV 20.0f                                           // 霍尔编码器每转脉冲数
#define GEAR_REDUCTION_RATIO   48.0f                                           // 减速比
#define COUNT_PER_WHEEL_REV    (ENCODER_PULSES_PER_REV * GEAR_REDUCTION_RATIO) // 960
#define DEGREE_PER_COUNT       (360.0f / COUNT_PER_WHEEL_REV)                  // 每个计数对应的角度（约 0.3750°） // Updated angle calculation

// 轮子直径及周长（直径单位：米）
#define WHEEL_DIAMETER_M      0.06768f                        // 67.68 mm
#define WHEEL_RADIUS          (WHEEL_DIAMETER_M / 2.0f)       // 半径，约 0.03384 m
#define WHEEL_CIRCUMFERENCE_M (WHEEL_DIAMETER_M * 3.1415926f) // 周长，约 0.2126 m

#define WHEEL_MOTOR_MAX_NUM   2 // 最大支持的轮子电机数量

/* 滤波系数设置为1的时候即关闭滤波 */
#define APS_BUFFER_SIZE    6
#define APS_LPF_ALPHA      0.5f   // 最好大于0.85
#define APS_JUMP_THRESHOLD 300.0f // 角速度跳变限制（度/秒）

typedef struct {
    PWM_Init_Config_s pwm_init_config;
    TIM_Encoder_Config encoder_init_config;
    GPIO_Init_Config_s gpio_init_config;
} WheelMotor_Init_Config_s;

/* 电机反馈数据 */
typedef struct
{
    volatile int16_t encoder;             // 当前编码器读数（16位计数器）
    volatile int32_t encoder_total_count; // 长距离累计编码器计数
    volatile float speed_aps;             // 角速度，单位：度/秒
    volatile float linear_speed;          // 线速度，单位：m/s
    uint8_t direction;                    // 旋转方向，0:正转，1:反转

    float aps_buffer[APS_BUFFER_SIZE];
    uint8_t aps_index;

    float total_angle;   // 累计角度，单位：度
    int32_t total_round; // 累计转数
} WheelMotor_Measurement_s;

typedef struct
{
    WheelMotor_Measurement_s measurement;   // 反馈数据
    Motor_Control_Setting_s motor_settings; // 电机设置
    Motor_Controller_s motor_controller;    // 电机控制器
    Motor_Type_e motor_type;                // 电机类型
    PWM_Instance *pwm;                      // PWM实例
    TIM_Encoder_Instance *encoder;          // 编码器实例
    GPIO_Instance *gpio_1;                  // GPIO实例
    Motor_Working_Type_e stop_flag;         // 启停标志

    Daemon_Instance *daemon;
    uint32_t feed_cnt;     // 用于 DWT 计时
    uint32_t last_tick_ms; // 上一次更新的毫秒时间

    float pwm_out;

    float dt; // 更新时间间隔（秒）
} WheelMotor_Instance;

WheelMotor_Instance *WheelMotorInit(Motor_Init_Config_s *config, WheelMotor_Init_Config_s *wheelmotor_config);
void WheelMotorEnable(WheelMotor_Instance *motor);
void WheelMotorStop(WheelMotor_Instance *motor);
void WheelMotorSetRef(WheelMotor_Instance *motor, float ref);
void WheelMotorControl(void);

// 外部获取累计编码器值接口
int32_t WheelMotorGetTotalEncoder(WheelMotor_Instance *motor);
