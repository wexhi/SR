#include "wheelmotor.h"
#include "general_def.h"
#include "bsp_dwt.h"
#include "bsp_pwm.h"
#include "bsp_gpio.h"
#include "tim.h"
#include "daemon.h"
#include <math.h>     // 计算三角函数时用到
#include "cmsis_os.h" // FreeRTOS头文件

static uint8_t idx                                                    = 0;
static WheelMotor_Instance *wheelmotor_instances[WHEEL_MOTOR_MAX_NUM] = {NULL}; // 电机实例列表

static void WheelMotorLossCallback(void *owner);
static void DecodeWheelMotor(WheelMotor_Instance *motor);

WheelMotor_Instance *WheelMotorInit(Motor_Init_Config_s *config, WheelMotor_Init_Config_s *wheelmotor_config)
{
    if (idx >= WHEEL_MOTOR_MAX_NUM)
        return NULL;

    WheelMotor_Instance *motor = (WheelMotor_Instance *)malloc(sizeof(WheelMotor_Instance));
    if (motor == NULL)
        return NULL;
    memset(motor, 0, sizeof(WheelMotor_Instance));

    motor->motor_type     = config->motor_type;
    motor->motor_settings = config->controller_setting_init_config;

    PIDInit(&motor->motor_controller.current_PID, &config->controller_param_init_config.current_PID);
    PIDInit(&motor->motor_controller.speed_PID, &config->controller_param_init_config.speed_PID);
    PIDInit(&motor->motor_controller.angle_PID, &config->controller_param_init_config.angle_PID);
    motor->motor_controller.other_angle_feedback_ptr = config->controller_param_init_config.other_angle_feedback_ptr;
    motor->motor_controller.other_speed_feedback_ptr = config->controller_param_init_config.other_speed_feedback_ptr;
    motor->motor_controller.speed_feedforward_ptr    = config->controller_param_init_config.speed_feedforward_ptr;
    motor->motor_controller.current_feedforward_ptr  = config->controller_param_init_config.current_feedforward_ptr;

    // 初始化PWM和编码器
    motor->pwm     = PWMRegister(&wheelmotor_config->pwm_init_config);
    motor->encoder = TIM_Encoder_Register(&wheelmotor_config->encoder_init_config);

    wheelmotor_config->gpio_init_config_1.exti_mode           = EXTI_MODE_NONE;
    wheelmotor_config->gpio_init_config_2.exti_mode           = EXTI_MODE_NONE;
    wheelmotor_config->gpio_init_config_1.gpio_model_callback = NULL;
    wheelmotor_config->gpio_init_config_2.gpio_model_callback = NULL;
    motor->gpio_1                                             = GPIORegister(&wheelmotor_config->gpio_init_config_1);
    // motor->gpio_2                                             = GPIORegister(&wheelmotor_config->gpio_init_config_2);

    // 初始化测量数据
    motor->measurement.encoder_total_count = 0;
    motor->measurement.total_angle         = 0;
    motor->measurement.total_round         = 0;
    motor->measurement.speed_aps           = 0;
    motor->measurement.linear_speed        = 0;
    motor->measurement.direction           = 0; // 初始设定为正转

    Daemon_Init_Config_s daemon_config = {
        .callback     = WheelMotorLossCallback,
        .owner_id     = motor,
        .reload_count = 2, // 20ms未收到数据则判定丢失
    };
    motor->daemon = DaemonRegister(&daemon_config);
    WheelMotorEnable(motor); // 设置电机启动标志

    wheelmotor_instances[idx++] = motor; // 保存到实例数组
    return motor;
}

void WheelMotorControl(void)
{
    // 遍历所有电机实例，更新数据
    for (size_t i = 0; i < idx; i++) {
        if (wheelmotor_instances[i] != NULL) {
            DecodeWheelMotor(wheelmotor_instances[i]);
        }
    }
    // 此处可添加其他周期处理函数
    WheelMotor_Instance *motor;
    // Motor_Control_Setting_s *motor_setting; // 电机控制参数
    // Motor_Controller_s *motor_controller;   // 电机控制器
    // WheelMotor_Measurement_s *measurement;  // 电机测量数据
    // float pid_measurement, pid_ref;

    for (size_t i = 0; i < idx; i++) {
        motor = wheelmotor_instances[i];
        if (motor == NULL) continue;

        if (motor->stop_flag == MOTOR_STOP) {
            motor->gpio_1->pin_state = GPIOReset(motor->gpio_1); // 停止电机
            PWMSetDutyRatio(motor->pwm, 0.0f);                   // 停止电机
        } else if (motor->stop_flag == MOTOR_ENABLE) {
            motor->gpio_1->pin_state = GPIOReset(motor->gpio_1);
            PWMSetDutyRatio(motor->pwm, 1.f); // 启动电机
        }
    }
}

/**
 * @brief 设置电机启动标志
 *
 * @param motor 电机实例指针
 */
void WheelMotorEnable(WheelMotor_Instance *motor)
{
    motor->stop_flag = MOTOR_ENABLE;
}

/**
 * @brief 停止电机：直接清零发送的电流值
 *
 * @param motor 电机实例指针
 */
void WheelMotorStop(WheelMotor_Instance *motor)
{
    motor->stop_flag = MOTOR_STOP;
}

/**
 * @brief 设置电机参考值
 *
 * @param motor 电机实例指针
 * @param ref 参考值
 */
void WheelMotorSetRef(WheelMotor_Instance *motor, float ref)
{
    motor->motor_controller.pid_ref = ref;
}

/**
 * @brief 电机通信丢失回调函数，停止电机
 */
static void WheelMotorLossCallback(void *owner)
{
    WheelMotor_Instance *motor = (WheelMotor_Instance *)owner;
    motor->stop_flag           = MOTOR_STOP;
}

static void DecodeWheelMotor(WheelMotor_Instance *motor)
{
    // 静态变量记录前一次编码器值（按实例编号区分）
    static uint16_t last_encoder[WHEEL_MOTOR_MAX_NUM] = {0};
    uint8_t id                                        = 0;
    for (; id < WHEEL_MOTOR_MAX_NUM; ++id) {
        if (wheelmotor_instances[id] == motor)
            break;
    }

    // 当前时间
    uint32_t now_ms     = osKernelGetTickCount();
    uint32_t dt_ms      = now_ms - motor->last_tick_ms;
    motor->last_tick_ms = now_ms;

    // 转为秒
    motor->dt = dt_ms / 1000.0f;
    if (motor->dt <= 0.0f) motor->dt = 0.001f; // 防止除以0

    // 当前编码器读数
    uint16_t curr_encoder = __HAL_TIM_GET_COUNTER(motor->encoder->htim);
    int16_t delta         = (int16_t)(curr_encoder - last_encoder[id]);
    last_encoder[id]      = curr_encoder;

    motor->measurement.encoder = curr_encoder;
    motor->measurement.encoder_total_count += delta;

    // 角度变化（度）
    float angle_increment = delta * DEGREE_PER_COUNT;

    // 角速度（度/秒）
    motor->measurement.speed_aps = angle_increment / motor->dt;

    // 线速度（m/s）= 角速度（deg/s）/ 360 × 轮周长
    motor->measurement.linear_speed = (motor->measurement.speed_aps / 360.0f) * WHEEL_CIRCUMFERENCE_M;

    // 累计角度与圈数
    motor->measurement.total_angle += angle_increment;
    motor->measurement.total_round = (int32_t)(motor->measurement.total_angle / 360.0f);

    // 方向判断（0 正转，1 反转）
    motor->measurement.direction = (delta >= 0) ? 0 : 1;

    // 重载守护进程
    DaemonReload(motor->daemon);
}

/**
 * @brief 获取电机累计编码器值（长距离计数）
 *
 * @param motor 电机实例指针
 * @return int32_t 当前累计编码器值
 */
int32_t WheelMotorGetTotalEncoder(WheelMotor_Instance *motor)
{
    return motor->measurement.encoder_total_count;
}
