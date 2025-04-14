#include "wheelmotor.h"
#include "general_def.h"
#include "bsp_dwt.h"
#include "daemon.h"
#include "cmsis_os.h" // FreeRTOS的osDelay函数需要包含此头文件
#include <math.h>     // 计算三角函数时用到

static uint8_t idx                                                    = 0;
static WheelMotor_Instance *wheelmotor_instances[WHEEL_MOTOR_MAX_NUM] = {NULL}; // 电机实例列表

static void WheelMotorLossCallback(void *owner);
static void DecodeWheelMotor(WheelMotor_Instance *motor);

WheelMotor_Instance *WheelMotorInit(Motor_Init_Config_s *config)
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
    motor->pwm     = PWMRegister(&config->pwm_init_config);
    motor->encoder = TIM_Encoder_Register(&config->encoder_init_config);

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
    for (uint8_t i = 0; i < idx; i++) {
        if (wheelmotor_instances[i] != NULL) {
            DecodeWheelMotor(wheelmotor_instances[i]);
        }
    }
    // 此处可添加其他周期处理函数
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

/**
 * @brief 更新编码器数据，并计算角速度、线速度及累计角度
 *
 * 处理步骤：
 *  1. 调用 osDelay(2) 以满足 FreeRTOS 任务周期要求（2ms周期）。
 *  2. 利用静态数组保存上一次16位编码器计数值，对本周期计数差（delta）进行计算（考虑溢出）。
 *  3. 利用 DWT_GetDeltaT 获取本周期时间 dt（秒）。
 *  4. 根据 delta 计算角度增量（单位：度），更新累计角度，并计算角速度（单位：deg/s）。
 *  5. 根据角速度换算线速度：线速度 = (角速度/360) × 轮子周长（m/s）。
 *  6. 根据累计角度计算完整转圈数。
 *  7. 更新旋转方向：delta>=0 为正转，反之为反转。
 */
static void DecodeWheelMotor(WheelMotor_Instance *motor)
{
    // 用静态数组记录各实例上一次的编码器数值
    static uint16_t last_encoder[WHEEL_MOTOR_MAX_NUM] = {0};
    uint8_t id                                        = 0;
    for (; id < WHEEL_MOTOR_MAX_NUM; ++id) {
        if (wheelmotor_instances[id] == motor)
            break;
    }

    // 读取当前编码器值（16位计数器）
    uint16_t curr_encoder = __HAL_TIM_GET_COUNTER(motor->encoder->htim);
    // 使用 int16_t 差分，自动处理溢出情况
    int16_t delta    = (int16_t)(curr_encoder - last_encoder[id]);
    last_encoder[id] = curr_encoder;

    motor->measurement.encoder = curr_encoder;
    motor->measurement.encoder_total_count += delta;

    // 通过 DWT 获取本周期间隔，单位为秒
    motor->dt = DWT_GetDeltaT(&motor->feed_cnt);

    // 根据每个计数对应的角度（度）计算本周期角度增量
    float angle_increment = delta * DEGREE_PER_COUNT; // 单位：度

    // 计算角速度（deg/s）
    if (motor->dt > 0)
        motor->measurement.speed_aps = angle_increment / motor->dt;
    else
        motor->measurement.speed_aps = 0;

    // 根据角速度换算线速度：
    // 每秒转速（rev/s） = 角速度 (deg/s) / 360，线速度 = rev/s × 轮周长
    motor->measurement.linear_speed = (motor->measurement.speed_aps / 360.0f) * WHEEL_CIRCUMFERENCE_M;

    // 更新累计角度和圈数
    motor->measurement.total_angle += angle_increment;
    motor->measurement.total_round = (int32_t)(motor->measurement.total_angle / 360.0f);

    // 更新旋转方向
    motor->measurement.direction = (delta >= 0) ? 0 : 1;

    // 重载看门狗，确保系统监控正常
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
