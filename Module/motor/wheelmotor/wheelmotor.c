#include "wheelmotor.h"
#include "general_def.h"
#include "bsp_dwt.h"
#include "daemon.h"

static uint8_t idx                                                    = 0;
static WheelMotor_Instance *wheelmotor_instances[WHEEL_MOTOR_MAX_NUM] = {NULL}; // 电机实例列表

static void WheelMotorLossCallback(void *owner);
static void DecodeWheelMotor(WheelMotor_Instance *motor);

WheelMotor_Instance *WheelMotorInit(Motor_Init_Config_s *config)
{
    if (idx >= WHEEL_MOTOR_MAX_NUM) return NULL;

    WheelMotor_Instance *motor = (WheelMotor_Instance *)malloc(sizeof(WheelMotor_Instance));
    if (motor == NULL) return NULL;
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

    // Init PWM
    motor->pwm     = PWMRegister(&config->pwm_init_config);
    motor->encoder = TIM_Encoder_Register(&config->encoder_init_config);

    Daemon_Init_Config_s daemon_config = {
        .callback     = WheelMotorLossCallback,
        .owner_id     = motor,
        .reload_count = 2, // 20ms未收到数据则丢失
    };
    motor->daemon = DaemonRegister(&daemon_config);
    WheelMotorEnable(motor); // 修改电机启动标志

    wheelmotor_instances[idx++] = motor; // 添加到实例数组
    return motor;                        // 返回实例指针
}

void WheelMotorControl(void)
{
    // Update the motor data
    for (uint8_t i = 0; i < idx; i++) {
        if (wheelmotor_instances[i] != NULL) {
            DecodeWheelMotor(wheelmotor_instances[i]);
        }
    }
    // TODO: Connect the encoder sensor
}

/**
 * @brief 修改电机启动标志
 *
 * @param motor 电机实例指针
 */
void WheelMotorEnable(WheelMotor_Instance *motor)
{
    motor->stop_flag = MOTOR_ENABLE;
}

/**
 * @brief 停止电机,注意不是将设定值设为零,而是直接给电机发送的电流值置零
 *
 */
void WheelMotorStop(WheelMotor_Instance *motor)
{
    motor->stop_flag = MOTOR_STOP;
}

void WheelMotorSetRef(WheelMotor_Instance *motor, float ref)
{
    motor->motor_controller.pid_ref = ref;
}

static void WheelMotorLossCallback(void *owner)
{
    WheelMotor_Instance *motor = (WheelMotor_Instance *)owner;
    motor->stop_flag           = MOTOR_STOP; // 停止电机
}

static void DecodeWheelMotor(WheelMotor_Instance *motor)
{
    motor->measurement.encoder = __HAL_TIM_GET_COUNTER(motor->encoder->htim);
    if (motor->measurement.encoder != 0) {
        DaemonReload(motor->daemon); // 重载守护进程
        motor->dt = DWT_GetDeltaT(&motor->feed_cnt);
    }
}