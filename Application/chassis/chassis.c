#include "chassis.h"
#include "robot_def.h"

#include "message_center.h"
#include "wheelmotor.h"
#include "JY901S.h"

#include "bsp_dwt.h"
#include "arm_math.h"

static Chassis_Ctrl_Cmd_s chassis_cmd_recv;       // The command to receive from the robot_cmd
static Chassis_Upload_Data_s chassis_upload_data; // The data to upload to the robot_cmd

static Publisher_t *chassis_upload_pub;        // The publisher for the upload topic
static Subscriber_t *chassis_cmd_sub;          // The subscriber for the command topic
static JY901S_attitude_t *attitude = NULL;     // Added missing variable declaration
static WheelMotor_Instance *motor_l, *motor_r; // The left and right wheel motors
static float chassis_vx, chassis_wz;           // The forward speed and angular speed of the robot
static float wheel_l_speed, wheel_r_speed;     // The speed of the left and right wheels

void ChassisInit()
{
    attitude = INS_Init(); // Initialize the JY901S sensor
    // TODO: Initialize the chassis motors and sensors here
    Motor_Init_Config_s motor_config = {
        .motor_type                   = WheelMotor,
        .controller_param_init_config = {
            .speed_PID = {
                .Kp     = 0.1f,
                .Ki     = 0.01f,
                .Kd     = 0.001f,
                .MaxOut = 100.0f,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type       = SPEED_LOOP,
            .close_loop_type       = SPEED_LOOP,
        },
    };
    WheelMotor_Init_Config_s wheelmotor_config = {
        .pwm_init_config = {
            .htim      = &htim1,
            .channel   = TIM_CHANNEL_1,
            .period    = 0.02f, // 20ms
            .dutyratio = 0.f,   // 50% duty cycle
            .callback  = NULL,  // No callback for now
            .id        = NULL,  // No ID for now
            .is_N      = 1,
        },
        .encoder_init_config = {
            .htim = &htim5,
        },
        .gpio_init_config_1 = {
            .GPIOx     = GPIOE,
            .GPIO_Pin  = GPIO_PIN_7,
            .pin_state = GPIO_PIN_RESET,
        },
        .gpio_init_config_2 = {
            .GPIOx     = GPIOE,
            .GPIO_Pin  = GPIO_PIN_8,
            .pin_state = GPIO_PIN_RESET,
        },
    };

    motor_l = WheelMotorInit(&motor_config, &wheelmotor_config); // Initialize the left wheel motor

    wheelmotor_config.pwm_init_config.channel     = TIM_CHANNEL_3; // Change the channel for the left motor
    wheelmotor_config.pwm_init_config.is_N        = 0;
    wheelmotor_config.encoder_init_config.htim    = &htim2;
    wheelmotor_config.gpio_init_config_1.GPIO_Pin = GPIO_PIN_12;
    wheelmotor_config.gpio_init_config_2.GPIO_Pin = GPIO_PIN_13;
    motor_r                                       = WheelMotorInit(&motor_config, &wheelmotor_config); // Initialize the right wheel motor

    chassis_cmd_sub    = SubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));       // Subscribe to the command topic
    chassis_upload_pub = PubRegister("chassis_upload", sizeof(Chassis_Upload_Data_s)); // Register the upload topic
}

// The core function of the chassis task
void ChassisTask(void)
{
    SubGetMessage(chassis_cmd_sub, &chassis_cmd_recv); // Get the command from the robot_cmd

    if (chassis_cmd_recv.chassis_mode == CHASSIS_ZERO_FORCE) {
        // TODO: Motor stop
        WheelMotorStop(motor_l);
        WheelMotorStop(motor_r);
    } else {
        // TODO: Motor Enable
        WheelMotorEnable(motor_l);
        WheelMotorEnable(motor_r);
    }

    chassis_vx = chassis_cmd_recv.vx; // Get the forward speed from the command
    chassis_wz = chassis_cmd_recv.wz; // Get the angular speed from the command

    // TODO: Control the motors based on the chassis_vx and chassis_wz values
    // TODO: Get the real speed and battery level from the motors and sensors
    wheel_l_speed = motor_l->measurement.linear_speed; // Get the speed of the left wheel
    wheel_r_speed = motor_r->measurement.linear_speed;

    PubPushMessage(chassis_upload_pub, (void *)&chassis_upload_data); // Publish the upload data
}