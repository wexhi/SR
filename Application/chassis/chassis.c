#include "chassis.h"
#include "robot_def.h"

#include "message_center.h"
#include "wheelmotor.h"

#include "bsp_dwt.h"
#include "arm_math.h"

static Chassis_Ctrl_Cmd_s chassis_cmd_recv;       // The command to receive from the robot_cmd
static Chassis_Upload_Data_s chassis_upload_data; // The data to upload to the robot_cmd

static Publisher_t *chassis_upload_pub; // The publisher for the upload topic
static Subscriber_t *chassis_cmd_sub;   // The subscriber for the command topic

static WheelMotor_Instance *motor_l, *motor_r; // The left and right wheel motors
static float chassis_vx, chassis_wz;           // The forward speed and angular speed of the robot

void ChassisInit()
{
    // TODO: Initialize the chassis motors and sensors here
    Motor_Init_Config_s motor_config = {
        .motor_type      = WheelMotor,
        .pwm_init_config = {
            .htim      = &htim1,
            .channel   = TIM_CHANNEL_1,
            .period    = 0.02f, // 20ms
            .dutyratio = 0.f,  // 50% duty cycle
            .callback  = NULL,  // No callback for now
            .id        = NULL,  // No ID for now
        },
        .encoder_init_config = {
            .htim     = &htim5,
        },
        .controller_param_init_config = {
            .speed_PID = {
                .Kp     = 0.1f,
                .Ki     = 0.01f,
                .Kd     = 0.001f,
                .MaxOut = 100.0f,
            },
            .current_PID = {
                .Kp     = 1.f,
                .Ki     = 0.f,
                .Kd     = 0.0f,
                .MaxOut = 100.0f,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type       = SPEED_LOOP,
            .close_loop_type       = SPEED_LOOP | CURRENT_LOOP,
        },
    };

    motor_l = WheelMotorInit(&motor_config); // Initialize the left wheel motor
    motor_config.pwm_init_config.channel = TIM_CHANNEL_3; // Change the channel for the left motor
    motor_config.encoder_init_config.htim = &htim2;
    motor_r = WheelMotorInit(&motor_config); // Initialize the right wheel motor

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

    PubPushMessage(chassis_upload_pub, (void *)&chassis_upload_data); // Publish the upload data
}