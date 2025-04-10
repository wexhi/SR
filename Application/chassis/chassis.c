#include "chassis.h"
#include "robot_def.h"

#include "message_center.h"

#include "bsp_dwt.h"
#include "arm_math.h"

static Chassis_Ctrl_Cmd_s chassis_cmd_recv;       // The command to receive from the robot_cmd
static Chassis_Upload_Data_s chassis_upload_data; // The data to upload to the robot_cmd

static Publisher_t *chassis_upload_pub; // The publisher for the upload topic
static Subscriber_t *chassis_cmd_sub;   // The subscriber for the command topic

static float chassis_vx, chassis_wz; // The forward speed and angular speed of the robot

void ChassisInit()
{
    // TODO: Initialize the chassis motors and sensors here

    chassis_cmd_sub    = SubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));       // Subscribe to the command topic
    chassis_upload_pub = PubRegister("chassis_upload", sizeof(Chassis_Upload_Data_s)); // Register the upload topic
}

// The core function of the chassis task
void ChassisTask(void)
{
    SubGetMessage(chassis_cmd_sub, &chassis_cmd_recv); // Get the command from the robot_cmd

    if (chassis_cmd_recv.chassis_mode == CHASSIS_ZERO_FORCE) {
        // TODO: Motor stop
    } else {
        // TODO: Motor Enable
    }

    chassis_vx = chassis_cmd_recv.vx; // Get the forward speed from the command
    chassis_wz = chassis_cmd_recv.wz; // Get the angular speed from the command

    // TODO: Control the motors based on the chassis_vx and chassis_wz values
    // TODO: Get the real speed and battery level from the motors and sensors

    PubPushMessage(chassis_upload_pub, (void *)&chassis_upload_data); // Publish the upload data
}