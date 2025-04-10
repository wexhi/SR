#include "robot_cmd.h"
#include "robot_def.h"

#include "key.h"
#include "message_center.h"
static Robot_Status_e robot_status = ROBOT_STOP; // The status of the robot

static KEY_Instance *key_l, *key_r; // The key instance for left and right

static Chassis_Ctrl_Cmd_s chassis_cmd_send;      // The command to send to the robot chassis
static Chassis_Upload_Data_s chassis_fetch_data; // The data to fetch from the robot chassis
static Publisher_t *chassis_cmd_pub;             // The publisher for the command topic
static Subscriber_t *chassis_fetch_sub;          // The subscriber for the command topic

static void RobotStop(void);                 // The function to stop the robot
static void RobotModeSet(KEY_Instance *key); // The function to set the robot mode

/**
 * @brief 机器人核心控制任务初始化,会被RobotInit()调用
 *
 */
void RobotCMDInit(void)
{
    // Initialize the key instances
    KEY_Config_s key_config = {
        .gpio_config = {
            .GPIOx     = KEY_L_GPIO_Port,
            .GPIO_Pin  = KEY_L_Pin,
            .pin_state = GPIO_PIN_RESET,
            .exti_mode = GPIO_EXTI_MODE_FALLING,
        },
        .init_state = GPIO_PIN_RESET,
        .on_press   = NULL, // No callback for now
    };
    key_l                           = KEYRegister(&key_config);
    key_config.gpio_config.GPIOx    = KEY_R_GPIO_Port;
    key_config.gpio_config.GPIO_Pin = KEY_R_Pin;
    key_r                           = KEYRegister(&key_config);

    // Register the command topic for the robot chassis
    chassis_cmd_pub   = PubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));
    chassis_fetch_sub = SubRegister("chassis_fetch", sizeof(Chassis_Upload_Data_s));

    robot_status = ROBOT_READY; // Set the robot status to ready
}

void RobotCMDTask(void)
{
    // Get the modules status
    SubGetMessage(chassis_fetch_sub, &chassis_fetch_data); // Get the data from the robot chassis
    RobotModeSet(key_l);                                   // Set the robot mode based on the key

    // TODO: Get the command from the PC and set the vx and wz values

    PubPushMessage(chassis_cmd_pub, (void *)&chassis_cmd_send); // Publish the command to the robot chassis
    // TODO: Publish the data to the PC
}

static void RobotModeSet(KEY_Instance *key)
{
    // Check the key count to determine the mode
    switch (key->count % 2) {
        case 0:
            RobotStop();
            break;
        default:
            chassis_cmd_send.chassis_mode = CHASSIS_NORMAL; // Set the chassis mode to normal
            break;
    }
}

static void RobotStop(void)
{
    chassis_cmd_send.chassis_mode = CHASSIS_ZERO_FORCE; // Set the chassis mode to stop
    chassis_cmd_send.vx           = 0.0f;               // Set the forward speed to 0
    chassis_cmd_send.wz           = 0.0f;               // Set the angular speed to 0
}