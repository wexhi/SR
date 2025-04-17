#include "robot_cmd.h"
#include "robot_def.h"

#include "key.h"
#include "message_center.h"
#include "miniPC_process.h"
#include "JY901S.h"
#include "cliff.h"

#include "math.h"

static Robot_Status_e robot_status = ROBOT_STOP;    // The status of the robot
static Obstacle_State_e obs_state  = OBSTACLE_NONE; // The state of the obstacle

static KEY_Instance *key_l, *key_r; // The key instance for left and right
static Vision_Recv_s *vision_ctrl;  // The vision receive data

static Chassis_Ctrl_Cmd_s chassis_cmd_send;      // The command to send to the robot chassis
static Chassis_Upload_Data_s chassis_fetch_data; // The data to fetch from the robot chassis
static Sensor_Upload_Data_s sensor_fetch_data;   // The data to upload from the sensors
static Publisher_t *chassis_cmd_pub;             // The publisher for the command topic
static Subscriber_t *chassis_fetch_sub;          // The subscriber for the command topic
static Subscriber_t *sensor_sub;

static int cliff_trigger_idx     = -1; // 触发的是哪个传感器
static float rotate_target_angle = 0.0f;
static float yaw_start           = 0.0f;
static float yaw_error_cliif     = 0.0f;

static JY901S_attitude_t *attitude_cmd = NULL;

static void RobotStop(void);                   // The function to stop the robot
static void RobotEnableSet(KEY_Instance *key); // The function to set the robot mode
static void RobotModeSet(KEY_Instance *key);
static void ObstacleAvoidance(void); // The function to set the robot mode to obstacle avoidance

/**
 * @brief 机器人核心控制任务初始化,会被RobotInit()调用
 *
 */
void RobotCMDInit(void)
{
    attitude_cmd = INS_Init(); // Initialize the JY901S sensor
    // Initialize the key instances
    KEY_Config_s key_config = {
        .gpio_config = {
            .GPIOx     = KEY_L_GPIO_Port,
            .GPIO_Pin  = KEY_L_Pin,
            .pin_state = GPIO_PIN_SET,
            .exti_mode = GPIO_EXTI_MODE_FALLING,
        },
        .on_press = NULL, // No callback for now
    };
    key_l                           = KEYRegister(&key_config);
    key_config.gpio_config.GPIOx    = KEY_R_GPIO_Port;
    key_config.gpio_config.GPIO_Pin = KEY_R_Pin;
    key_r                           = KEYRegister(&key_config);

    vision_ctrl = VisionInit(&huart1); // 初始化视觉控制

    // Register the command topic for the robot chassis
    chassis_cmd_pub   = PubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));
    chassis_fetch_sub = SubRegister("chassis_fetch", sizeof(Chassis_Upload_Data_s));
    sensor_sub        = SubRegister("sensor_fetch", sizeof(Sensor_Upload_Data_s)); // Register the cliff data topic
    robot_status      = ROBOT_READY;                                               // Set the robot status to ready
}

void RobotCMDTask(void)
{
    // Get the modules status
    SubGetMessage(chassis_fetch_sub, &chassis_fetch_data); // Get the data from the robot chassis
    SubGetMessage(sensor_sub, &sensor_fetch_data);         // Get the data from the sensors

    RobotEnableSet(key_l); // Set the robot mode based on the key

    // TODO: Get the command from the PC and set the vx and wz values

    PubPushMessage(chassis_cmd_pub, (void *)&chassis_cmd_send); // Publish the command to the robot chassis
    // TODO: Publish the data to the PC
    VisionSend(); // Send the data to the PC
}

static void RobotEnableSet(KEY_Instance *key)
{
    // Check the key count to determine the mode
    switch (key->count % 2) {
        case 0:
            RobotStop();
            break;
        default:
            robot_status                  = ROBOT_READY;    // Set the robot status to run
            chassis_cmd_send.chassis_mode = CHASSIS_NORMAL; // Set the chassis mode to normal
            RobotModeSet(key_r);
            break;
    }
}

static void RobotModeSet(KEY_Instance *key)
{
    // Check the key count to determine the mode
    switch (key->count % 2) {
        case 0:
            ObstacleAvoidance();
            break;
        default:
            chassis_cmd_send.vx = 0.15f; // Set the forward speed to 0
            chassis_cmd_send.wz = -0.0f; // Set the angular speed to 0
            break;
    }
}

static void RobotStop(void)
{
    // Stop the robot
    robot_status                  = ROBOT_STOP;         // Set the robot status to stop
    chassis_cmd_send.chassis_mode = CHASSIS_ZERO_FORCE; // Set the chassis mode to stop
    chassis_cmd_send.vx           = 0.0f;               // Set the forward speed to 0
    chassis_cmd_send.wz           = 0.0f;               // Set the angular speed to 0
}

static void ObstacleAvoidance(void)
{
    float current_yaw = attitude_cmd->YawTotalAngle; // Get the current yaw angle
    switch (obs_state) {
        case OBSTACLE_NONE:
            // 检测是否触发悬崖
            for (int i = 0; i < 4; ++i) {
                if (sensor_fetch_data.cliff_detected[i]) {
                    obs_state         = OBSTACLE_DETECTED;
                    cliff_trigger_idx = i;
                    break;
                }
            }
            chassis_cmd_send.vx = 0.15f; // Set the forward speed to 0
            chassis_cmd_send.wz = -0.0f; // Set the angular speed to 0
            break;
        case OBSTACLE_DETECTED:
            // 设置转向目标角度
            yaw_start           = current_yaw;
            rotate_target_angle = yaw_start + 180.0f;
            // TODO: Add the direction of the turn
            obs_state = OBSTACLE_TURNING;
            break;
        case OBSTACLE_TURNING:
            // 旋转中（固定角速度）
            chassis_cmd_send.vx = 0.0f;
            chassis_cmd_send.wz = -0.7f; // fan转

            yaw_error_cliif = rotate_target_angle - current_yaw;

            if (fabsf(yaw_error_cliif) < 5.0f) { // 角度接近目标
                obs_state = OBSTACLE_RECOVERY;
            }
            break;
        case OBSTACLE_RECOVERY:
            chassis_cmd_send.vx = 0.15f;
            chassis_cmd_send.wz = 0.0f;
            obs_state           = OBSTACLE_NONE;
            break;
    }
}