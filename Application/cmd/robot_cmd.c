#include "robot_cmd.h"
#include "robot_def.h"
#include "goto_controller.h"

#include "key.h"
#include "message_center.h"
#include "miniPC_process.h"
#include "JY901S.h"
#include "cliff.h"
#include "general_def.h"

#include "math.h"

static Robot_Status_e robot_status = ROBOT_STOP;    // The status of the robot
static Obstacle_State_e obs_state  = OBSTACLE_NONE; // The state of the obstacle

static KEY_Instance *key_l, *key_r; // The key instance for left and right
static Vision_Recv_s *vision_ctrl;  // The vision receive data

static Chassis_Ctrl_Cmd_s chassis_cmd_send;      // The command to send to the robot chassis
static Chassis_Upload_Data_s chassis_fetch_data; // The data to fetch from the robot chassis
static Sensor_Upload_Data_s sensor_fetch_data;   // The data to upload from the sensors
static Publisher_t *chassis_cmd_pub;             // The publisher for the command topic
static Publisher_t *robot_state_pub;             // The publisher for the robot state
static Subscriber_t *chassis_fetch_sub;          // The subscriber for the command topic
static Subscriber_t *sensor_sub;

static int cliff_trigger_idx     = -1; // 触发的是哪个传感器
static float rotate_target_angle = 0.0f;
static float yaw_start           = 0.0f;
static float yaw_error_cliif     = 0.0f;
static float norm; // Check the quaternion norm

static JY901S_attitude_t *attitude_cmd = NULL;

static Pose2D_t init_pose = {0};               /* 机器人上电原点 (0,0,0°) */
static void RobotStop(void);                   // The function to stop the robot
static void RobotEnableSet(KEY_Instance *key); // The function to set the robot mode
static void RobotModeSet(KEY_Instance *key);
static void ObstacleAvoidance(void); // The function to set the robot mode to obstacle avoidance
static void RobotGoTo(void);

static void VisionControl(void);

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
    GotoCtrl_Init(&init_pose);         /* 初始化控制器 */

    // Register the command topic for the robot chassis
    chassis_cmd_pub   = PubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));
    chassis_fetch_sub = SubRegister("chassis_fetch", sizeof(Chassis_Upload_Data_s));
    robot_state_pub   = PubRegister("robot_state", sizeof(Robot_Status_e));
    // Register the command topic for the sensors
    sensor_sub   = SubRegister("sensor_fetch", sizeof(Sensor_Upload_Data_s)); // Register the cliff data topic
    robot_status = ROBOT_READY;                                               // Set the robot status to ready
}

void RobotCMDTask(void)
{
    // Get the modules status
    SubGetMessage(chassis_fetch_sub, &chassis_fetch_data); // Get the data from the robot chassis
    SubGetMessage(sensor_sub, &sensor_fetch_data);         // Get the data from the sensors
    float dt = chassis_fetch_data.dt;
    if (chassis_cmd_send.chassis_mode == CHASSIS_GOTO_POINT &&
        robot_status == ROBOT_READY) // 只有启动车才导航
    {
        VelocityCmd_t out = GotoCtrl_Step(chassis_fetch_data.real_vx,
                                          chassis_fetch_data.real_wz,
                                          attitude_cmd->YawTotalAngle,
                                          dt);

        chassis_cmd_send.vx = out.vx;
        chassis_cmd_send.wz = out.wz;

        /* 到点即停 */
        if (GotoCtrl_IsArrived()) {
            chassis_cmd_send.chassis_mode = CHASSIS_ZERO_FORCE;
        }
    }
    RobotEnableSet(key_l); // Set the robot mode based on the key

    // TODO: Get the command from the PC and set the vx and wz values

    PubPushMessage(chassis_cmd_pub, (void *)&chassis_cmd_send); // Publish the command to the robot chassis
    PubPushMessage(robot_state_pub, (void *)&robot_status);     // Publish the robot state to the PC

    // TODO: Publish the data to the PC
    float vx, wz, ax, ay, az;
    vx         = chassis_fetch_data.real_vx;
    wz         = chassis_fetch_data.real_wz;
    float q[4] = {attitude_cmd->Quaternion[0],
                  +attitude_cmd->Quaternion[1],
                  +attitude_cmd->Quaternion[2],
                  +attitude_cmd->Quaternion[3]};

    ax = attitude_cmd->Accel[0];
    ay = attitude_cmd->Accel[1];
    az = attitude_cmd->Accel[2];

    norm = sqrtf(q[0] * q[0] + q[1] * q[1] +
                 +q[2] * q[2] + q[3] * q[3]); // Check the quaternion norm
    VisionValueSet(vx, wz, q, ax, ay, az);    // Send the data to the PC

    VisionSend(); // Send the data to the PC
}

static uint8_t goal_executed = 0; /* 0 = 还没跑过 GOTO, 1 = 已执行 */

static void RobotEnableSet(KEY_Instance *key)
{
    // Check the key count to determine the mode
    switch (key->count % 2) {
        case 0:
            RobotStop();
            break;
        default:
            robot_status = ROBOT_READY; // Set the robot status to run

            /***** Only Select one of the below three options  *****/

            // RobotModeSet(key);           // Set the robot mode based on the key,
            // if (goal_executed == 0) { RobotGoTo(); }
            VisionControl();
            break;
    }
}

__unused static void RobotModeSet(KEY_Instance *key_r)
{

    switch (key_r->count % 2) {
        case 0: /* NORMAL / 避障 */
            chassis_cmd_send.chassis_mode = CHASSIS_NORMAL;
            ObstacleAvoidance();
            break;
        case 1: /* 手动直行示例 */
            chassis_cmd_send.chassis_mode = CHASSIS_NORMAL;
            VisionControl();
            break;
    }
}

/**
 * @brief Only run once, can not be used with RobotModeSet
 *
 */
__unused static void RobotGoTo(void)
{
    chassis_cmd_send.chassis_mode = CHASSIS_GOTO_POINT;
    GotoCtrl_SetGoal(1.5, 0); // Set the goal to (1, 0)
    goal_executed = 1;
}

static void RobotStop(void)
{
    robot_status                  = ROBOT_STOP;
    chassis_cmd_send.chassis_mode = CHASSIS_ZERO_FORCE;
    chassis_cmd_send.vx           = 0.0f;
    chassis_cmd_send.wz           = 0.0f;
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

static void VisionControl(void)
{
    if (fabsf(vision_ctrl->vx) < 0.01 && fabsf(vision_ctrl->wz) < 0.01) {
        chassis_cmd_send.chassis_mode = CHASSIS_ZERO_FORCE;
    } else {
        chassis_cmd_send.chassis_mode = CHASSIS_NORMAL;
    }
    // Set the chassis command based on the vision control
    chassis_cmd_send.vx = vision_ctrl->vx;
    chassis_cmd_send.wz = vision_ctrl->wz;
}