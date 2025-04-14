#include "JY901S.h"
#include "JY901S_typedef.h"
#include "bsp_usart.h"
#include "stm32f4xx_it.h"

#define JY901S_FRAME_LEN  11
#define JY901S_FRAME_HEAD 0x55

static JY901S_Instance *INS = NULL;
static struct SAcc stcAcc;
static struct SGyro stcGyro;
static struct SAngle stcAngle;
static struct SMag stcMag;
static struct SQ stcQ;

extern DMA_HandleTypeDef hdma_usart2_rx;

static void JY901S_Parsedata(void);

JY901S_attitude_t *INS_Init(void)
{
    if (INS == NULL) {
        INS = (JY901S_Instance *)malloc(sizeof(JY901S_Instance));
        memset(INS, 0, sizeof(JY901S_Instance));
        INS->is_init = 1;
    } else {
        return &INS->attitude;
    }

    // 1. 初始化串口
    USART_Init_Config_s usart_config = {
        .usart_handle    = &huart2,
        .recv_buff_size  = RXBUFFER_LEN,
        .module_callback = JY901S_Parsedata,
    };
    INS->usart_instance     = USARTRegister(&usart_config);
    INS->rx_data.frame_head = JY901S_FRAME_HEAD;
    return &INS->attitude;
}

static float JY901S_UnwrapYaw(JY901S_attitude_t *att, float current_yaw)
{
    float delta = current_yaw - att->_last_yaw;

    if (delta > 180.0f)
        delta -= 360.0f;
    else if (delta < -180.0f)
        delta += 360.0f;

    att->_yaw_total += delta;
    att->_last_yaw = current_yaw;
    return att->_yaw_total;
}

static void JY901S_Parsedata(void)
{
    uint32_t temp;
    temp                = hdma_usart2_rx.Instance->NDTR;
    INS->rx_data.rx_len = RXBUFFER_LEN - temp;
    if (INS->rx_data.rx_len < RXBUFFER_LEN) return;
    memcpy(INS->rx_data.rx_buffer, INS->usart_instance->recv_buff, RXBUFFER_LEN);
    for (uint8_t i = 0; i < 9; i++) {
        if (INS->rx_data.rx_buffer[i * JY901S_FRAME_LEN] != INS->rx_data.frame_head) return;
        switch (INS->rx_data.rx_buffer[i * JY901S_FRAME_LEN + 1]) {
            case 0x51:
                memcpy(&stcAcc, &INS->rx_data.rx_buffer[2 + i * JY901S_FRAME_LEN], 8);
                for (uint8_t j = 0; j < 3; j++) { INS->attitude.Accel[j] = (float)stcAcc.a[j] / 32768.f * 16.f; }
                break;
            case 0x52:
                memcpy(&stcGyro, &INS->rx_data.rx_buffer[2 + i * JY901S_FRAME_LEN], 8);
                for (uint8_t j = 0; j < 3; j++) { INS->attitude.Gyro[j] = (float)stcGyro.w[j] / 32768.f * 2000.f; }
                break;
            case 0x53:
                memcpy(&stcAngle, &INS->rx_data.rx_buffer[2 + i * JY901S_FRAME_LEN], 8);
                for (uint8_t j = 0; j < 3; j++) { INS->attitude.Angle[j] = (float)stcAngle.Angle[j] / 32768.f * 180.f; }
                INS->attitude.Yaw_Angle     = INS->attitude.Angle[2];
                INS->attitude.Pitch_Angle   = INS->attitude.Angle[0];
                INS->attitude.Roll_Angle    = INS->attitude.Angle[1];
                INS->attitude.YawTotalAngle = JY901S_UnwrapYaw(&INS->attitude, INS->attitude.Yaw_Angle);
                break;
            case 0x54:
                memcpy(&stcMag, &INS->rx_data.rx_buffer[2 + i * JY901S_FRAME_LEN], 8);
                for (uint8_t j = 0; j < 3; j++) { INS->attitude.Mag[j] = (float)stcMag.h[j]; }
                break;
            case 0x59:
                memcpy(&stcQ, &INS->rx_data.rx_buffer[2 + i * JY901S_FRAME_LEN], 8);
                for (uint8_t j = 0; j < 4; j++) { INS->attitude.Quaternion[j] = (float)stcQ.q[j] / 32768.f; }
                break;
                // TODO: Parse the rest of imu data
            default:
                break;
        }
    }
}