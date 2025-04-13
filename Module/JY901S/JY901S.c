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

static void JY901S_Parsedata(void)
{
    uint32_t temp;
    temp                = hdma_usart2_rx.Instance->NDTR;
    INS->rx_data.rx_len = RXBUFFER_LEN - temp;
    if (INS->rx_data.rx_len < RXBUFFER_LEN) return;
    memcpy(INS->rx_data.rx_buffer, INS->usart_instance->recv_buff, RXBUFFER_LEN);
    for (uint8_t i = 0; i < 4; i++) {
        if (INS->rx_data.rx_buffer[i * JY901S_FRAME_LEN] != INS->rx_data.frame_head) return;
        switch (INS->rx_data.rx_buffer[i * JY901S_FRAME_LEN + 1]) {
            case 0x51:
                memcpy(&stcAcc, &INS->rx_data.rx_buffer[2 + i * JY901S_FRAME_LEN], 8);
                for (uint8_t j = 0; j < 3; j++) { INS->attitude.Accel[j] = (float)stcAcc.a[j] / 32768 * 16; }
                break;
            case 0x52:
                memcpy(&stcGyro, &INS->rx_data.rx_buffer[2 + i * JY901S_FRAME_LEN], 8);
                for (uint8_t j = 0; j < 3; j++) { INS->attitude.Gyro[j] = (float)stcGyro.w[j] / 32768 * 2000; }
                break;
                // TODO: Parse the rest of imu data
            default:
                break;
        }
    }
}