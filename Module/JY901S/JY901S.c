#include "JY901S.h"
#include "bsp_usart.h"
#define RXBUFFER_LEN 44

static JY901S_Instance *INS;

static void JY901S_Parsedata(void);

JY901S_attitude_t *INS_Init(void)
{
    if (!INS->is_init) {
        INS = (JY901S_Instance *)malloc(sizeof(JY901S_Instance));
        memset(INS, 0, sizeof(JY901S_Instance));
        INS->is_init = 1;
    } else
        return &INS->attitude;

    // 1. 初始化串口
    USART_Init_Config_s usart_config = {
        .usart_handle    = &huart2,
        .recv_buff_size  = RXBUFFER_LEN,
        .module_callback = JY901S_Parsedata,
    };
    INS->usart_instance = USARTRegister(&usart_config);
    return &INS->attitude;
}
static int r_s = 0;
static void JY901S_Parsedata(void)
{
    r_s++;
}