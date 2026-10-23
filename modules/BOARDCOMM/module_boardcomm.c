#include "module_boardcomm.h"
#include "bsp_can.h"
#include "module_offline.h"
#include <string.h>

#define LOG_TAG "BoardComm"
#define LOG_LVL LOG_LVL_INFO
#include "ulog_def.h"

static Can_Device            *boardcomm_dev         = NULL;
static Offline_Device        *boardcomm_offline_dev = NULL;
static BoardComm_RxCallback_t app_rx_callback       = NULL;

/* RegisterRxBuffer 模式 */
static void   *rx_buffer       = NULL;
static uint8_t rx_expected_len = 0;

#if !SINGLE_BOARD

/* BSP CAN 内部回调 → 转发给 app 注册的回调 / 直接 memcpy 到注册的缓冲区 */
static void boardcomm_rx_callback(Can_Device *dev, const uint8_t *data, uint8_t len)
{
    (void)dev;

    if (rx_buffer != NULL && len == rx_expected_len)
    {
        memcpy(rx_buffer, data, len);
        Module_Offline_device_update(boardcomm_offline_dev);
    }
    else if (app_rx_callback != NULL)
    {
        app_rx_callback(data, len);
        Module_Offline_device_update(boardcomm_offline_dev);
    }
}
#endif

void Module_BoardComm_Init(void)
{
#if SINGLE_BOARD
    LOG_I("BoardComm skipped (single board)");
    return;
#endif

#if CHASSIS_BOARD
    Can_Device_Init_Config_s cfg = {
        .hcan        = BOARDCOMM_CAN,
        .tx_id       = BOARDCOMM_CHASSIS_ID,
        .rx_id       = BOARDCOMM_GIMBAL_ID,
        .rx_callback = boardcomm_rx_callback,
    };
    boardcomm_dev = BSP_CAN_Device_Init(&cfg);
#endif

#if GIMBAL_BOARD
    Can_Device_Init_Config_s cfg = {
        .hcan        = BOARDCOMM_CAN,
        .tx_id       = BOARDCOMM_GIMBAL_ID,
        .rx_id       = BOARDCOMM_CHASSIS_ID,
        .rx_callback = boardcomm_rx_callback,
    };
    boardcomm_dev = BSP_CAN_Device_Init(&cfg);
#endif

    if (boardcomm_dev == NULL)
    {
        LOG_E("Failed to init can device");
    }

    Offline_Init_config_t offlineconfig = {
        .name       = "boardcomm",
        .beep_times = 10,
        .enable     = BOARDCOMM_OFFLINE_ENABLE,
        .timeout_ms = 100,
    };
    boardcomm_offline_dev = Module_Offline_register(&offlineconfig);
    if (boardcomm_offline_dev == NULL)
    {
        LOG_E("offline device register error");
        return;
    }

    LOG_I("BoardComm initialized");
}

void Module_BoardComm_Send(uint8_t *data, uint8_t len)
{
    if (boardcomm_dev == NULL || data == NULL || len == 0 || len > 8)
    {
        return;
    }
    BSP_CAN_Send(boardcomm_dev, data, len);
}

uint8_t Module_BoardComm_Get_Offline_State(void)
{
    if (boardcomm_offline_dev == NULL)
    {
        return STATE_OFFLINE;
    }
    return Module_Offline_get_device_status(boardcomm_offline_dev);
}

void Module_BoardComm_RegisterRx(BoardComm_RxCallback_t callback) { app_rx_callback = callback; }

void Module_BoardComm_RegisterRxBuffer(void *buffer, uint8_t expected_len)
{
    rx_buffer       = buffer;
    rx_expected_len = expected_len;
}
