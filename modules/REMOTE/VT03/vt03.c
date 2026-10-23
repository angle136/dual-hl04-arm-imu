#include "vt03.h"
#include "bsp_def.h"
#include "bsp_uart.h"
#include "crc_rm.h"
#include "module_offline.h"
#include "module_remote.h"
#include <stdint.h>
#include <stdlib.h>
#include "usart.h"

#define LOG_TAG "remote_vt03"
#define LOG_LVL LOG_LVL_DBG
#include "ulog_def.h"

#if defined(REMOTE_VT_SOURCE) && (REMOTE_VT_SOURCE == 2)

#define VT03_FRAME_SIZE 21
#define VT03_HEADER0    0xA9
#define VT03_HEADER1    0x53

BUFFER_SECTION static uint8_t vt03_rx_buf[64];
static UART_Device           *vt03_uart;
static bool                   vt03_initialized;

int8_t remote_vt03_init(Offline_Device **out_offline)
{
    REMOTE_VT_UART.Init.BaudRate = 921600;
    if (HAL_UART_Init(&REMOTE_VT_UART) != HAL_OK)
    {
        LOG_E("uart init error");
        return -1;
    }

    Offline_Init_config_t offline_cfg = {
        .name       = "vt03",
        .timeout_ms = 100,
        .beep_times = 0,
        .enable     = REMOTE_VT_OFFLINE_ENABLE,
    };
    Offline_Device *offline = Module_Offline_register(&offline_cfg);
    if (offline == NULL)
    {
        LOG_E("offline register error");
        return -1;
    }
    if (out_offline) *out_offline = offline;

    UART_Device_init_config uart_cfg = {
        .huart           = &REMOTE_VT_UART,
        .expected_rx_len = VT03_FRAME_SIZE,
        .rx_buf          = vt03_rx_buf,
        .rx_buf_size     = 64,
        .rx_mode         = UART_MODE_DMA,
        .tx_mode         = UART_MODE_BLOCKING,
    };
    vt03_uart = BSP_UART_Device_Init(&uart_cfg);
    if (vt03_uart == NULL)
    {
        LOG_E("uart device init error");
        return -1;
    }

    vt03_initialized = true;
    LOG_I("vt03 init success");
    return 0;
}

void remote_vt03_decode(Remote_Data_t *data, Offline_Device *offline)
{
    if (!vt03_initialized || !data) return;

    static uint8_t buf[64];
    uint32_t       rx_len;
    if (BSP_UART_Read(vt03_uart, buf, sizeof(buf), &rx_len, TX_WAIT_FOREVER) <= 0) return;

    for (uint32_t i = 0; i + VT03_FRAME_SIZE - 1 < rx_len; i++)
    {
        /* 帧头校验 */
        if (buf[i] != VT03_HEADER0 || buf[i + 1] != VT03_HEADER1) continue;

        /* CRC16 校验 */
        if (Verify_CRC16_Check_Sum(&buf[i], VT03_FRAME_SIZE) != RM_TRUE) continue;

        /* 摇杆通道解码 (零偏 + 死区)  */
        int16_t ch1 = (buf[i + 2] | (buf[i + 3] << 8)) & 0x07FF;
        int16_t ch2 = ((buf[i + 3] >> 3) | (buf[i + 4] << 5)) & 0x07FF;
        int16_t ch3 = ((buf[i + 4] >> 6) | (buf[i + 5] << 2) | (buf[i + 6] << 10)) & 0x07FF;
        int16_t ch4 = ((buf[i + 6] >> 1) | (buf[i + 7] << 7)) & 0x07FF;

        if (ch1 < VT03_CH_VALUE_MIN || ch1 > VT03_CH_VALUE_MAX || ch2 < VT03_CH_VALUE_MIN || ch2 > VT03_CH_VALUE_MAX || ch3 < VT03_CH_VALUE_MIN ||
            ch3 > VT03_CH_VALUE_MAX || ch4 < VT03_CH_VALUE_MIN || ch4 > VT03_CH_VALUE_MAX)
        {
            continue;
        }

        data->channels[0] = ch1 - VT03_CH_VALUE_OFFSET;
        data->channels[1] = ch2 - VT03_CH_VALUE_OFFSET;
        data->channels[2] = ch3 - VT03_CH_VALUE_OFFSET;
        data->channels[3] = ch4 - VT03_CH_VALUE_OFFSET;

        if (abs(data->channels[0]) <= REMOTE_DEAD_ZONE) data->channels[0] = 0;
        if (abs(data->channels[1]) <= REMOTE_DEAD_ZONE) data->channels[1] = 0;
        if (abs(data->channels[2]) <= REMOTE_DEAD_ZONE) data->channels[2] = 0;
        if (abs(data->channels[3]) <= REMOTE_DEAD_ZONE) data->channels[3] = 0;

        /* 滚轮 */
        int16_t wheel = ((buf[i + 8] >> 1) | (buf[i + 9] << 7)) & 0x07FF;
        wheel -= VT03_CH_VALUE_OFFSET;
        data->vt03.wheel = wheel;

        /* 按键 */
        data->vt03.switch_pos   = (buf[i + 7] >> 4) & 0x03;
        data->vt03.pause        = (buf[i + 7] >> 6) & 0x01;
        data->vt03.custom_left  = (buf[i + 7] >> 7) & 0x01;
        data->vt03.custom_right = (buf[i + 8] >> 0) & 0x01;
        data->vt03.trigger      = (buf[i + 9] >> 4) & 0x01;

        /* 鼠标 */
        data->mouse.mouse_x = (int16_t)(buf[i + 10] | (buf[i + 11] << 8));
        data->mouse.mouse_y = (int16_t)(buf[i + 12] | (buf[i + 13] << 8));
        data->mouse.mouse_z = (int16_t)(buf[i + 14] | (buf[i + 15] << 8));
        data->mouse.mouse_l = buf[i + 16] & 0x03;
        data->mouse.mouse_r = (buf[i + 16] >> 2) & 0x03;
        data->mouse.mouse_m = (buf[i + 16] >> 4) & 0x03;

        if (data->mouse.mouse_x < -32768 || data->mouse.mouse_x > 32767) data->mouse.mouse_x = 0;
        if (data->mouse.mouse_y < -32768 || data->mouse.mouse_y > 32767) data->mouse.mouse_y = 0;
        if (data->mouse.mouse_z < -32768 || data->mouse.mouse_z > 32767) data->mouse.mouse_z = 0;

        /* 键盘 */
        data->keyboard.key_code = buf[i + 17] | (buf[i + 18] << 8);

        if (offline) Module_Offline_device_update(offline);
    }
}

#else

int8_t remote_vt03_init(Offline_Device **out_offline)
{
    (void)out_offline;
    return -1;
}
void remote_vt03_decode(Remote_Data_t *data, Offline_Device *offline)
{
    (void)data;
    (void)offline;
}

#endif
