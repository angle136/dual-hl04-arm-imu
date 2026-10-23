#include "dt7.h"
#include "bsp_def.h"
#include "bsp_uart.h"
#include "module_offline.h"
#include "module_remote.h"
#include <stdlib.h>
#include "usart.h"

#define LOG_TAG "remote_dt7"
#define LOG_LVL LOG_LVL_DBG
#include "ulog_def.h"

#if defined(REMOTE_SOURCE) && (REMOTE_SOURCE == 2)

#define DT7_FRAME_SIZE 18

BUFFER_SECTION static uint8_t dt7_rx_buf[64];
static UART_Device           *dt7_uart;
static bool                   dt7_initialized;

int8_t remote_dt7_init(Offline_Device **out_offline)
{
    REMOTE_UART.Init.BaudRate = 100000;
    if (HAL_UART_Init(&REMOTE_UART) != HAL_OK)
    {
        LOG_E("uart init error");
        return -1;
    }

    Offline_Init_config_t offline_cfg = {
        .name       = "dt7",
        .timeout_ms = 50,
        .beep_times = 0,
        .enable     = REMOTE_OFFLINE_ENABLE,
    };
    Offline_Device *offline = Module_Offline_register(&offline_cfg);
    if (offline == NULL)
    {
        LOG_E("offline register error");
        return -1;
    }
    if (out_offline) *out_offline = offline;

    UART_Device_init_config uart_cfg = {
        .huart           = &REMOTE_UART,
        .expected_rx_len = DT7_FRAME_SIZE,
        .rx_buf          = dt7_rx_buf,
        .rx_buf_size     = 64,
        .rx_mode         = UART_MODE_DMA,
        .tx_mode         = UART_MODE_BLOCKING,
    };
    dt7_uart = BSP_UART_Device_Init(&uart_cfg);
    if (dt7_uart == NULL)
    {
        LOG_E("uart device init error");
        return -1;
    }

    dt7_initialized = true;
    LOG_I("dt7 init success");
    return 0;
}

void remote_dt7_decode(Remote_Data_t *data, Offline_Device *offline)
{
    if (!dt7_initialized || !data) return;

    static uint8_t buf[64];
    uint32_t       rx_len;
    if (BSP_UART_Read(dt7_uart, buf, sizeof(buf), &rx_len, TX_WAIT_FOREVER) <= 0) return;

    for (uint32_t i = 0; i + DT7_FRAME_SIZE - 1 < rx_len; i++)
    {
        int16_t ch1 = (buf[i] | buf[i + 1] << 8) & 0x07FF;
        int16_t ch2 = (buf[i + 1] >> 3 | buf[i + 2] << 5) & 0x07FF;
        int16_t ch3 = (buf[i + 2] >> 6 | buf[i + 3] << 2 | buf[i + 4] << 10) & 0x07FF;
        int16_t ch4 = (buf[i + 4] >> 1 | buf[i + 5] << 7) & 0x07FF;

        if (ch1 < DT7_CH_VALUE_MIN || ch1 > DT7_CH_VALUE_MAX || ch2 < DT7_CH_VALUE_MIN || ch2 > DT7_CH_VALUE_MAX || ch3 < DT7_CH_VALUE_MIN ||
            ch3 > DT7_CH_VALUE_MAX || ch4 < DT7_CH_VALUE_MIN || ch4 > DT7_CH_VALUE_MAX)
        {
            continue;
        }

        data->channels[0] = ch1 - DT7_CH_VALUE_OFFSET;
        data->channels[1] = ch2 - DT7_CH_VALUE_OFFSET;
        data->channels[2] = ch3 - DT7_CH_VALUE_OFFSET;
        data->channels[3] = ch4 - DT7_CH_VALUE_OFFSET;

        if (abs(data->channels[0]) <= REMOTE_DEAD_ZONE) data->channels[0] = 0;
        if (abs(data->channels[1]) <= REMOTE_DEAD_ZONE) data->channels[1] = 0;
        if (abs(data->channels[2]) <= REMOTE_DEAD_ZONE) data->channels[2] = 0;
        if (abs(data->channels[3]) <= REMOTE_DEAD_ZONE) data->channels[3] = 0;

        data->dt7.sw1 = ((buf[i + 5] >> 4) & 0x000C) >> 2;
        data->dt7.sw2 = (buf[i + 5] >> 4) & 0x0003;

#if (REMOTE_VT_SOURCE == 0)
        /* 鼠标 — 仅在无外部图传时使用 DT7 自带键鼠 */
        data->mouse.mouse_x = (int16_t)(buf[i + 6] | (buf[i + 7] << 8));
        data->mouse.mouse_y = (int16_t)(buf[i + 8] | (buf[i + 9] << 8));
        data->mouse.mouse_z = (int16_t)(buf[i + 10] | (buf[i + 11] << 8));
        data->mouse.mouse_l = buf[i + 12];
        data->mouse.mouse_r = buf[i + 13];

        if (data->mouse.mouse_x < -32768 || data->mouse.mouse_x > 32767) data->mouse.mouse_x = 0;
        if (data->mouse.mouse_y < -32768 || data->mouse.mouse_y > 32767) data->mouse.mouse_y = 0;
        if (data->mouse.mouse_z < -32768 || data->mouse.mouse_z > 32767) data->mouse.mouse_z = 0;

        data->keyboard.key_code = buf[i + 14] | (buf[i + 15] << 8);
#endif

        int16_t wheel = (buf[i + 16] | buf[i + 17] << 8) - DT7_CH_VALUE_OFFSET;
        if (abs(wheel) <= REMOTE_DEAD_ZONE * 10) wheel = 0;
        data->dt7.wheel = wheel;

        if (offline) Module_Offline_device_update(offline);
    }
}

#else

int8_t remote_dt7_init(Offline_Device **out_offline)
{
    (void)out_offline;
    return -1;
}
void remote_dt7_decode(Remote_Data_t *data, Offline_Device *offline)
{
    (void)data;
    (void)offline;
}

#endif
