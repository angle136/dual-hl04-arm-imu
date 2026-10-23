#include "module_lora.h"
#include "bsp_def.h"
#include "bsp_dwt.h"
#include "module_offline.h"
#include "tx_api.h"
#include "usart.h"
#include <stdbool.h>
#include <string.h>

#define LOG_TAG "module_lora"
#define LOG_LVL LOG_LVL_INFO
#include "ulog_def.h"

#define LORA_FRAME_HDR0  0xAAu
#define LORA_FRAME_HDR1  0x55u
#define LORA_MAX_PAYLOAD 59u
#define LORA_MAX_REGS    16u
#define LORA_RX_BUF_SIZE 512u

typedef struct
{
    void          *value;
    const char    *name;
    Lora_Data_Type type;
    uint8_t        size;
} Lora_Registration;

static UART_Device               *lora_uart;
static Offline_Device            *lora_offline;
static Lora_Registration          lora_regs[LORA_MAX_REGS];
static uint8_t                    lora_reg_count;
static uint8_t                    lora_payload_len;
static uint8_t                    lora_tx_seq;
static volatile uint8_t           lora_started;
static uint32_t                   lora_last_tx_ms;
static TX_THREAD                  lora_thread;
APPS_STACK_SECTION static uint8_t lora_stack[LORA_TASK_STACK_SIZE];
BUFFER_SECTION static uint8_t     lora_rxbuf[LORA_RX_BUF_SIZE];
/* DMA 发送源必须落在 DMA 可访问内存；F4 的 CCMRAM 和 H7 的 DTCM 均不能被 DMA 读取。 */
BUFFER_SECTION static uint8_t lora_txbuf[2u + 1u + 1u + LORA_MAX_PAYLOAD + 1u];

static void lora_task_entry(ULONG arg);

static uint8_t lora_crc8(uint8_t crc, uint8_t byte)
{
    crc ^= byte;
    for (uint8_t bit = 0; bit < 8; bit++) crc = (crc & 0x80u) ? (uint8_t)((crc << 1) ^ 0x07u) : (uint8_t)(crc << 1);
    return crc;
}

static uint8_t lora_type_size(Lora_Data_Type type)
{
    switch (type)
    {
    case LORA_TYPE_INT8:
    case LORA_TYPE_UINT8:
        return 1;
    case LORA_TYPE_INT16:
    case LORA_TYPE_UINT16:
        return 2;
    case LORA_TYPE_INT32:
    case LORA_TYPE_UINT32:
    case LORA_TYPE_FLOAT:
        return 4;
    case LORA_TYPE_DOUBLE:
        return 8;
    default:
        return 0;
    }
}

static void lora_put_le(uint8_t *out, const void *value, uint8_t size)
{
    uint64_t number = 0;
    memcpy(&number, value, size);
    for (uint8_t i = 0; i < size; i++) out[i] = (uint8_t)(number >> (8u * i));
}

#if LORA_RX_ENABLE
static void lora_get_le(void *value, const uint8_t *in, uint8_t size)
{
    uint64_t number = 0;
    for (uint8_t i = 0; i < size; i++) number |= (uint64_t)in[i] << (8u * i);
    memcpy(value, &number, size);
}
#endif

static uint8_t lora_serialize(uint8_t *payload)
{
    uint8_t offset = 0;
    for (uint8_t i = 0; i < lora_reg_count; i++)
    {
        lora_put_le(&payload[offset], lora_regs[i].value, lora_regs[i].size);
        offset = (uint8_t)(offset + lora_regs[i].size);
    }
    return offset;
}

#if LORA_RX_ENABLE
static bool lora_deserialize(const uint8_t *payload, uint8_t length)
{
    uint8_t offset = 0;
    for (uint8_t i = 0; i < lora_reg_count; i++)
    {
        if ((uint16_t)offset + lora_regs[i].size > length) return false;
        lora_get_le(lora_regs[i].value, &payload[offset], lora_regs[i].size);
        offset = (uint8_t)(offset + lora_regs[i].size);
    }
    if (offset != length) return false;
    if (lora_offline != NULL) Module_Offline_device_update(lora_offline);
    return true;
}
#endif

static bool lora_wait_idle(uint32_t timeout_ms)
{
#if defined(LORA_AUX_GPIO_PORT) && defined(LORA_AUX_GPIO_PIN)
    if (LORA_AUX_ENABLE)
    {
        uint32_t start = (uint32_t)BSP_DWT_GetTimeline_ms();
        while (HAL_GPIO_ReadPin(LORA_AUX_GPIO_PORT, LORA_AUX_GPIO_PIN) == GPIO_PIN_RESET)
        {
            if ((uint32_t)BSP_DWT_GetTimeline_ms() - start > timeout_ms) return false;
        }
    }
#else
    (void)timeout_ms;
#endif
    return true;
}

static bool lora_send_frame(void)
{
    uint8_t *frame = lora_txbuf;
    uint8_t  payload[LORA_MAX_PAYLOAD];
    uint8_t  length = lora_serialize(payload);
    uint8_t  crc    = 0;

    frame[0] = LORA_FRAME_HDR0;
    frame[1] = LORA_FRAME_HDR1;
    frame[2] = length;
    frame[3] = lora_tx_seq++;
    for (uint8_t i = 0; i < length; i++) frame[4 + i] = payload[i];

    crc = lora_crc8(crc, frame[2]);
    crc = lora_crc8(crc, frame[3]);
    for (uint8_t i = 0; i < length; i++) crc = lora_crc8(crc, payload[i]);
    frame[4 + length] = crc;

    if (!lora_wait_idle(1000)) return false;
    return BSP_UART_Send(lora_uart, frame, (uint32_t)(5 + length), 100) > 0;
}

#if LORA_RX_ENABLE
static struct
{
    uint8_t state;
    uint8_t length;
    uint8_t index;
    uint8_t seq;
    uint8_t crc;
    uint8_t payload[LORA_MAX_PAYLOAD];
} lora_parser;

static bool lora_feed_byte(uint8_t byte)
{
    switch (lora_parser.state)
    {
    case 0:
        lora_parser.state = byte == LORA_FRAME_HDR0 ? 1 : 0;
        return false;
    case 1:
        lora_parser.state = byte == LORA_FRAME_HDR1 ? 2 : 0;
        return false;
    case 2:
        lora_parser.length = byte;
        if (byte > LORA_MAX_PAYLOAD || byte != lora_payload_len)
            lora_parser.state = 0;
        else
        {
            lora_parser.index = 0;
            lora_parser.crc   = lora_crc8(0, byte);
            lora_parser.state = 3;
        }
        return false;
    case 3:
        lora_parser.seq   = byte;
        lora_parser.crc   = lora_crc8(lora_parser.crc, byte);
        lora_parser.state = lora_parser.length == 0 ? 5 : 4;
        return false;
    case 4:
        lora_parser.payload[lora_parser.index++] = byte;
        lora_parser.crc                          = lora_crc8(lora_parser.crc, byte);
        if (lora_parser.index == lora_parser.length) lora_parser.state = 5;
        return false;
    case 5:
    {
        bool received     = byte == lora_parser.crc && lora_deserialize(lora_parser.payload, lora_parser.length);
        lora_parser.state = 0;
        return received;
    }
    default:
        lora_parser.state = 0;
        return false;
    }
}
#endif

void Module_Lora_Init(void)
{
    /* M0/M1 使用普通输出模式，保持低电平进入透明传输模式。 */
    GPIO_InitTypeDef lora_gpio = {
        .Pin   = LORA_M0_GPIO_PIN,
        .Mode  = GPIO_MODE_OUTPUT_PP,
        .Pull  = GPIO_PULLDOWN,
        .Speed = GPIO_SPEED_FREQ_HIGH,
    };
    HAL_GPIO_Init(LORA_M0_GPIO_PORT, &lora_gpio);

    lora_gpio.Pin = LORA_M1_GPIO_PIN;
    HAL_GPIO_Init(LORA_M1_GPIO_PORT, &lora_gpio);

#if defined(LORA_AUX_GPIO_PORT) && defined(LORA_AUX_GPIO_PIN)
    if (LORA_AUX_ENABLE)
    {
        lora_gpio.Mode = GPIO_MODE_INPUT;
        lora_gpio.Pull = GPIO_NOPULL;
        lora_gpio.Pin  = LORA_AUX_GPIO_PIN;
        HAL_GPIO_Init(LORA_AUX_GPIO_PORT, &lora_gpio);
    }
#endif

    HAL_GPIO_WritePin(LORA_M0_GPIO_PORT, LORA_M0_GPIO_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LORA_M1_GPIO_PORT, LORA_M1_GPIO_PIN, GPIO_PIN_RESET);

    LORA_UART.Init.BaudRate = 115200;
    if (HAL_UART_Init(&LORA_UART) != HAL_OK) return;

    UART_Device_init_config config = {
        .huart           = &LORA_UART,
        .expected_rx_len = 0,
        .rx_buf          = lora_rxbuf,
        .rx_buf_size     = LORA_RX_BUF_SIZE,
        .rx_mode         = LORA_RX_ENABLE ? UART_MODE_DMA : UART_MODE_BLOCKING,
        .tx_mode         = UART_MODE_DMA,
    };
    lora_uart = BSP_UART_Device_Init(&config);
    if (lora_uart == NULL) return;

    Offline_Init_config_t offline = {
        .name       = "lora",
        .beep_times = 0,
        .enable     = LORA_OFFLINE_ENABLE,
        .timeout_ms = 100,
    };
    lora_offline = Module_Offline_register(&offline);

    if (tx_thread_create(&lora_thread, "lora", lora_task_entry, 0, lora_stack, LORA_TASK_STACK_SIZE, LORA_TASK_PRIORITY, LORA_TASK_PRIORITY,
                         TX_NO_TIME_SLICE, TX_AUTO_START) != TX_SUCCESS)
    {
        LOG_E("lora thread create failed");
        return;
    }

    LOG_I("module lora init finished");
}

int Lora_Register(const char *name, void *value, Lora_Data_Type type)
{
    uint8_t size = lora_type_size(type);
    if (lora_started || name == NULL || value == NULL || size == 0 || lora_reg_count >= LORA_MAX_REGS) return -1;
    if ((uint16_t)lora_payload_len + size > LORA_MAX_PAYLOAD) return -1;
    lora_regs[lora_reg_count++] = (Lora_Registration){.value = value, .name = name, .type = type, .size = size};
    lora_payload_len            = (uint8_t)(lora_payload_len + size);
    return 0;
}

int Lora_Start(void)
{
    if (lora_uart == NULL || lora_reg_count == 0 || lora_payload_len > LORA_MAX_PAYLOAD) return -1;
    lora_last_tx_ms = (uint32_t)BSP_DWT_GetTimeline_ms();
    lora_started    = 1;
    return 0;
}

static void lora_process(void)
{
#if LORA_RX_ENABLE
    uint8_t  buffer[64];
    uint32_t length = 0;
#endif
    if (!lora_started) return;

#if LORA_RX_ENABLE
    if (BSP_UART_Read(lora_uart, buffer, sizeof(buffer), &length, 0) > 0)
        for (uint32_t i = 0; i < length; i++) lora_feed_byte(buffer[i]);
#endif

    uint32_t now = (uint32_t)BSP_DWT_GetTimeline_ms();
#if LORA_TX_ENABLE
    if (now - lora_last_tx_ms >= LORA_TX_INTERVAL_MS)
    {
        if (lora_send_frame()) lora_last_tx_ms = now;
    }
#else
    (void)now;
#endif
}

uint8_t Module_Lora_get_offline_status(void) { return Module_Offline_get_device_status(lora_offline); }

static void lora_task_entry(ULONG arg)
{
    (void)arg;
    while (1)
    {
        lora_process();
        tx_thread_sleep(10);
    }
}
