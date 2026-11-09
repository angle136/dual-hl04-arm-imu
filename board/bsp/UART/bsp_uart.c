#include "bsp_uart.h"
#include "bsp_def.h"
#include "kfifo.h"
#include "usart.h"

#if defined(STM32F407xx)
#include "stm32f4xx_hal_uart.h"
#elif defined(STM32H723xx)
#include "stm32h7xx_hal_uart.h"
#elif defined(STM32F105xC) || defined(STM32F103xB)
#include "stm32f1xx_hal_uart.h"
#endif

#include "tx_api.h"
#include <string.h>

#define LOG_TAG "bsp_uart"
#define LOG_LVL LOG_LVL_WARNING
#include "ulog_def.h"

/* 事件标志位定义 */
#define BSP_UART_EVENT_RX ((ULONG)0x01) /* RX 数据就绪 */
#define BSP_UART_EVENT_TX ((ULONG)0x02) /* TX 发送完成 */

typedef struct UART_Device
{
    UART_HandleTypeDef  *huart;           // uart 句柄
    struct kfifo         rx_fifo;         // 接收 FIFO
    uint8_t             *rx_buf;          // DMA / IT 硬件接收缓冲区指针
    uint32_t             buf_size;        // 硬件接收缓冲区大小
    volatile uint32_t    last_buf_pos;    // 上次 DMA 缓冲区处理位置（仅 DMA 模式）
    volatile uint32_t    rx_event_count;
    volatile uint32_t    dma_bytes_received;
    volatile uint32_t    fifo_bytes_enqueued;
    volatile uint32_t    fifo_bytes_read;
    volatile uint32_t    fifo_bytes_dropped;
    volatile uint32_t    fifo_bytes_discarded_on_restart;
    volatile uint32_t    uart_error_count;
    volatile uint32_t    parity_error_count;
    volatile uint32_t    noise_error_count;
    volatile uint32_t    frame_error_count;
    volatile uint32_t    overrun_error_count;
    volatile uint32_t    dma_error_count;
    volatile uint32_t    dma_start_count;
    volatile uint32_t    rx_start_failure_count;
    volatile uint32_t    last_error_code;
    volatile uint8_t     recent_byte_count;
    volatile uint8_t     recent_byte_write_index;
    volatile uint8_t     recent_bytes[BSP_UART_DIAG_RECENT_BYTE_COUNT];
    uint32_t             expected_rx_len; // 期望接收数据长度，0 = 不定长
    TX_EVENT_FLAGS_GROUP event_flags;     // 事件标志组 (RX / TX)
    UART_Mode            rx_mode;         // 接收模式
    UART_Mode            tx_mode;         // 发送模式
    uint8_t              initialed;       // 设备是否已初始化
} UART_Device;

static UART_Device uart_device_table[BSP_UART_DEVICE_NUM];

/* 内部辅助函数  */

static UART_Device *Find_Device(UART_HandleTypeDef *huart)
{
    for (int i = 0; i < BSP_UART_DEVICE_NUM; ++i)
        if (uart_device_table[i].initialed && uart_device_table[i].huart == huart) return &uart_device_table[i];
    return NULL;
}

static int Find_Free_Slot(void)
{
    for (int i = 0; i < BSP_UART_DEVICE_NUM; ++i)
        if (!uart_device_table[i].initialed) return i;
    return -1;
}

static void record_recent_bytes(UART_Device *dev, const uint8_t *bytes, size_t length)
{
    for (size_t i = 0U; i < length; ++i)
    {
        dev->recent_bytes[dev->recent_byte_write_index] = bytes[i];
        dev->recent_byte_write_index =
            (uint8_t)((dev->recent_byte_write_index + 1U) % BSP_UART_DIAG_RECENT_BYTE_COUNT);
        if (dev->recent_byte_count < BSP_UART_DIAG_RECENT_BYTE_COUNT) dev->recent_byte_count++;
    }
}

static void discard_pending_fifo(UART_Device *dev)
{
    uint32_t pending = kfifo_len(&dev->rx_fifo);
    if (pending > dev->buf_size) pending = dev->buf_size;
    dev->fifo_bytes_discarded_on_restart += pending;
    dev->last_buf_pos = 0U;
    dev->rx_fifo.in   = 0U;
    dev->rx_fifo.out  = 0U;
}

static HAL_StatusTypeDef start_rx_it(UART_Device *dev)
{
    uint16_t expect = dev->expected_rx_len > 0 ? (uint16_t)dev->expected_rx_len : (uint16_t)dev->buf_size;
    HAL_StatusTypeDef status = HAL_UARTEx_ReceiveToIdle_IT(dev->huart, dev->rx_buf, expect);
    if (status != HAL_OK) dev->rx_start_failure_count++;
    return status;
}

static HAL_StatusTypeDef start_rx_dma(UART_Device *dev)
{
    discard_pending_fifo(dev);
    HAL_StatusTypeDef status = HAL_UARTEx_ReceiveToIdle_DMA(dev->huart, dev->rx_buf, (uint16_t)dev->buf_size);
    if (status != HAL_OK) dev->rx_start_failure_count++;
    else dev->dma_start_count++;
    return status;
}

int BSP_UART_Rearm_DMA(UART_Device *device)
{
    if (device == NULL || device->initialed == 0U || device->rx_mode != UART_MODE_DMA) return -1;

    if (HAL_UART_AbortReceive(device->huart) != HAL_OK) return -1;

    /* 清除传感器在主控接收器尚未启动期间留下的 UART 状态位。 */
    __HAL_UART_CLEAR_OREFLAG(device->huart);
    __HAL_UART_CLEAR_IDLEFLAG(device->huart);

    discard_pending_fifo(device);
    if (HAL_UARTEx_ReceiveToIdle_DMA(device->huart, device->rx_buf, (uint16_t)device->buf_size) != HAL_OK)
    {
        device->rx_start_failure_count++;
        return -1;
    }
    device->dma_start_count++;

    return 0;
}

int BSP_UART_GetDiagnostics(UART_Device *device, BSP_UART_Diagnostics_t *diagnostics)
{
    if (device == NULL || diagnostics == NULL || device->initialed == 0U) return -1;

    diagnostics->rx_event_count        = device->rx_event_count;
    diagnostics->dma_bytes_received    = device->dma_bytes_received;
    diagnostics->fifo_bytes_enqueued   = device->fifo_bytes_enqueued;
    diagnostics->fifo_bytes_read       = device->fifo_bytes_read;
    diagnostics->fifo_bytes_dropped    = device->fifo_bytes_dropped;
    diagnostics->fifo_bytes_discarded_on_restart = device->fifo_bytes_discarded_on_restart;
    diagnostics->fifo_depth            = kfifo_len(&device->rx_fifo);
    if (diagnostics->fifo_depth > device->buf_size) diagnostics->fifo_depth = device->buf_size;
    diagnostics->uart_error_count      = device->uart_error_count;
    diagnostics->parity_error_count    = device->parity_error_count;
    diagnostics->noise_error_count     = device->noise_error_count;
    diagnostics->frame_error_count     = device->frame_error_count;
    diagnostics->overrun_error_count   = device->overrun_error_count;
    diagnostics->dma_error_count       = device->dma_error_count;
    diagnostics->dma_start_count       = device->dma_start_count;
    diagnostics->rx_start_failure_count = device->rx_start_failure_count;
    diagnostics->last_error_code       = device->last_error_code;
    diagnostics->uart_rx_state         = (uint8_t)device->huart->RxState;
    diagnostics->dma_state             = (device->huart->hdmarx != NULL) ? (uint8_t)device->huart->hdmarx->State : 0xFFU;
    diagnostics->dma_remaining         = (device->huart->hdmarx != NULL) ? __HAL_DMA_GET_COUNTER(device->huart->hdmarx) : 0U;
    diagnostics->dma_buffer_size        = device->buf_size;
    diagnostics->dma_position           = (diagnostics->dma_buffer_size >= diagnostics->dma_remaining)
                                             ? diagnostics->dma_buffer_size - diagnostics->dma_remaining
                                             : 0U;
    diagnostics->rx_active              = (device->huart->RxState == HAL_UART_STATE_BUSY_RX) ? 1U : 0U;
    diagnostics->dma_active             = (device->huart->hdmarx != NULL && device->huart->hdmarx->State == HAL_DMA_STATE_BUSY) ? 1U : 0U;
    diagnostics->recent_byte_count = device->recent_byte_count;
    uint8_t recent_start = (uint8_t)((device->recent_byte_write_index + BSP_UART_DIAG_RECENT_BYTE_COUNT -
                                      diagnostics->recent_byte_count) % BSP_UART_DIAG_RECENT_BYTE_COUNT);
    for (uint8_t i = 0U; i < diagnostics->recent_byte_count; ++i)
    {
        diagnostics->recent_bytes[i] = device->recent_bytes[(recent_start + i) % BSP_UART_DIAG_RECENT_BYTE_COUNT];
    }
    for (uint8_t i = diagnostics->recent_byte_count; i < BSP_UART_DIAG_RECENT_BYTE_COUNT; ++i)
    {
        diagnostics->recent_bytes[i] = 0U;
    }
    return 0;
}

/* 对外函数 */

UART_Device *BSP_UART_Device_Init(UART_Device_init_config *config)
{
    if (!config || !config->huart)
    {
        LOG_E("Invalid config or huart");
        return NULL;
    }
    if (Find_Device((UART_HandleTypeDef *)config->huart))
    {
        LOG_E("Device already exists");
        return NULL;
    }

    int slot = Find_Free_Slot();
    if (slot < 0)
    {
        LOG_E("No free slot (max %d)", BSP_UART_DEVICE_NUM);
        return NULL;
    }

    UART_Device *dev = &uart_device_table[slot];
    memset(dev, 0, sizeof(UART_Device));
    dev->huart           = (UART_HandleTypeDef *)config->huart;
    dev->rx_mode         = config->rx_mode;
    dev->tx_mode         = config->tx_mode;
    dev->expected_rx_len = config->expected_rx_len;

    /* ── RX 初始化 ── */
    if (config->rx_mode != UART_MODE_BLOCKING)
    {
        /* 参数检查 */
        if (!config->rx_buf || config->rx_buf_size < 2)
        {
            LOG_E("RX buffer invalid");
            return NULL;
        }
        if (config->rx_buf_size & (config->rx_buf_size - 1))
        {
            LOG_E("rx_buf_size must be power of 2");
            return NULL;
        }
        if (BSP_IS_DMA_INACCESSIBLE(config->rx_buf))
        {
            LOG_E("RX buffer in DMA inaccessible memory");
            return NULL;
        }

        /* 初始化 kfifo */
        if (kfifo_init(&dev->rx_fifo, config->rx_buf, config->rx_buf_size, 1) != 0)
        {
            LOG_E("kfifo_init failed");
            return NULL;
        }

        dev->buf_size = config->rx_buf_size;
        dev->rx_buf   = config->rx_buf;
    }
    else
    {
        /* 阻塞模式：仅保存缓冲区信息，不使用 kfifo */
        dev->buf_size = config->rx_buf_size;
        dev->rx_buf   = config->rx_buf;
    }

    /* 创建事件标志组 */
    if (tx_event_flags_create(&dev->event_flags, "uart_evt") != TX_SUCCESS)
    {
        LOG_E("Event flags create failed");
        if (config->rx_mode != UART_MODE_BLOCKING)
        {
            tx_byte_release(dev->rx_fifo.data);
            dev->rx_fifo.data = NULL;
        }
        return NULL;
    }

    dev->initialed = 1;

    /* 启动首次接收 */
    if (dev->rx_mode == UART_MODE_IT)
    {
        start_rx_it(dev);
    }
    else if (dev->rx_mode == UART_MODE_DMA)
    {
        start_rx_dma(dev);
    }

    LOG_I("UART init OK: slot=%d, rx=%d, tx=%d", slot, (int)dev->rx_mode, (int)dev->tx_mode);
    return dev;
}

int BSP_UART_Send(UART_Device *device, uint8_t *data, uint32_t len, uint32_t timeout)
{
    if (!device || !data || len == 0) return -1;
    if (!device->initialed) return -1;

    switch (device->tx_mode)
    {
    case UART_MODE_BLOCKING:
    {
        unsigned int      tmo = (timeout == TX_WAIT_FOREVER) ? HAL_MAX_DELAY : timeout;
        HAL_StatusTypeDef st  = HAL_UART_Transmit(device->huart, data, len, tmo);
        return (st == HAL_OK) ? (int)len : -1;
    }
    case UART_MODE_IT:
        HAL_UART_Transmit_IT(device->huart, data, len);
        break;
    case UART_MODE_DMA:
        if (BSP_IS_DMA_INACCESSIBLE(data)) return -1;
        BSP_CACHE_CLEAN(data, len);
        HAL_UART_Transmit_DMA(device->huart, data, len);
        break;
    }

    /* 等待 TX 完成事件 */
    ULONG actual_flags = 0;
    UINT  evt_st       = tx_event_flags_get(&device->event_flags, BSP_UART_EVENT_TX, TX_OR_CLEAR, &actual_flags, timeout);
    return (evt_st == TX_SUCCESS) ? (int)len : -1;
}

int BSP_UART_Read(UART_Device *device, uint8_t *buf, uint32_t buf_size, uint32_t *rx_len, uint32_t timeout)
{
    if (!device || !device->initialed || !buf || buf_size == 0)
    {
        if (rx_len) *rx_len = 0;
        return -1;
    }

    /* 阻塞模式：直接调用 HAL */
    if (device->rx_mode == UART_MODE_BLOCKING)
    {
        uint16_t tmp    = 0;
        uint32_t tmo    = (timeout == TX_WAIT_FOREVER) ? HAL_MAX_DELAY : timeout;
        uint16_t expect = device->expected_rx_len > 0 ? (uint16_t)device->expected_rx_len : (uint16_t)device->buf_size;
        if (expect > buf_size) expect = (uint16_t)buf_size;

        if (HAL_UARTEx_ReceiveToIdle(device->huart, buf, expect, &tmp, tmo) == HAL_OK)
        {
            if (rx_len) *rx_len = tmp;
            return (int)tmp;
        }
        if (rx_len) *rx_len = 0;
        return -1;
    }

    /* IT / DMA 模式：等待 RX 事件 → 从 kfifo 读出 */
    ULONG actual_flags = 0;
    if (tx_event_flags_get(&device->event_flags, BSP_UART_EVENT_RX, TX_OR_CLEAR, &actual_flags, timeout) != TX_SUCCESS)
    {
        if (rx_len) *rx_len = 0;
        return -1;
    }

    /* 从 kfifo 中读出数据 */
    uint32_t avail   = kfifo_len(&device->rx_fifo);
    uint32_t to_read = (avail <= buf_size) ? avail : buf_size;
    uint32_t actual  = kfifo_out(&device->rx_fifo, buf, to_read);
    device->fifo_bytes_read += actual;
    if (rx_len) *rx_len = actual;
    return (int)actual;
}

/*  HAL 回调  */

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    UART_Device *dev = Find_Device(huart);
    if (dev)
    {
        tx_event_flags_set(&dev->event_flags, BSP_UART_EVENT_TX, TX_OR);
    }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    UART_Device *dev = Find_Device(huart);
    if (!dev) return;

    if (dev->rx_mode == UART_MODE_IT)
    {
        /* IT 模式：HAL 每次从 rx_buf[0] 开始写，直接重置 kfifo 索引 */
        if (Size > 0)
        {
            dev->rx_event_count++;
            dev->dma_bytes_received += Size;
            dev->rx_fifo.in  = Size;
            dev->rx_fifo.out = 0;
            dev->fifo_bytes_enqueued += Size;
            record_recent_bytes(dev, dev->rx_buf, Size);
            __DMB();
            tx_event_flags_set(&dev->event_flags, BSP_UART_EVENT_RX, TX_OR);
        }
        start_rx_it(dev);
    }
    else if (dev->rx_mode == UART_MODE_DMA)
    {
        dev->rx_event_count++;
        /* DMA 模式：环形缓冲区，计算新增数据长度并更新写索引 */
        size_t dma_size = dev->buf_size;
        size_t ndtr     = __HAL_DMA_GET_COUNTER(huart->hdmarx);
        if (ndtr > dma_size) ndtr = dma_size;
        size_t curr_pos = dma_size - ndtr;
        size_t last_pos = dev->last_buf_pos;

        /* 确保 CPU 能看到 DMA 写入的最新数据 */
        BSP_CACHE_INVALIDATE(dev->rx_buf, dma_size);

        size_t new_len = 0;
        if (curr_pos >= last_pos)
        {
            new_len = curr_pos - last_pos;
        }
        else
        {
            new_len = dma_size - last_pos + curr_pos;
        }

        if (new_len > 0)
        {
            const size_t received_len = new_len;
            const size_t tail_len = (received_len < BSP_UART_DIAG_RECENT_BYTE_COUNT)
                                        ? received_len
                                        : BSP_UART_DIAG_RECENT_BYTE_COUNT;
            dev->dma_bytes_received += (uint32_t)received_len;
            uint8_t tail[BSP_UART_DIAG_RECENT_BYTE_COUNT];
            for (size_t i = 0U; i < tail_len; ++i)
            {
                size_t dma_index = (curr_pos + dma_size - tail_len + i) % dma_size;
                tail[i] = dev->rx_buf[dma_index];
            }
            record_recent_bytes(dev, tail, tail_len);

            /* 保护 kfifo 容量: in 不超过 out + mask + 1 (即 fifo 容量) */
            unsigned int fifo_cap = dev->rx_fifo.mask + 1;
            unsigned int max_in   = dev->rx_fifo.out + fifo_cap;
            unsigned int space    = (dev->rx_fifo.in < max_in) ? (max_in - dev->rx_fifo.in) : 0;
            if (new_len > space) new_len = space;

            if (new_len > 0)
            {
                dev->rx_fifo.in += new_len;
                dev->fifo_bytes_enqueued += (uint32_t)new_len;
                __DMB();
                tx_event_flags_set(&dev->event_flags, BSP_UART_EVENT_RX, TX_OR);
            }
            dev->fifo_bytes_dropped += (uint32_t)(received_len - new_len);
            dev->last_buf_pos = curr_pos;
        }
    }
}
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    UART_Device *dev = Find_Device(huart);
    if (dev != NULL)
    {
        uint32_t error = huart->ErrorCode;
        dev->uart_error_count++;
        dev->last_error_code = error;
        if ((error & HAL_UART_ERROR_PE) != 0U) dev->parity_error_count++;
        if ((error & HAL_UART_ERROR_NE) != 0U) dev->noise_error_count++;
        if ((error & HAL_UART_ERROR_FE) != 0U) dev->frame_error_count++;
        if ((error & HAL_UART_ERROR_ORE) != 0U) dev->overrun_error_count++;
        if ((error & HAL_UART_ERROR_DMA) != 0U) dev->dma_error_count++;
    }

    HAL_UART_Abort(huart); /* 同步 abort，消除异步竞态 */
    if (!dev) return;
    if (dev->rx_mode == UART_MODE_IT)
    {
        start_rx_it(dev);
    }
    else if (dev->rx_mode == UART_MODE_DMA)
    {
        start_rx_dma(dev);
    }
}

void HAL_UART_AbortCpltCallback(UART_HandleTypeDef *huart) { (void)huart; }
