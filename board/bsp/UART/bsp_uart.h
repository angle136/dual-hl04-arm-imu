/*
 * @Author: laladuduqq 2807523947@qq.com
 * @Date: 2026-05-04 12:11:52
 * @LastEditors: laladuduqq 2807523947@qq.com
 * @LastEditTime: 2026-05-05 08:38:28
 * @FilePath: /mas_embedded_threadx/board/bsp/UART/bsp_uart.h
 * @Description:
 */
#ifndef _BSP_UART_H_
#define _BSP_UART_H_

#include <stdint.h>

#if defined(STM32H723xx)
#define BSP_UART_DEVICE_NUM 6
#elif defined(STM32F407xx)
#define BSP_UART_DEVICE_NUM 3
#elif defined(STM32F105xC) || defined(STM32F103xB)
#define BSP_UART_DEVICE_NUM 3
#endif

typedef struct UART_Device UART_Device;

// 模式选择
typedef enum
{
    UART_MODE_BLOCKING,
    UART_MODE_IT,
    UART_MODE_DMA
} UART_Mode;

#define BSP_UART_DIAG_RECENT_BYTE_COUNT 16U

// 初始化配置结构体
typedef struct
{
    void     *huart;           // UART句柄 (UART_HandleTypeDef*)
    uint8_t  *rx_buf;          // RX缓冲区（2的幂）
    uint32_t  rx_buf_size;     // RX缓冲区总大小
    uint32_t  expected_rx_len; // 期望接收的数据包长度(0为不定长)
    UART_Mode rx_mode;         // 接收模式
    UART_Mode tx_mode;         // 发送模式
} UART_Device_init_config;

typedef struct
{
    uint32_t rx_event_count;
    uint32_t dma_bytes_received;
    uint32_t fifo_bytes_enqueued;
    uint32_t fifo_bytes_read;
    uint32_t fifo_bytes_dropped;
    uint32_t fifo_bytes_discarded_on_restart;
    uint32_t fifo_depth;
    uint32_t uart_error_count;
    uint32_t parity_error_count;
    uint32_t noise_error_count;
    uint32_t frame_error_count;
    uint32_t overrun_error_count;
    uint32_t dma_error_count;
    uint32_t dma_start_count;
    uint32_t rx_start_failure_count;
    uint32_t last_error_code;
    uint32_t dma_remaining;
    uint32_t dma_position;
    uint32_t dma_buffer_size;
    uint8_t  uart_rx_state;
    uint8_t  dma_state;
    uint8_t  rx_active;
    uint8_t  dma_active;
    uint8_t  recent_byte_count;
    uint8_t  recent_bytes[BSP_UART_DIAG_RECENT_BYTE_COUNT];
} BSP_UART_Diagnostics_t;

/**
 * @brief UART初始化
 * @param config 初始化配置
 * @return UART_Device* 成功返回设备指针，失败返回NULL
 */
UART_Device *BSP_UART_Device_Init(UART_Device_init_config *config);

/**
 * @brief 清理 UART 接收状态并重新启动 DMA 接收。
 * @param device 已初始化且使用 DMA 接收的 UART 设备。
 * @return 0 表示成功，-1 表示参数、模式或 HAL 操作失败。
 * @note 仅重启主控侧接收，不会向外部设备发送任何数据。
 */
int BSP_UART_Rearm_DMA(UART_Device *device);

/**
 * @brief 获取 UART DMA 接收计数和当前接收状态。
 * @param device 已初始化的 UART 设备。
 * @param diagnostics 输出诊断快照。
 * @return 0 表示成功，-1 表示参数或设备无效。
 */
int BSP_UART_GetDiagnostics(UART_Device *device, BSP_UART_Diagnostics_t *diagnostics);

/**
 * @brief UART发送（阻塞至DMA完成或超时）
 * @param device UART设备
 * @param data   发送数据指针（DMA模式下需保持有效至发送完成）
 * @param len    数据长度
 * @param timeout 超时时间
 * @return >0：实际发送字节数，-1：失败
 */
int BSP_UART_Send(UART_Device *device, uint8_t *data, uint32_t len, uint32_t timeout);

/**
 * @brief UART接收
 * @param device   UART设备
 * @param buf      接收数据缓冲区
 * @param buf_size 接收缓冲区容量 (字节), 实际读取不会超过此值
 * @param rx_len   输出实际接收数据长度 (字节)
 * @param timeout  超时时间
 * @return >0：实际接收字节数，-1：失败/超时
 */
int BSP_UART_Read(UART_Device *device, uint8_t *buf, uint32_t buf_size, uint32_t *rx_len, uint32_t timeout);

#endif // _BSP_UART_H_
