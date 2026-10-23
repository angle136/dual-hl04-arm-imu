# BSP UART 驱动文档

## 概述

BSP UART 是基于 STM32 HAL 库的串口驱动层，提供阻塞 / 中断 / DMA 三种收发模式。

## 架构设计

### 数据流（DMA 模式）

```
DMA 硬件 ──→ rx_buf (环形, 2的幂) ──→ kfifo.in 递增
    │                                      │
    │ (ISR 通过 NDTR 算增量)               │ (只更新索引指针)
    │                                      │
    └── BSP_CACHE_INVALIDATE ──→ DMB ──→ tx_event_flags_set(RX)
                                              │
                                              ▼
                                   BSP_UART_Read() 阻塞等待
                                              │  
                                   kfifo_out(buf, avail) → 用户 buf
```

### `UART_Device` 内部结构

| 字段 | 类型 | 说明 |
|------|------|------|
| `huart` | `UART_HandleTypeDef *` | HAL 库 UART 句柄 |
| `rx_fifo` | `struct kfifo` | 接收 FIFO（esize=1，buffer = rx_buf） |
| `rx_buf` | `uint8_t *` | 指向 kfifo 底层 buffer（同时也是 DMA 目标地址） |
| `buf_size` | `uint32_t` | 缓冲区大小（必须为 2 的幂） |
| `last_buf_pos` | `volatile uint32_t` | 上次 DMA 处理位置（仅 DMA 模式，用于计算增量） |
| `expected_rx_len` | `uint32_t` | 期望接收长度（0 = 不定长，使用 IDLE 检测） |
| `event_flags` | `TX_EVENT_FLAGS_GROUP` | 事件标志组（bit0: RX, bit1: TX） |
| `rx_mode` | `UART_Mode` | 接收模式 |
| `tx_mode` | `UART_Mode` | 发送模式 |
| `initialed` | `uint8_t` | 初始化完成标志 |

### 事件标志位

| 宏 | 值 | 触发者 | 等待者 |
|----|-----|--------|--------|
| `BSP_UART_EVENT_RX` | `0x01` | `HAL_UARTEx_RxEventCallback` (ISR) | `BSP_UART_Read` (任务) |
| `BSP_UART_EVENT_TX` | `0x02` | `HAL_UART_TxCpltCallback` (ISR) | `BSP_UART_Send` (任务) |


## API 接口说明

### 初始化

```c
UART_Device *BSP_UART_Device_Init(UART_Device_init_config *config);
```

```c
int BSP_UART_Send(UART_Device *device, uint8_t *data, uint32_t len, uint32_t timeout);
```

```c
int BSP_UART_Read(UART_Device *device, uint8_t *buf, uint32_t buf_size, uint32_t *rx_len, uint32_t timeout);
```


## 使用示例

### DMA 接收 + 阻塞发送

```c
#include "bsp_uart.h"
#include "bsp_def.h"

extern UART_HandleTypeDef huart1;

#define UART_BUF_SIZE 512  /* 必须是 2 的幂 */

static UART_Device      *g_uart;
static uint8_t           g_rx_buf[UART_BUF_SIZE] BUFFER_SECTION;
static TX_EVENT_FLAGS_GROUP g_evt; /* 可复用 BSP 内部 event_flags */

void uart_demo_init(void)
{
    UART_Device_init_config cfg = {
        .huart           = &huart1,
        .rx_buf          = g_rx_buf,
        .rx_buf_size     = UART_BUF_SIZE,
        .expected_rx_len = 0,              /* 不定长 */
        .rx_mode         = UART_MODE_DMA,
        .tx_mode         = UART_MODE_BLOCKING,
    };

    g_uart = BSP_UART_Device_Init(&cfg);
    if (g_uart == NULL)
    {
        /* 错误处理 */
    }
}

void uart_recv_task(ULONG unused)
{
    uint8_t  buf[256];
    uint32_t rx_len;

    while (1)
    {
        int ret = BSP_UART_Read(g_uart, buf, sizeof(buf), &rx_len, TX_WAIT_FOREVER);
        if (ret > 0)
        {
            /* 处理 buf[0..rx_len-1] */
            process_data(buf, rx_len);
        }
    }
}
```

### IT 模式收发

```c
UART_Device_init_config cfg = {
    .huart       = &huart2,
    .rx_buf      = g_rx_buf,
    .rx_buf_size = 256,
    .rx_mode     = UART_MODE_IT,
    .tx_mode     = UART_MODE_IT,
};

g_uart = BSP_UART_Device_Init(&cfg);

/* 发送 */
BSP_UART_Send(g_uart, tx_data, tx_len, TX_WAIT_FOREVER);

/* 接收 */
uint8_t  buf[128];
uint32_t len;
BSP_UART_Read(g_uart, buf, sizeof(buf), &len, TX_WAIT_FOREVER);
```

### 阻塞模式（简单轮询）

```c
cfg.rx_mode = UART_MODE_BLOCKING;
cfg.tx_mode = UART_MODE_BLOCKING;

g_uart = BSP_UART_Device_Init(&cfg);

uint8_t  buf[64];
uint32_t len;
BSP_UART_Read(g_uart, buf, sizeof(buf), &len, 100); /* 100ms 超时 */
```

## 内部实现细节

### 零拷贝 kfifo

`rx_buf` 直接作为 kfifo 的底层 buffer：

```
kfifo_init(&dev->rx_fifo, config->rx_buf, config->rx_buf_size, 1);
dev->rx_buf = config->rx_buf;
```

ISR 收到数据后不拷贝，仅递增 `fifo.in`：

```c
/* DMA 模式 */
dev->rx_fifo.in += new_len;  // 零拷贝更新写索引

/* IT 模式 */
dev->rx_fifo.in += Size;     // 零拷贝更新写索引
```

任务侧通过 `kfifo_out` 读出数据（消费端拷贝），但 ISR→kfifo 路径完全零拷贝。

### kfifo 溢出保护

ISR 写入 kfifo 前检查组 fifo 容量，避免 `in` 超过 `out + mask + 1` 导致数据被覆盖：

```c
unsigned int fifo_cap = dev->rx_fifo.mask + 1;
unsigned int max_in   = dev->rx_fifo.out + fifo_cap;
unsigned int space    = (dev->rx_fifo.in < max_in) ? (max_in - dev->rx_fifo.in) : 0;
if (new_len > space) new_len = space;
if (new_len == 0) return;

dev->rx_fifo.in += new_len;
```

用户侧同样受 `buf_size` 参数保护，`BSP_UART_Read` 每次读取不会超过用户缓冲区容量。

### DMA 增量计算

DMA 模式下通过 NDTR 寄存器追踪环形写入位置：

```c
curr_pos = buf_size - __HAL_DMA_GET_COUNTER(huart->hdmarx); // DMA 当前写入位置
new_len  = curr_pos >= last_pos ? curr_pos - last_pos        // 线性
                                : buf_size - last_pos + curr_pos; // 回绕
```

### H7 Cache 一致性

- **TX DMA 前**：`BSP_CACHE_CLEAN(data, len)` — 将 CPU 写入刷到内存，确保 DMA 读到最新数据。
- **RX DMA 后**：`BSP_CACHE_INVALIDATE(rx_buf, buf_size)` — 无效化 CPU Cache，确保读到 DMA 最新写入。
- F4 平台上两个宏为空操作（无 D-Cache）。

## 注意事项

1. **rx_buf_size 必须为 2 的幂** — kfifo 依赖 `mask = size - 1` 位运算定位，非 2 的幂会导致 kfifo 初始化失败。
2. **rx_buf 必须在 DMA 可访问区域** — H7 上禁止使用 DTCM (`0x20000000`) 和 ITCM (`0x00000000`)。推荐使用 `BUFFER_SECTION` 宏将 buffer 放在 `.RAM_D1` 段。
3. **DMA 模式下 rx_buf 建议 32 字节对齐** — 虽然 cache 操作宏内部做了对齐，但 32 字节对齐可避免 Cache Line 共享问题（与相邻变量互相干扰）。
4. **BLOCKING 模式不使用 kfifo** — 阻塞模式直接调用 HAL 函数，不会设置 RX 事件标志，也不分配 kfifo 内部资源。
5. **DMA 模式持续运行** — `HAL_UARTEx_ReceiveToIdle_DMA` 配置为环形 DMA，启动后持续接收，出错后通过 `HAL_UART_ErrorCallback` → `HAL_UART_Abort_IT` → `HAL_UART_AbortCpltCallback` 自动重启。
6. **TX 数据生命周期** — DMA / IT 发送模式下，`data` 指针指向的数据必须在发送完成（`BSP_UART_Send` 返回）之前保持有效。


## 版本历史

| 版本 | 日期 | 说明 |
|------|------|------|
| v1.0 | 2026-05-05 | 初始版本：阻塞/IT/DMA 三模式，零拷贝 kfifo，事件标志组同步 |
| v1.1 | 2026-05-23 | `BSP_UART_Read` 新增 `buf_size` 参数防止用户缓冲区溢出；DMA ISR 增加 kfifo 容量保护 |
