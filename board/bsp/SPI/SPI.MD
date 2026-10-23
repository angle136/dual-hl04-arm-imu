# BSP SPI 驱动文档

## 概述

BSP SPI 是基于 STM32 HAL 库的 SPI 驱动层，提供阻塞 / 中断 / DMA 三种传输模式。

## 架构设计

### 总线与设备模型

```
spi_bus_table[SPI_BUS_NUM]
    │
    ├── SPI_Bus
    │   ├── hspi          ─── HAL 句柄（一条物理 SPI 总线）
    │   ├── bus_mutex     ─── 总线互斥锁（多设备共享）
    │   ├── event_flags   ─── 事件标志组（IT/DMA 完成通知）
    │   └── devices       ─── 设备链表头
    │         ├── SPI_Device (dev0)
    │         │   ├── cs_port / cs_pin  ─── 片选引脚
    │         │   ├── tx_mode / rx_mode ─── 发送/接收模式
    │         │   └── node              ─── 链表节点
    │         ├── SPI_Device (dev1)
    │         └── ...
    │
    └── SPI_Bus (另一条物理总线)
        └── ...
```

### 传输流程（IT / DMA 模式）

```
任务调用 BSP_SPI_Transmit() / BSP_SPI_Receive() / BSP_SPI_TransReceive()
    │
    ├── transfer_begin()
    │   ├── find_bus(hspi)                    ── 查找总线管理器
    │   ├── tx_mutex_get()                    ── 获取总线锁（仅任务上下文）
    │   ├── HAL_SPI_GetState() == READY       ── 检查外设状态
    │   └── spi_cs_low(dev)                   ── 拉低片选
    │
    ├── transfer_once()
    │   ├── BLOCKING: HAL_SPI_Transmit/Receive/TransmitReceive
    │   ├── IT:      HAL_SPI_xxx_IT  →  tx_event_flags_get() 等待事件
    │   └── DMA:     HAL_SPI_xxx_DMA →  BSP_CACHE_CLEAN(tx)  →  tx_event_flags_get()
    │                                →  BSP_CACHE_INVALIDATE(rx)
    │
    └── transfer_end()
        ├── spi_cs_high(dev)                   ── 拉高片选
        └── tx_mutex_put()                     ── 释放总线锁

ISR 路径:
    HAL_SPI_TxCpltCallback  ──→ tx_event_flags_set(TX)
    HAL_SPI_RxCpltCallback  ──→ tx_event_flags_set(RX)
    HAL_SPI_TxRxCpltCallback ──→ tx_event_flags_set(TXRX)
```

### `SPI_Bus` 管理器

| 字段 | 类型 | 说明 |
|------|------|------|
| `hspi` | `SPI_HandleTypeDef *` | HAL 库 SPI 句柄 |
| `devices_list` | `SPI_Device *` | 设备链表头（挂载该总线上的所有 `SPI_Device`） |
| `event_flags` | `TX_EVENT_FLAGS_GROUP` | 事件标志组（bit0: TX, bit1: RX, bit2: TXRX） |
| `bus_mutex` | `TX_MUTEX` | 总线互斥锁（多设备、多任务共享同一物理总线时互斥） |
| `initialized` | `uint8_t` | 总线初始化完成标志 |

### `SPI_Device` 设备实例

| 字段 | 类型 | 说明 |
|------|------|------|
| `next` | `SPI_Device *` | 指向同一总线下一个设备的指针，NULL 表尾 |
| `hspi` | `SPI_HandleTypeDef *` | 指向所属总线的 HAL 句柄 |
| `cs_port` | `GPIO_TypeDef *` | 片选 GPIO 端口，NULL = 无硬件 CS |
| `cs_pin` | `uint16_t` | 片选引脚号，0 = 无硬件 CS |
| `tx_mode` | `SPI_Mode` | 发送模式 |
| `rx_mode` | `SPI_Mode` | 接收模式 |

### 事件标志位

| 宏 | 值 | 触发者 | 等待者 |
|----|-----|--------|--------|
| `BSP_SPI_EVENT_TX` | `0x01` | `HAL_SPI_TxCpltCallback` (ISR) | `transfer_once` (任务，IT/DMA 模式) |
| `BSP_SPI_EVENT_RX` | `0x02` | `HAL_SPI_RxCpltCallback` (ISR) | `transfer_once` (任务，IT/DMA 模式) |
| `BSP_SPI_EVENT_TXRX` | `0x04` | `HAL_SPI_TxRxCpltCallback` (ISR) | `transfer_once` (任务，IT/DMA 模式) |

## API 接口说明

```c
SPI_Device *BSP_SPI_Device_Init(SPI_Device_Init_Config *config);
```
```c
uint8_t BSP_SPI_Transmit(SPI_Device *dev, const uint8_t *tx_data, uint16_t size, uint32_t timeout);
```

```c
uint8_t BSP_SPI_Receive(SPI_Device *dev, uint8_t *rx_data, uint16_t size, uint32_t timeout);
```

```c
uint8_t BSP_SPI_TransReceive(SPI_Device *dev, const uint8_t *tx_data, uint8_t *rx_data, uint16_t size, uint32_t timeout);
```

```c
uint8_t BSP_SPI_TransAndTrans(SPI_Device *dev, const uint8_t *tx_data1, uint16_t size1, const uint8_t *tx_data2, uint16_t size2, uint32_t timeout);
```

```c
uint8_t BSP_SPI_ReceAndRece(SPI_Device *dev, uint8_t *rx_data1, uint16_t size1, uint8_t *rx_data2, uint16_t size2, uint32_t timeout);
```


## 使用示例

### 阻塞模式（简单轮询）

```c
#include "bsp_spi.h"
#include "bsp_def.h"

extern SPI_HandleTypeDef hspi1;

static SPI_Device *g_spi_dev;

void spi_blocking_demo_init(void)
{
    SPI_Device_Init_Config cfg = {
        .hspi    = &hspi1,
        .cs_port = GPIOA,
        .cs_pin  = GPIO_PIN_4,
        .tx_mode = SPI_MODE_BLOCKING,
        .rx_mode = SPI_MODE_BLOCKING,
    };

    g_spi_dev = BSP_SPI_Device_Init(&cfg);
    if (g_spi_dev == NULL)
    {
        /* 错误处理 */
    }
}

void spi_read_register(uint8_t reg_addr, uint8_t *val)
{
    /* 发送寄存器地址，然后读回一个字节 */
    uint8_t ret = BSP_SPI_TransAndTrans(g_spi_dev, &reg_addr, 1, NULL, 0, 100);
    if (ret != 0) return;

    ret = BSP_SPI_Receive(g_spi_dev, val, 1, 100);
}
```

### DMA 模式收发

```c
#include "bsp_spi.h"
#include "bsp_def.h"

extern SPI_HandleTypeDef hspi2;

static SPI_Device *g_spi_dma;
static uint8_t     g_tx_buf[256] BUFFER_SECTION;
static uint8_t     g_rx_buf[256] BUFFER_SECTION;

void spi_dma_demo_init(void)
{
    SPI_Device_Init_Config cfg = {
        .hspi    = &hspi2,
        .cs_port = GPIOB,
        .cs_pin  = GPIO_PIN_6,
        .tx_mode = SPI_MODE_DMA,
        .rx_mode = SPI_MODE_DMA,
    };

    g_spi_dma = BSP_SPI_Device_Init(&cfg);
}

void spi_dma_task(ULONG unused)
{
    while (1)
    {
        /* 准备发送数据 */
        memset(g_tx_buf, 0xAA, sizeof(g_tx_buf));

        /* 全双工 DMA 收发 */
        uint8_t ret = BSP_SPI_TransReceive(g_spi_dma, g_tx_buf, g_rx_buf, sizeof(g_tx_buf), TX_WAIT_FOREVER);
        if (ret == 0)
        {
            /* 处理 g_rx_buf 中的数据 */
            process_data(g_rx_buf, sizeof(g_rx_buf));
        }

        tx_thread_sleep(100);
    }
}
```

### IT 模式收发

```c
SPI_Device_Init_Config cfg = {
    .hspi    = &hspi1,
    .cs_port = GPIOA,
    .cs_pin  = GPIO_PIN_4,
    .tx_mode = SPI_MODE_IT,
    .rx_mode = SPI_MODE_IT,
};

g_spi_dev = BSP_SPI_Device_Init(&cfg);

/* 命令+数据连续发送（同一个 CS 周期） */
uint8_t cmd  = 0x90;
uint8_t data = 0x55;
BSP_SPI_TransAndTrans(g_spi_dev, &cmd, 1, &data, 1, TX_WAIT_FOREVER);

/* 全双工收发 */
uint8_t tx[8] = {1, 2, 3, 4, 5, 6, 7, 8};
uint8_t rx[8];
BSP_SPI_TransReceive(g_spi_dev, tx, rx, 8, TX_WAIT_FOREVER);
```

## 内部实现细节

### 总线互斥锁

同一物理 SPI 总线上可能挂载多个设备，互斥锁确保同一时间只有一个设备占用总线。锁仅在任务上下文（`tx_thread_identify() != NULL`）中获取和释放：

- **任务调用**：`transfer_begin()` 调用 `tx_mutex_get()` 获取锁，`transfer_end()` 调用 `tx_mutex_put()` 释放锁。
- **非任务上下文**（中断 / 初始化）：跳过锁操作，直接访问总线。

### 非线程上下文强制 BLOCKING

`transfer_once()` 在 `tx_thread_identify() == NULL` 时（如系统初始化阶段）强制使用 BLOCKING 模式，忽略设备配置的 IT/DMA 模式。这确保了在 ThreadX 调度器尚未就绪或中断上下文中的调用能够同步完成。

### 设备链表

`SPI_Bus.devices` 使用单向链表串联同一总线上所有 `SPI_Device`，新设备头插入。`SPI_Device.next` 指向下一个设备，NULL 表尾。

## 注意事项

- **DMA 缓冲区必须在 DMA 可访问区域** — H7 上禁止使用 DTCM (`0x20000000`) 和 ITCM (`0x00000000`)。
- **DMA 模式下缓冲区建议 32 字节对齐** — 使用 `BUFFER_SECTION` 宏自动对齐，避免 Cache Line 共享问题。
- **TransReceive 的 size** — 全双工收发时发送和接收使用相同的 `size`，这是 SPI 协议的特性决定的。
- **TransAndTrans / ReceAndRece** — 两次传输在同一 CS 周期内完成。如果第一次传输失败，第二次不会执行。
- **总线互斥锁** — 仅在任务上下文中生效。中断上下文调用时不会尝试获取锁。

## 版本历史

| 版本 | 日期 | 说明 |
|------|------|------|
| v1.0 | 2026-05-05 | 初始版本 |
| v1.1 | 2026-05-21 | 改用单向链表，方便调试,合并重复函数 |
