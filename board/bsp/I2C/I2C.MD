# BSP I2C 驱动文档

## 概述

BSP I2C 是基于 STM32 HAL 库的 I2C 驱动层，提供阻塞 / 中断 / DMA 三种传输模式。

## 架构设计

### 总线与设备模型

```
i2c_bus_table[I2C_BUS_NUM]
    │
    ├── I2C_Bus
    │   ├── hi2c          ─── HAL 句柄（一条物理 I2C 总线）
    │   ├── bus_mutex     ─── 总线互斥锁（多设备共享）
    │   ├── event_flags   ─── 事件标志组（IT/DMA 完成通知）
    │   └── devices_list  ─── 设备链表头
    │         ├── I2C_Device (dev0)
    │         │   ├── dev_address       ─── 从设备地址（左移1位）
    │         │   ├── tx_mode / rx_mode ─── 发送/接收模式
    │         │   └── node             ─── 链表节点
    │         ├── I2C_Device (dev1)
    │         └── ...
    │
    └── I2C_Bus (另一条物理总线)
        └── ...
```

### 传输流程（IT / DMA 模式）

```
任务调用 BSP_I2C_Transmit() / BSP_I2C_Receive() / BSP_I2C_Mem_Write() / BSP_I2C_Mem_Read()
    │
    ├── transfer_begin()
    │   ├── find_bus(hi2c)                    ── 查找总线管理器
    │   ├── tx_mutex_get()                    ── 获取总线锁（仅任务上下文）
    │   ├── HAL_I2C_GetState() == READY       ── 检查外设状态
    │   └── 清除残留事件标志                   ── TX_NO_WAIT 清理 TX|RX
    │
    ├── transfer_once()
    │   ├── 非线程上下文 → 强制 BLOCKING
    │   ├── BLOCKING: HAL_I2C_Master_Transmit/Receive / HAL_I2C_Mem_Write/Read
    │   ├── IT:      HAL_I2C_xxx_IT  →  tx_event_flags_get() 等待事件
    │   └── DMA:     HAL_I2C_xxx_DMA →  BSP_CACHE_CLEAN(tx)  →  tx_event_flags_get()
    │                                →  BSP_CACHE_INVALIDATE(rx)
    │
    └── transfer_end()
        └── tx_mutex_put()                     ── 释放总线锁

ISR 路径:
    HAL_I2C_MasterTxCpltCallback ──→ tx_event_flags_set(TX)
    HAL_I2C_MasterRxCpltCallback ──→ tx_event_flags_set(RX)
    HAL_I2C_MemTxCpltCallback    ──→ tx_event_flags_set(TX)
    HAL_I2C_MemRxCpltCallback    ──→ tx_event_flags_set(RX)
    HAL_I2C_ErrorCallback        ──→ abort + 状态重置（不设事件标志，任务超时感知）
```

### `I2C_Bus` 管理器

| 字段 | 类型 | 说明 |
|------|------|------|
| `hi2c` | `I2C_HandleTypeDef *` | HAL 库 I2C 句柄 |
| `devices_list` | `I2C_Device *` | 设备链表头（挂载该总线上的所有 `I2C_Device`） |
| `event_flags` | `TX_EVENT_FLAGS_GROUP` | 事件标志组（bit0: TX, bit1: RX） |
| `bus_mutex` | `TX_MUTEX` | 总线互斥锁（多设备、多任务共享同一物理总线时互斥） |
| `initialized` | `uint8_t` | 总线初始化完成标志 |

### `I2C_Device` 设备实例

| 字段 | 类型 | 说明 |
|------|------|------|
| `next` | `I2C_Device *` | 指向同一总线下一个设备的指针，NULL 表尾 |
| `hi2c` | `I2C_HandleTypeDef *` | 指向所属总线的 HAL 句柄 |
| `dev_address` | `uint16_t` | 从设备地址（STM32 中需左移 1 位，如 7-bit 地址 `0x0E` → 写入 `0x1C`） |
| `tx_mode` | `I2C_Mode` | 发送模式 |
| `rx_mode` | `I2C_Mode` | 接收模式 |

### 事件标志位

| 宏 | 值 | 触发者 | 等待者 |
|----|-----|--------|--------|
| `BSP_I2C_EVENT_TX` | `0x01` | `HAL_I2C_MasterTxCpltCallback` / `HAL_I2C_MemTxCpltCallback` (ISR) | `transfer_once` (任务，IT/DMA 模式) |
| `BSP_I2C_EVENT_RX` | `0x02` | `HAL_I2C_MasterRxCpltCallback` / `HAL_I2C_MemRxCpltCallback` (ISR) | `transfer_once` (任务，IT/DMA 模式) |

## API 接口说明

### 初始化

```c
I2C_Device *BSP_I2C_Device_Init(I2C_Device_Init_Config *config);
```

```c
uint8_t BSP_I2C_Transmit(I2C_Device *dev, const uint8_t *tx_data, uint16_t size, uint32_t timeout);
```

```c
uint8_t BSP_I2C_Receive(I2C_Device *dev, uint8_t *rx_data, uint16_t size, uint32_t timeout);
```

```c
uint8_t BSP_I2C_Mem_Write(I2C_Device *dev, uint16_t mem_address, uint16_t mem_add_size, const uint8_t *data, uint16_t size, uint32_t timeout);
```

```c
uint8_t BSP_I2C_Mem_Read(I2C_Device *dev, uint16_t mem_address, uint16_t mem_add_size, uint8_t *data, uint16_t size, uint32_t timeout);
```

## 使用示例

### 阻塞模式（简单轮询）

```c
#include "bsp_i2c.h"

extern I2C_HandleTypeDef hi2c3;

static I2C_Device *g_i2c_dev;

void i2c_blocking_demo_init(void)
{
    I2C_Device_Init_Config cfg = {
        .hi2c        = &hi2c3,
        .dev_address = 0x0E << 1,  /* 7-bit addr 0x0E → 左移1位 */
        .tx_mode     = I2C_MODE_BLOCKING,
        .rx_mode     = I2C_MODE_BLOCKING,
    };

    g_i2c_dev = BSP_I2C_Device_Init(&cfg);
    if (g_i2c_dev == NULL)
    {
        /* 错误处理 */
    }
}

void i2c_read_register(uint8_t reg, uint8_t *val)
{
    uint8_t ret = BSP_I2C_Mem_Read(g_i2c_dev, reg, I2C_MEMADD_SIZE_8BIT, val, 1, 100);
    if (ret != 0)
    {
        /* 超时或错误处理 */
    }
}
```

### DMA 模式收发

```c
#include "bsp_i2c.h"
#include "bsp_def.h"

extern I2C_HandleTypeDef hi2c1;

static I2C_Device *g_i2c_dma;
static uint8_t     g_tx_buf[64] BUFFER_SECTION;
static uint8_t     g_rx_buf[64] BUFFER_SECTION;

void i2c_dma_demo_init(void)
{
    I2C_Device_Init_Config cfg = {
        .hi2c        = &hi2c1,
        .dev_address = 0x50 << 1,  /* EEPROM 地址 */
        .tx_mode     = I2C_MODE_DMA,
        .rx_mode     = I2C_MODE_DMA,
    };

    g_i2c_dma = BSP_I2C_Device_Init(&cfg);
}

void i2c_dma_task(ULONG unused)
{
    while (1)
    {
        /* 从寄存器 0x00 开始读取 64 字节 */
        uint8_t ret = BSP_I2C_Mem_Read(g_i2c_dma, 0x00, I2C_MEMADD_SIZE_8BIT, g_rx_buf, 64, TX_WAIT_FOREVER);
        if (ret == 0)
        {
            process_data(g_rx_buf, 64);
        }

        tx_thread_sleep(100);
    }
}
```

## 内部实现细节

### 总线互斥锁

同一物理 I2C 总线上可能挂载多个设备，互斥锁确保同一时间只有一个设备占用总线。锁仅在任务上下文（`tx_thread_identify() != NULL`）中获取和释放：

- **任务调用**：`transfer_begin()` 调用 `tx_mutex_get()` 获取锁，`transfer_end()` 调用 `tx_mutex_put()` 释放锁。
- **非任务上下文**（中断 / 初始化）：跳过锁操作，直接访问总线。

### 非线程上下文强制 BLOCKING

`transfer_once()` 在 `tx_thread_identify() == NULL` 时（如系统初始化阶段）强制使用 BLOCKING 模式，忽略设备配置的 IT/DMA 模式。这确保了在 ThreadX 调度器尚未就绪或中断上下文中的调用能够同步完成。


### 设备链表

`I2C_Bus.devices` 使用单向链表串联同一总线上所有 `I2C_Device`，新设备头插入。`I2C_Device.next` 指向下一个设备，NULL 表尾。

## 注意事项

- **dev_address 必须左移 1 位** — STM32 HAL 库要求传入的地址已经是 8-bit 格式（7-bit 地址 << 1）。例如设备数据手册标注地址为 `0x0E`，应传入 `0x1C`。
- **DMA 缓冲区必须在 DMA 可访问区域** — H7 上禁止使用 DTCM (`0x20000000`) 和 ITCM (`0x00000000`)。推荐使用 `BUFFER_SECTION` 宏将 buffer 放在 `.RAM_D1` 段。
- **Mem_Write / Mem_Read 的 mem_add_size** — 必须与从设备的寄存器地址宽度匹配：8-bit 地址用 `I2C_MEMADD_SIZE_8BIT`，16-bit 地址用 `I2C_MEMADD_SIZE_16BIT`。

## 版本历史

| 版本 | 日期 | 说明 |
|------|------|------|
| v1.0 | 2026-05-05 | 初始版本 |
| v1.1 | 2026-05-21 | 改用单向链表，方便调试,合并重复函数 |
