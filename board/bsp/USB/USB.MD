# BSP USB CDC ACM 驱动文档

## 概述

基于 CherryUSB 协议栈的 CDC ACM（虚拟串口）驱动层

## 架构设计

### 数据流

```
USB 硬件 OUT 端点 ──→ cdc_read_buffer (环形, 1024B) ──→ kfifo.in 递增
         │                                                    │
         │ (OUT 端点回调)                                     │ (只更新索引指针)
         │                                                    │
         └── 4 字节对齐处理 ──→ event_flags_set(RX) ──────────┘
                                               │
                                               ▼
                                    cdc_acm_recv() 阻塞等待
                                               │  
                                    kfifo_out(buf, to_read) → 用户 buf
```

### 数据写入路径

```
cdc_acm_send() ──→ memcpy(cdc_write_buffer) ──→ usbd_ep_start_write(IN)
                           │
                           │ (等待上次 TX 完成事件)
                           │
                           └── ZLP 处理 (整除包长时自动发 0 长度包)
```

### 内部结构

| 字段 / 资源 | 类型 | 说明 |
|-------------|------|------|
| `cdc_rx_fifo` | `struct kfifo` | 接收 FIFO，底层 buffer 为 `cdc_read_buffer` |
| `cdc_read_buffer` | `uint8_t[1024]` | 接收环形缓冲区（USB_NOCACHE_RAM_SECTION） |
| `cdc_write_buffer` | `uint8_t[CDC_MAX_MPS]` | 发送缓冲区，单次最多一个 MPS 包 |
| `usb_event_flags` | `TX_EVENT_FLAGS_GROUP` | 事件标志组（bit0: RX, bit1: TX） |
| `CDC_MAX_MPS` | `512` (HS) / `64` (FS) | 批量传输最大包长 |

### 事件标志位

| 宏 | 值 | 触发者 | 等待者 |
|----|-----|--------|--------|
| `BSP_USB_EVENT_RX` | `0x01` | `usbd_cdc_acm_bulk_out` (端点回调) | `cdc_acm_recv` (任务) |
| `BSP_USB_EVENT_TX` | `0x02` | `usbd_cdc_acm_bulk_in` (端点回调) | `cdc_acm_send` (任务) |

## API 接口说明

```c
void cdc_acm_init(uint8_t busid, uintptr_t reg_base);
```

```c
int cdc_acm_send(const uint8_t *data, uint32_t len, uint32_t timeout);
```

```c
int cdc_acm_recv(uint8_t *data, uint32_t buf_size, uint32_t *rx_len, uint32_t timeout);
```


## 使用示例

```c
#include "usbd_cdc_acm_user.h"

void usb_demo_init(void)
{
    /* 初始化 CDC ACM，使用 USB1 (busid=0) */
    cdc_acm_init(0, 0x40040000); /* USB1 OTG HS */
}

void usb_send_task(ULONG unused)
{
    uint8_t payload[] = "Hello USB\r\n";

    while (1)
    {
        int ret = cdc_acm_send(payload, sizeof(payload), 100);
        if (ret > 0)
        {
            /* 发送成功 */
        }
        tx_thread_sleep(TX_TIMER_TICKS_PER_SECOND);
    }
}

void usb_recv_task(ULONG unused)
{
    uint8_t  buf[256];
    uint32_t rx_len;

    while (1)
    {
        int ret = cdc_acm_recv(buf, sizeof(buf), &rx_len, TX_WAIT_FOREVER);
        if (ret > 0)
        {
            /* 处理 buf[0..rx_len-1] */
        }
    }
}
```

## 内部实现细节

### 零拷贝 kfifo

`cdc_read_buffer`（USB_NOCACHE_RAM_SECTION，放在 Cache 不缓存区域）直接作为 kfifo 的底层 buffer：

```c
USB_NOCACHE_RAM_SECTION USB_MEM_ALIGNX uint8_t cdc_read_buffer[CDC_RX_FIFO_SIZE];
kfifo_init(&cdc_rx_fifo, cdc_read_buffer, CDC_RX_FIFO_SIZE, 1);
```

OUT 端点回调收到数据后不拷贝，仅递增 `cdc_rx_fifo.in`：

```c
cdc_rx_fifo.in += nbytes;
```

### kfifo 溢出保护

端点回调写入前检查 kfifo 容量，避免 `in` 超过 `out + mask + 1` 导致环形缓冲区数据覆盖：

```c
unsigned int fifo_cap = cdc_rx_fifo.mask + 1;
unsigned int max_in   = cdc_rx_fifo.out + fifo_cap;
unsigned int space    = (cdc_rx_fifo.in < max_in) ? (max_in - cdc_rx_fifo.in) : 0;
if (nbytes > space) nbytes = space;
if (nbytes == 0) return;
```

用户侧同样受 `buf_size` 参数保护，`cdc_acm_recv` 每次读取不会超过用户缓冲区容量。

### DWC2 4 字节对齐处理

DWC2 USB 控制器要求 DMA 目标地址 4 字节对齐。kfifo 推进后若 `pos` 非 4 字节对齐，跳过补齐字节：

```c
unsigned int align_gap = (4 - (pos & 0x3)) & 0x3;
if (align_gap)
{
    cdc_rx_fifo.in += align_gap;
    pos = cdc_rx_fifo.in & cdc_rx_fifo.mask;
}
```

之后检查尾部剩余连续空间是否足够容纳一个 MPS 包，不足则回绕到缓冲区起始：

```c
unsigned int tail = (cdc_rx_fifo.mask + 1) - pos;
if (tail < CDC_MAX_MPS)
{
    cdc_rx_fifo.in += tail;
    pos = 0;
}
```

### ZLP 自动处理

IN 端点回调中，若发送字节数为端点 MPS 的整数倍且非零，自动追加一个零长度包（ZLP）标志传输结束：

```c
if ((nbytes % usbd_get_ep_mps(busid, ep)) == 0 && nbytes)
{
    usbd_ep_start_write(busid, CDC_IN_EP, NULL, 0); // ZLP
}
else
{
    tx_event_flags_set(&usb_event_flags, BSP_USB_EVENT_TX, TX_OR);
}
```

### 缓冲区位域

`cdc_read_buffer` 和 `cdc_write_buffer` 通过 `USB_NOCACHE_RAM_SECTION` 放置在非 Cache 区域：

- H7 (`STM32H723xx`): `.RAM_D2` 段
- F4 (`STM32F407xx`): `.ram` 段


## 版本历史

| 版本 | 日期 | 说明 |
|------|------|------|
| v1.0 | 2026-05-09 | 初始版本 |
| v1.1 | 2026-05-23 | `cdc_acm_recv` 新增 `buf_size` 参数防止用户缓冲区溢出；OUT 端点回调增加 kfifo 容量保护 |
