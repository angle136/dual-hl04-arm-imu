# REMOTE 遥控/图传模块文档

## 概述

REMOTE 模块统一封装**遥控器**（SBUS / DT7）和**图传键鼠**（VT02 / VT03）。

## 架构设计

### 数据流

```
┌──────────────┐    ┌──────────────┐
│ REMOTE_UART  │    │REMOTE_VT_UART│
│ (huart5/3)   │    │ (huart1/6)   │
└──────┬───────┘    └──────┬───────┘
       │ DMA               │ DMA
       │ (IDLE 中断)       │ (IDLE 中断)
       ▼                   ▼
  BSP_UART_Read()    BSP_UART_Read()
       │                   │
       ▼                   ▼
  sbus_decode()       vt02_decode()
      or                    or
  dt7_decode()        vt03_decode()
       │                    │
       │  直接写入          │  直接写入
       ▼                    ▼
  ┌──────────────────────────────────┐
  │         g_remote_data            │
  │  Remote_Data_t (唯一全局实例)    │
  │  channels[16] / wheel / mouse …  │
  └──────────────────────────────────┘
       ▲
       │ const Remote_Data_t *
       │
  ┌────┴─────────────────────────────┐
  │         上层应用                 │
  │  Module_Remote_get_channel(1..16)│
  │  Module_Remote_get_mouse()       │
  │  Module_Remote_get_keyboard()    │
  │  Module_Remote_get_wheel()       │
  └──────────────────────────────────┘
```

### 子模块封装模式

各子模块对外仅暴露 `_init` / `_decode` 两个函数：

```
          sbus.h
   ┌─────────────────┐
   │ sbus_init()     │  ← 配置 UART / 注册 Offline
   │ sbus_decode()   │  ← for 循环扫描, 直接写入 Remote_Data_t
   └─────────────────┘
          sbus.c
   ┌──────────────────────────────┐
   │ static UART_Device *uart;    │
   │ static uint8_t rx_buf[64];   │  DMA 接收缓冲区
   │ static bool initialized;     │
   │ static sbus_decode();        │
   └──────────────────────────────┘
```

### `Remote_Data_t` 统一结构体

| 字段 | 类型 | 说明 |
|------|------|------|
| `channels[0..3]` | `int16_t` | 摇杆通道，零偏后 ±660 范围 (SBUS/DT7/VT03) |
| `channels[4..15]` | `int16_t` | 扩展通道，原始值 0..2047 (仅 SBUS) |
| `wheel` | `int16_t` | 滚轮值 (DT7 / VT03) |
| `sw1` | `uint8_t` | 拨杆1 (DT7: 1=UP, 2=DOWN, 3=MID) |
| `sw2` | `uint8_t` | 拨杆2 (DT7) |
| `switch_pos` | `uint8_t` | 拨杆位置 (VT03) |
| `mouse` | `mouse_state_t` | 鼠标状态 (DT7 / VT02 / VT03) |
| `keyboard` | `keyboard_state_t` | 键盘状态 (DT7 / VT02 / VT03) |

无效字段保持为 0。

### 各子模块参数

| 子模块 | 波特率 | 帧长 | rx_buf | offline 超时 | 校验 |
|--------|--------|------|--------|:---:|------|
| SBUS | 100000 | 25 字节 | 64 B | 50 ms | 帧头 0x0F + 帧尾 0x00 |
| DT7 | 100000 | 18 字节 | 64 B | 50 ms | 通道边界检查 (364..1684) |
| VT02 | 115200 | 不定长 | 128 B | 100 ms | CRC8 (帧头 5 字节) |
| VT03 | 921600 | 21 字节 | 64 B | 100 ms | 帧头 0xA9 0x53 + CRC16 |

## API 接口说明

```c
void Module_Remote_init(void);
```

```c
const Remote_Data_t *Module_Remote_get_data(void);
```

```c
int16_t Module_Remote_get_channel(uint8_t ch);
```

```c
const mouse_state_t *Module_Remote_get_mouse(void);
```

```c
const keyboard_state_t *Module_Remote_get_keyboard(void);
```

```c
int16_t Module_Remote_get_wheel(void);
```

## 使用示例

### SBUS + VT02 方案

```c
#include "module_remote.h"

/* 修改 module_remote.h 中编译期配置:
 *   #define REMOTE_SOURCE     1   // sbus
 *   #define REMOTE_VT_SOURCE  1   // vt02
 */

void app_init(void)
{
    Module_Offline_init();   /* 先初始化离线检测 */
    Module_Remote_init();    /* 再初始化遥控/图传 */
}

void chassis_task(ULONG unused)
{
    while (1)
    {
        /* 读取摇杆通道 (零偏后) */
        int16_t ch1 = Module_Remote_get_channel(1);
        int16_t ch2 = Module_Remote_get_channel(2);
        int16_t ch3 = Module_Remote_get_channel(3);
        int16_t ch4 = Module_Remote_get_channel(4);

        chassis_control(ch1, ch2, ch3, ch4);
        tx_thread_sleep(10);
    }
}

void gimbal_task(ULONG unused)
{
    while (1)
    {
        const mouse_state_t *mouse = Module_Remote_get_mouse();
        if (mouse)
        {
            gimbal_update(mouse->mouse_x, mouse->mouse_y, mouse->mouse_z);
        }

        /* 检查键盘按键 */
        const keyboard_state_t *kb = Module_Remote_get_keyboard();
        if (kb && kb->bit.KEY_Q)
        {
            trigger_action();
        }

        tx_thread_sleep(5);
    }
}
```

### 直接获取整个数据结构

```c
void debug_task(ULONG unused)
{
    while (1)
    {
        const Remote_Data_t *data = Module_Remote_get_data();
        if (data)
        {
            LOG_I("ch1=%d ch2=%d ch3=%d ch4=%d wheel=%d",
                  data->channels[0], data->channels[1],
                  data->channels[2], data->channels[3],
                  data->wheel);
        }
        tx_thread_sleep(100);
    }
}
```

## 内部实现细节

### 摇杆通道零偏与死区

SBUS / DT7 / VT03 的摇杆通道原始值范围 364..1684，中间值 1024 为零位。解码后将值减去 1024 得到 ±660 范围，再通过 `REMOTE_DEAD_ZONE` (10) 去除零漂：

```c
ch -= DT7_CH_VALUE_OFFSET;        // 零偏
if (abs(ch) <= REMOTE_DEAD_ZONE)  // 死区
    ch = 0;
```

## 注意事项

1. **初始化顺序** — `Module_Offline_init()` → `Module_Remote_init()`，否则子模块 Offline 注册失败。

2. **UART 波特率被子模块修改** — 子模块 init 会调用 `HAL_UART_Init()` 重配波特率（SBUS/DT7: 100000, VT02: 115200, VT03: 921600），确保 `REMOTE_UART` / `REMOTE_VT_UART` 不与其它外设共用。


## 版本历史

| 版本 | 日期 | 说明 |
|------|------|------|
| v1.0 | 2026-05-10 | 初始版本 |
| v1.1 | 2026-05-23 | 适配最新bsp_uart |
| v1.2 | 2026-05-24 | SBUS/DT7/VT03 改为 for 循环帧头扫描模式，支持多帧累积解码 |
