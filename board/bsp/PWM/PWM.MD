# BSP PWM 驱动文档

## 概述

BSP PWM 是基于 STM32 HAL 库的 PWM 输出驱动层，提供阻塞 / 中断 / DMA 三种工作模式。

## 架构设计

### 设备管理模型

```
全局链表 devices_list 
    │
    ├── PWM_Device (dev0)
    │   ├── htim / Channel ── 绑定的定时器和通道
    │   ├── Period / dutyx10 ── 周期和占空比
    │   ├── Mode ── BLOCKING / IT / DMA
    │   ├── event_flags ── 独立事件标志组
    │   └── next ── 指向下一个设备
    │
    ├── PWM_Device (dev1)
    └── ...
```

### 传输流程（IT / DMA 模式）

```
BSP_PWM_Start(dev, data, len)
    │
    ├── 清除残留事件标志 ── TX_NO_WAIT 清理 COMPLETE
    ├── BLOCKING: HAL_TIM_PWM_Start()
    ├── IT:       HAL_TIM_PWM_Start_IT()
    └── DMA:      BSP_IS_DMA_INACCESSIBLE 检查
                  BSP_CACHE_CLEAN(data)
                  HAL_TIM_PWM_Start_DMA()
    │
    └── 返回成功

BSP_PWM_WaitForComplete(dev, timeout)
    └── tx_event_flags_get(&dev->event_flags, COMPLETE, …)  ── 阻塞等待

ISR 路径:
    TIMx_IRQHandler
      └── HAL_TIM_IRQHandler
            └── HAL_TIM_PWM_PulseFinishedCallback(htim)
                  └── for (dev = devices; dev; dev = dev->next)
                        └── 匹配 htim 的设备 → tx_event_flags_set(COMPLETE)
```

> **注意**：HAL 回调仅传入 `htim`，不含 channel 号。因此同一 TIM 上所有匹配设备的 `event_flags` 都会被设置。`BSP_PWM_Start` 启动前清除残留标志保证不会误触发。

### `PWM_Device` 设备实例

| 字段 | 类型 | 说明 |
|------|------|------|
| `next` | `PWM_Device *` | 指向下一个设备指针，NULL 表尾 |
| `htim` | `TIM_HandleTypeDef *` | HAL 定时器句柄 |
| `Channel` | `uint32_t` | PWM 通道（`TIM_CHANNEL_1` ~ `TIM_CHANNEL_4`） |
| `Period` | `uint32_t` | 自动重装载值（从 CubeMX 配置 `htim->Init.Period` 读取） |
| `dutyx10` | `uint32_t` | 占空比（0-1000，对应 0.0%-100.0%） |
| `Mode` | `PWM_Mode` | 工作模式 |
| `event_flags` | `TX_EVENT_FLAGS_GROUP` | 独立事件标志组（bit0: COMPLETE） |

### 事件标志位

| 宏 | 值 | 触发者 | 等待者 |
|----|-----|--------|--------|
| `BSP_PWM_EVENT_COMPLETE` | `0x01` | `HAL_TIM_PWM_PulseFinishedCallback` (ISR) | `BSP_PWM_WaitForComplete` (任务) |


## API 接口说明

```c
PWM_Device *BSP_PWM_Device_Init(PWM_Init_Config *config);
```

```c
uint8_t BSP_PWM_Start(PWM_Device *dev, uint8_t *data, uint8_t len);
```

```c
uint8_t BSP_PWM_Stop(PWM_Device *dev);
```

```c
uint8_t BSP_PWM_SetDutyCycle(PWM_Device *dev, uint16_t duty);
```

```c
uint8_t BSP_PWM_SetPulse(PWM_Device *dev, uint32_t pulse);
```

```c
uint8_t BSP_PWM_SetPSC(PWM_Device *dev, uint32_t psc);
```

```c
uint8_t BSP_PWM_WaitForComplete(PWM_Device *dev, uint32_t wait_option);
```

## 使用示例

### BLOCKING 模式（简单开关）

```c
#include "bsp_pwm.h"

extern TIM_HandleTypeDef htim1;

static PWM_Device *g_pwm;

void pwm_blocking_demo(void)
{
    PWM_Init_Config cfg = {
        .htim    = &htim1,
        .Channel = TIM_CHANNEL_1,
        .dutyx10 = 500,              /* 50.0% */
        .Mode    = PWM_MODE_BLOCKING,
    };

    g_pwm = BSP_PWM_Device_Init(&cfg);
    if (g_pwm == NULL) return;

    BSP_PWM_Start(g_pwm, NULL, 0);   /* 启动，阻塞至完成 */

    /* 运行时调节占空比 */
    BSP_PWM_SetDutyCycle(g_pwm, 750); /* 75.0% */
}
```

### IT/DMA 模式

```c
#include "bsp_pwm.h"
#include "bsp_def.h"

extern TIM_HandleTypeDef htim3;

static PWM_Device *g_pwm_dma;
static uint32_t    g_dma_buf[16] BUFFER_SECTION;

void pwm_dma_demo(void)
{
    /* 准备 DMA 数据 */
    for (int i = 0; i < 16; i++)
    {
        g_dma_buf[i] = (i * 100); /* 占空比 0% → 100% 步进 */
    }

    PWM_Init_Config cfg = {
        .htim    = &htim3,
        .Channel = TIM_CHANNEL_3,
        .dutyx10 = 0,
        .Mode    = PWM_MODE_DMA,
    };

    g_pwm_dma = BSP_PWM_Device_Init(&cfg);

    BSP_PWM_Start(g_pwm_dma, (uint8_t *)g_dma_buf, 16);
    BSP_PWM_WaitForComplete(g_pwm_dma, TX_WAIT_FOREVER);
}
```

## 注意事项

- **DMA 缓冲区必须在 DMA 可访问区域** — H7 上禁止 DTCM/ITCM。
- **DMA 数据为 uint32_t 数组** — `BSP_PWM_Start` 的 `data` 参数被视为 `uint32_t *`，每个元素对应一个 CCR 比较值。`len` 是元素个数，不是字节数。
- **IT/DMA 模式配对** — `Start` 之后必须调用 `WaitForComplete` 等待脉冲完成，否则 `event_flags` 可能残留。
- **无互斥锁** — 同一设备不应被多任务并发操作。多任务共享设备需上层保证互斥。
- **SetDutyCycle 实时性** — `__HAL_TIM_SET_COMPARE` 直接写寄存器，可在 PWM 运行中随时调用。但修改在下一个周期才生效。

## 版本历史

| 版本 | 日期 | 说明 |
|------|------|------|
| v1.0 | 2026-05-05 | 初始版本 |
| v1.1 | 2026-05-21 | 改用单向链表 |
