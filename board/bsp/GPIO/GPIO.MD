# BSP GPIO 驱动文档

## 概述

BSP GPIO 是外部中断（EXTI）回调管理层，提供基于引脚号的回调注册/注销机制。HAL 的 `HAL_GPIO_EXTI_Callback` 统一入口根据引脚号分发到对应的用户回调函数。

## 架构设计

### 回调分发模型

```
硬件中断 ──→ HAL_GPIO_EXTI_IRQHandler(pin)
                  │
                  └── HAL_GPIO_EXTI_Callback(pin)
                          │
                          ├── get_callback_index(pin) ── 查找静态数组
                          │
                          └── exti_callbacks[index].callback()
                                  │
                                  ├── 回调0: 用户函数（pin=某值）
                                  ├── 回调1: 用户函数（pin=某值）
                                  └── ...
```

### `EXTI_Callback_t` 回调槽位

| 字段 | 类型 | 说明 |
|------|------|------|
| `pin` | `uint16_t` | 绑定的 GPIO 引脚号（0-15） |
| `callback` | `void (*)(void)` | 用户回调函数指针 |
| `initialized` | `uint8_t` | 槽位是否已占用 |

### 静态数组

```c
static EXTI_Callback_t exti_callbacks[MAX_EXTI_CALLBACKS]; /* 最大 8 个回调 */
static uint8_t         callback_count = 0;                 /* 当前已注册数量 */
```

## API 接口说明

```c
uint8_t BSP_GPIO_EXTI_Register(uint16_t pin, void (*callback)(void));
```

```c
void BSP_GPIO_EXTI_Unregister(uint8_t index);
```

## 使用示例

### 按键中断（单个 pin）

```c
#include "bsp_gpio.h"

static uint8_t g_key_cb_index;

/* 用户回调 */
static void key_callback(void)
{
    /* 处理按键，如发送信号量 */
}

void key_exti_init(void)
{
    g_key_cb_index = BSP_GPIO_EXTI_Register(GPIO_PIN_13, key_callback);
    if (g_key_cb_index == 0xFF)
    {
        /* 注册失败 */
    }
}

void key_exti_deinit(void)
{
    BSP_GPIO_EXTI_Unregister(g_key_cb_index);
}
```


## 内部实现细节

### 查找空闲槽位

`find_empty_slot()` 遍历 `exti_callbacks[]` 数组，返回第一个 `initialized == 0` 的槽位。注销后槽位被标记为未初始化，后续注册可复用该位置。

### 重复注册处理

`BSP_GPIO_EXTI_Register` 先通过 `get_callback_index(pin)` 检查是否已有同 pin 的回调。如已存在，返回已有索引（幂等操作），不创建新槽位。

### 无碎片整理

注销时仅清除单个槽位（`initialized = 0`），不移动后续数组元素。频繁注册/注销可能导致数组中存在"空洞"，但 `find_empty_slot` 会优先使用第一个空洞。

### `callback_count` 计数

仅用于日志输出，不参与槽位查找逻辑。注销时递减，注册时递增。非线程安全。

## 注意事项

1. **无互斥锁保护** — 注册/注销操作非线程安全。必须在任务启动前（初始化阶段）完成所有注册，或由上层保证互斥。
2. **回调在 ISR 上下文执行** — 用户回调函数运行在 EXTI 中断上下文中，应尽量简短。避免在回调中调用 ThreadX 阻塞 API（`tx_semaphore_get` 等），应使用 ISR 安全版本（如 `tx_semaphore_put`）。
3. **最大回调数量** — `MAX_EXTI_CALLBACKS` 默认为 8。引脚数超过 8 时需要增大此值。
4. **pin 范围** — 仅支持 `0`-`15`（对应 16 个 EXTI 线）。超出范围返回 `0xFF`。
5. **用户回调返回值** — `void` 无返回值。中断上下文中通常通过信号量/队列通知任务。
6. **HAL 弱函数覆盖** — 确保 `bsp_gpio.c` 中的 `HAL_GPIO_EXTI_Callback` 覆盖了 HAL 库的默认弱定义。链接顺序可能导致覆盖失败。

## 版本历史

| 版本 | 日期 | 说明 |
|------|------|------|
| v1.0 | 2026-05-05 | 初始版本 |
