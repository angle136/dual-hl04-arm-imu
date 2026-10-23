# BSP DWT 驱动文档

## 概述

BSP DWT 是一个基于 Cortex-M 内核的 DWT (Data Watchpoint and Trace) 单元的高精度定时驱动程序。该驱动提供了微秒级甚至更高精度的时间测量和延时功能，相比传统的 SysTick 定时器具有更高的精度和更广泛的应用场景。


## 架构设计

### `DWT_Time_t`

```c
typedef struct
{
    uint32_t s;   // 秒
    uint16_t ms;  // 毫秒
    uint16_t us;  // 微秒
} DWT_Time_t;
```

### 全局变量

| 变量名 | 类型 | 说明 |
|--------|------|------|
| `SysTime` | `DWT_Time_t` | 当前系统时间（分解为 s/ms/us） |
| `CPU_FREQ_Hz` | `uint32_t` | CPU 频率（Hz） |
| `CPU_FREQ_Hz_ms` | `uint32_t` | CPU 频率（cycles/ms） |
| `CPU_FREQ_Hz_us` | `uint32_t` | CPU 频率（cycles/us） |
| `CYCCNT_RountCount` | `uint32_t` | CYCCNT 溢出次数 |
| `CYCCNT_LAST` | `uint32_t` | 上次读取的 CYCCNT 值 |
| `CYCCNT64` | `uint64_t` | 64 位扩展计数器值 |
| `dwt_lock_state` | `volatile uint8_t` | 重入保护标志（0-未持有，1-已持有） |

### 代码架构

```
内部辅助函数层：
├─ DWT_CNT_Update()              // 检查并更新计数器溢出状态
└─ DWT_SysTimeUpdate()           // 更新系统时间轴（计算 s/ms/us）

公开 API 层：
├─ BSP_DWT_Init()                // 初始化 DWT 外设和全局变量
├─ BSP_DWT_GetDeltaT()           // 获取时间间隔（float 精度，秒）
├─ BSP_DWT_GetDeltaT64()         // 获取时间间隔（double 高精度，秒）
├─ BSP_DWT_GetTimeline_s()       // 获取系统运行时间（秒）
├─ BSP_DWT_GetTimeline_ms()      // 获取系统运行时间（毫秒）
├─ BSP_DWT_GetTimeline_us()      // 获取系统运行时间（微秒）
└─ BSP_DWT_Delay()               // 高精度延时（秒）

性能测量宏：
└─ BSP_DWT_MEASURE_START/END()   // 代码段性能测量（自动打印耗时）
```

## API 接口说明

```c
void BSP_DWT_Init(uint32_t CPU_Freq_mHz);
```
```c
float BSP_DWT_GetDeltaT(uint32_t *cnt_last);
```
```c
float BSP_DWT_GetTimeline_s(void);
```
```c
float BSP_DWT_GetTimeline_ms(void);
```
```c
uint64_t BSP_DWT_GetTimeline_us(void);
```
```c
void BSP_DWT_Delay(float Delay);
```

### BSP_DWT_MEASURE_START/END 宏

**功能：** 便捷的性能测量宏，自动打印代码段执行时间

## 注意事项

**必须在使用任何 DWT 功能前调用 `BSP_DWT_Init()`**

## 使用流程
  ```c
  // 初始化DWT，传入CPU频率(MHz)
  // C板为168MHz，喵板为480MHz
  BSP_DWT_Init(168);  // 对于C板
  // 或
  BSP_DWT_Init(480);  // 对于喵板

  // 性能测量
  BSP_DWT_MEASURE_START();
  
  // 执行需要测量的代码
  some_function();
  
  // 结束测量，并通过log打印输出
  BSP_DWT_MEASURE_END(function_name);

  //获取自初始化以来的系统运行时间：
  // 获取系统运行时间（秒）
  float time_s = BSP_DWT_GetTimeline_s();
  
  // 获取系统运行时间（毫秒）
  float time_ms = BSP_DWT_GetTimeline_ms();
  
  // 获取系统运行时间（微秒）
  uint64_t time_us = BSP_DWT_GetTimeline_us();

  // 延时1毫秒
  BSP_DWT_Delay(0.001f);
  
  // 延时100微秒
  BSP_DWT_Delay(0.0001f);
  ```
  

## 版本历史

| 版本 | 日期 | 说明 |
|------|------|------|
| v1.0 | 2026-04-22 | 初始版本 |
