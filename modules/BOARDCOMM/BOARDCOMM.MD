# BOARDCOMM 板间通信模块文档

## 概述

BOARDCOMM 模块负责云台板与底盘板之间的 CAN 总线通信，提供 8 字节固定帧长的收发接口，内置离线检测。

**双板架构:**

```
┌──────────────┐         CAN (8 bytes)        ┌──────────────┐
│  云台板      │ ◄──────────────────────────► │  底盘板      │
│  GIMBAL_BOARD│   0x310: 云台→底盘           │ CHASSIS_BOARD│
│              │   0x311: 底盘→云台           │              │
└──────────────┘                              └──────────────┘
```

## 架构设计

### 数据流

```
                  ┌────────────────────────────────────┐
                  │         BOARDCOMM 模块             │
                  │                                    │
CAN 帧到达 ──────►│  boardcomm_bsp_rx_callback()       │
                  │  ├── buffer 模式: memcpy → 更新离线│
                  │  └── callback 模式: 调用 app 回调  │
                  │                                    │
app 发送 ────────►│  Module_BoardComm_Send()           │
                  │  └── BSP_CAN_Send() (≤8 bytes)     │
                  └────────────────────────────────────┘
```

> 两种接收模式互斥: `RegisterRxBuffer` 与 `RegisterRx`。

## API 接口说明

```c
void Module_BoardComm_Init(void);
```

```c
void Module_BoardComm_Send(uint8_t *data, uint8_t len);
```

```c
void Module_BoardComm_RegisterRx(BoardComm_RxCallback_t callback);
```

注册接收回调函数。收到 CAN 帧时由 BOARDCOMM 自动调用。重复调用会覆盖，传 `NULL` 注销。

```c
typedef void (*BoardComm_RxCallback_t)(const uint8_t *data, uint8_t len);
```

```c
void Module_BoardComm_RegisterRxBuffer(void *buffer, uint8_t expected_len);
```

直接注册接收缓冲区。CAN 帧长度与 `expected_len` 匹配时，BOARDCOMM 内部自动 `memcpy` 到 `buffer`。


```c
uint8_t Module_BoardComm_Get_Offline_State(void);
```

## 配置

### CAN ID

| 宏 | 值 | 方向 |
|----|-----|------|
| `BOARDCOMM_GIMBAL_ID` | `0x310` | 云台 → 底盘 |
| `BOARDCOMM_CHASSIS_ID` | `0x311` | 底盘 → 云台 |

### CAN 总线

```c
#define BOARDCOMM_CAN  BSP_CAN_HANDLE2  // 默认 CAN2，可覆盖
```

### 编译选项

| 宏 | 说明 |
|----|------|
| `SINGLE_BOARD` | 单板模式，`Module_BoardComm_Init()` 直接跳过 |
| `CHASSIS_BOARD` | 编译为底盘板（TX=0x311, RX=0x310） |
| `GIMBAL_BOARD` | 编译为云台板（TX=0x310, RX=0x311） |

## 使用示例

### 方式一：RegisterRxBuffer **接收结构体强制8字节，并且有静态校验**

```c
#include "module_boardcomm.h"

/* 板间通讯结构体 */
#pragma pack(1)

typedef struct
{
    int8_t  vx;           /* X速度, -10 ～ +10 */
    int8_t  vy;           /* Y速度, -10 ～ +10 */
    int8_t  wz;           /* 旋转速度, -10 ～ +10 */
    int16_t offset_angle; /* yaw偏移角度 */
    uint8_t chassis_mode; /* 底盘控制指令 (chassis_mode_e) */
    int8_t  reserved[2];  /* 填充至8字节 */
} GimbalToChassis_cmd_t;
_Static_assert(sizeof(GimbalToChassis_cmd_t) == 8, "GimbalToChassis_cmd_t must be 8 bytes");

typedef struct
{
    uint8_t  robot_color;      /* 机器人颜色 (0=红,1=蓝) */
    uint8_t  game_progress;    /* 比赛阶段 */
    uint16_t current_hp;       /* 当前血量 */
    uint16_t bullet_allow;     /* 可发射17mm弹丸剩余量 */
    uint16_t shooter_heat_pct; /* 枪管热量% = heat/limit*100 */
} ChassisToGimbal_referee_t;
_Static_assert(sizeof(ChassisToGimbal_referee_t) == 8, "ChassisToGimbal_referee_t must be 8 bytes");

#pragma pack()

static GimbalToChassis_cmd_t     chassis_send_cmd; 
static ChassisToGimbal_referee_t chassis_upload_data;

void app_init(void)
{
    Module_BoardComm_Init();

    // 直接注册接收缓冲区，BOARDCOMM 内部 memcpy
    Module_BoardComm_RegisterRxBuffer(&chassis_upload_data,
                                       sizeof(ChassisToGimbal_referee_t));
}

void app_task(void)
{
    // 发送：直接填结构体，强转为 uint8_t* 发送
    chassis_send_cmd.vx = 5;
    chassis_send_cmd.vy = 0;
    Module_BoardComm_Send((uint8_t *)&chassis_send_cmd,
                           sizeof(GimbalToChassis_cmd_t));

    // 接收：chassis_upload_data 已被自动更新，直接读取
    uint16_t hp = chassis_upload_data.current_hp;

    // 检查板间通信是否离线
    if (Module_BoardComm_Get_Offline_State() == STATE_OFFLINE)
    {
        // 进入安全模式
    }
}
```

### 方式二：RegisterRx（需要自定义解析逻辑）

```c
static void my_rx_callback(const uint8_t *data, uint8_t len)
{
    if (len == 8)
    {
        // 自定义解析逻辑
        uint16_t hp = *(uint16_t *)(data + 2);
        process_data(hp);
    }
}

void app_init(void)
{
    Module_BoardComm_Init();
    Module_BoardComm_RegisterRx(my_rx_callback);
}
```

## 内部实现细节

### BSP CAN 回调

```c
static void boardcomm_bsp_rx_callback(Can_Device *dev,
                                       const uint8_t *data, uint8_t len)
{
    // buffer 模式
    if (rx_buffer != NULL && len == rx_expected_len)
    {
        memcpy(rx_buffer, data, len);
        Module_Offline_device_update(boardcomm_offline_dev);
    }
    // callback 模式
    else if (app_rx_callback != NULL)
    {
        app_rx_callback(data, len);
        Module_Offline_device_update(boardcomm_offline_dev);
    }
}
```

## 注意事项

1. **帧长限制** — CAN 帧最大 8 字节。结构体超过 8 字节会导致 `_Static_assert` 编译失败。

2. **RegisterRxBuffer vs RegisterRx** — 两种模式互斥。

5. **结构体对齐** — 使用RegisterRxBuffer时定义结构体必须用 `#pragma pack(1)` 保证无填充，`_Static_assert(sizeof(...) == 8)` 编译期校验。

## 版本历史

| 版本 | 日期 | 说明 |
|------|------|------|
| v1.0 | 2026-05-13 | 初始版本 |
| v1.1 | 2026-05-17 | 重构，增加RegisterRxBuffer与RegisterRx |
| v1.1.1 | 2026-05-17 | fix:修复send函数传参 |
