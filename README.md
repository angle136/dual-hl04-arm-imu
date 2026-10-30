# Dual HL04 Arm IMU

STM32F407 + ThreadX 三 IMU 三段手臂姿态采集与机械臂位姿解算工程。姿态源包括两只 HL04 和手背主控板内置 BMI088。仓库包含完整可配置固件环境、`arm_imu` APP，以及配套 MATLAB/J-Link RTT 可视化工具。

## 当前硬件配置

- 控制器：DJI C 型开发板，STM32F407IG；
- 大臂 HL04：USART1，`921600 baud`；
- 前臂 HL04：CAN2，`500 kbit/s`；
- 手背姿态：主控板内置 BMI088，经 INS EKF 输出 `w,x,y,z` 四元数；
- CAN 协议：CANopen 标准帧，节点 8，TPDO4 四元数 ID `0x488`；
- 三段长度：大臂 `0.204 m`、前臂 `0.120 m`、腕部到手部目标点 `0.090 m`；
- 手动标零：板载 KEY（PA0）；
- J-Link RTT Channel 0：`ARMPOSE` 文本姿态；
- J-Link RTT Channel 1：兼容机械臂板间协议的 36 字节六轴逆解帧。

本 APP 不初始化或控制电机。两只 HL04 与 BMI088 的姿态、三段位置链和六轴解析逆解均用于观测、调试和后续集成。

## 目录

```text
apps/arm_imu/                 三 IMU 三段手臂 APP 与参数
board/                        STM32 板级工程与 BSP
modules/                      公共功能模块
threadx/                      Azure RTOS ThreadX
utils/                        日志、RTT 与通用工具
cmake/                        公共构建脚本
tools/matlab_arm_pose/        三 IMU 姿态与 RPY 曲线
tools/matlab_imu_raw_pose/    四点三连杆原坐标视图
```

## 构建

默认目标已经设为 `ROBOT=arm_imu`、`BOARD=single`。从 `board/dji_c` 配置：

```bash
cmake -S board/dji_c -B build/dji_c/Debug --preset Debug -DROBOT=arm_imu -DBOARD=single
cmake --build build/dji_c/Debug -j
```

生成固件为 `build/dji_c/Debug/base.elf`。切换旧构建目录时仍应显式传入 `-DROBOT=arm_imu -DBOARD=single`，避免 CMake 缓存沿用其他目标。

## MATLAB 工具

关闭 Ozone、Cortex-Debug、OpenOCD 和其他占用 J-Link 的进程后，可运行：

- `tools/matlab_arm_pose/start_arm_pose.bat`
- `tools/matlab_imu_raw_pose/start_imu_raw_pose.bat`

两套工具只读取 RTT，不复位、暂停、烧写或修改目标芯片。

`ARMPOSE` 文本协议在原双 HL04 字段尾部追加手背 BMI088 有效位和四元数；旧解析器仍可读取前 13 个字段，两套 MATLAB 工具可显示三 IMU。

## 来源与历史

公共固件环境源自 `mas_embedded_threadx`。本仓库保留了双 HL04 APP 从初次集成、手动标零、CAN 接收修复到 CANopen `0x488` 配置的提交历史，并在此基础上继续维护 IMU 专属功能。
