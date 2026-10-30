# 三 IMU 三段原坐标姿态

该工具位于仓库的 `tools/` 目录，只读取 J-Link RTT Channel 0 中的
`ARMPOSE` 文本帧，不加载 URDF，不执行 IK，不复位、暂停或烧写芯片。

双击 `start_imu_raw_pose.bat` 一键启动。启动前需关闭 Ozone、Cortex-Debug、
OpenOCD 和其他 J-Link RTT Logger。

画面只包含：

- 位于画面中心的基座/肩点球和世界 RGB 坐标轴；
- 肘点球，以及大臂 HL04 的 RGB 姿态轴；
- 腕点球，以及前臂 HL04 的 RGB 姿态轴；
- 手部目标点球，以及手背 BMI088 的 RGB 姿态轴；
- `0.204 m` 大臂、`0.120 m` 前臂和 `0.090 m` 手部连杆。

四个点的位置为：

```text
p_base  = [0, 0, 0]^T
p_elbow = p_base  + R_upper   * [0.204, 0, 0]^T
p_wrist = p_elbow + R_forearm * [0.120, 0, 0]^T
p_hand  = p_wrist + R_hand    * [0.090, 0, 0]^T
```

三个姿态直接使用 Channel 0 中的 `[w,x,y,z]` 四元数，不套用
MATLAB 机械臂的关节取反、URDF 零位偏置或末端坐标转换。固件完成
软件标零后，`ARMPOSE` 中的四元数是相对最新标零姿态的旋转。

新版固件在旧 13 字段 `ARMPOSE` 尾部追加 BMI088 有效位和四元数；
工具仍兼容旧帧，读取旧固件时只显示原来的两段。

三维窗口复用 `matlab_arm_visualizer` 的视角控制：

- 主视图采用右手系：`+Z` 向上，`+X` 向前；
- 左键拖动自由旋转，中键拖动绕世界 Z 轴旋转；
- 滚轮缩放，`F` 重新取景，`H` 或 `Home` 恢复主视图；
- 数字键 `1` 到 `0` 选择视图 01 到 10，`V` 循环切换。
