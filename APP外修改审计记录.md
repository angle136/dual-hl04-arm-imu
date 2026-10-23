# App 层外修改审计记录

## 规则

本工程的机械臂业务实现原则上只修改 `apps/` 层。任何确实需要修改 `apps/` 之外文件的操作，都必须在本文件追加记录，并在交付说明中明确告知：

- 修改了哪些文件；
- 修改的目的和影响范围；
- 是否经过构建或其他验证；
- 该修改是本轮主动实施，还是工作区既有改动被保留并同步。

本记录只审计 `apps/` 之外的文件；`apps/` 内的业务改动不在这里展开。

## 2026-10-01：机械臂第一阶段基线

对应提交：`c5296db feat(arm): add dji_c motor bring-up baseline`

### 本轮主动修改

| 文件 | 修改内容 | 目的 | 验证 |
| --- | --- | --- | --- |
| `.vscode/tasks.json` | 将 `Build dji_c` 设为默认构建任务；构建命令显式传入 `-DROBOT=arm -DBOARD=single`。保留工作区原有的 F103/F105 构建任务。 | 让快捷键构建机械臂时固定指向 DJI C/F407 板，而不是 H7；避免复用错误的 CMake 机器人缓存。 | `cmake --build build/dji_c/Debug --config Debug -j 4` 通过。 |
| `README.md` | 将项目说明改为 ARM 机械臂第一阶段目标，补充电机注册表、构建命令和分阶段验收顺序。 | 让仓库入口文档与当前“先注册电机、确认在线，再做重力补偿”的目标一致。 | 与 `dji_c/arm` 构建结果一致。 |
| `modules/MOTOR/motor_def.h` | 增加 `DM4340` 电机类型枚举。 | 允许 J2 按实际型号注册，不再落回错误的默认电机类型。 | 工程编译通过。 |
| `modules/MOTOR/DAMIAO/motor_damiao.c` | 将达妙参数表改为按枚举索引，并补充 DM6220、DM4340、DM3507 的位置、速度、力矩量程。 | 让七个电机的反馈解码和 MIT 报文编码使用对应型号量程。参数仍需结合实物手册和反馈实测复核。 | 工程编译通过；未接实物验证量程。 |

### 工作区既有改动：未由本轮主动设计，但随远端覆盖同步

以下改动在本轮开始前已存在于工作树。为完成用户要求的整目录覆盖，它们被包含在 `c5296db` 中；这里记录来源，后续不应把它们误认为机械臂 App 逻辑修改：

| 范围 | 现状 | 备注 |
| --- | --- | --- |
| `board/105_rc/**` | 删除/调整 F1 外设、HAL CAN/TIM 文件及 CubeMX/CMake 配置。 | 属于工作区原有的 105 RC 板裁剪改动；本阶段构建目标是 `board/dji_c`，未依赖它。 |
| `board/bsp/bsp_init.c` | 移除蜂鸣器初始化和 BSP 层 `BSP_CAN_TaskInit()` 调用。 | 属于工作区原有初始化调整；机械臂应用在注册电机前显式启动 CAN 任务。 |
| `.vscode/tasks.json` 中 F103/F105 条目 | 新增两个板卡构建任务。 | 属于工作区原有任务配置；本轮只改变 DJI C 默认任务和其 ARM 参数。 |

## 2026-10-01：状态日志标签修正

对应提交：`2b1b893 fix(arm): label motor status logs by joint`

本次只修改 `apps/arm/single_board/robot_control.c`，将达妙状态日志从按数组下标显示改为正确显示 `J1/J2/J3/J5/J6/J7`。没有发生 App 层外修改，因此没有新增审计项。

## 当前验证状态

- 本地分支 `arm` 与 `origin/main` 同步。
- `build/dji_c/Debug/base.elf` 已成功生成。
- 尚未接入真实机械臂硬件；在线/掉线、反馈 ID 和量程仍需烧录后通过 `arm_status` 日志确认。

## 2026-10-01：达妙电机 CAN 发送节奏调整

### 本轮主动修改

| 文件 | 修改内容 | 目的 | 验证 |
| --- | --- | --- | --- |
| `modules/MOTOR/DAMIAO/motor_damiao.c` | 达妙清错和启动命令均重复发送两次；普通 DM 控制帧发送后增加 200us 间隔，命令帧发送后增加 400us 间隔。 | 复用旧机械臂工程对达妙 CAN 抢占/响应时序的处理方式，避免 6 个 DM 电机在同一周期内无间隔排队发送导致反馈停止、离线检测超时。 | `cmake --build build/dji_c/Debug --parallel 4` 通过；需烧录后观察 `arm_status`/`g_arm_feedback_snapshot` 是否持续更新。 |
| `modules/MOTOR/DJI/motor_dji.c` | DJI/GM6020 共享控制帧发送后增加 200us 间隔。 | 复用旧机械臂工程中 GM6020 对前后 CAN 空窗的要求，避免 6020 控制帧与 DM 帧紧贴发送导致反馈异常。 | `cmake --build build/dji_c/Debug --parallel 4` 通过；需烧录后观察 J4 反馈和 DM 反馈是否持续更新。 |

## 2026-10-01：电机驱动注释补全

### 本轮主动修改

| 文件 | 修改内容 | 目的 | 验证 |
| --- | --- | --- | --- |
| `modules/MOTOR/DAMIAO/motor_damiao.c` | 为本阶段改动过或直接受影响的达妙函数补充标准中文嵌入式注释，包括 `mit_ctrl()`、`pos_speed_ctrl()`、`speed_ctrl()`、`psi_ctrl()`、`dm_ControlAndSend()`、`Motor_DM_Cmd()` 和 `Motor_DM_Init()`。 | 明确函数输入、输出/返回、调用关系和 CAN 帧间隔设计原因，便于后续审查和接手调试；不改变运行逻辑。 | `cmake --build build/dji_c/Debug --parallel 4` 通过。 |
| `modules/MOTOR/DJI/motor_dji.c` | 为 `Motor_DJI_Flush()` 补充标准中文嵌入式注释。 | 明确 GM6020 控制帧发送流程和帧间隔原因，便于后续审查；不改变运行逻辑。 | `cmake --build build/dji_c/Debug --parallel 4` 通过。 |

## 2026-10-01：电机 CAN 槽位调度调整

### 本轮主动修改

| 文件 | 修改内容 | 目的 | 验证 |
| --- | --- | --- | --- |
| `modules/MOTOR/motor_base.c` | `Motor_UpdateAll()` 从单次遍历全部电机，调整为 CAN 槽位轮询：每个槽位内每条 CAN 总线最多调用一台电机的 `ControlAndSend()`，槽位之间插入 200us 等待；新增内部辅助函数 `motor_is_can_bus()` 和 `motor_find_next_can()`。 | 复刻旧机械臂工程 `Core/Src/freertos.c` 的节奏：CAN1 和 CAN2 互不抢占，可在同一时间片各发一帧；真正需要间隔的是同一条 CAN 内部相邻报文。 | `cmake --build build/dji_c/Debug --parallel 4` 通过。 |
| `modules/MOTOR/motor_base.h` | 更新 `Motor_UpdateAll()` 接口注释，说明 CAN 槽位轮询调度和调用关系。 | 让调度行为在公共头文件中可审查。 | `cmake --build build/dji_c/Debug --parallel 4` 通过。 |
| `modules/MOTOR/DAMIAO/motor_damiao.c` | 删除达妙运行控制帧发送函数内部的 200us 延时，并删除 `dm_ControlAndSend()` 尾部已失效的注释掉延时代码；达妙特殊命令帧 400us 间隔保持不变。 | 避免把 CAN1/CAN2 跨总线发送也人为串行拉开；运行期同一 CAN 内部间隔统一由 `Motor_UpdateAll()` 的槽位调度保证。 | `cmake --build build/dji_c/Debug --parallel 4` 通过。 |

## 2026-10-01：升级到主仓库底座并迁移机械臂必要补丁

### 本轮主动修改

| 文件 | 修改内容 | 目的 | 验证 |
| --- | --- | --- | --- |
| 全工程底座 | 将当前工作区底座切换到 `upstream/main`（`HebutMas/mas_embedded_threadx` 的 `2d39c0f`），再叠加机械臂 APP 和必要模块兼容补丁。 | 复用主仓库最新两阶段电机框架、CAN TX FIFO、模块结构等改动，减少继续维护旧底座的风险。 | 独立 worktree 试迁移构建通过；主工作区 `cmake -S board/dji_c -B build/dji_c/Debug --preset Debug -G Ninja -DROBOT=arm -DBOARD=single` 配置通过，`cmake --build build/dji_c/Debug --parallel 4` 构建通过。 |
| `apps/arm/**` | 保留机械臂 APP 逻辑和中文标准注释；将旧的 `Motor_DM_Stop()` / `Motor_DJI_Stop()` 适配为主仓库通用 `Motor_Stop(&base)`。 | 适配主仓库电机 API，同时保持当前“只注册电机、输出失能、读取反馈快照和状态日志”的 APP 行为。 | 主工作区构建通过。 |
| `modules/MOTOR/motor_def.h` | 增加 `DM4340` 电机类型枚举。 | 允许 J2 按实际达妙 DM4340 型号注册。 | 主工作区构建通过。 |
| `modules/MOTOR/DAMIAO/motor_damiao.c` | 将达妙参数表扩展为按电机枚举索引，补充 DM4310、DM4340、DM6220、DM3507 参数；特殊命令帧间隔调整为 400us；初始化阶段清错和启动命令各重复发送一次；补充中文标准注释。 | 防止主仓库默认全部落回 DM4310 参数导致 J1/J2/J6/J7 反馈和控制量程错误；保留旧工程上电命令节奏。 | 主工作区构建通过。 |
| `modules/MOTOR/motor_base.c` / `modules/MOTOR/motor_base.h` | 在主仓库两阶段框架的 `Motor_ApplyAll()` 中加入 CAN 槽位轮询：每个槽位每条 CAN 总线最多应用一台电机，槽位之间等待 200us；补充中文标准注释。 | 主仓库已有 CAN TX FIFO，但未保证同一 CAN 内部相邻运行控制帧间隔；该补丁复刻旧工程 CAN1/CAN2 可同片发送、同 CAN 内部要隔开的节奏。 | 主工作区构建通过。 |

## 2026-10-01：DM4340 参数复核与默认构建任务修复

### 本轮主动修改

| 文件 | 修改内容 | 目的 | 验证 |
| --- | --- | --- | --- |
| `.vscode/tasks.json` | 将默认 `Build selected board`、`Build dji_c`、`Build damiao_h7` 构建任务从 `type: shell` 改为 `type: process`；Windows 下直接启动 `powershell`，避免 VSCode 外层 PowerShell 再套一层内部 PowerShell；三个构建任务均显式传入 `-DROBOT=arm -DBOARD=single`。 | 修复快捷键构建时 `$board='dji_c'` 被外层引号吃掉、变成 `$board= dji_c` 的错误，同时避免 `cmake` 去错误读取 `board/CMakePresets.json`；保证手动选择任意板子时仍编译机械臂 APP。 | `tasks.json` 通过 `ConvertFrom-Json` 校验；按任务等价命令执行 `dji_c` 构建通过，输出 `Robot: arm`、`Board: single`，`ninja: no work to do`。 |
| `modules/MOTOR/DAMIAO/motor_damiao.c` | 对 DM4340 参数来源补充说明：位置 ±12.5 rad、速度 ±10 rad/s、力矩 ±28 N·m 参照旧机械臂工程；Kp/Kd 下限保留 0。 | 新建 4340 型号时匹配旧工程实际量程；同时保留旧工程 `DM_4340_cmd()` 只发力矩、位置/速度/Kp/Kd 原始字节为 0 的行为，避免 Kp=0/Kd=0 被负下限量程错误编码成非零原始值。 | 主工作区 `dji_c/arm` 构建通过。 |

## 2026-10-01：电机底层安全限位和致命故障蜂鸣

### 本轮主动修改

| 文件 | 修改内容 | 目的 | 验证 |
| --- | --- | --- | --- |
| `modules/MOTOR/motor_def.h` | 新增 `Motor_Safety_Limit_s`，并在 `Motor_Init_Config_s` 中加入 `safety_limit_config`。 | 让各 APP 在注册电机时传入 raw/角度限位参数，但限位检查逻辑留在电机底层。 | `cmake --build build/dji_c/Debug --parallel 4` 通过。 |
| `modules/MOTOR/motor_base.h` / `modules/MOTOR/motor_base.c` | 在 `Motor_Base` 中保存安全限位、名称和原始位置读取回调；`Motor_ApplyAll()` 在调用驱动 `Apply()` 前执行底层限位检查；超限后锁存全局安全故障、失能全部电机、清零输出/PID 积分，并拒绝后续 `Motor_Start()` 重新使能。 | 保证不管上层如何调用 start/set/output，真正下发前都必须经过底层安全闸门；任一电机超限即全局急停。 | `cmake --build build/dji_c/Debug --parallel 4` 通过。 |
| `modules/MOTOR/DAMIAO/motor_damiao.h` / `modules/MOTOR/DAMIAO/motor_damiao.c` | 达妙反馈结构保存反馈包 16bit 原始位置值；初始化时接入安全限位配置和 raw 读取回调。 | 让达妙关节可直接使用实测 raw 值做底层限位，避免用角度反推造成调试偏差。 | `cmake --build build/dji_c/Debug --parallel 4` 通过。 |
| `modules/MOTOR/DJI/motor_dji.c` | GM6020/DJI 初始化时接入安全限位配置和 raw 读取回调，raw 值使用 `ecd`。 | 让 J4 GM6020 可使用 0~8191 编码器原始值做底层限位。 | `cmake --build build/dji_c/Debug --parallel 4` 通过。 |
| `modules/OFFLINE/module_offline.h` / `modules/OFFLINE/module_offline.c` | 新增 `Module_Offline_SetFatalFault()` 和 `Module_Offline_HasFatalFault()`；Offline 线程在 fatal fault 锁存后统一持续红灯和蜂鸣。 | 使用统一系统报警模块处理蜂鸣，避免电机底层直接操作蜂鸣器；本次不改变看门狗喂狗/复位逻辑。 | `cmake --build build/dji_c/Debug --parallel 4` 通过。 |

## 2026-10-02：底层限位预扫描和 raw 跨零处理

### 本轮主动修改

| 文件 | 修改内容 | 目的 | 验证 |
| --- | --- | --- | --- |
| `modules/MOTOR/motor_base.h` / `modules/MOTOR/motor_base.c` | 在 `Motor_Base` 中新增 `feedback_valid`；`Motor_ApplyAll()` 改为先对全部电机做安全限位预扫描，再开始任何 CAN/PWM/UART Apply；raw 限位支持跨编码器 0 点区间；限位在收到首帧有效反馈前不参与判定。 | 修复“前面电机已下发、后面电机才发现超限”的半周期风险；避免上电 raw 初始 0 在首帧反馈前误触发；支持 GM6020/达妙 raw 区间跨 0 点。 | `cmake --build build/dji_c/Debug --parallel 4` 通过。 |
| `modules/MOTOR/DAMIAO/motor_damiao.c` | 达妙接收回调在完成反馈解码后置 `base.feedback_valid=1`。 | 明确 raw/角度限位必须基于真实反馈帧，避免未收到反馈时误判。 | `cmake --build build/dji_c/Debug --parallel 4` 通过。 |
| `modules/MOTOR/DJI/motor_dji.c` | DJI/GM6020 接收回调在完成反馈解码后置 `base.feedback_valid=1`。 | 明确 raw/角度限位必须基于真实反馈帧，避免未收到反馈时误判。 | `cmake --build build/dji_c/Debug --parallel 4` 通过。 |

## 2026-10-02：具体驱动 Apply 入口增加单电机最终安全校验

### 本轮主动修改

| 文件 | 修改内容 | 目的 | 验证 |
| --- | --- | --- | --- |
| `modules/MOTOR/motor_base.h` / `modules/MOTOR/motor_base.c` | 新增 `Motor_SafetyCheckBeforeApply()`；在具体驱动真正编码 MIT/位置/速度/电流报文前，重复检查当前电机的反馈限位。若刚刚越限，只失能当前电机、清零当前输出，并返回失败状态。 | 在 `Motor_ApplyAll()` 全局预扫描之外增加单电机最后一道拦截，缩短“预扫描通过到具体电机发送”之间的窗口；保持实现简单，不重复重写各协议编码函数。 | `cmake --build build/dji_c/Debug --parallel 4` 通过。 |
| `modules/MOTOR/DAMIAO/motor_damiao.c` | `dm_apply()` 入口调用 `Motor_SafetyCheckBeforeApply()`；失败后进入已有零 MIT/位置/速度/PSI 输出分支。 | 保证达妙具体模式发送前，当前电机越限时不再发正常力矩/速度/位置指令。 | `cmake --build build/dji_c/Debug --parallel 4` 通过。 |
| `modules/MOTOR/DJI/motor_dji.c` | `dji_apply()` 入口调用 `Motor_SafetyCheckBeforeApply()`；失败后进入已有零电流帧分支。 | 保证 GM6020 具体控制帧发送前，当前电机越限时只发零电流，不继续输出正常电流。 | `cmake --build build/dji_c/Debug --parallel 4` 通过。 |

## 2026-10-02：机械臂角度限位来源整理

### 本轮主动修改

| 文件 | 修改内容 | 目的 | 验证 |
| --- | --- | --- | --- |
| `modules/MOTOR/motor_def.h` / `modules/MOTOR/motor_base.c` | 新增 `Motor_Safety_Angle_Source_e`，角度限位可明确选择单圈/协议角度或软件累计总角度；安全检查统一通过选择器读取角度。 | 让不跨圈关节使用 `single_round_angle`，J4/J6 跨圈关节使用 `total_angle`，不改变任何电机控制模式。 | `cmake --build build/dji_c/Debug --parallel 4` 通过。 |
| `apps/arm/arm_def.h` / `apps/arm/single_board/robot_control.c` | 写入 J2~J7 角度限位（单位 rad）：J2 `[-1.9,1.8]`、J3 `[0.2,5.53]`、J4 `[-4.5,9.0]`、J5 `[-1.6,1.6]`、J6 `[-5.2,7.0]`、J7 `[-2.0,2.0]`；J2/J3/J5/J7 使用单圈/协议角度，J4/J6 使用累计角度；J1 限位保持关闭。 | 将实测机械限位直接配置到安全层，避免把角度值误当成 raw 编码值。 | `cmake --build build/dji_c/Debug --parallel 4` 通过。 |
| `apps/arm/single_board/robot_control.c` | 将 J1 电机型号由 DM6220 改为 DM4310。 | 匹配底盘一号电机更换后的实物型号，避免速度/力矩编解码量程复用错误参数。 | `cmake --build build/dji_c/Debug --parallel 4` 通过。 |
| `modules/MOTOR/DAMIAO/motor_damiao.c` / `modules/MOTOR/DJI/motor_dji.c` | 首次收到有效反馈时只建立累计角度基线，不根据默认 0 值判断跨圈。 | 防止 J4/J6 上电第一帧因与默认值差异过大而误加/减一圈，导致累计角度限位误触发。 | `cmake --build build/dji_c/Debug --parallel 4` 通过。 |

## 后续记录格式

后续每次发生 App 层外修改时，在本文件末尾新增日期、提交、文件、目的、验证和来源；如果只修改 `apps/`，在交付说明中注明“无 App 层外修改”。

## 2026-10-09：增加 ARM 软件限位总开关

| 文件 | 修改内容 | 目的 | 风险与验证 |
| --- | --- | --- | --- |
| `apps/config.cmake` / `apps/arm/robot.cmake` / `apps/module_config.h.in` | 新增 `ARM_SOFTWARE_LIMIT_ENABLE` 配置宏；通用默认值为 `1`，当前 ARM 配置设为 `0`。 | 允许在 CMake 配置阶段统一关闭 ARM 软件角度/raw 限位及其由限位触发的失能，保留遥控器、掉线等其它安全路径。 | 当前 ARM 构建配置关闭软件限位；按用户要求未构建，仅做静态检查。 |
| `modules/MOTOR/motor_base.c` | 在全局限位预扫描和单电机最终限位检查入口增加宏旁路。 | 宏为 `0` 时不执行软件限位判断、不触发限位失能；宏为 `1` 时恢复原有限位逻辑。 | 未修改 J2/J3 角度换算；按用户要求未构建。 |

## 2026-10-02：离线检测线程初始化时序修复

### 本轮主动修改

| 文件 | 修改内容 | 目的 | 验证 |
| --- | --- | --- | --- |
| `modules/OFFLINE/module_offline.c` | 将看门狗/蜂鸣器和模块状态初始化放到 `TX_AUTO_START` 离线线程创建之前；在线程创建成功前设置 `g_initialized`，创建失败时回滚；并将初始化标志声明为 `volatile`。 | 避免自动启动线程在模块标志置位前运行并直接退出，确保离线检测、红灯和致命故障蜂鸣始终能够工作。 | 已完成 `cmake -S board/dji_c -B build/dji_c/Debug --preset Debug -G Ninja -DROBOT=arm -DBOARD=single` 配置，并执行清理后全量构建；构建通过。 |

## 2026-10-02：达妙初始化保持失能

### 本轮主动修改

| 文件 | 修改内容 | 目的 | 验证 |
| --- | --- | --- | --- |
| `modules/MOTOR/DAMIAO/motor_damiao.c` | `Motor_DM_Init()` 初始化阶段不再发送 `DM_CMD_MOTOR_START`；清错后改为重复发送 `DM_CMD_MOTOR_STOP`，并补充中文注释说明第一阶段只读反馈。 | 避免软件 `enableflag=0` 但达妙本体已进入内部使能态，导致上电后不同电机出现灯色/阻尼不一致；后续重力补偿阶段再设计独立使能流程。 | 已执行 `cmake --build build/dji_c/Debug --config Debug --parallel 4`，构建通过。 |

## 2026-10-02：致命故障报警增加闪灯

### 本轮主动修改

| 文件 | 修改内容 | 目的 | 验证 |
| --- | --- | --- | --- |
| `modules/OFFLINE/module_offline.c` | `g_fatal_fault` 锁存后不再红灯常亮，改为 100ms 周期红灯/灭灯闪烁；蜂鸣器仍保持持续报警。 | 让限位等安全锁存报警与普通静默离线红灯常亮区分开，便于现场快速识别安全故障。 | 已执行 `cmake --build build/dji_c/Debug --config Debug --parallel 4`，构建通过。 |

## 2026-10-04：重力补偿调试阶段输出力矩硬限幅

### 本轮主动修改

| 文件 | 修改内容 | 目的 | 验证 |
| --- | --- | --- | --- |
| `modules/MOTOR/motor_base.c` | 新增 `motor_limit_output_torque()`，当 `Motor_Base.info.max_torque > 0` 时，将 `controller.output_torque` 硬限制到 `[-max_torque, max_torque]`；在 `Motor_SetOutputTorque()` 写入力矩后和 `Motor_ApplyAll()` 调用具体驱动 `Apply()` 前均执行限幅。 | 在重力补偿探索阶段提供下发前的通用力矩保险，防止上层试参或未来控制路径写入过大力矩导致疯转或机械损伤；`max_torque <= 0` 保持旧行为。 | 已执行 `git diff --check`；完整构建当前卡在 Ninja/CMake `Re-checking globbed directories` 阶段，未进入源码编译。 |

## 2026-10-04：达妙硬件启停与软件使能同步

### 本轮主动修改

| 文件 | 修改内容 | 目的 | 验证 |
| --- | --- | --- | --- |
| `modules/MOTOR/DAMIAO/motor_damiao.h` | 在 `DM_Motor_t` 中新增 `hardware_start_sent` 状态字段。 | 为每一台达妙电机独立保存 START 命令是否已入队；达妙没有启动确认帧，因此该字段不冒充硬件 ACK，也避免用全局状态误启动其它电机。 | 已用 STM32F407 `board/dji_c` 的生成编译命令验证头文件可用。 |
| `modules/MOTOR/DAMIAO/motor_damiao.c` | 将 `DM_CMD_MOTOR_START/STOP` 同步逻辑放入已有 `dm_apply()`；仅当当前对象软件 `enableflag=1`、反馈有效、在线且安全检查通过时发送一次 START；失能、掉线、反馈无效或安全故障时发送一次 STOP；START 所在周期不发送正常控制帧。 | 让 `Motor_Start()` 保持软件层职责，由现有电机调度线程完成逐电机硬件启停；保证 J5 的启动不会遍历或带起 J2/J3/J6/J7 等其它电机。 | 已直接编译 `motor_damiao.c` 和 arm APP 源文件；正常 Ninja 全量构建仍卡在 `Re-checking globbed directories`，未以该命令作为通过依据。 |

## 2026-10-10：移植 LoRa 模块并接入机械臂发送端

### 本轮主动修改

| 文件 | 修改内容 | 目的 | 风险与验证 |
| --- | --- | --- | --- |
| `modules/LORA/module_lora.c` / `modules/LORA/module_lora.h` / `modules/LORA/README.md` | 从参考工程移植塔石 L33 LoRa 模块，保留注册表、ThreadX 线程、小端序序列化、`AA 55 LEN SEQ PAYLOAD CRC8` 帧协议和收发能力；新增 `LORA_RX_ENABLE` 以允许当前发送端关闭接收解析。 | 以固定周期发送 APP 注册的持久化变量，并保持与重装机器人接收端协议一致；当前 ARM 配置仅发送，不让接收数据覆盖发送数组。 | 模块独占一个 UART；DMA 发送缓冲区必须位于 DMA 可访问区；两端串口参数、空中速率和信道必须一致。新增 C/H 的 clang-format 检查与 `git diff --check` 均通过；按要求未构建。 |
| `modules/CMakeLists.txt` / `modules/module_config.cmake` / `modules/module_init.c` | 增加 LORA 条件编译源、默认参数和框架初始化入口。 | 让模块仍按仓库现有的模块开关机制构建和初始化，未启用 LORA 的其它机器人保持原行为。 | 若启用 LORA 却未配置 UART 或 M0/M1 GPIO，配置生成后会编译失败；当前 ARM 已显式配置，模块开关、生成宏、条件源和初始化入口已完成静态闭环检查；按要求未构建。 |

本轮未修改 `board/**`、CAN 或电机调度文件。USART1（PA9 TX、PB7 RX）及其 DMA 已由现有 `board/dji_c` 配置提供；当前 ARM 未启用会占用 USART1 的图传、裁判系统或 WT606。ARM APP 将 LoRa M0/M1 设为 PE13/PE9，运行时会把这两个原 TIM1 通道引脚改为普通 GPIO，因此不可再同时作为 PWM 使用。

## 2026-10-11：LoRa 发送端 USART1 波特率闭环和发送诊断

| 文件 | 修改内容 | 目的 | 风险与验证 |
| --- | --- | --- | --- |
| `board/dji_c/Src/usart.c`、`board/dji_c/base.ioc` | 将 USART1 板级初始波特率从 921600 改为 LoRa 目标值 115200。 | 消除板级初始化与 LoRa 模块运行配置不一致，避免发送端仍以旧波特率输出导致接收端直连只能收到 `FE/FF` 类乱码。 | USART1 当前由 ARM LoRa 独占；其他启用 USART1 的机器人需复核自己的串口配置。按项目规则未构建。 |
| `modules/LORA/module_lora.c` | 增加发送尝试、成功、失败计数，并每秒打印 UART 波特率、BRR、状态、错误码及 TX DMA 状态/NDTR。 | 区分未启动、DMA 发送失败和实际完成发送，支持直连现场定位。 | 只增加低频 RTT 日志和计数，不改变帧格式、发送周期或载荷。按项目规则未构建。 |

## 2026-10-11：LoRa 发送端迁移到 USART6

| 文件 | 修改内容 | 目的 | 风险与验证 |
| --- | --- | --- | --- |
| `apps/arm/robot.cmake` | 将 ARM LoRa 的 UART 绑定从 `huart1` 改为 `huart6`，对应 `PG14(TX)` / `PG9(RX)`。 | 改用发送板 USART6 输出 LoRa 数据。 | ARM 当前未启用同样使用 `huart6` 的 VOFA、图传或裁判模块；LoRa 仍为 115200 8N1、TX DMA 发送。按项目规则未构建。 |
| `board/dji_c/Src/usart.c`、`board/dji_c/base.ioc` | USART1 恢复为迁移 LoRa 前的 921600；USART6 保持已有的 115200 8N1 和 DMA 配置。 | LoRa 已不再使用 USART1，撤销此前为 LoRa 增加的板级 USART1 全局改动，避免影响未来 USART1 使用者。 | USART6 TX 使用 DMA2 Stream6；仅做静态检查，按项目规则未构建。 |

## 2026-10-11：清理 LoRa 发送端排障日志

| 文件 | 修改内容 | 目的 | 风险与验证 |
| --- | --- | --- | --- |
| `modules/LORA/module_lora.c` | 删除 TX 尝试/成功/失败计数、每秒 UART/DMA 诊断日志及详细启动诊断；保留 DMA 可访问发送缓冲区、帧协议、发送失败返回值和 USART6 配置。 | 无线收发链路验证完成后移除临时排障负担，为后续方向修正保留干净运行路径。 | 不改变 37 字节帧、CRC8、序号、50 ms 周期或载荷顺序；按项目规则未构建。 |

## 2026-10-15：发送端由 LoRa 替换为 NRF24L01

| 文件 | 修改内容 | 目的 | 风险与验证 |
| --- | --- | --- | --- |
| `apps/arm/single_board/arm_lora.c` / `arm_lora.h`、`robot_control.c` / `.h` | 删除 LoRa 应用注册与更新路径，新增 `arm_nrf24l01.c` / `.h`，注册 8 个 `float` 关节/工具字段并在控制周期刷新。 | 将机械臂发送端应用切换到 NRF24L01 注册式模块，保持 8×float=32 字节的载荷规模。 | 注册顺序与接收端必须一致；静态检查通过。 |
| `apps/arm/robot.cmake`、`modules/CMakeLists.txt`、`modules/module_config.cmake`、`modules/module_init.c` | 启用 `NRF24L01`，设置 TX-only、10 ms 周期和统一 5 字节地址，补齐模块源文件、配置宏和初始化入口。 | 按现有 ThreadX 模块框架接入 NRF24L01，避免继续走独立 LoRa 路径。 | `NRF24L01_TX_ENABLE=1`、`NRF24L01_RX_ENABLE=0` 已形成配置闭环；未执行完整构建。 |
| `modules/NRF24L01/module_nrf24l01.c` / `.h`、`README.md` | 采用模块默认 CE/CSN/IRQ：dji_c 为 PF0/PB12/PF1；SPI2 PB13/PB14/PB15 与 DMA 仍由板级配置提供；README 同步说明 TX-only 可不接 IRQ、RX 必须接。 | 明确模块层与板级层职责，优先沿用现有 dji_c 默认定义，不强行改用 PB7/PC7/PA7/PC1。 | 已完成 `git diff --check` 前的静态内容核对；单文件编译检查曾通过，按本轮要求不再构建。 |
| `board/dji_c/Inc/spi.h`、`Src/spi.c`、`Src/dma.c`、`Src/stm32f4xx_it.c`、`base.ioc` | 保留/同步 SPI2 DMA RX/TX（Stream3/4，Channel 0）及对应中断和句柄链接。 | 为 NRF24L01 模块提供 HAL/BSP DMA SPI2 底层。 | 仅做静态配置检查；未执行完整构建。 |

## 2026-10-15：增加 NRF24L01 通信与发送诊断日志

| 文件 | 修改内容 | 目的 | 风险与验证 |
| --- | --- | --- | --- |
| `modules/NRF24L01/module_nrf24l01.c` | 初始化后回读 `CONFIG`、`RF_CH`、`FEATURE` 并打印 DMA 句柄状态；首次有载荷时打印 `TX started`；首次发送结果打印 ACK/无 ACK；成功和失败累计统计按低频周期打印。 | 区分 MCU 与 NRF24L01 SPI 寄存器通信失败、发送线程未启动、以及 NRF 无法收到对端 ACK。 | 不改变无线协议、发送周期或寄存器配置；已通过 `git diff --check`，按要求未构建。 |
