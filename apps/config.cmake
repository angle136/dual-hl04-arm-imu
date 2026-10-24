# 机器人 & 板型 配置
# 默认值在这里改；也可用 -DROBOT=xxx -DBOARD=xxx 覆盖
# 注意：已配置过的 build 目录以缓存值为准，改本文件默认值不影响旧 build 目录

# 目标机器人 & 板型
set(ROBOT "arm" CACHE STRING "Target robot")
set_property(CACHE ROBOT PROPERTY STRINGS arm arm_dual_imu)
set(BOARD "single" CACHE STRING "Board role")
set_property(CACHE BOARD PROPERTY STRINGS single gimbal chassis)

# 板型校验
if(NOT BOARD MATCHES "^(single|gimbal|chassis)$")
    message(FATAL_ERROR "Unknown BOARD '${BOARD}', expected: single / gimbal / chassis")
endif()

# 加载默认模块配置
include(${CMAKE_CURRENT_LIST_DIR}/../modules/module_config.cmake)

# ARM 重力补偿遥控安全门默认关闭；apps/<robot>/robot.cmake 可按目标机器人覆盖。
set(ARM_GRAVITY_REMOTE_GUARD_ENABLE 0)

# ARM 软件角度/raw 限位默认开启；apps/<robot>/robot.cmake 可按目标机器人覆盖。
set(ARM_SOFTWARE_LIMIT_ENABLE 1)

# 双 IMU 机械臂 APP 参数默认关闭；apps/arm_dual_imu/robot.cmake 覆盖。
set(ARM_DUAL_IMU_ENABLE             0)
set(ARM_DUAL_IMU_UART               huart1)
set(ARM_DUAL_IMU_UART_BAUD          921600)
set(ARM_DUAL_IMU_TASK_STACK_SIZE    2048)
set(ARM_DUAL_IMU_TASK_PRIORITY      11)
set(ARM_DUAL_IMU_READ_TIMEOUT_MS    2)
set(ARM_DUAL_IMU_UPPER_TIMEOUT_MS   100)
set(ARM_DUAL_IMU_AUTO_ZERO_ENABLE   0)
set(ARM_DUAL_IMU_AUTO_ZERO_DELAY_MS 1000)

# 加载机器人差异配置
include(${CMAKE_CURRENT_LIST_DIR}/${ROBOT}/robot.cmake)

# 派生变量
string(TOUPPER ${ROBOT} ROBOT_UPPER)
string(TOUPPER ${BOARD} BOARD_UPPER)

message(STATUS "Robot: ${ROBOT}")
message(STATUS "Board: ${BOARD}")

# 板型宏（只有一个为 1）
set(SINGLE_BOARD  0)
set(GIMBAL_BOARD  0)
set(CHASSIS_BOARD 0)
set(${BOARD_UPPER}_BOARD 1)

# 模块开关
foreach(_m OFFLINE REMOTE BMI088 INS REFEREE SUPERCAP WT606 MOTOR VISION BOARDCOMM LORA VOFA NRF24L01)
    set(MODULE_${_m} 0)
endforeach()

set(_enabled ${MODULES_${BOARD_UPPER}})
foreach(_m ${_enabled})
    set(MODULE_${_m} 1)
endforeach()
