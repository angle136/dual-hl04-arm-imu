#include "robot_control.h"

#include "arm_dual_imu.h"
#include "bsp_def.h"

#if (ARM_IMU_DEBUG_ENABLE != 0)
#include "arm_imu_debug.h"
#endif

#define LOG_TAG "app_arm_imu"
#define LOG_LVL LOG_LVL_INFO
#include "ulog_def.h"

#if (ARM_DUAL_IMU_ENABLE != 0)
void robot_control_init(void)
{
    arm_dual_imu_init();

#if (ARM_IMU_DEBUG_ENABLE != 0)
    arm_imu_debug_init();
#endif

    LOG_I("dual HL04 arm application initialized");
}
#else
void robot_control_init(void) { LOG_W("dual IMU arm application disabled by configuration"); }
#endif
