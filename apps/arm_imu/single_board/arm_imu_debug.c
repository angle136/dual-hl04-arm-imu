#include "bsp_def.h"

#if (ARM_IMU_DEBUG_ENABLE != 0)

#include "arm_imu_debug.h"

#include "arm_dual_imu.h"
#include "tx_api.h"

#include <math.h>
#include <stdint.h>

#define LOG_TAG "arm_imu_debug"
#define LOG_LVL LOG_LVL_INFO
#include "ulog_def.h"

#define ARM_IMU_DEBUG_STREAM_HZ           50U
#define ARM_IMU_DEBUG_STREAM_PERIOD_TICKS (TX_TIMER_TICKS_PER_SECOND / ARM_IMU_DEBUG_STREAM_HZ)
#define ARM_IMU_DEBUG_STATUS_PERIOD_TICKS TX_TIMER_TICKS_PER_SECOND
#define ARM_IMU_DEBUG_QUATERNION_SCALE    1000000.0f
#define ARM_IMU_DEBUG_TASK_STACK_SIZE     1024U
#define ARM_IMU_DEBUG_TASK_PRIORITY       20U

static TX_THREAD                  g_arm_imu_debug_thread;
APPS_STACK_SECTION static uint8_t g_arm_imu_debug_stack[ARM_IMU_DEBUG_TASK_STACK_SIZE];

static float arm_imu_debug_clampf(float value, float minimum, float maximum)
{
    if (value < minimum) return minimum;
    if (value > maximum) return maximum;
    return value;
}

static int32_t arm_imu_debug_quaternion_to_fixed(float value)
{
    if (!isfinite(value)) return 0;
    return (int32_t)(arm_imu_debug_clampf(value, -1.0f, 1.0f) * ARM_IMU_DEBUG_QUATERNION_SCALE);
}

static void arm_imu_debug_log_status(void)
{
    Arm_Dual_Imu_State_t state;
    if (arm_dual_imu_get_state(&state) == 0U)
    {
        LOG_W("snapshot busy");
        return;
    }

    LOG_I("valid upper=%u forearm=%u cal=%u pose=%u ik=%u", (unsigned)state.upper_imu_valid, (unsigned)state.forearm_imu_valid,
          (unsigned)state.calibrated, (unsigned)state.pose_valid, (unsigned)state.ik_valid);
    LOG_I("rx uart[pkt=%lu frame=%lu] can[pkt=%lu frame=%lu]", (unsigned long)state.upper_packet_count, (unsigned long)state.upper_frame_error_count,
          (unsigned long)state.forearm_packet_count, (unsigned long)state.forearm_frame_error_count);
    LOG_I("target p=[%.3f %.3f %.3f] q=[%.3f %.3f %.3f %.3f %.3f %.3f]", state.target_position_m[0], state.target_position_m[1],
          state.target_position_m[2], state.legacy_joint_target_rad[0], state.legacy_joint_target_rad[1], state.legacy_joint_target_rad[2],
          state.legacy_joint_target_rad[3], state.legacy_joint_target_rad[4], state.legacy_joint_target_rad[5]);
}

static void arm_imu_debug_log_matlab_pose(void)
{
    Arm_Dual_Imu_State_t state;
    if (arm_dual_imu_get_state(&state) == 0U) return;

    int32_t upper_quaternion[4];
    int32_t forearm_quaternion[4];
    for (uint8_t i = 0U; i < 4U; ++i)
    {
        upper_quaternion[i]   = arm_imu_debug_quaternion_to_fixed(state.upper_quaternion[i]);
        forearm_quaternion[i] = arm_imu_debug_quaternion_to_fixed(state.forearm_quaternion[i]);
    }

    LOG_RAW("ARMPOSE,%lu,%u,%u,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld\r\n", (unsigned long)state.sequence, (unsigned)state.upper_imu_valid,
            (unsigned)state.forearm_imu_valid, (long)upper_quaternion[0], (long)upper_quaternion[1], (long)upper_quaternion[2],
            (long)upper_quaternion[3], (long)forearm_quaternion[0], (long)forearm_quaternion[1], (long)forearm_quaternion[2],
            (long)forearm_quaternion[3]);
}

static void arm_imu_debug_task_entry(ULONG thread_input)
{
    (void)thread_input;
    ULONG status_elapsed_ticks = ARM_IMU_DEBUG_STATUS_PERIOD_TICKS;

    while (1)
    {
        arm_imu_debug_log_matlab_pose();
        if (status_elapsed_ticks >= ARM_IMU_DEBUG_STATUS_PERIOD_TICKS)
        {
            arm_imu_debug_log_status();
            status_elapsed_ticks = 0U;
        }
        tx_thread_sleep(ARM_IMU_DEBUG_STREAM_PERIOD_TICKS);
        status_elapsed_ticks += ARM_IMU_DEBUG_STREAM_PERIOD_TICKS;
    }
}

void arm_imu_debug_init(void)
{
    UINT status =
        tx_thread_create(&g_arm_imu_debug_thread, "arm_imu_debug", arm_imu_debug_task_entry, 0U, g_arm_imu_debug_stack, sizeof(g_arm_imu_debug_stack),
                         ARM_IMU_DEBUG_TASK_PRIORITY, ARM_IMU_DEBUG_TASK_PRIORITY, TX_NO_TIME_SLICE, TX_AUTO_START);
    if (status != TX_SUCCESS) LOG_E("debug thread create failed: %u", (unsigned)status);
}

#endif /* ARM_IMU_DEBUG_ENABLE */
