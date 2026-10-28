#include "bsp_def.h"

#if (ARM_IMU_DEBUG_ENABLE != 0)

#include "arm_imu_debug.h"

#include "arm_dual_imu.h"
#include "tx_api.h"

#if (ARM_IMU_DEBUG_RTT_ENABLE != 0)
#include "SEGGER_RTT.h"
#endif

#include <math.h>
#include <stdint.h>
#include <string.h>

#define LOG_TAG "arm_imu_debug"
#define LOG_LVL LOG_LVL_INFO
#include "ulog_def.h"

#define ARM_IMU_DEBUG_STREAM_HZ           50U
#define ARM_IMU_DEBUG_STREAM_PERIOD_TICKS (TX_TIMER_TICKS_PER_SECOND / ARM_IMU_DEBUG_STREAM_HZ)
#define ARM_IMU_DEBUG_STATUS_PERIOD_TICKS TX_TIMER_TICKS_PER_SECOND
#define ARM_IMU_DEBUG_QUATERNION_SCALE    1000000.0f
#define ARM_IMU_DEBUG_TASK_STACK_SIZE     1024U
#define ARM_IMU_DEBUG_TASK_PRIORITY       20U
#define ARM_IMU_DEBUG_FRAME_HEADER        0xAAU
#define ARM_IMU_DEBUG_FRAME_PAYLOAD_SIZE  32U
#define ARM_IMU_DEBUG_FRAME_SIZE          36U
#define ARM_IMU_DEBUG_FRAME_TAIL          0xFFU

static TX_THREAD                  g_arm_imu_debug_thread;
APPS_STACK_SECTION static uint8_t g_arm_imu_debug_stack[ARM_IMU_DEBUG_TASK_STACK_SIZE];

#if (ARM_IMU_DEBUG_RTT_ENABLE != 0)
static uint8_t g_arm_imu_debug_rtt_buffer[ARM_IMU_DEBUG_RTT_BUFFER_SIZE];
static uint8_t g_arm_imu_debug_frame[ARM_IMU_DEBUG_FRAME_SIZE];
static float   g_arm_imu_debug_last_valid_joint[ARM_DUAL_IMU_IK_DOF];
static uint8_t g_arm_imu_debug_sequence;
static uint8_t g_arm_imu_debug_rtt_ready;
static uint8_t g_arm_imu_debug_have_valid_joint;
#endif

/**
 * @brief 将调试浮点值限制在指定闭区间内。
 * @param value 输入值。
 * @param minimum 区间下限。
 * @param maximum 区间上限。
 * @return 限幅后的浮点数。
 * @note 调用关系：由四元数定点化函数调用，保证输出范围为 [-1, 1]。
 */
static float arm_imu_debug_clampf(float value, float minimum, float maximum)
{
    if (value < minimum) return minimum;
    if (value > maximum) return maximum;
    return value;
}

/**
 * @brief 将四元数分量转换为 MATLAB 文本协议使用的定点整数。
 * @param value 输入四元数分量。
 * @return 放大 ARM_IMU_DEBUG_QUATERNION_SCALE 倍后的定点值；非有限输入返回 0。
 * @note 调用关系：由 arm_imu_debug_log_matlab_pose() 逐分量调用。
 */
static int32_t arm_imu_debug_quaternion_to_fixed(float value)
{
    if (!isfinite(value)) return 0;
    return (int32_t)(arm_imu_debug_clampf(value, -1.0f, 1.0f) * ARM_IMU_DEBUG_QUATERNION_SCALE);
}

/**
 * @brief 将三阶旋转矩阵转换为单位四元数。
 * @param rotation 输入 3x3 旋转矩阵。
 * @param quaternion 输出四元数，顺序为 w、x、y、z。
 * @retval 无。
 * @note 调用关系：由 MATLAB 位姿日志函数在相对姿态有效时调用；退化输入输出单位四元数。
 */
static void arm_imu_debug_rotation_to_quaternion(const float rotation[3][3], float quaternion[4])
{
    float trace = rotation[0][0] + rotation[1][1] + rotation[2][2];

    if (trace > 0.0f)
    {
        float scale   = 2.0f * sqrtf(trace + 1.0f);
        quaternion[0] = 0.25f * scale;
        quaternion[1] = (rotation[2][1] - rotation[1][2]) / scale;
        quaternion[2] = (rotation[0][2] - rotation[2][0]) / scale;
        quaternion[3] = (rotation[1][0] - rotation[0][1]) / scale;
    }
    else if (rotation[0][0] > rotation[1][1] && rotation[0][0] > rotation[2][2])
    {
        float scale   = 2.0f * sqrtf(1.0f + rotation[0][0] - rotation[1][1] - rotation[2][2]);
        quaternion[0] = (rotation[2][1] - rotation[1][2]) / scale;
        quaternion[1] = 0.25f * scale;
        quaternion[2] = (rotation[0][1] + rotation[1][0]) / scale;
        quaternion[3] = (rotation[0][2] + rotation[2][0]) / scale;
    }
    else if (rotation[1][1] > rotation[2][2])
    {
        float scale   = 2.0f * sqrtf(1.0f + rotation[1][1] - rotation[0][0] - rotation[2][2]);
        quaternion[0] = (rotation[0][2] - rotation[2][0]) / scale;
        quaternion[1] = (rotation[0][1] + rotation[1][0]) / scale;
        quaternion[2] = 0.25f * scale;
        quaternion[3] = (rotation[1][2] + rotation[2][1]) / scale;
    }
    else
    {
        float scale   = 2.0f * sqrtf(1.0f + rotation[2][2] - rotation[0][0] - rotation[1][1]);
        quaternion[0] = (rotation[1][0] - rotation[0][1]) / scale;
        quaternion[1] = (rotation[0][2] + rotation[2][0]) / scale;
        quaternion[2] = (rotation[1][2] + rotation[2][1]) / scale;
        quaternion[3] = 0.25f * scale;
    }

    float norm = sqrtf(quaternion[0] * quaternion[0] + quaternion[1] * quaternion[1] + quaternion[2] * quaternion[2] + quaternion[3] * quaternion[3]);
    if (!isfinite(norm) || norm < 1.0e-6f)
    {
        quaternion[0] = 1.0f;
        quaternion[1] = 0.0f;
        quaternion[2] = 0.0f;
        quaternion[3] = 0.0f;
        return;
    }
    for (uint8_t i = 0U; i < 4U; ++i) quaternion[i] /= norm;
}

/**
 * @brief 输出双 IMU 有效性、收包统计、末端位置和逆解角摘要。
 * @param state 输入一致的双 IMU 状态快照。
 * @retval 无。
 * @note 调用关系：由调试线程按 ARM_IMU_DEBUG_STATUS_PERIOD_TICKS 周期调用。
 */
static void arm_imu_debug_log_status(const Arm_Dual_Imu_State_t *state)
{
    LOG_I("valid upper=%u forearm=%u cal=%u zero=%lu pose=%u ik=%u", (unsigned)state->upper_imu_valid, (unsigned)state->forearm_imu_valid,
          (unsigned)state->calibrated, (unsigned long)state->zero_count, (unsigned)state->pose_valid, (unsigned)state->ik_valid);
    LOG_I("rx uart[pkt=%lu frame=%lu] can[pkt=%lu frame=%lu]", (unsigned long)state->upper_packet_count,
          (unsigned long)state->upper_frame_error_count, (unsigned long)state->forearm_packet_count, (unsigned long)state->forearm_frame_error_count);
    LOG_I("target p=[%.3f %.3f %.3f] q=[%.3f %.3f %.3f %.3f %.3f %.3f]", state->target_position_m[0], state->target_position_m[1],
          state->target_position_m[2], state->legacy_joint_target_rad[0], state->legacy_joint_target_rad[1], state->legacy_joint_target_rad[2],
          state->legacy_joint_target_rad[3], state->legacy_joint_target_rad[4], state->legacy_joint_target_rad[5]);
}

/**
 * @brief 输出 MATLAB 可解析的 ARMPOSE 文本帧。
 * @param state 输入一致的双 IMU 状态快照。
 * @retval 无。
 * @note 调用关系：由调试线程按流输出周期调用；优先输出校准后的相对旋转，位姿无效时退回传感器原始四元数。
 */
static void arm_imu_debug_log_matlab_pose(const Arm_Dual_Imu_State_t *state)
{
    float upper_pose_quaternion[4];
    float forearm_pose_quaternion[4];
    if (state->pose_valid != 0U)
    {
        arm_imu_debug_rotation_to_quaternion(state->upper_rotation, upper_pose_quaternion);
        arm_imu_debug_rotation_to_quaternion(state->forearm_rotation, forearm_pose_quaternion);
    }
    else
    {
        for (uint8_t i = 0U; i < 4U; ++i)
        {
            upper_pose_quaternion[i]   = state->upper_quaternion[i];
            forearm_pose_quaternion[i] = state->forearm_quaternion[i];
        }
    }

    int32_t upper_quaternion[4];
    int32_t forearm_quaternion[4];
    for (uint8_t i = 0U; i < 4U; ++i)
    {
        upper_quaternion[i]   = arm_imu_debug_quaternion_to_fixed(upper_pose_quaternion[i]);
        forearm_quaternion[i] = arm_imu_debug_quaternion_to_fixed(forearm_pose_quaternion[i]);
    }

    LOG_RAW("ARMPOSE,%lu,%u,%u,%u,%lu,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld\r\n", (unsigned long)state->sequence, (unsigned)state->upper_imu_valid,
            (unsigned)state->forearm_imu_valid, (unsigned)state->calibrated, (unsigned long)state->zero_count, (long)upper_quaternion[0],
            (long)upper_quaternion[1], (long)upper_quaternion[2], (long)upper_quaternion[3], (long)forearm_quaternion[0], (long)forearm_quaternion[1],
            (long)forearm_quaternion[2], (long)forearm_quaternion[3]);
}

#if (ARM_IMU_DEBUG_RTT_ENABLE != 0)
/**
 * @brief 通过 SEGGER RTT 发布兼容板间协议格式的六轴逆解帧。
 * @param state 输入一致的双 IMU 状态快照。
 * @retval 无。
 * @note 调用关系：由调试线程在 RTT 启用时调用；逆解暂时无效时保持并发送最后一个有效关节目标。
 */
static void arm_imu_debug_stream_joint_frame(const Arm_Dual_Imu_State_t *state)
{
    if (g_arm_imu_debug_rtt_ready == 0U) return;

    if (state->ik_valid != 0U)
    {
        memcpy(g_arm_imu_debug_last_valid_joint, state->legacy_joint_target_rad, sizeof(g_arm_imu_debug_last_valid_joint));
        g_arm_imu_debug_have_valid_joint = 1U;
    }
    if (state->calibrated == 0U || g_arm_imu_debug_have_valid_joint == 0U) return;

    float payload[8] = {0.0f};
    memcpy(payload, g_arm_imu_debug_last_valid_joint, sizeof(g_arm_imu_debug_last_valid_joint));

    g_arm_imu_debug_frame[0] = ARM_IMU_DEBUG_FRAME_HEADER;
    g_arm_imu_debug_frame[1] = ARM_IMU_DEBUG_FRAME_PAYLOAD_SIZE;
    g_arm_imu_debug_frame[2] = g_arm_imu_debug_sequence++;
    memcpy(&g_arm_imu_debug_frame[3], payload, sizeof(payload));
    g_arm_imu_debug_frame[ARM_IMU_DEBUG_FRAME_SIZE - 1U] = ARM_IMU_DEBUG_FRAME_TAIL;

    (void)SEGGER_RTT_Write(ARM_IMU_DEBUG_RTT_CHANNEL, g_arm_imu_debug_frame, sizeof(g_arm_imu_debug_frame));
}
#endif

/**
 * @brief 双 IMU 调试与遥测线程入口。
 * @param thread_input ThreadX 线程入口参数，当前未使用。
 * @retval 无，线程启动后持续运行。
 * @note 调用关系：由 arm_imu_debug_init() 创建；每周期读取一次一致快照，并输出 MATLAB 文本、可选 RTT 帧和低频状态日志。
 */
static void arm_imu_debug_task_entry(ULONG thread_input)
{
    (void)thread_input;
    ULONG                status_elapsed_ticks = ARM_IMU_DEBUG_STATUS_PERIOD_TICKS;
    Arm_Dual_Imu_State_t state;

    while (1)
    {
        uint8_t snapshot_valid = arm_dual_imu_get_state(&state);
        if (snapshot_valid != 0U)
        {
            arm_imu_debug_log_matlab_pose(&state);
#if (ARM_IMU_DEBUG_RTT_ENABLE != 0)
            arm_imu_debug_stream_joint_frame(&state);
#endif
        }
        if (status_elapsed_ticks >= ARM_IMU_DEBUG_STATUS_PERIOD_TICKS)
        {
            if (snapshot_valid != 0U)
                arm_imu_debug_log_status(&state);
            else
                LOG_W("snapshot busy");
            status_elapsed_ticks = 0U;
        }
        tx_thread_sleep(ARM_IMU_DEBUG_STREAM_PERIOD_TICKS);
        status_elapsed_ticks += ARM_IMU_DEBUG_STREAM_PERIOD_TICKS;
    }
}

/**
 * @brief 初始化可选 RTT 上行缓冲区并启动双 IMU 调试线程。
 * @param 无。
 * @retval 无。
 * @note 调用关系：由 robot_control_init() 在双 IMU 主模块初始化后调用；线程创建失败只记录错误日志。
 */
void arm_imu_debug_init(void)
{
#if (ARM_IMU_DEBUG_RTT_ENABLE != 0)
    int rtt_result = SEGGER_RTT_ConfigUpBuffer(ARM_IMU_DEBUG_RTT_CHANNEL, "ArmIk", g_arm_imu_debug_rtt_buffer, sizeof(g_arm_imu_debug_rtt_buffer),
                                               SEGGER_RTT_MODE_NO_BLOCK_SKIP);
    g_arm_imu_debug_rtt_ready = (rtt_result == 0) ? 1U : 0U;
    if (rtt_result != 0) LOG_E("joint RTT channel init failed: channel=%u result=%d", (unsigned)ARM_IMU_DEBUG_RTT_CHANNEL, rtt_result);
#endif

    UINT status =
        tx_thread_create(&g_arm_imu_debug_thread, "arm_imu_debug", arm_imu_debug_task_entry, 0U, g_arm_imu_debug_stack, sizeof(g_arm_imu_debug_stack),
                         ARM_IMU_DEBUG_TASK_PRIORITY, ARM_IMU_DEBUG_TASK_PRIORITY, TX_NO_TIME_SLICE, TX_AUTO_START);
    if (status != TX_SUCCESS) LOG_E("debug thread create failed: %u", (unsigned)status);
}

#endif /* ARM_IMU_DEBUG_ENABLE */
