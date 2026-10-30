#include "dual_imu.h"

#include "hl04.h"
#include "imu_math.h"
#include "onboard_imu.h"
#include "pose_solver.h"
#include "bsp_def.h"
#include "cmsis_compiler.h"
#include "tx_api.h"

#include <stddef.h>
#include <string.h>

#define LOG_TAG "arm_imu"
#define LOG_LVL LOG_LVL_INFO
#include "ulog_def.h"

typedef struct
{
    Arm_Dual_Imu_State_t state;
    Arm_Pose_Solver_t    pose_solver;
    uint32_t             all_ready_since_tick;
    volatile uint8_t     zero_requested;
} Arm_Dual_Imu_Context_t;

static TX_THREAD                  g_arm_dual_imu_thread;
APPS_STACK_SECTION static uint8_t g_arm_dual_imu_stack[ARM_DUAL_IMU_TASK_STACK_SIZE];
static Arm_Dual_Imu_Context_t     g_arm_dual_imu_context;

volatile Arm_Dual_Imu_State_t g_arm_dual_imu_state;

/**
 * @brief 以序列锁方式发布完整三 IMU 解算状态。
 * @param state 输入待发布状态。
 * @retval 无。
 * @note 调用关系：由 arm_dual_imu_calculate() 的提前退出路径和正常解算末尾调用。
 */
static void arm_dual_imu_publish(const Arm_Dual_Imu_State_t *state)
{
    const size_t payload_offset   = offsetof(Arm_Dual_Imu_State_t, initialized);
    uint32_t     odd_sequence     = (g_arm_dual_imu_state.sequence + 1U) | 1U;
    g_arm_dual_imu_state.sequence = odd_sequence;
    __DMB();
    memcpy((uint8_t *)(void *)&g_arm_dual_imu_state + payload_offset, (const uint8_t *)state + payload_offset, sizeof(*state) - payload_offset);
    __DMB();
    g_arm_dual_imu_state.sequence = odd_sequence + 1U;
}

/**
 * @brief 在三路 IMU 都有效时处理自动和手动标零请求。
 * @param context 输入输出三 IMU 上下文。
 * @param now 当前 ThreadX 时基。
 * @param upper_raw 输入大臂 IMU 当前原始旋转矩阵。
 * @param forearm_raw 输入前臂 IMU 当前原始旋转矩阵。
 * @param hand_raw 输入手背 BMI088 当前原始旋转矩阵。
 * @retval 无。
 * @note 调用关系：由 arm_dual_imu_calculate() 在完成三路四元数校验后调用。
 */
static void arm_dual_imu_update_zero(Arm_Dual_Imu_Context_t *context, ULONG now, const float upper_raw[3][3], const float forearm_raw[3][3],
                                     const float hand_raw[3][3])
{
    Arm_Dual_Imu_State_t *state = &context->state;
    if (state->upper_imu_valid != 0U && state->forearm_imu_valid != 0U && state->hand_imu_valid != 0U)
    {
        if (context->all_ready_since_tick == 0U) context->all_ready_since_tick = now;
#if (ARM_DUAL_IMU_AUTO_ZERO_ENABLE != 0)
        if (context->pose_solver.calibrated == 0U && (now - context->all_ready_since_tick) >= ARM_DUAL_IMU_AUTO_ZERO_DELAY_MS)
        {
            context->zero_requested = 1U;
        }
#endif
        if (context->zero_requested != 0U)
        {
            arm_pose_solver_capture_zero(&context->pose_solver, upper_raw, forearm_raw, hand_raw);
            context->zero_requested = 0U;
            LOG_I("three-IMU neutral pose captured, count=%lu", (unsigned long)context->pose_solver.zero_count);
        }
    }
    else
    {
        context->all_ready_since_tick = 0U;
    }
}

/**
 * @brief 从三路最新 IMU 数据计算相对姿态、三段位置链和六轴解析逆解。
 * @param context 输入输出三 IMU 上下文，保存标零请求、求解器和发布状态。
 * @retval 无。
 * @note 调用关系：由手臂 IMU 工作线程每轮调用；任一路数据超时或未校准时仅发布有效性状态，不执行位姿和逆解。
 */
static void arm_dual_imu_calculate(Arm_Dual_Imu_Context_t *context)
{
    Arm_Dual_Imu_State_t  *state = &context->state;
    const ULONG            now   = tx_time_get();
    Arm_Hl04_Uart_Data_t   upper_data;
    Arm_Hl04_Can_Data_t    forearm_data;
    Arm_Onboard_Imu_Data_t hand_data;
    Arm_Hl04_Stats_t       stats;
    float                  upper_raw[3][3];
    float                  forearm_raw[3][3];
    float                  hand_raw[3][3];

    uint8_t upper_snapshot_valid   = arm_hl04_get_upper(&upper_data);
    uint8_t forearm_snapshot_valid = arm_hl04_get_forearm(&forearm_data);
    uint8_t hand_snapshot_valid    = arm_onboard_imu_get(&hand_data);
    arm_hl04_get_stats(&stats);

    state->initialized               = 1U;
    state->upper_last_rx_tick        = upper_snapshot_valid != 0U ? upper_data.last_rx_tick : 0U;
    state->upper_packet_count        = stats.upper_packet_count;
    state->upper_frame_error_count   = stats.upper_frame_error_count;
    state->forearm_last_rx_tick      = forearm_snapshot_valid != 0U ? forearm_data.last_rx_tick : 0U;
    state->forearm_packet_count      = stats.forearm_packet_count;
    state->forearm_frame_error_count = stats.forearm_frame_error_count;
    state->hand_last_update_tick     = hand_snapshot_valid != 0U ? hand_data.last_update_tick : 0U;
    state->hand_update_count         = hand_snapshot_valid != 0U ? hand_data.update_count : 0U;
    state->upper_imu_valid =
        (upper_snapshot_valid != 0U && upper_data.frame_valid != 0U && (now - upper_data.last_rx_tick) <= ARM_DUAL_IMU_UPPER_TIMEOUT_MS &&
         arm_imu_math_normalize_quaternion(upper_data.quaternion, state->upper_quaternion) != 0U)
            ? 1U
            : 0U;
    state->forearm_imu_valid =
        (forearm_snapshot_valid != 0U && forearm_data.frame_valid != 0U && (now - forearm_data.last_rx_tick) <= ARM_DUAL_IMU_FOREARM_TIMEOUT_MS &&
         arm_imu_math_normalize_quaternion(forearm_data.quaternion, state->forearm_quaternion) != 0U)
            ? 1U
            : 0U;
    state->hand_imu_valid =
        (hand_snapshot_valid != 0U && hand_data.frame_valid != 0U && (now - hand_data.last_update_tick) <= ARM_DUAL_IMU_HAND_TIMEOUT_MS &&
         arm_imu_math_normalize_quaternion(hand_data.quaternion, state->hand_quaternion) != 0U)
            ? 1U
            : 0U;

    if (state->upper_imu_valid != 0U) arm_imu_math_quaternion_to_rotation(state->upper_quaternion, upper_raw);
    if (state->forearm_imu_valid != 0U) arm_imu_math_quaternion_to_rotation(state->forearm_quaternion, forearm_raw);
    if (state->hand_imu_valid != 0U) arm_imu_math_quaternion_to_rotation(state->hand_quaternion, hand_raw);
    arm_dual_imu_update_zero(context, now, upper_raw, forearm_raw, hand_raw);

    state->calibrated = context->pose_solver.calibrated;
    state->zero_count = context->pose_solver.zero_count;
    state->pose_valid = 0U;
    state->ik_valid   = 0U;
    if (state->upper_imu_valid == 0U || state->forearm_imu_valid == 0U || state->hand_imu_valid == 0U || context->pose_solver.calibrated == 0U)
    {
        arm_dual_imu_publish(state);
        return;
    }

    (void)arm_pose_solver_solve(&context->pose_solver, upper_raw, forearm_raw, hand_raw, state);
    arm_dual_imu_publish(state);
}

/**
 * @brief 三 IMU 主工作线程入口。
 * @param thread_input ThreadX 线程入口参数，当前未使用。
 * @retval 无，线程启动后持续运行。
 * @note 调用关系：由 arm_dual_imu_init() 创建；循环执行两路 HL04 取数、手背 INS 抓取和三段位姿解算。
 */
static void arm_dual_imu_task_entry(ULONG thread_input)
{
    (void)thread_input;

    while (1)
    {
        arm_hl04_poll();
        arm_onboard_imu_poll();
        arm_dual_imu_calculate(&g_arm_dual_imu_context);
    }
}

/**
 * @brief 初始化双 HL04、手背 BMI088 适配层、共享状态和解算线程。
 * @param 无。
 * @retval 无。
 * @note 调用关系：由 robot_control_init() 调用；依次初始化求解状态、USART1 DMA 接收、CAN2 接收和 ThreadX 工作线程，任一步失败即停止后续初始化。
 */
void arm_dual_imu_init(void)
{
    memset(&g_arm_dual_imu_context, 0, sizeof(g_arm_dual_imu_context));
    memset((void *)&g_arm_dual_imu_state, 0, sizeof(g_arm_dual_imu_state));
    arm_pose_solver_init(&g_arm_dual_imu_context.pose_solver);
    arm_onboard_imu_init();

    if (arm_hl04_init() == 0U) return;

    UINT status = tx_thread_create(&g_arm_dual_imu_thread, "arm_imu", arm_dual_imu_task_entry, 0U, g_arm_dual_imu_stack, sizeof(g_arm_dual_imu_stack),
                                   ARM_DUAL_IMU_TASK_PRIORITY, ARM_DUAL_IMU_TASK_PRIORITY, TX_NO_TIME_SLICE, TX_AUTO_START);
    if (status != TX_SUCCESS)
    {
        LOG_E("dual IMU thread create failed: %u", (unsigned)status);
        return;
    }

    LOG_I("3 IMUs ready: UART=%lu CAN=0x%03X BMI088, L=%.3f/%.3f/%.3f m, zero=%u", (unsigned long)ARM_DUAL_IMU_UART_BAUD,
          (unsigned)ARM_DUAL_IMU_CAN_RX_ID, (double)ARM_IMU_UPPER_LENGTH_M, (double)ARM_IMU_FOREARM_LENGTH_M, (double)ARM_IMU_HAND_LENGTH_M,
          (unsigned)ARM_DUAL_IMU_AUTO_ZERO_ENABLE);
}

/**
 * @brief 请求在下一次三路 IMU 均有效时捕获当前姿态为伸直中立零位。
 * @param 无。
 * @retval 无。
 * @note 调用关系：由按键回调或其他 APP 调用；这里只置请求标志，实际矩阵更新在三 IMU 工作线程中完成。
 */
void arm_dual_imu_request_zero(void) { g_arm_dual_imu_context.zero_requested = 1U; }

/**
 * @brief 获取一致的三 IMU 解算状态快照。
 * @param state 输出状态缓冲区。
 * @return 1 表示序列锁读取成功，0 表示参数无效或八次尝试期间持续发生并发写入。
 * @note 调用关系：主要由调试线程及后续业务消费者调用；函数只读共享状态，不触发传感器采集或重新计算。
 */
uint8_t arm_dual_imu_get_state(Arm_Dual_Imu_State_t *state)
{
    if (state == NULL) return 0U;

    for (uint8_t attempt = 0U; attempt < 8U; ++attempt)
    {
        uint32_t sequence_before = g_arm_dual_imu_state.sequence;
        if ((sequence_before & 1U) != 0U) continue;
        __DMB();
        memcpy(state, (const void *)&g_arm_dual_imu_state, sizeof(*state));
        __DMB();
        uint32_t sequence_after = g_arm_dual_imu_state.sequence;
        if (sequence_before == sequence_after && (sequence_after & 1U) == 0U)
        {
            state->sequence = sequence_after;
            return 1U;
        }
    }
    return 0U;
}
