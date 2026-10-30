#include "onboard_imu.h"

#include "imu_math.h"
#include "cmsis_compiler.h"
#include "module_ins.h"
#include "tx_api.h"

#include <stddef.h>
#include <string.h>

static uint32_t                 g_hand_last_ins_dwt_count;
volatile Arm_Onboard_Imu_Data_t g_hand_bmi088_data;

/**
 * @brief 以序列锁方式发布手背 BMI088/INS 姿态。
 * @param data 输入待发布姿态。
 * @retval 无。
 * @note 调用关系：由 arm_onboard_imu_poll() 在 INS 产生新样本时调用。
 */
static void arm_onboard_imu_publish(const Arm_Onboard_Imu_Data_t *data)
{
    const size_t payload_offset = offsetof(Arm_Onboard_Imu_Data_t, last_update_tick);
    uint32_t     odd_sequence   = (g_hand_bmi088_data.sequence + 1U) | 1U;
    g_hand_bmi088_data.sequence = odd_sequence;
    __DMB();
    memcpy((uint8_t *)(void *)&g_hand_bmi088_data + payload_offset, (const uint8_t *)data + payload_offset, sizeof(*data) - payload_offset);
    __DMB();
    g_hand_bmi088_data.sequence = odd_sequence + 1U;
}

/**
 * @brief 清空手背 BMI088 适配层状态。
 * @param 无。
 * @retval 无。
 * @note BMI088 和 INS 已由 MODULE_Init() 在 APP_Init() 前初始化，本层不重复操作硬件。
 */
void arm_onboard_imu_init(void)
{
    g_hand_last_ins_dwt_count = 0U;
    memset((void *)&g_hand_bmi088_data, 0, sizeof(g_hand_bmi088_data));
}

/**
 * @brief 从模块层 INS 抓取最新的手背姿态。
 * @param 无。
 * @retval 无。
 * @note 使用 INS 的 DWT 计数识别新样本；只有四元数有效时才发布。
 */
void arm_onboard_imu_poll(void)
{
    const Ins_t *ins = Module_INS_get();
    if (ins == NULL || ins->initialized == 0U) return;

    uint32_t dwt_count_before = ins->dwt_cnt;
    if (dwt_count_before == 0U || dwt_count_before == g_hand_last_ins_dwt_count) return;

    Arm_Onboard_Imu_Data_t data = {0};
    memcpy(data.quaternion, ins->q, sizeof(data.quaternion));
    memcpy(data.euler_rad, ins->euler_rad, sizeof(data.euler_rad));
    __DMB();
    uint32_t dwt_count_after = ins->dwt_cnt;
    if (dwt_count_before != dwt_count_after) return;

    float normalized[4];
    if (arm_imu_math_normalize_quaternion(data.quaternion, normalized) == 0U) return;

    g_hand_last_ins_dwt_count = dwt_count_after;
    data.last_update_tick     = tx_time_get();
    data.update_count         = g_hand_bmi088_data.update_count + 1U;
    data.ins_dwt_count        = dwt_count_after;
    data.frame_valid          = 1U;
    arm_onboard_imu_publish(&data);
}

/**
 * @brief 获取一致的手背 BMI088/INS 姿态快照。
 * @param data 输出快照。
 * @return 1 表示读取成功，0 表示参数无效或四次尝试期间持续发生并发写入。
 */
uint8_t arm_onboard_imu_get(Arm_Onboard_Imu_Data_t *data)
{
    if (data == NULL) return 0U;

    for (uint8_t attempt = 0U; attempt < 4U; ++attempt)
    {
        uint32_t sequence_before = g_hand_bmi088_data.sequence;
        if ((sequence_before & 1U) != 0U) continue;
        __DMB();
        memcpy(data, (const void *)&g_hand_bmi088_data, sizeof(*data));
        __DMB();
        if (sequence_before == g_hand_bmi088_data.sequence) return 1U;
    }
    return 0U;
}
