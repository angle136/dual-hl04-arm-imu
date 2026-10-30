#ifndef _ARM_DUAL_IMU_H_
#define _ARM_DUAL_IMU_H_

#include "hl04.h"
#include "imu_types.h"

/** Ozone 观察用三 IMU 解算快照；业务代码优先使用 arm_dual_imu_get_state()。 */
extern volatile Arm_Dual_Imu_State_t g_arm_dual_imu_state;

/** 初始化大臂 HL04、前臂 HL04、手背 BMI088 适配层、解算状态和线程。 */
void arm_dual_imu_init(void);

/** 请求在下一次三路 IMU 都有效时，把当前伸直姿势记作中立位。 */
void arm_dual_imu_request_zero(void);

/**
 * @brief 获取一致的三 IMU 状态快照。
 * @param state 输出状态。
 * @return 1 表示成功，0 表示参数无效或连续读取期间一直有写入。
 */
uint8_t arm_dual_imu_get_state(Arm_Dual_Imu_State_t *state);

#endif /* _ARM_DUAL_IMU_H_ */
