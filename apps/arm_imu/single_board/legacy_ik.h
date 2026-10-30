#ifndef _ARM_LEGACY_IK_H_
#define _ARM_LEGACY_IK_H_

#include "imu_types.h"

/** YUAN 六轴近似逆解的跨周期连续角状态。 */
typedef struct
{
    float   previous_joint_target[ARM_DUAL_IMU_IK_DOF];
    uint8_t have_previous_joint_target;
} Arm_Legacy_Ik_Context_t;

/** 清除上一帧分支，使下一次逆解重新选择角度分支。 */
void arm_legacy_ik_reset(Arm_Legacy_Ik_Context_t *context);

/** 使用 YUAN 版几何模型求解六自由度机械臂关节目标。 */
uint8_t arm_legacy_ik_solve(Arm_Legacy_Ik_Context_t *context, const float position[3], const float target_rotation[3][3],
                            float joint_target[ARM_DUAL_IMU_IK_DOF]);

#endif /* _ARM_LEGACY_IK_H_ */
