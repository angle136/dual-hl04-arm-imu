#ifndef _ARM_POSE_SOLVER_H_
#define _ARM_POSE_SOLVER_H_

#include "imu_types.h"
#include "legacy_ik.h"

/** 三 IMU 零位矩阵和旧六轴逆解的跨周期状态。 */
typedef struct
{
    float                   upper_zero_transpose[3][3];
    float                   forearm_zero_transpose[3][3];
    float                   hand_zero_transpose[3][3];
    uint32_t                zero_count;
    uint8_t                 calibrated;
    Arm_Legacy_Ik_Context_t legacy_ik;
} Arm_Pose_Solver_t;

/** 初始化姿态求解器为未标零状态。 */
void arm_pose_solver_init(Arm_Pose_Solver_t *solver);

/** 将当前三路 IMU 姿态捕获为伸直中立零位。 */
void arm_pose_solver_capture_zero(Arm_Pose_Solver_t *solver, const float upper_raw[3][3], const float forearm_raw[3][3], const float hand_raw[3][3]);

/** 计算相对旋转、肘部、腕部和手部目标位姿，并执行旧六轴近似逆解。 */
uint8_t arm_pose_solver_solve(Arm_Pose_Solver_t *solver, const float upper_raw[3][3], const float forearm_raw[3][3], const float hand_raw[3][3],
                              Arm_Dual_Imu_State_t *state);

#endif /* _ARM_POSE_SOLVER_H_ */
