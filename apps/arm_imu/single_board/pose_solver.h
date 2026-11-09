#ifndef _ARM_POSE_SOLVER_H_
#define _ARM_POSE_SOLVER_H_

#include "imu_types.h"
#include "arm_ik.h"

/** 三 IMU 零位矩阵和真实 URDF 六轴逆解状态。 */
typedef struct
{
    float                   upper_zero_transpose[3][3];
    float                   forearm_zero_transpose[3][3];
    float                   hand_zero_transpose[3][3];
    uint32_t                zero_count;
    uint32_t                last_ik_tick;
    uint8_t                 calibrated;
    uint8_t                 ik_due_immediately;
    Arm_Ik_Context_t        arm_ik;
} Arm_Pose_Solver_t;

/** 初始化姿态求解器为未标零状态。 */
void arm_pose_solver_init(Arm_Pose_Solver_t *solver);

/** 将当前三路 IMU 姿态捕获为伸直中立零位。 */
void arm_pose_solver_capture_zero(Arm_Pose_Solver_t *solver, const float upper_raw[3][3], const float forearm_raw[3][3], const float hand_raw[3][3]);

/** 计算相对旋转和人体手臂位姿，并通过真实 URDF 六轴模型求解协议角。 */
uint8_t arm_pose_solver_solve(Arm_Pose_Solver_t *solver, const float upper_raw[3][3], const float forearm_raw[3][3], const float hand_raw[3][3],
                              uint32_t now, Arm_Dual_Imu_State_t *state);

#endif /* _ARM_POSE_SOLVER_H_ */
