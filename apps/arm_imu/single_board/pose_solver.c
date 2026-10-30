#include "pose_solver.h"

#include "imu_math.h"

#include <stddef.h>
#include <string.h>

/**
 * @brief 初始化姿态求解器为未标零状态。
 * @param solver 输入输出姿态求解器上下文。
 * @retval 无。
 * @note 调用关系：由 arm_dual_imu_init() 在传感器初始化前调用。
 */
void arm_pose_solver_init(Arm_Pose_Solver_t *solver)
{
    if (solver == NULL) return;

    memset(solver, 0, sizeof(*solver));
    arm_imu_math_matrix_identity(solver->upper_zero_transpose);
    arm_imu_math_matrix_identity(solver->forearm_zero_transpose);
    arm_imu_math_matrix_identity(solver->hand_zero_transpose);
    arm_legacy_ik_reset(&solver->legacy_ik);
}

/**
 * @brief 将当前三路 IMU 姿态捕获为伸直中立零位。
 * @param solver 输入输出姿态求解器上下文，保存三路零位转置矩阵和校准状态。
 * @param upper_raw 输入大臂 IMU 当前原始旋转矩阵。
 * @param forearm_raw 输入前臂 HL04 当前原始旋转矩阵。
 * @param hand_raw 输入手背 BMI088 当前原始旋转矩阵。
 * @retval 无。
 * @note 调用关系：由 arm_dual_imu_calculate() 在三路数据均有效且收到自动或手动置零请求时调用。
 */
void arm_pose_solver_capture_zero(Arm_Pose_Solver_t *solver, const float upper_raw[3][3], const float forearm_raw[3][3], const float hand_raw[3][3])
{
    if (solver == NULL || upper_raw == NULL || forearm_raw == NULL || hand_raw == NULL) return;

    arm_imu_math_matrix_transpose(upper_raw, solver->upper_zero_transpose);
    arm_imu_math_matrix_transpose(forearm_raw, solver->forearm_zero_transpose);
    arm_imu_math_matrix_transpose(hand_raw, solver->hand_zero_transpose);
    solver->calibrated = 1U;
    solver->zero_count++;
    arm_legacy_ik_reset(&solver->legacy_ik);
}

/**
 * @brief 从三路原始旋转矩阵计算相对姿态、三段位置链和六轴解析逆解。
 * @param solver 输入输出姿态求解器上下文，保存校准矩阵和连续角状态。
 * @param upper_raw 输入大臂 IMU 原始旋转矩阵。
 * @param forearm_raw 输入前臂 HL04 原始旋转矩阵。
 * @param hand_raw 输入手背 BMI088 原始旋转矩阵。
 * @param state 输入输出三 IMU 状态，写入相对旋转、三段位置和逆解结果。
 * @return 1 表示姿态已计算，0 表示参数无效或尚未标零。
 * @note 调用关系：由 arm_dual_imu_calculate() 在三路 IMU 数据有效时调用。
 */
uint8_t arm_pose_solver_solve(Arm_Pose_Solver_t *solver, const float upper_raw[3][3], const float forearm_raw[3][3], const float hand_raw[3][3],
                              Arm_Dual_Imu_State_t *state)
{
    if (solver == NULL || upper_raw == NULL || forearm_raw == NULL || hand_raw == NULL || state == NULL || solver->calibrated == 0U) return 0U;

    arm_imu_math_matrix_multiply(solver->upper_zero_transpose, upper_raw, state->upper_rotation);
    arm_imu_math_matrix_multiply(solver->forearm_zero_transpose, forearm_raw, state->forearm_rotation);
    arm_imu_math_matrix_multiply(solver->hand_zero_transpose, hand_raw, state->hand_rotation);

    const float upper_local[3]   = {ARM_IMU_UPPER_LENGTH_M, 0.0f, 0.0f};
    const float forearm_local[3] = {ARM_IMU_FOREARM_LENGTH_M, 0.0f, 0.0f};
    const float hand_local[3]    = {ARM_IMU_HAND_LENGTH_M, 0.0f, 0.0f};
    float       forearm_vector[3];
    float       hand_vector[3];
    arm_imu_math_matrix_vector(state->upper_rotation, upper_local, state->elbow_position_m);
    arm_imu_math_matrix_vector(state->forearm_rotation, forearm_local, forearm_vector);
    arm_imu_math_matrix_vector(state->hand_rotation, hand_local, hand_vector);

    /* ponytail: 保留 YUAN 版上臂 Z 镜像；闭环前应替换成实测的完整 3x3 外参。 */
    state->elbow_position_m[2] = -state->elbow_position_m[2];
    for (uint8_t i = 0U; i < 3U; ++i)
    {
        state->wrist_position_m[i]  = state->elbow_position_m[i] + forearm_vector[i];
        state->target_position_m[i] = state->wrist_position_m[i] + hand_vector[i];
    }

    /* YUAN 版末端坐标变换，等价于绕 Y 轴旋转 -90 度。 */
    static const float end_frame_map[3][3] = {
        {0.0f, 0.0f, -1.0f},
        {0.0f, 1.0f, 0.0f},
        {1.0f, 0.0f, 0.0f},
    };
    arm_imu_math_matrix_multiply(end_frame_map, state->hand_rotation, state->target_rotation);
    state->pose_valid = 1U;
    /* 旧 IK 的两连杆几何求腕部中心；手长是腕后工具偏置，不计入 L1/L2 可达性。 */
    state->ik_valid = arm_legacy_ik_solve(&solver->legacy_ik, state->wrist_position_m, state->target_rotation, state->legacy_joint_target_rad);
    return 1U;
}
