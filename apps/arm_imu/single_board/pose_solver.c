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
    arm_ik_reset(&solver->arm_ik);
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
    solver->last_ik_tick       = 0U;
    solver->ik_due_immediately = 1U;
    arm_ik_reset(&solver->arm_ik);
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
                              uint32_t now, Arm_Dual_Imu_State_t *state)
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

    /* 两个肢段都由同型号 HL04 给出同一右手系姿态。旧版“大臂 HL04 +
     * 前臂 BMI088”遗留的单段 Z 镜像不能继续使用，否则位置链不是刚体变换。 */
    for (uint8_t i = 0U; i < 3U; ++i)
    {
        state->wrist_position_m[i]  = state->elbow_position_m[i] + forearm_vector[i];
        state->target_position_m[i] = state->wrist_position_m[i] + hand_vector[i];
    }

    /* 前两路 IMU 控制 J1~J6：前臂相对旋转给出六轴末端方向。
     * 固定左乘矩阵会在 IK 的 neutral^T * current 中完全抵消，真正的人体到
     * 机器人基座变换统一由 arm_ik.c 处理。手背 BMI088 只生成 J7。 */
    memcpy(state->target_rotation, state->forearm_rotation, sizeof(state->target_rotation));
    state->pose_valid = 1U;

    /* 姿态和三段位置每轮更新；计算量较大的数值 IK 只按固定周期运行。 */
    if (solver->ik_due_immediately == 0U && (uint32_t)(now - solver->last_ik_tick) < ARM_DUAL_IMU_IK_PERIOD_MS) return 1U;
    solver->last_ik_tick       = now;
    solver->ik_due_immediately = 0U;

    /* 前六轴按 URDF 链求解腕部中心；手长向量保留用于目标点和姿态观测。 */
    state->ik_valid = arm_ik_solve_protocol(&solver->arm_ik, state->wrist_position_m, state->target_rotation, state->hand_rotation,
                                            state->protocol_joint_target_rad, state->raw_joint_target_rad, &state->wrist_pitch_rad);
    state->ik_position_error_m   = solver->arm_ik.last_position_error_m;
    state->ik_rotation_error_rad = solver->arm_ik.last_rotation_error_rad;
    state->ik_position_scale     = solver->arm_ik.position_scale;
    memcpy(state->robot_target_position_m, solver->arm_ik.robot_target_position, sizeof(state->robot_target_position_m));
    state->ik_iterations         = solver->arm_ik.last_iterations;
    state->ik_attempt            = solver->arm_ik.last_attempt;
    state->protocol_valid        = solver->arm_ik.have_previous_protocol_target;
    if (state->ik_valid != 0U)
    {
        memcpy(state->legacy_joint_target_rad, state->protocol_joint_target_rad, sizeof(state->legacy_joint_target_rad));
    }
    return 1U;
}
