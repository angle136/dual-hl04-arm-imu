#include "legacy_ik.h"

#include "imu_math.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#define ARM_LEGACY_IK_PI                3.14159265358979323846f
#define ARM_LEGACY_IK_TWO_PI            (2.0f * ARM_LEGACY_IK_PI)
#define ARM_LEGACY_IK_SINGULAR_EPSILON  1.0e-4f
#define ARM_LEGACY_IK_REACH_TOLERANCE_M 1.0e-3f

/**
 * @brief 将角度展开到最靠近参考角的 2pi 等价分支。
 * @param angle 输入待展开角度，单位 rad。
 * @param reference 输入上一周期参考角度，单位 rad。
 * @return 与参考角差值不超过 pi 的等价角度，单位 rad。
 * @note 调用关系：由解析逆解逐轴调用，用于抑制跨越正负 pi 时的目标跳变。
 */
static float arm_legacy_ik_unwrap_near(float angle, float reference)
{
    while ((angle - reference) > ARM_LEGACY_IK_PI) angle -= ARM_LEGACY_IK_TWO_PI;
    while ((angle - reference) < -ARM_LEGACY_IK_PI) angle += ARM_LEGACY_IK_TWO_PI;
    return angle;
}

/**
 * @brief 清除上一帧逆解角，下一帧不执行跨周期角度展开。
 * @param context 输入输出逆解上下文。
 * @retval 无。
 * @note 调用关系：由位姿解算器初始化和捕获新零位时调用。
 */
void arm_legacy_ik_reset(Arm_Legacy_Ik_Context_t *context)
{
    if (context == NULL) return;
    memset(context->previous_joint_target, 0, sizeof(context->previous_joint_target));
    context->have_previous_joint_target = 0U;
}

/**
 * @brief 使用 YUAN 版几何模型求解六自由度机械臂关节目标。
 * @param context 输入输出逆解上下文，用于保存上一周期关节解。
 * @param position 输入末端目标位置，单位 m。
 * @param target_rotation 输入末端目标旋转矩阵。
 * @param joint_target 输出 J1~J6 解析逆解角，单位 rad。
 * @return 1 表示目标可达且求解结果有效，0 表示奇异、越界或结果非有限值。
 * @note 调用关系：由 arm_pose_solver_solve() 在标定位姿有效后调用；当前结果只用于观测和遥测，不驱动电机。
 */
uint8_t arm_legacy_ik_solve(Arm_Legacy_Ik_Context_t *context, const float position[3], const float target_rotation[3][3],
                            float joint_target[ARM_DUAL_IMU_IK_DOF])
{
    if (context == NULL || position == NULL || target_rotation == NULL || joint_target == NULL) return 0U;

    const float x         = position[0];
    const float y         = position[1];
    const float z         = position[2];
    const float radius_sq = x * x + y * y + z * z;
    const float radius    = sqrtf(radius_sq);
    const float reach_min = fabsf(ARM_IMU_UPPER_LENGTH_M - ARM_IMU_FOREARM_LENGTH_M);
    const float reach_max = ARM_IMU_UPPER_LENGTH_M + ARM_IMU_FOREARM_LENGTH_M;

    if (!isfinite(radius) || radius < (reach_min - ARM_LEGACY_IK_REACH_TOLERANCE_M) || radius > (reach_max + ARM_LEGACY_IK_REACH_TOLERANCE_M) ||
        radius < ARM_LEGACY_IK_SINGULAR_EPSILON)
    {
        return 0U;
    }

    float shoulder_cos = (ARM_IMU_UPPER_LENGTH_M * ARM_IMU_UPPER_LENGTH_M + radius_sq - ARM_IMU_FOREARM_LENGTH_M * ARM_IMU_FOREARM_LENGTH_M) /
                         (2.0f * ARM_IMU_UPPER_LENGTH_M * radius);
    float elbow_cos = (ARM_IMU_UPPER_LENGTH_M * ARM_IMU_UPPER_LENGTH_M + ARM_IMU_FOREARM_LENGTH_M * ARM_IMU_FOREARM_LENGTH_M - radius_sq) /
                      (2.0f * ARM_IMU_UPPER_LENGTH_M * ARM_IMU_FOREARM_LENGTH_M);

    joint_target[0] = atan2f(y, x);
    joint_target[1] = acosf(arm_imu_math_clampf(shoulder_cos, -1.0f, 1.0f)) + atan2f(z, sqrtf(x * x + y * y)) - ARM_LEGACY_IK_PI / 2.0f;
    joint_target[2] = acosf(arm_imu_math_clampf(elbow_cos, -1.0f, 1.0f)) - ARM_LEGACY_IK_PI;

    float rotation_1[3][3];
    float rotation_2[3][3];
    float rotation_3[3][3];
    float rotation_12[3][3];
    float rotation_123[3][3];
    float rotation_123_transpose[3][3];
    float rotation_456[3][3];
    arm_imu_math_rotation_z(joint_target[0], rotation_1);
    arm_imu_math_rotation_y(joint_target[1], rotation_2);
    arm_imu_math_rotation_y(joint_target[2], rotation_3);
    arm_imu_math_matrix_multiply(rotation_1, rotation_2, rotation_12);
    arm_imu_math_matrix_multiply(rotation_12, rotation_3, rotation_123);
    arm_imu_math_matrix_transpose(rotation_123, rotation_123_transpose);
    arm_imu_math_matrix_multiply(rotation_123_transpose, target_rotation, rotation_456);

    joint_target[4]   = atan2f(sqrtf(rotation_456[2][0] * rotation_456[2][0] + rotation_456[2][1] * rotation_456[2][1]), rotation_456[2][2]);
    float middle_sine = sinf(joint_target[4]);
    if (fabsf(middle_sine) < 1.0e-2f)
    {
        joint_target[3] = 0.0f;
        joint_target[5] = (joint_target[4] < ARM_LEGACY_IK_PI / 2.0f) ? atan2f(-rotation_456[0][1], rotation_456[0][0])
                                                                      : atan2f(rotation_456[0][1], -rotation_456[0][0]);
    }
    else
    {
        joint_target[3] = atan2f(rotation_456[1][2] / middle_sine, rotation_456[0][2] / middle_sine);
        joint_target[5] = atan2f(rotation_456[2][1] / middle_sine, -rotation_456[2][0] / middle_sine);
    }

    for (uint8_t i = 0U; i < ARM_DUAL_IMU_IK_DOF; ++i)
    {
        if (!isfinite(joint_target[i])) return 0U;
        if (context->have_previous_joint_target != 0U)
        {
            joint_target[i] = arm_legacy_ik_unwrap_near(joint_target[i], context->previous_joint_target[i]);
        }
    }

    memcpy(context->previous_joint_target, joint_target, sizeof(context->previous_joint_target));
    context->have_previous_joint_target = 1U;
    return 1U;
}
