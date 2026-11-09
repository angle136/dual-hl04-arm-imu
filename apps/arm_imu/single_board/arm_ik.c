#include "arm_ik.h"

#include "imu_math.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#define ARM_IK_PI                     3.14159265358979323846f
#define ARM_IK_TWO_PI                 (2.0f * ARM_IK_PI)
#define ARM_IK_JOINT_COUNT            6U
#define ARM_IK_MAX_ITERATIONS         ARM_DUAL_IMU_IK_MAX_ITERATIONS
#define ARM_IK_DAMPING                0.025f
#define ARM_IK_MAX_STEP_RAD           0.20f
#define ARM_IK_MAX_OUTPUT_DELTA_RAD   0.10f
#define ARM_IK_POSITION_TOLERANCE_M   0.015f
#define ARM_IK_ROTATION_TOLERANCE_RAD 0.025f
#define ARM_IK_J7_ZERO_RAW_RAD        0.066f

/* Protocol-space limits copied from arm_small_machine's board-link command
 * contract.  The remote arm still has its own motor safety backstop; this
 * APP must never send a command outside the explicit board-link range. */
static const float g_protocol_joint_min[ARM_DUAL_IMU_PROTOCOL_DOF] = {-6.2831853f, -1.90f, -3.20f, -6.2831853f, -2.10f, -6.2831853f, -2.00f};
static const float g_protocol_joint_max[ARM_DUAL_IMU_PROTOCOL_DOF] = {6.2831853f, 1.80f, 3.00f, 6.2831853f, 2.70f, 6.2831853f, 2.00f};

/* map_arm_configuration.m: URDF q = direction * protocol_q + zero_offset. */
static const float g_urdf_direction[ARM_DUAL_IMU_PROTOCOL_DOF]   = {-1.0f, -1.0f, -1.0f, -1.0f, -1.0f, 1.0f, 1.0f};
static const float g_urdf_zero_offset[ARM_DUAL_IMU_PROTOCOL_DOF] = {0.0f, -1.278566f, -2.918680f, 3.110406f, -0.031736f, -0.000452f, -1.100000f};

/* 由真实 URDF 正解选出的水平参考分支。主连杆沿 +Z 水平，J3 保留轻微
 * 弯曲以避开实机 J3 下限，后三轴补偿该弯曲带来的末端姿态变化。 */
static const float g_horizontal_reference_protocol[ARM_IK_JOINT_COUNT] = {0.0f, 1.66f, 0.25f, -0.23f, -0.16f, 0.23f};

/* 原姿态工具采用人体 +X 向前、+Z 向上（故 +Y 向左）；机械臂基座
 * 采用 +Y 向上，水平参考臂沿 +Z。保持右手系的轴置换为：
 * human +X -> robot +Z, human +Y -> robot +X, human +Z -> robot +Y。 */
static const float g_human_to_robot_local[3][3] = {
    {0.0f, 1.0f, 0.0f},
    {0.0f, 0.0f, 1.0f},
    {1.0f, 0.0f, 0.0f},
};

/* robot_dynamics.urdf joints link_002_joint through link_007_joint. */
static const float g_joint_origin_xyz[ARM_IK_JOINT_COUNT][3] = {
    {-0.000008662165f, 0.025000001f, 0.00002769516f}, {0.036069598f, 0.026573411f, -0.063260000f},  {0.062312248f, 0.19425031f, -0.0060000001f},
    {-0.0027078612f, 0.041912617f, -0.053000000f},    {0.020706928f, -0.010234868f, -0.078586752f}, {-0.0092938797f, -0.034467356f, 0.022003512f},
};

static const float g_joint_origin_rpy[ARM_IK_JOINT_COUNT][3] = {
    {1.5707963f, 0.0f, 0.0f},
    {0.0f, 1.5707963f, 3.1389782f},
    {-3.1415927f, 0.0f, 0.0f},
    {1.5707929f, -1.5706278f, 0.064521089f},
    {1.5707963f, -0.065249392f, -1.7207424f},
    {-1.5707963f, 1.4208503f, 3.0763433f},
};

typedef struct
{
    float rotation[3][3];
    float position[3];
} Arm_IK_Pose_t;

typedef struct
{
    Arm_IK_Pose_t current;
    float         origin_pose_rotation[3][3];
    float         joint_rotation[3][3];
} Arm_IK_Forward_Workspace_t;

typedef struct
{
    Arm_IK_Forward_Workspace_t forward;
    Arm_IK_Pose_t              current;
    float                      joint_axis[ARM_IK_JOINT_COUNT][3];
    float                      joint_origin[ARM_IK_JOINT_COUNT][3];
    float                      neutral_rotation_transpose[3][3];
    float                      relative_rotation[3][3];
    float                      mapped_relative_rotation[3][3];
    float                      local_map_transpose[3][3];
    float                      provisional_position_rotation[3][3];
    float                      position_alignment_rotation[3][3];
    float                      position_delta[3];
    float                      mapped_delta[3];
    float                      solution[ARM_IK_JOINT_COUNT];
    float                      error[6];
    float                      jacobian[6][ARM_IK_JOINT_COUNT];
    float                      normal[6][6];
    float                      rhs[6];
    float                      step[6];
    float                      position_error;
    float                      rotation_error;
    uint8_t                    iterations_used;
    float                      candidate_protocol[ARM_DUAL_IMU_PROTOCOL_DOF];
    float                      candidate_raw[ARM_DUAL_IMU_PROTOCOL_DOF];
} Arm_IK_Workspace_t;

/* IK is called only by the single arm_imu thread. Keeping its matrix workspace
 * outside the 2 KB thread stack prevents the iterative solver from corrupting
 * the saved thread context after zero calibration. */
static Arm_IK_Workspace_t g_arm_ik_workspace;
static float              g_joint_origin_rotation[ARM_IK_JOINT_COUNT][3][3];
static uint8_t            g_arm_ik_model_ready;

static float arm_ik_unwrap_near(float angle, float reference)
{
    while ((angle - reference) > ARM_IK_PI) angle -= ARM_IK_TWO_PI;
    while ((angle - reference) < -ARM_IK_PI) angle += ARM_IK_TWO_PI;
    return angle;
}

static float arm_ik_clamp_protocol_joint(uint8_t joint, float angle)
{
    if (angle < g_protocol_joint_min[joint]) return g_protocol_joint_min[joint];
    if (angle > g_protocol_joint_max[joint]) return g_protocol_joint_max[joint];
    return angle;
}

static void arm_ik_seed_variant(uint8_t variant, float seed[ARM_IK_JOINT_COUNT])
{
    memcpy(seed, g_horizontal_reference_protocol, sizeof(g_horizontal_reference_protocol));
    if (variant == 2U)
    {
        seed[1] = 0.15f;
        seed[2] = 1.20f;
        seed[3] = -0.70f;
    }
    else if (variant == 3U)
    {
        seed[1] = 1.20f;
        seed[2] = 0.65f;
        seed[3] = -0.55f;
    }
    else if (variant == 4U)
    {
        seed[0] = -1.40f;
        seed[1] = 0.50f;
        seed[2] = 1.55f;
        seed[3] = -0.90f;
    }

    for (uint8_t joint = 0U; joint < ARM_IK_JOINT_COUNT; ++joint) seed[joint] = arm_ik_clamp_protocol_joint(joint, seed[joint]);
}

static float arm_ik_vector_norm(const float vector[3])
{
    return sqrtf(vector[0] * vector[0] + vector[1] * vector[1] + vector[2] * vector[2]);
}

static void arm_ik_vector_cross(const float left[3], const float right[3], float output[3])
{
    output[0] = left[1] * right[2] - left[2] * right[1];
    output[1] = left[2] * right[0] - left[0] * right[2];
    output[2] = left[0] * right[1] - left[1] * right[0];
}

/** 生成把单位向量 from 旋转到单位向量 to 的最小旋转。 */
static void arm_ik_vector_alignment(const float from[3], const float to[3], float rotation[3][3])
{
    float cross[3];
    arm_ik_vector_cross(from, to, cross);
    const float sine_squared = cross[0] * cross[0] + cross[1] * cross[1] + cross[2] * cross[2];
    const float cosine       = arm_imu_math_clampf(from[0] * to[0] + from[1] * to[1] + from[2] * to[2], -1.0f, 1.0f);
    arm_imu_math_matrix_identity(rotation);

    if (sine_squared < 1.0e-8f)
    {
        if (cosine > 0.0f) return;

        float basis[3] = {1.0f, 0.0f, 0.0f};
        if (fabsf(from[0]) > 0.8f)
        {
            basis[0] = 0.0f;
            basis[1] = 1.0f;
        }
        float axis[3];
        arm_ik_vector_cross(from, basis, axis);
        const float inverse_norm = 1.0f / arm_ik_vector_norm(axis);
        for (uint8_t i = 0U; i < 3U; ++i) axis[i] *= inverse_norm;
        for (uint8_t row = 0U; row < 3U; ++row)
            for (uint8_t column = 0U; column < 3U; ++column)
                rotation[row][column] = 2.0f * axis[row] * axis[column] - (row == column ? 1.0f : 0.0f);
        return;
    }

    const float factor = (1.0f - cosine) / sine_squared;
    const float skew[3][3] = {
        {0.0f, -cross[2], cross[1]},
        {cross[2], 0.0f, -cross[0]},
        {-cross[1], cross[0], 0.0f},
    };
    float skew_squared[3][3];
    arm_imu_math_matrix_multiply(skew, skew, skew_squared);
    for (uint8_t row = 0U; row < 3U; ++row)
        for (uint8_t column = 0U; column < 3U; ++column)
            rotation[row][column] += skew[row][column] + factor * skew_squared[row][column];
}

static void arm_ik_prepare_position_map(Arm_Ik_Context_t *context, const float neutral_position[3], Arm_IK_Workspace_t *workspace)
{
    const float human_reach = arm_ik_vector_norm(neutral_position);
    float       robot_neutral_vector[3];
    for (uint8_t i = 0U; i < 3U; ++i)
        robot_neutral_vector[i] = context->reference_robot_position[i] - context->robot_shoulder_position[i];
    const float robot_reach = arm_ik_vector_norm(robot_neutral_vector);

    /* 位置向量位于基座坐标系，只应用人体到机器人基座的轴置换。末端
     * 参考姿态不能混入这里，否则水平参考会额外翻转人体竖直轴。 */
    memcpy(workspace->provisional_position_rotation, g_human_to_robot_local, sizeof(workspace->provisional_position_rotation));
    if (human_reach < 1.0e-5f || robot_reach < 1.0e-5f)
    {
        memcpy(context->human_to_robot_position_rotation, workspace->provisional_position_rotation,
               sizeof(context->human_to_robot_position_rotation));
        context->position_scale = 1.0f;
        return;
    }

    float human_neutral_unit[3];
    float mapped_neutral_unit[3];
    float robot_neutral_unit[3];
    for (uint8_t i = 0U; i < 3U; ++i)
    {
        human_neutral_unit[i] = neutral_position[i] / human_reach;
        robot_neutral_unit[i] = robot_neutral_vector[i] / robot_reach;
    }
    arm_imu_math_matrix_vector(workspace->provisional_position_rotation, human_neutral_unit, mapped_neutral_unit);
    arm_ik_vector_alignment(mapped_neutral_unit, robot_neutral_unit, workspace->position_alignment_rotation);
    arm_imu_math_matrix_multiply(workspace->position_alignment_rotation, workspace->provisional_position_rotation,
                                 context->human_to_robot_position_rotation);
    context->position_scale = robot_reach / human_reach;
}

static void arm_ik_rotation_rpy(const float rpy[3], float rotation[3][3])
{
    const float cr = cosf(rpy[0]);
    const float sr = sinf(rpy[0]);
    const float cp = cosf(rpy[1]);
    const float sp = sinf(rpy[1]);
    const float cy = cosf(rpy[2]);
    const float sy = sinf(rpy[2]);

    rotation[0][0] = cy * cp;
    rotation[0][1] = cy * sp * sr - sy * cr;
    rotation[0][2] = cy * sp * cr + sy * sr;
    rotation[1][0] = sy * cp;
    rotation[1][1] = sy * sp * sr + cy * cr;
    rotation[1][2] = sy * sp * cr - cy * sr;
    rotation[2][0] = -sp;
    rotation[2][1] = cp * sr;
    rotation[2][2] = cp * cr;
}

/** 固定 URDF origin.rpy 只需在启动时转换一次，避免每轮 IK 重复调用三角函数。 */
static void arm_ik_prepare_model(void)
{
    if (g_arm_ik_model_ready != 0U) return;
    for (uint8_t joint = 0U; joint < ARM_IK_JOINT_COUNT; ++joint)
        arm_ik_rotation_rpy(g_joint_origin_rpy[joint], g_joint_origin_rotation[joint]);
    g_arm_ik_model_ready = 1U;
}

static void arm_ik_forward(const float protocol_q[ARM_IK_JOINT_COUNT], Arm_IK_Pose_t *end_pose, float joint_axis[ARM_IK_JOINT_COUNT][3],
                           float joint_origin[ARM_IK_JOINT_COUNT][3], Arm_IK_Forward_Workspace_t *workspace)
{
    arm_ik_prepare_model();
    memset(&workspace->current, 0, sizeof(workspace->current));
    arm_imu_math_matrix_identity(workspace->current.rotation);

    for (uint8_t joint = 0U; joint < ARM_IK_JOINT_COUNT; ++joint)
    {
        arm_imu_math_matrix_multiply(workspace->current.rotation, g_joint_origin_rotation[joint], workspace->origin_pose_rotation);
        arm_imu_math_matrix_vector(workspace->current.rotation, g_joint_origin_xyz[joint], joint_origin[joint]);
        for (uint8_t axis = 0U; axis < 3U; ++axis) joint_origin[joint][axis] += workspace->current.position[axis];
        joint_axis[joint][0] = workspace->origin_pose_rotation[0][2];
        joint_axis[joint][1] = workspace->origin_pose_rotation[1][2];
        joint_axis[joint][2] = workspace->origin_pose_rotation[2][2];

        arm_imu_math_rotation_z(g_urdf_direction[joint] * protocol_q[joint] + g_urdf_zero_offset[joint], workspace->joint_rotation);
        arm_imu_math_matrix_multiply(workspace->origin_pose_rotation, workspace->joint_rotation, workspace->current.rotation);
        memcpy(workspace->current.position, joint_origin[joint], sizeof(workspace->current.position));
    }
    *end_pose = workspace->current;
}

static void arm_ik_rotation_error(const float target[3][3], const float actual[3][3], float error[3])
{
    float actual_transpose[3][3];
    float delta[3][3];
    arm_imu_math_matrix_transpose(actual, actual_transpose);
    arm_imu_math_matrix_multiply(target, actual_transpose, delta);

    const float cosine = arm_imu_math_clampf((delta[0][0] + delta[1][1] + delta[2][2] - 1.0f) * 0.5f, -1.0f, 1.0f);
    const float angle  = acosf(cosine);
    if (angle < 1.0e-5f)
    {
        error[0] = 0.5f * (delta[2][1] - delta[1][2]);
        error[1] = 0.5f * (delta[0][2] - delta[2][0]);
        error[2] = 0.5f * (delta[1][0] - delta[0][1]);
        return;
    }

    /* sin(pi) approaches zero, so the usual skew extraction is unstable for
     * a nearly opposite orientation. Recover a finite axis from R + I. */
    if ((ARM_IK_PI - angle) < 1.0e-3f)
    {
        uint8_t axis = 0U;
        if (delta[1][1] > delta[axis][axis]) axis = 1U;
        if (delta[2][2] > delta[axis][axis]) axis = 2U;

        float vector[3] = {delta[0][axis], delta[1][axis], delta[2][axis]};
        vector[axis] += 1.0f;
        const float vector_norm = arm_ik_vector_norm(vector);
        if (vector_norm > 1.0e-6f)
        {
            const float scale = angle / vector_norm;
            for (uint8_t i = 0U; i < 3U; ++i) error[i] = scale * vector[i];
            return;
        }
    }

    const float scale = angle / (2.0f * sinf(angle));
    error[0]          = scale * (delta[2][1] - delta[1][2]);
    error[1]          = scale * (delta[0][2] - delta[2][0]);
    error[2]          = scale * (delta[1][0] - delta[0][1]);
}

static uint8_t arm_ik_solve_linear_6(float matrix[6][6], float vector[6], float solution[6])
{
    for (uint8_t pivot = 0U; pivot < 6U; ++pivot)
    {
        uint8_t best = pivot;
        for (uint8_t row = (uint8_t)(pivot + 1U); row < 6U; ++row)
        {
            if (fabsf(matrix[row][pivot]) > fabsf(matrix[best][pivot])) best = row;
        }
        if (fabsf(matrix[best][pivot]) < 1.0e-8f) return 0U;
        if (best != pivot)
        {
            for (uint8_t column = pivot; column < 6U; ++column)
            {
                const float temp      = matrix[pivot][column];
                matrix[pivot][column] = matrix[best][column];
                matrix[best][column]  = temp;
            }
            const float temp = vector[pivot];
            vector[pivot]    = vector[best];
            vector[best]     = temp;
        }

        const float inverse_pivot = 1.0f / matrix[pivot][pivot];
        for (uint8_t column = pivot; column < 6U; ++column) matrix[pivot][column] *= inverse_pivot;
        vector[pivot] *= inverse_pivot;
        for (uint8_t row = 0U; row < 6U; ++row)
        {
            if (row == pivot) continue;
            const float factor = matrix[row][pivot];
            for (uint8_t column = pivot; column < 6U; ++column) matrix[row][column] -= factor * matrix[pivot][column];
            vector[row] -= factor * vector[pivot];
        }
    }
    memcpy(solution, vector, sizeof(float) * 6U);
    return 1U;
}

static uint8_t arm_ik_iterative_solve(const float target_position[3], const float target_rotation[3][3], float protocol_q[ARM_IK_JOINT_COUNT],
                                      Arm_IK_Workspace_t *workspace)
{
    workspace->iterations_used = 0U;
    for (uint8_t iteration = 0U; iteration < ARM_IK_MAX_ITERATIONS; ++iteration)
    {
        arm_ik_forward(protocol_q, &workspace->current, workspace->joint_axis, workspace->joint_origin, &workspace->forward);

        for (uint8_t i = 0U; i < 3U; ++i) workspace->error[i] = target_position[i] - workspace->current.position[i];
        arm_ik_rotation_error(target_rotation, workspace->current.rotation, &workspace->error[3]);
        workspace->position_error =
            sqrtf(workspace->error[0] * workspace->error[0] + workspace->error[1] * workspace->error[1] + workspace->error[2] * workspace->error[2]);
        workspace->rotation_error =
            sqrtf(workspace->error[3] * workspace->error[3] + workspace->error[4] * workspace->error[4] + workspace->error[5] * workspace->error[5]);
        workspace->iterations_used = iteration;
        if (workspace->position_error < ARM_IK_POSITION_TOLERANCE_M && workspace->rotation_error < ARM_IK_ROTATION_TOLERANCE_RAD) return 1U;

        for (uint8_t joint = 0U; joint < ARM_IK_JOINT_COUNT; ++joint)
        {
            float radius[3];
            float linear[3];
            for (uint8_t axis = 0U; axis < 3U; ++axis) radius[axis] = workspace->current.position[axis] - workspace->joint_origin[joint][axis];
            linear[0] = workspace->joint_axis[joint][1] * radius[2] - workspace->joint_axis[joint][2] * radius[1];
            linear[1] = workspace->joint_axis[joint][2] * radius[0] - workspace->joint_axis[joint][0] * radius[2];
            linear[2] = workspace->joint_axis[joint][0] * radius[1] - workspace->joint_axis[joint][1] * radius[0];
            for (uint8_t axis = 0U; axis < 3U; ++axis)
            {
                /* protocol_q 与 URDF q 的方向并不全部相同。位置和角速度
                 * Jacobian 必须对同一个协议变量求导，二者都要乘 direction。 */
                workspace->jacobian[axis][joint]      = g_urdf_direction[joint] * linear[axis];
                workspace->jacobian[axis + 3U][joint] = g_urdf_direction[joint] * workspace->joint_axis[joint][axis];
            }
        }

        memset(workspace->normal, 0, sizeof(workspace->normal));
        memset(workspace->rhs, 0, sizeof(workspace->rhs));
        for (uint8_t row = 0U; row < 6U; ++row)
        {
            for (uint8_t column = 0U; column < 6U; ++column)
            {
                for (uint8_t axis = 0U; axis < 6U; ++axis)
                    workspace->normal[row][column] += workspace->jacobian[axis][row] * workspace->jacobian[axis][column];
            }
            workspace->normal[row][row] += ARM_IK_DAMPING * ARM_IK_DAMPING;
            for (uint8_t axis = 0U; axis < 6U; ++axis) workspace->rhs[row] += workspace->jacobian[axis][row] * workspace->error[axis];
        }

        if (arm_ik_solve_linear_6(workspace->normal, workspace->rhs, workspace->step) == 0U) return 0U;
        for (uint8_t joint = 0U; joint < ARM_IK_JOINT_COUNT; ++joint)
        {
            const float bounded_step = arm_imu_math_clampf(workspace->step[joint], -ARM_IK_MAX_STEP_RAD, ARM_IK_MAX_STEP_RAD);
            protocol_q[joint] = arm_ik_clamp_protocol_joint(joint, protocol_q[joint] + bounded_step);
        }
    }

    /* 最后一次迭代会先更新 q 再退出循环，因此必须对更新后的候选解再验收一次。 */
    arm_ik_forward(protocol_q, &workspace->current, workspace->joint_axis, workspace->joint_origin, &workspace->forward);
    for (uint8_t i = 0U; i < 3U; ++i) workspace->error[i] = target_position[i] - workspace->current.position[i];
    arm_ik_rotation_error(target_rotation, workspace->current.rotation, &workspace->error[3]);
    workspace->position_error =
        sqrtf(workspace->error[0] * workspace->error[0] + workspace->error[1] * workspace->error[1] + workspace->error[2] * workspace->error[2]);
    workspace->rotation_error =
        sqrtf(workspace->error[3] * workspace->error[3] + workspace->error[4] * workspace->error[4] + workspace->error[5] * workspace->error[5]);
    workspace->iterations_used = ARM_IK_MAX_ITERATIONS;
    return (workspace->position_error < ARM_IK_POSITION_TOLERANCE_M && workspace->rotation_error < ARM_IK_ROTATION_TOLERANCE_RAD) ? 1U : 0U;
}

void arm_ik_reset(Arm_Ik_Context_t *context)
{
    if (context == NULL) return;
    memset(context, 0, sizeof(*context));
    arm_imu_math_matrix_identity(context->neutral_target_rotation);

    Arm_IK_Workspace_t *workspace = &g_arm_ik_workspace;
    memcpy(workspace->solution, g_horizontal_reference_protocol, sizeof(workspace->solution));
    arm_ik_forward(workspace->solution, &workspace->current, workspace->joint_axis, workspace->joint_origin, &workspace->forward);
    memcpy(context->reference_robot_position, workspace->current.position, sizeof(context->reference_robot_position));
    memcpy(context->reference_robot_rotation, workspace->current.rotation, sizeof(context->reference_robot_rotation));
    /* Retargeting is defined between the human straight-rod shoulder and
     * robot J1, while the forearm sensor endpoint maps to robot J6.  Using J2
     * here silently shortens and offsets the scale baseline. */
    memcpy(context->robot_shoulder_position, workspace->joint_origin[0], sizeof(context->robot_shoulder_position));
}

static float arm_ik_extract_pitch(const float rotation[3][3])
{
    const float horizontal = sqrtf(rotation[0][0] * rotation[0][0] + rotation[1][0] * rotation[1][0]);
    return atan2f(-rotation[2][0], horizontal);
}

static void arm_ik_raw_from_protocol(const float protocol[ARM_DUAL_IMU_PROTOCOL_DOF], float raw[ARM_DUAL_IMU_PROTOCOL_DOF])
{
    raw[0] = protocol[0] - 1.836f;
    raw[1] = protocol[1] - 0.952f;
    raw[2] = protocol[2] + 3.148f;
    raw[3] = protocol[3] + 1.01242732f;
    raw[4] = 0.040f - protocol[4];
    raw[5] = protocol[5] + 1.04290104f;
    raw[6] = protocol[6] + ARM_IK_J7_ZERO_RAW_RAD;
}

uint8_t arm_ik_solve_protocol(Arm_Ik_Context_t *context, const float position[3], const float target_rotation[3][3], const float hand_rotation[3][3],
                              float protocol_target[ARM_DUAL_IMU_PROTOCOL_DOF], float raw_target[ARM_DUAL_IMU_PROTOCOL_DOF], float *wrist_pitch_rad)
{
    if (context == NULL || position == NULL || target_rotation == NULL || hand_rotation == NULL || protocol_target == NULL || raw_target == NULL)
        return 0U;

    Arm_IK_Workspace_t *workspace = &g_arm_ik_workspace;

    /* J7 只依赖手背 BMI088。它必须独立于前六轴 IK 的成功或失败实时更新。 */
    const float pitch = arm_ik_extract_pitch(hand_rotation);
    const float j7_seed = -pitch;
    const float j7      = arm_ik_clamp_protocol_joint(
        6U, arm_ik_unwrap_near(j7_seed, context->have_previous_j7 != 0U ? context->previous_j7 : j7_seed));
    if (!isfinite(j7)) return 0U;
    protocol_target[6]       = j7;
    raw_target[6]            = j7 + ARM_IK_J7_ZERO_RAW_RAD;
    context->previous_j7     = j7;
    context->have_previous_j7 = 1U;
    if (wrist_pitch_rad != NULL) *wrist_pitch_rad = j7;

    if (context->neutral_reference_valid == 0U)
    {
        memcpy(context->neutral_position, position, sizeof(context->neutral_position));
        memcpy(context->neutral_target_rotation, target_rotation, sizeof(context->neutral_target_rotation));
        arm_ik_prepare_position_map(context, position, workspace);
        context->neutral_reference_valid = 1U;
    }

    arm_imu_math_matrix_transpose(context->neutral_target_rotation, workspace->neutral_rotation_transpose);
    arm_imu_math_matrix_multiply(workspace->neutral_rotation_transpose, target_rotation, workspace->relative_rotation);
    arm_imu_math_matrix_transpose(g_human_to_robot_local, workspace->local_map_transpose);
    arm_imu_math_matrix_multiply(g_human_to_robot_local, workspace->relative_rotation, workspace->mapped_relative_rotation);
    arm_imu_math_matrix_multiply(workspace->mapped_relative_rotation, workspace->local_map_transpose, workspace->relative_rotation);
    /* relative_rotation 是机器人基座系中的空间旋转；它必须左乘参考姿态，
     * 与位置目标使用同一个空间变换。 */
    arm_imu_math_matrix_multiply(workspace->relative_rotation, context->reference_robot_rotation, context->robot_target_rotation);

    arm_imu_math_matrix_vector(context->human_to_robot_position_rotation, position, workspace->mapped_delta);
    for (uint8_t i = 0U; i < 3U; ++i)
        context->robot_target_position[i] = context->robot_shoulder_position[i] + context->position_scale * workspace->mapped_delta[i];

    memcpy(workspace->solution, g_horizontal_reference_protocol, sizeof(workspace->solution));
    if (context->have_previous_protocol_target != 0U)
        memcpy(workspace->solution, context->previous_protocol_target, sizeof(workspace->solution));
    context->last_attempt = 1U;
    uint8_t solved = arm_ik_iterative_solve(context->robot_target_position, context->robot_target_rotation, workspace->solution, workspace);
    context->last_position_error_m   = workspace->position_error;
    context->last_rotation_error_rad = workspace->rotation_error;
    context->last_iterations         = workspace->iterations_used;

    /* A failed previous-branch solve is retried from several physically
     * distinct, limit-clamped elbow/shoulder branches.  This is deliberately
     * a fallback: normal motion remains on the previous continuous branch. */
    for (uint8_t variant = 1U; solved == 0U && variant <= 4U; ++variant)
    {
        if (variant == 1U && context->have_previous_protocol_target == 0U) continue;
        arm_ik_seed_variant(variant, workspace->solution);
        context->last_attempt = (uint8_t)(variant + 1U);
        solved = arm_ik_iterative_solve(context->robot_target_position, context->robot_target_rotation, workspace->solution, workspace);
        context->last_position_error_m   = workspace->position_error;
        context->last_rotation_error_rad = workspace->rotation_error;
        context->last_iterations         = workspace->iterations_used;
    }
    if (solved == 0U) return 0U;

    /* 保持同一连续角分支，并限制单周期输出变化，避免数值 IK 跨分支跳变。 */
    for (uint8_t joint = 0U; joint < ARM_IK_JOINT_COUNT; ++joint)
    {
        const float reference = context->have_previous_protocol_target != 0U ? context->previous_protocol_target[joint]
                                                                            : g_horizontal_reference_protocol[joint];
        const float continuous_solution = arm_ik_unwrap_near(workspace->solution[joint], reference);
        const float delta = arm_imu_math_clampf(continuous_solution - reference, -ARM_IK_MAX_OUTPUT_DELTA_RAD, ARM_IK_MAX_OUTPUT_DELTA_RAD);
        workspace->candidate_protocol[joint] = arm_ik_clamp_protocol_joint(joint, reference + delta);
    }
    workspace->candidate_protocol[6] = j7;
    arm_ik_raw_from_protocol(workspace->candidate_protocol, workspace->candidate_raw);
    for (uint8_t i = 0U; i < ARM_DUAL_IMU_PROTOCOL_DOF; ++i)
    {
        if (!isfinite(workspace->candidate_raw[i]) || !isfinite(workspace->candidate_protocol[i])) return 0U;
    }

    /* 候选解完整通过校验后再同时提交，失败路径不会污染上一帧有效输出。 */
    memcpy(protocol_target, workspace->candidate_protocol, sizeof(workspace->candidate_protocol));
    memcpy(raw_target, workspace->candidate_raw, sizeof(workspace->candidate_raw));
    memcpy(context->previous_protocol_target, workspace->candidate_protocol, sizeof(context->previous_protocol_target));
    context->have_previous_protocol_target = 1U;
    return 1U;
}
