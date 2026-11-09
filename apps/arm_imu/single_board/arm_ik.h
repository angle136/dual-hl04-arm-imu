#ifndef _ARM_IK_H_
#define _ARM_IK_H_

#include "imu_types.h"

/** 六轴几何逆解到 arm_small_machine 七轴协议的 APP 层适配状态。 */
typedef struct
{
    float   neutral_position[3];
    float   neutral_target_rotation[3][3];
    float   reference_robot_position[3];
    float   reference_robot_rotation[3][3];
    float   robot_shoulder_position[3];
    float   human_to_robot_position_rotation[3][3];
    float   position_scale;
    float   robot_target_position[3];
    float   robot_target_rotation[3][3];
    float   previous_protocol_target[ARM_DUAL_IMU_IK_DOF];
    float   previous_j7;
    float   last_position_error_m;
    float   last_rotation_error_rad;
    uint8_t neutral_reference_valid;
    uint8_t have_previous_protocol_target;
    uint8_t have_previous_j7;
    uint8_t last_iterations;
    uint8_t last_attempt;
} Arm_Ik_Context_t;

/** 清除连续 IK 分支和当前 IMU 零位对应的参考位姿。 */
void arm_ik_reset(Arm_Ik_Context_t *context);

/**
 * 根据 robot_dynamics.urdf 的前六关节固定变换求 IK，输出 arm_small_machine 协议角。
 * IMU 水平标零姿态对应机械臂水平伸展参考位；raw_target 是将协议角反算得到的 r1~r7。
 */
uint8_t arm_ik_solve_protocol(Arm_Ik_Context_t *context, const float position[3], const float target_rotation[3][3], const float hand_rotation[3][3],
                              float protocol_target[ARM_DUAL_IMU_PROTOCOL_DOF], float raw_target[ARM_DUAL_IMU_PROTOCOL_DOF], float *wrist_pitch_rad);

#endif /* _ARM_IK_H_ */
