#ifndef _ARM_IMU_TYPES_H_
#define _ARM_IMU_TYPES_H_

#include <stdint.h>

#define ARM_DUAL_IMU_IK_DOF 6U
#define ARM_DUAL_IMU_PROTOCOL_DOF 7U

/**
 * @brief 三 IMU 人体手臂映射结果。
 *
 * @note target_position_m / target_rotation 是机械臂末端目标位姿。
 * @note protocol_joint_target_rad 严格使用 arm_small_machine 的 J1~J7 协议角契约。
 */
typedef struct
{
    uint32_t sequence;
    uint8_t  initialized;
    uint8_t  upper_imu_valid;
    uint8_t  forearm_imu_valid;
    uint8_t  hand_imu_valid;
    uint8_t  calibrated;
    uint8_t  pose_valid;
    uint8_t  ik_valid;
    uint8_t  protocol_valid;
    uint32_t zero_count;
    uint32_t upper_last_rx_tick;
    uint32_t upper_packet_count;
    uint32_t upper_frame_error_count;
    uint32_t forearm_last_rx_tick;
    uint32_t forearm_packet_count;
    uint32_t forearm_frame_error_count;
    uint32_t hand_last_update_tick;
    uint32_t hand_update_count;
    float    upper_quaternion[4];
    float    forearm_quaternion[4];
    float    hand_quaternion[4];
    float    upper_rotation[3][3];
    float    forearm_rotation[3][3];
    float    hand_rotation[3][3];
    float    elbow_position_m[3];
    float    wrist_position_m[3];
    float    target_position_m[3];
    float    robot_target_position_m[3];
    float    target_rotation[3][3];
    float    legacy_joint_target_rad[ARM_DUAL_IMU_IK_DOF]; /* compatibility mirror of protocol J1~J6 */
    float    raw_joint_target_rad[ARM_DUAL_IMU_PROTOCOL_DOF];
    float    protocol_joint_target_rad[ARM_DUAL_IMU_PROTOCOL_DOF];
    float    wrist_pitch_rad;
    float    ik_position_error_m;
    float    ik_rotation_error_rad;
    float    ik_position_scale;
    uint8_t  ik_iterations;
    uint8_t  ik_attempt;
} Arm_Dual_Imu_State_t;

#endif /* _ARM_IMU_TYPES_H_ */
