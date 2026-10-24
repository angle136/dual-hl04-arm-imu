#ifndef _ARM_DUAL_IMU_H_
#define _ARM_DUAL_IMU_H_

#include <stdint.h>

#define ARM_DUAL_IMU_IK_DOF 6U

/**
 * @brief 双 IMU 人体手臂映射结果。
 *
 * @note target_position_m / target_rotation 是机械臂末端目标位姿。
 * @note legacy_joint_target_rad 沿用 YUAN 版解析逆解坐标，目前只用于观测，
 *       尚未换算为本工程 J1~J6 的电机关节坐标，也不会直接驱动电机。
 */
typedef struct
{
    uint32_t sequence;
    uint8_t  initialized;
    uint8_t  upper_imu_valid;
    uint8_t  forearm_imu_valid;
    uint8_t  calibrated;
    uint8_t  pose_valid;
    uint8_t  ik_valid;
    uint8_t  reserved[2];
    uint32_t upper_last_rx_tick;
    uint32_t upper_packet_count;
    uint32_t upper_crc_error_count;
    uint32_t upper_frame_error_count;
    float    upper_quaternion[4];
    float    forearm_quaternion[4];
    float    upper_rotation[3][3];
    float    forearm_rotation[3][3];
    float    elbow_position_m[3];
    float    target_position_m[3];
    float    target_rotation[3][3];
    float    legacy_joint_target_rad[ARM_DUAL_IMU_IK_DOF];
} Arm_Dual_Imu_State_t;

/** Ozone 观察用双 IMU 解算快照；业务代码优先使用 arm_dual_imu_get_state()。 */
extern volatile Arm_Dual_Imu_State_t g_arm_dual_imu_state;

/** 初始化 HL04 串口、解算状态和双 IMU 线程。 */
void arm_dual_imu_init(void);

/** 请求在下一次两路 IMU 都有效时，把当前姿势记作中立位。 */
void arm_dual_imu_request_zero(void);

/**
 * @brief 获取一致的双 IMU 状态快照。
 * @param state 输出状态。
 * @return 1 表示成功，0 表示参数无效或连续读取期间一直有写入。
 */
uint8_t arm_dual_imu_get_state(Arm_Dual_Imu_State_t *state);

/** 输出一行双 IMU 有效性、末端位置和旧解析 IK 调试日志。 */
void arm_dual_imu_log_status(void);

#endif /* _ARM_DUAL_IMU_H_ */
