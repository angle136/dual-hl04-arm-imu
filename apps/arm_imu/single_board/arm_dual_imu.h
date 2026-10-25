#ifndef _ARM_DUAL_IMU_H_
#define _ARM_DUAL_IMU_H_

#include <stdint.h>

#define ARM_DUAL_IMU_IK_DOF              6U
#define ARM_HL04_UART_RAW_FRAME_CAPACITY 100U
#define ARM_HL04_CAN_RAW_FRAME_SIZE      8U

/** USART1 HL04 最近一个完整帧，供 Ozone 直接观察。 */
typedef struct
{
    uint32_t sequence;
    uint32_t capture_count;
    uint16_t payload_length;
    uint16_t frame_length;
    uint8_t  raw_frame[ARM_HL04_UART_RAW_FRAME_CAPACITY];
} Arm_Hl04_Uart_Raw_Frame_t;

/** USART1 HL04 最近一帧的原始数据与解析结果，供 Ozone 直接观察。 */
typedef struct
{
    uint32_t sequence;
    uint32_t last_rx_tick;
    uint32_t packet_count;
    uint32_t frame_error_count;
    uint16_t payload_length;
    uint16_t frame_length;
    uint8_t  frame_valid;
    uint8_t  tag;
    uint8_t  reserved[2];
    uint8_t  raw_frame[ARM_HL04_UART_RAW_FRAME_CAPACITY];
    float    pressure;
    uint32_t sensor_timestamp;
    float    acceleration[3];
    float    angular_velocity[3];
    float    magnetic_field[3];
    float    euler[3];
    float    quaternion[4];
} Arm_Hl04_Uart_Data_t;

/** CAN2 HL04 最近一帧的原始数据与解析结果，供 Ozone 直接观察。 */
typedef struct
{
    uint32_t sequence;
    uint32_t last_rx_tick;
    uint32_t packet_count;
    uint32_t frame_error_count;
    uint32_t can_id;
    uint8_t  dlc;
    uint8_t  frame_valid;
    uint8_t  reserved[2];
    uint8_t  raw_frame[ARM_HL04_CAN_RAW_FRAME_SIZE];
    int16_t  raw_quaternion[4];
    float    quaternion[4];
} Arm_Hl04_Can_Data_t;

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
    uint32_t upper_frame_error_count;
    uint32_t forearm_last_rx_tick;
    uint32_t forearm_packet_count;
    uint32_t forearm_frame_error_count;
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

/** Ozone 观察变量：大臂 USART1 HL04 原始数据。 */
extern volatile Arm_Hl04_Uart_Data_t g_upper_hl04_uart_data;

/** Ozone 观察变量：大臂 USART1 HL04 最近一个完整帧。 */
extern volatile Arm_Hl04_Uart_Raw_Frame_t g_upper_hl04_uart_raw_frame;

/** Ozone 观察变量：前臂 CAN2 HL04 原始数据。 */
extern volatile Arm_Hl04_Can_Data_t g_forearm_hl04_can_data;

/** 初始化大臂 HL04 串口、前臂 HL04 CAN2 接收、解算状态和线程。 */
void arm_dual_imu_init(void);

/** 请求在下一次两路 IMU 都有效时，把当前姿势记作中立位。 */
void arm_dual_imu_request_zero(void);

/**
 * @brief 获取一致的双 IMU 状态快照。
 * @param state 输出状态。
 * @return 1 表示成功，0 表示参数无效或连续读取期间一直有写入。
 */
uint8_t arm_dual_imu_get_state(Arm_Dual_Imu_State_t *state);

#endif /* _ARM_DUAL_IMU_H_ */
