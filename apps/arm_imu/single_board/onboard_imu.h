#ifndef _ARM_ONBOARD_IMU_H_
#define _ARM_ONBOARD_IMU_H_

#include <stdint.h>

/** 主控板手背 BMI088/INS 最近姿态，供业务和 Ozone 观察。 */
typedef struct
{
    uint32_t sequence;
    uint32_t last_update_tick;
    uint32_t update_count;
    uint32_t ins_dwt_count;
    uint8_t  frame_valid;
    uint8_t  reserved[3];
    float    quaternion[4];
    float    euler_rad[3];
} Arm_Onboard_Imu_Data_t;

/** Ozone 观察变量：手背 BMI088 经 INS 融合后的最新姿态。 */
extern volatile Arm_Onboard_Imu_Data_t g_hand_bmi088_data;

/** 清空手背 BMI088 适配层状态。BMI088/INS 本体由模块层初始化。 */
void arm_onboard_imu_init(void);

/** 从模块层 INS 抓取一个最新姿态快照并更新时间统计。 */
void arm_onboard_imu_poll(void);

/** 获取一致的手背 BMI088/INS 姿态快照。 */
uint8_t arm_onboard_imu_get(Arm_Onboard_Imu_Data_t *data);

#endif /* _ARM_ONBOARD_IMU_H_ */
