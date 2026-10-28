#ifndef _ARM_HL04_H_
#define _ARM_HL04_H_

#include <stdint.h>

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

/** 两路 HL04 接收统计快照。 */
typedef struct
{
    uint32_t upper_packet_count;
    uint32_t upper_frame_error_count;
    uint32_t forearm_packet_count;
    uint32_t forearm_frame_error_count;
} Arm_Hl04_Stats_t;

/** Ozone 观察变量：大臂 USART1 HL04 原始数据。 */
extern volatile Arm_Hl04_Uart_Data_t g_upper_hl04_uart_data;

/** Ozone 观察变量：大臂 USART1 HL04 最近一个完整帧。 */
extern volatile Arm_Hl04_Uart_Raw_Frame_t g_upper_hl04_uart_raw_frame;

/** Ozone 观察变量：前臂 CAN2 HL04 原始数据。 */
extern volatile Arm_Hl04_Can_Data_t g_forearm_hl04_can_data;

/** 初始化大臂 HL04 串口和前臂 HL04 CAN2 接收。 */
uint8_t arm_hl04_init(void);

/** 处理一批 UART 字节并清空 CAN2 接收 FIFO。 */
void arm_hl04_poll(void);

/** 获取一致的大臂 UART 数据快照。 */
uint8_t arm_hl04_get_upper(Arm_Hl04_Uart_Data_t *data);

/** 获取一致的前臂 CAN 数据快照。 */
uint8_t arm_hl04_get_forearm(Arm_Hl04_Can_Data_t *data);

/** 获取两路传感器累计收包和错误计数。 */
void arm_hl04_get_stats(Arm_Hl04_Stats_t *stats);

#endif /* _ARM_HL04_H_ */
