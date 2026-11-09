#include "hl04.h"

#include "imu_math.h"
#include "bsp_can.h"
#include "bsp_def.h"
#include "bsp_uart.h"
#include "cmsis_compiler.h"
#include "tx_api.h"
#include "usart.h"

#include <stddef.h>
#include <string.h>

#define LOG_TAG "arm_hl04"
#define LOG_LVL LOG_LVL_INFO
#include "ulog_def.h"

#define ARM_HL04_HEADER_0         0x5AU
#define ARM_HL04_HEADER_1         0xA5U
#define ARM_HL04_TAG              0x91U
#define ARM_HL04_MIN_PAYLOAD      76U
#define ARM_HL04_MAX_PAYLOAD      92U
#define ARM_HL04_FRAME_OVERHEAD   6U
#define ARM_HL04_UART_BUFFER_SIZE 256U
#define ARM_HL04_READ_BUFFER_SIZE 128U
#define ARM_HL04_CAN_FILTER_BANK  14U

typedef enum
{
    HL04_WAIT_HEADER_0 = 0,
    HL04_WAIT_HEADER_1,
    HL04_READ_LENGTH_LOW,
    HL04_READ_LENGTH_HIGH,
    HL04_READ_BODY,
} Arm_Hl04_Parser_State_e;

typedef struct
{
    Arm_Hl04_Parser_State_e state;
    uint8_t                 frame[ARM_HL04_UART_RAW_FRAME_CAPACITY];
    uint16_t                index;
    uint16_t                payload_length;
    uint16_t                expected_length;
    uint32_t                packet_count;
    uint32_t                frame_error_count;
} Arm_Hl04_Parser_t;

typedef struct
{
    UART_Device      *uart;
    Arm_Hl04_Parser_t parser;
    volatile uint32_t forearm_packet_count;
    volatile uint32_t forearm_frame_error_count;
    ULONG             uart_start_tick;
    ULONG             uart_diag_log_tick;
    uint32_t          uart_diag_prev_dma_position;
    uint32_t          uart_diag_prev_rx_event_count;
    uint32_t          uart_diag_prev_bytes_received;
    uint32_t          uart_diag_prev_bytes_read;
    uint32_t          uart_diag_prev_error_count;
    uint32_t          uart_current_baud;
    uint8_t           uart_recovery_attempted;
    uint8_t           uart_data_detected_logged;
    uint8_t           uart_diag_have_dma_position;
} Arm_Hl04_Context_t;

BUFFER_SECTION static uint8_t g_arm_hl04_uart_buffer[ARM_HL04_UART_BUFFER_SIZE];
static Arm_Hl04_Context_t     g_arm_hl04_context;

volatile Arm_Hl04_Uart_Data_t      g_upper_hl04_uart_data;
volatile Arm_Hl04_Uart_Raw_Frame_t g_upper_hl04_uart_raw_frame;
volatile Arm_Hl04_Can_Data_t       g_forearm_hl04_can_data;

/**
 * @brief 从 HL04 小端字节流读取一个 float。
 * @param bytes 输入至少 4 字节的缓冲区地址。
 * @return 按本机 float 表示解释得到的数值。
 * @note 调用关系：由 UART 完整帧解码函数读取压力、惯性数据、欧拉角和四元数。
 */
static float arm_hl04_read_float(const uint8_t *bytes)
{
    float value;
    memcpy(&value, bytes, sizeof(value));
    return value;
}

/**
 * @brief 从 HL04 小端字节流读取一个 32 位无符号整数。
 * @param bytes 输入至少 4 字节的缓冲区地址。
 * @return 解码后的 uint32_t 数值。
 * @note 调用关系：由 UART 完整帧解码函数读取传感器时间戳。
 */
static uint32_t arm_hl04_read_u32(const uint8_t *bytes)
{
    uint32_t value;
    memcpy(&value, bytes, sizeof(value));
    return value;
}

/**
 * @brief 以序列锁方式发布最近一个大臂 HL04 UART 原始帧。
 * @param parser 输入已收齐完整帧的解析器状态。
 * @retval 无。
 * @note 调用关系：由 arm_hl04_decode_frame() 在字段校验前调用，供 Ozone 保留原始诊断数据。
 */
static void arm_hl04_uart_raw_frame_publish(const Arm_Hl04_Parser_t *parser)
{
    uint32_t odd_sequence = (g_upper_hl04_uart_raw_frame.sequence + 1U) | 1U;

    g_upper_hl04_uart_raw_frame.sequence = odd_sequence;
    __DMB();
    g_upper_hl04_uart_raw_frame.capture_count++;
    g_upper_hl04_uart_raw_frame.payload_length = parser->payload_length;
    g_upper_hl04_uart_raw_frame.frame_length   = parser->expected_length;
    memcpy((void *)g_upper_hl04_uart_raw_frame.raw_frame, parser->frame, parser->expected_length);
    if (parser->expected_length < ARM_HL04_UART_RAW_FRAME_CAPACITY)
    {
        memset((void *)&g_upper_hl04_uart_raw_frame.raw_frame[parser->expected_length], 0,
               ARM_HL04_UART_RAW_FRAME_CAPACITY - parser->expected_length);
    }
    __DMB();
    g_upper_hl04_uart_raw_frame.sequence = odd_sequence + 1U;
}

/**
 * @brief 以序列锁方式发布大臂 HL04 UART 解码结果。
 * @param data 输入一帧已校验的 UART 解码数据。
 * @retval 无。
 * @note 调用关系：由 arm_hl04_poll() 在流解析器返回完整有效帧后调用。
 */
static void arm_hl04_uart_publish(const Arm_Hl04_Uart_Data_t *data)
{
    const size_t payload_offset     = offsetof(Arm_Hl04_Uart_Data_t, last_rx_tick);
    uint32_t     odd_sequence       = (g_upper_hl04_uart_data.sequence + 1U) | 1U;
    g_upper_hl04_uart_data.sequence = odd_sequence;
    __DMB();
    memcpy((uint8_t *)(void *)&g_upper_hl04_uart_data + payload_offset, (const uint8_t *)data + payload_offset, sizeof(*data) - payload_offset);
    __DMB();
    g_upper_hl04_uart_data.sequence = odd_sequence + 1U;
}

/**
 * @brief 以序列锁方式发布前臂 HL04 CAN 解码结果。
 * @param data 输入当前 CAN 帧的解码数据；无效帧也会携带错误计数发布。
 * @retval 无。
 * @note 调用关系：由 arm_hl04_can_decode() 在成功和错误路径调用。
 */
static void arm_hl04_can_publish(const Arm_Hl04_Can_Data_t *data)
{
    const size_t payload_offset      = offsetof(Arm_Hl04_Can_Data_t, last_rx_tick);
    uint32_t     odd_sequence        = (g_forearm_hl04_can_data.sequence + 1U) | 1U;
    g_forearm_hl04_can_data.sequence = odd_sequence;
    __DMB();
    memcpy((uint8_t *)(void *)&g_forearm_hl04_can_data + payload_offset, (const uint8_t *)data + payload_offset, sizeof(*data) - payload_offset);
    __DMB();
    g_forearm_hl04_can_data.sequence = odd_sequence + 1U;
}

/**
 * @brief 将 HL04 UART 流解析器恢复到等待首字节状态。
 * @param parser 输入输出解析器。
 * @retval 无。
 * @note 调用关系：由初始化、异常状态处理以及每个完整帧处理结束后调用；累计计数不会被清零。
 */
static void arm_hl04_parser_reset(Arm_Hl04_Parser_t *parser)
{
    parser->state           = HL04_WAIT_HEADER_0;
    parser->index           = 0U;
    parser->payload_length  = 0U;
    parser->expected_length = 0U;
}

/**
 * @brief 仅重配主控 USART 波特率并重新启动 DMA，用于兼容未保存高速配置的 HI04。
 * @param baud_rate 目标波特率。
 * @return 1 表示重配成功，0 表示 HAL 或 DMA 重启失败。
 * @note 不向传感器发送指令，也不修改传感器 Flash。
 */
static uint8_t arm_hl04_uart_set_baud(uint32_t baud_rate)
{
    if (HAL_UART_AbortReceive(&ARM_DUAL_IMU_UART) != HAL_OK) return 0U;

    ARM_DUAL_IMU_UART.Init.BaudRate = baud_rate;
    if (HAL_UART_Init(&ARM_DUAL_IMU_UART) != HAL_OK) return 0U;

    arm_hl04_parser_reset(&g_arm_hl04_context.parser);
    if (BSP_UART_Rearm_DMA(g_arm_hl04_context.uart) != 0) return 0U;

    g_arm_hl04_context.uart_current_baud = baud_rate;
    return 1U;
}

/**
 * @brief 解码并校验一个完整的大臂 HL04 UART 帧。
 * @param parser 输入解析器及完整原始帧，同时更新包计数和错误计数。
 * @param data 输出解码后的传感器数据。
 * @return 1 表示标签和四元数有效，0 表示帧内容无效。
 * @note 调用关系：由 arm_hl04_parser_feed() 收齐 expected_length 字节后调用。
 */
static uint8_t arm_hl04_decode_frame(Arm_Hl04_Parser_t *parser, Arm_Hl04_Uart_Data_t *data)
{
    arm_hl04_uart_raw_frame_publish(parser);
    if (parser->frame[6] != ARM_HL04_TAG)
    {
        parser->frame_error_count++;
        return 0U;
    }

    memset(data, 0, sizeof(*data));
    data->last_rx_tick     = tx_time_get();
    data->payload_length   = parser->payload_length;
    data->frame_length     = parser->expected_length;
    data->tag              = parser->frame[6];
    data->pressure         = arm_hl04_read_float(&parser->frame[10]);
    data->sensor_timestamp = arm_hl04_read_u32(&parser->frame[14]);
    memcpy(data->raw_frame, parser->frame, parser->expected_length);
    for (uint8_t i = 0U; i < 3U; ++i)
    {
        data->acceleration[i]     = arm_hl04_read_float(&parser->frame[18U + 4U * i]);
        data->angular_velocity[i] = arm_hl04_read_float(&parser->frame[30U + 4U * i]);
        data->magnetic_field[i]   = arm_hl04_read_float(&parser->frame[42U + 4U * i]);
        data->euler[i]            = arm_hl04_read_float(&parser->frame[54U + 4U * i]);
    }
    for (uint8_t i = 0U; i < 4U; ++i) data->quaternion[i] = arm_hl04_read_float(&parser->frame[66U + 4U * i]);

    float normalized[4];
    if (arm_imu_math_normalize_quaternion(data->quaternion, normalized) == 0U)
    {
        parser->frame_error_count++;
        return 0U;
    }

    parser->packet_count++;
    data->packet_count      = parser->packet_count;
    data->frame_error_count = parser->frame_error_count;
    data->frame_valid       = 1U;
    return 1U;
}

/**
 * @brief 向 HL04 UART 状态机逐字节输入数据。
 * @param parser 输入输出流解析器。
 * @param byte 输入当前字节。
 * @param data 输出完整有效帧的解码结果。
 * @return 1 表示本字节结束了一个有效完整帧，0 表示仍在组帧或帧无效。
 * @note 调用关系：由 arm_hl04_poll() 遍历 BSP_UART_Read() 返回的字节块时调用。
 */
static uint8_t arm_hl04_parser_feed(Arm_Hl04_Parser_t *parser, uint8_t byte, Arm_Hl04_Uart_Data_t *data)
{
    switch (parser->state)
    {
    case HL04_WAIT_HEADER_0:
        if (byte == ARM_HL04_HEADER_0)
        {
            parser->frame[0] = byte;
            parser->index    = 1U;
            parser->state    = HL04_WAIT_HEADER_1;
        }
        break;

    case HL04_WAIT_HEADER_1:
        if (byte == ARM_HL04_HEADER_1)
        {
            parser->frame[1] = byte;
            parser->index    = 2U;
            parser->state    = HL04_READ_LENGTH_LOW;
        }
        else if (byte != ARM_HL04_HEADER_0)
        {
            arm_hl04_parser_reset(parser);
        }
        break;

    case HL04_READ_LENGTH_LOW:
        parser->frame[2] = byte;
        parser->index    = 3U;
        parser->state    = HL04_READ_LENGTH_HIGH;
        break;

    case HL04_READ_LENGTH_HIGH:
        parser->frame[3]        = byte;
        parser->index           = 4U;
        parser->payload_length  = (uint16_t)parser->frame[2] | ((uint16_t)parser->frame[3] << 8U);
        parser->expected_length = (uint16_t)(parser->payload_length + ARM_HL04_FRAME_OVERHEAD);
        if (parser->payload_length < ARM_HL04_MIN_PAYLOAD || parser->payload_length > ARM_HL04_MAX_PAYLOAD ||
            parser->expected_length > ARM_HL04_UART_RAW_FRAME_CAPACITY)
        {
            parser->frame_error_count++;
            arm_hl04_parser_reset(parser);
        }
        else
        {
            parser->state = HL04_READ_BODY;
        }
        break;

    case HL04_READ_BODY:
        parser->frame[parser->index++] = byte;
        if (parser->index == parser->expected_length)
        {
            uint8_t decoded = arm_hl04_decode_frame(parser, data);
            arm_hl04_parser_reset(parser);
            return decoded;
        }
        break;

    default:
        arm_hl04_parser_reset(parser);
        break;
    }
    return 0U;
}

/**
 * @brief 解码并发布一个前臂 HL04 CAN 四元数帧。
 * @param context 输入输出 HL04 上下文，用于累计前臂包数和错误数。
 * @param can_id 输入接收帧标准标识符。
 * @param bytes 输入 CAN 数据区。
 * @param length 输入数据长度，单位字节。
 * @retval 无。
 * @note 调用关系：由 arm_hl04_can_poll() 对通过标识符筛选的帧调用。
 */
static void arm_hl04_can_decode(Arm_Hl04_Context_t *context, uint32_t can_id, const uint8_t *bytes, uint8_t length)
{
    if (context == NULL || bytes == NULL) return;

    Arm_Hl04_Can_Data_t data = {0};
    data.last_rx_tick        = tx_time_get();
    data.can_id              = can_id;
    data.dlc                 = length;
    context->forearm_packet_count++;
    data.packet_count = context->forearm_packet_count;

    uint8_t copy_length = length > ARM_HL04_CAN_RAW_FRAME_SIZE ? ARM_HL04_CAN_RAW_FRAME_SIZE : length;
    memcpy(data.raw_frame, bytes, copy_length);
    if (length != ARM_HL04_CAN_RAW_FRAME_SIZE)
    {
        context->forearm_frame_error_count++;
        data.frame_error_count = context->forearm_frame_error_count;
        arm_hl04_can_publish(&data);
        return;
    }

    for (uint8_t i = 0U; i < 4U; ++i)
    {
        uint16_t packed        = (uint16_t)bytes[2U * i] | ((uint16_t)bytes[2U * i + 1U] << 8U);
        data.raw_quaternion[i] = (int16_t)packed;
        data.quaternion[i]     = (float)data.raw_quaternion[i] * 0.0001f;
    }

    float normalized[4];
    if (arm_imu_math_normalize_quaternion(data.quaternion, normalized) == 0U)
    {
        context->forearm_frame_error_count++;
        data.frame_error_count = context->forearm_frame_error_count;
        arm_hl04_can_publish(&data);
        return;
    }

    data.frame_error_count = context->forearm_frame_error_count;
    data.frame_valid       = 1U;
    arm_hl04_can_publish(&data);
}

/**
 * @brief 初始化前臂 HL04 所用 CAN2 和精确接收滤波器。
 * @param 无。
 * @return 1 表示 CAN 初始化、滤波器配置和启动全部成功，0 表示任一步骤失败。
 * @note 调用关系：仅由 arm_hl04_init() 调用。
 */
static uint8_t arm_hl04_can_init(void)
{
    CAN_HandleTypeDef *const can    = BSP_CAN_HANDLE2;
    CAN_FilterTypeDef        filter = {
               .FilterMode           = CAN_FILTERMODE_IDMASK,
               .FilterScale          = CAN_FILTERSCALE_32BIT,
               .FilterFIFOAssignment = CAN_FILTER_FIFO0,
               .FilterBank           = ARM_HL04_CAN_FILTER_BANK,
               .SlaveStartFilterBank = ARM_HL04_CAN_FILTER_BANK,
               .FilterIdHigh         = (uint16_t)(((uint32_t)ARM_DUAL_IMU_CAN_RX_ID & 0x7FFU) << 5U),
               .FilterIdLow          = 0U,
               .FilterMaskIdHigh     = 0xFFE0U,
               .FilterMaskIdLow      = 0x0006U,
               .FilterActivation     = CAN_FILTER_ENABLE,
    };

    can->Init.Prescaler = ARM_DUAL_IMU_CAN_PRESCALER;
    if (HAL_CAN_Init(can) != HAL_OK) return 0U;
    if (HAL_CAN_ConfigFilter(can, &filter) != HAL_OK) return 0U;
    if (HAL_CAN_Start(can) != HAL_OK) return 0U;
    return 1U;
}

/**
 * @brief 轮询并清空 CAN2 FIFO0 中待处理的前臂 HL04 帧。
 * @param context 输入输出 HL04 上下文，用于解码和累计错误计数。
 * @retval 无。
 * @note 调用关系：由 arm_hl04_poll() 每轮在 UART 处理后调用；非法帧只累计错误，不进入姿态解算。
 */
static void arm_hl04_can_poll(Arm_Hl04_Context_t *context)
{
    CAN_HandleTypeDef *const can = BSP_CAN_HANDLE2;

    while (HAL_CAN_GetRxFifoFillLevel(can, CAN_RX_FIFO0) > 0U)
    {
        CAN_RxHeaderTypeDef rx_header;
        uint8_t             rx_data[ARM_HL04_CAN_RAW_FRAME_SIZE];
        if (HAL_CAN_GetRxMessage(can, CAN_RX_FIFO0, &rx_header, rx_data) != HAL_OK)
        {
            context->forearm_frame_error_count++;
            continue;
        }

        if (rx_header.IDE != CAN_ID_STD || rx_header.StdId != ARM_DUAL_IMU_CAN_RX_ID || rx_header.RTR != CAN_RTR_DATA ||
            rx_header.DLC != ARM_HL04_CAN_RAW_FRAME_SIZE)
        {
            context->forearm_frame_error_count++;
            continue;
        }

        arm_hl04_can_decode(context, rx_header.StdId, rx_data, (uint8_t)rx_header.DLC);
    }
}

/**
 * @brief 获取一致的大臂 HL04 UART 数据快照。
 * @param data 输出快照。
 * @return 1 表示序列锁读取成功，0 表示参数无效或四次尝试期间持续发生并发写入。
 * @note 调用关系：由 arm_dual_imu_calculate() 每周期读取最新大臂数据。
 */
uint8_t arm_hl04_get_upper(Arm_Hl04_Uart_Data_t *data)
{
    if (data == NULL) return 0U;

    for (uint8_t attempt = 0U; attempt < 4U; ++attempt)
    {
        uint32_t sequence_before = g_upper_hl04_uart_data.sequence;
        if ((sequence_before & 1U) != 0U) continue;
        __DMB();
        memcpy(data, (const void *)&g_upper_hl04_uart_data, sizeof(*data));
        __DMB();
        if (sequence_before == g_upper_hl04_uart_data.sequence) return 1U;
    }
    return 0U;
}

/**
 * @brief 获取一致的前臂 HL04 CAN 数据快照。
 * @param data 输出快照。
 * @return 1 表示序列锁读取成功，0 表示参数无效或四次尝试期间持续发生并发写入。
 * @note 调用关系：由 arm_dual_imu_calculate() 每周期读取最新前臂数据。
 */
uint8_t arm_hl04_get_forearm(Arm_Hl04_Can_Data_t *data)
{
    if (data == NULL) return 0U;

    for (uint8_t attempt = 0U; attempt < 4U; ++attempt)
    {
        uint32_t sequence_before = g_forearm_hl04_can_data.sequence;
        if ((sequence_before & 1U) != 0U) continue;
        __DMB();
        memcpy(data, (const void *)&g_forearm_hl04_can_data, sizeof(*data));
        __DMB();
        if (sequence_before == g_forearm_hl04_can_data.sequence) return 1U;
    }
    return 0U;
}

/**
 * @brief 获取两路 HL04 累计接收统计。
 * @param stats 输出统计快照。
 * @retval 无。
 * @note 调用关系：由 arm_dual_imu_calculate() 用于填充共享状态中的诊断字段。
 */
void arm_hl04_get_stats(Arm_Hl04_Stats_t *stats)
{
    if (stats == NULL) return;

    stats->upper_packet_count        = g_arm_hl04_context.parser.packet_count;
    stats->upper_frame_error_count   = g_arm_hl04_context.parser.frame_error_count;
    stats->forearm_packet_count      = g_arm_hl04_context.forearm_packet_count;
    stats->forearm_frame_error_count = g_arm_hl04_context.forearm_frame_error_count;
}

/**
 * @brief 初始化大臂 HL04 USART1 DMA 接收和前臂 HL04 CAN2 接收。
 * @param 无。
 * @return 1 表示两路接口均初始化成功，0 表示任一步骤失败。
 * @note 调用关系：由 arm_dual_imu_init() 在线程创建前调用。
 */
uint8_t arm_hl04_init(void)
{
    memset(&g_arm_hl04_context, 0, sizeof(g_arm_hl04_context));
    memset((void *)&g_upper_hl04_uart_data, 0, sizeof(g_upper_hl04_uart_data));
    memset((void *)&g_upper_hl04_uart_raw_frame, 0, sizeof(g_upper_hl04_uart_raw_frame));
    memset((void *)&g_forearm_hl04_can_data, 0, sizeof(g_forearm_hl04_can_data));
    arm_hl04_parser_reset(&g_arm_hl04_context.parser);

    ARM_DUAL_IMU_UART.Init.BaudRate = ARM_DUAL_IMU_UART_BAUD;
    if (HAL_UART_Init(&ARM_DUAL_IMU_UART) != HAL_OK)
    {
        LOG_E("HL04 UART init failed");
        return 0U;
    }

    UART_Device_init_config uart_config = {
        .huart           = &ARM_DUAL_IMU_UART,
        .rx_buf          = g_arm_hl04_uart_buffer,
        .rx_buf_size     = sizeof(g_arm_hl04_uart_buffer),
        .expected_rx_len = 0U,
        .rx_mode         = UART_MODE_DMA,
        .tx_mode         = UART_MODE_BLOCKING,
    };
    g_arm_hl04_context.uart = BSP_UART_Device_Init(&uart_config);
    if (g_arm_hl04_context.uart == NULL)
    {
        LOG_E("HL04 UART device registration failed");
        return 0U;
    }

    /* HL04 可能在主控进入 APP 前已经输出，先丢弃半帧并从干净状态重新接收。 */
    if (BSP_UART_Rearm_DMA(g_arm_hl04_context.uart) != 0)
    {
        LOG_E("HL04 UART DMA start failed");
        return 0U;
    }
    g_arm_hl04_context.uart_start_tick = tx_time_get();
    g_arm_hl04_context.uart_current_baud = ARM_DUAL_IMU_UART_BAUD;

    if (arm_hl04_can_init() == 0U)
    {
        LOG_E("forearm HL04 CAN2 init failed: rx_id=0x%08lX", (unsigned long)ARM_DUAL_IMU_CAN_RX_ID);
        return 0U;
    }
    return 1U;
}

/**
 * @brief 读取并解析一批大臂 UART 字节，随后清空前臂 CAN2 接收 FIFO。
 * @param 无。
 * @retval 无。
 * @note 调用关系：由三 IMU 工作线程每轮调用；UART 读取保持原有超时时间。
 */
void arm_hl04_poll(void)
{
    uint8_t  read_buffer[ARM_HL04_READ_BUFFER_SIZE];
    uint32_t read_length = 0U;
    int      read_status = BSP_UART_Read(g_arm_hl04_context.uart, read_buffer, sizeof(read_buffer), &read_length, ARM_DUAL_IMU_READ_TIMEOUT_MS);
    if (read_status > 0)
    {
        for (uint32_t i = 0U; i < read_length; ++i)
        {
            Arm_Hl04_Uart_Data_t data;
            if (arm_hl04_parser_feed(&g_arm_hl04_context.parser, read_buffer[i], &data) != 0U) arm_hl04_uart_publish(&data);
        }
    }

    ULONG now = tx_time_get();
    if (g_arm_hl04_context.uart_data_detected_logged == 0U && (now - g_arm_hl04_context.uart_diag_log_tick) >= TX_TIMER_TICKS_PER_SECOND)
    {
        BSP_UART_Diagnostics_t diag;
        g_arm_hl04_context.uart_diag_log_tick = now;
        if (BSP_UART_GetDiagnostics(g_arm_hl04_context.uart, &diag) == 0)
        {
            const char *classification = "NO_RX_ACTIVITY_OBSERVED";
            uint32_t new_rx_events = diag.rx_event_count - g_arm_hl04_context.uart_diag_prev_rx_event_count;
            uint32_t new_rx_bytes = diag.dma_bytes_received - g_arm_hl04_context.uart_diag_prev_bytes_received;
            uint32_t new_read_bytes = diag.fifo_bytes_read - g_arm_hl04_context.uart_diag_prev_bytes_read;
            uint32_t new_uart_errors = diag.uart_error_count - g_arm_hl04_context.uart_diag_prev_error_count;
            uint8_t dma_position_changed =
                ((g_arm_hl04_context.uart_diag_have_dma_position == 0U && diag.dma_position != 0U) ||
                 (g_arm_hl04_context.uart_diag_have_dma_position != 0U &&
                  diag.dma_position != g_arm_hl04_context.uart_diag_prev_dma_position));

            if (diag.rx_active == 0U || diag.dma_active == 0U)
                classification = "RX_NOT_ACTIVE";
            else if (new_uart_errors != 0U)
                classification = "UART_HARDWARE_ERROR";
            else if (dma_position_changed != 0U && new_rx_events == 0U)
                classification = "DMA_MOVED_NO_CALLBACK";
            else if (new_rx_events != 0U && new_rx_bytes == 0U)
                classification = "RX_EVENT_NO_BYTE_DELTA";
            else if (new_rx_bytes != 0U && new_read_bytes == 0U)
                classification = "BYTES_RECEIVED_NOT_READ";
            else if (new_read_bytes != 0U && g_arm_hl04_context.parser.packet_count == 0U)
                classification = "BYTES_READ_NO_VALID_FRAME";

            g_arm_hl04_context.uart_diag_prev_dma_position = diag.dma_position;
            g_arm_hl04_context.uart_diag_prev_rx_event_count = diag.rx_event_count;
            g_arm_hl04_context.uart_diag_prev_bytes_received = diag.dma_bytes_received;
            g_arm_hl04_context.uart_diag_prev_bytes_read = diag.fifo_bytes_read;
            g_arm_hl04_context.uart_diag_prev_error_count = diag.uart_error_count;
            g_arm_hl04_context.uart_diag_have_dma_position = 1U;

            if (g_arm_hl04_context.parser.packet_count != 0U)
            {
                g_arm_hl04_context.uart_data_detected_logged = 1U;
                LOG_I("HL04 UART data detected: baud=%lu bytes=%lu events=%lu frames=%lu",
                      (unsigned long)g_arm_hl04_context.uart_current_baud,
                      (unsigned long)diag.dma_bytes_received, (unsigned long)diag.rx_event_count,
                      (unsigned long)g_arm_hl04_context.parser.packet_count);
            }
            else
            {
                LOG_W("HL04 RX[%s] +B=%lu +E=%lu +R=%lu fifo=%lu dma=%lu/%lu", classification,
                      (unsigned long)new_rx_bytes, (unsigned long)new_rx_events, (unsigned long)new_read_bytes,
                      (unsigned long)diag.fifo_depth, (unsigned long)diag.dma_position,
                      (unsigned long)diag.dma_buffer_size);
                LOG_W("HL04 ERR +%lu total=%lu last=%08lX", (unsigned long)new_uart_errors,
                      (unsigned long)diag.uart_error_count, (unsigned long)diag.last_error_code);
                LOG_W("HL04 ERRBITS ORE=%lu FE=%lu NE=%lu PE=%lu DMA=%lu",
                      (unsigned long)diag.overrun_error_count, (unsigned long)diag.frame_error_count,
                      (unsigned long)diag.noise_error_count, (unsigned long)diag.parity_error_count,
                      (unsigned long)diag.dma_error_count);
                LOG_W("HL04 DMA starts=%lu discard=%lu drop=%lu fail=%lu", (unsigned long)diag.dma_start_count,
                      (unsigned long)diag.fifo_bytes_discarded_on_restart, (unsigned long)diag.fifo_bytes_dropped,
                      (unsigned long)diag.rx_start_failure_count);
                LOG_W("HL04 HW rx=%02X dm=%02X pos=%lu/%lu ndtr=%lu", (unsigned)diag.uart_rx_state,
                      (unsigned)diag.dma_state, (unsigned long)diag.dma_position,
                      (unsigned long)diag.dma_buffer_size, (unsigned long)diag.dma_remaining);
                LOG_W("HL04 RAW0 n=%u: %02X %02X %02X %02X %02X %02X %02X %02X",
                      (unsigned)diag.recent_byte_count,
                      diag.recent_bytes[0], diag.recent_bytes[1], diag.recent_bytes[2], diag.recent_bytes[3],
                      diag.recent_bytes[4], diag.recent_bytes[5], diag.recent_bytes[6], diag.recent_bytes[7]);
                LOG_W("HL04 RAW1: %02X %02X %02X %02X %02X %02X %02X %02X",
                      diag.recent_bytes[8], diag.recent_bytes[9], diag.recent_bytes[10], diag.recent_bytes[11],
                      diag.recent_bytes[12], diag.recent_bytes[13], diag.recent_bytes[14], diag.recent_bytes[15]);
                LOG_W("HL04 PARSE state=%u idx=%u frames=%lu err=%lu", (unsigned)g_arm_hl04_context.parser.state,
                      (unsigned)g_arm_hl04_context.parser.index,
                      (unsigned long)g_arm_hl04_context.parser.packet_count,
                      (unsigned long)g_arm_hl04_context.parser.frame_error_count);
            }
        }
    }

#if (ARM_DUAL_IMU_UART_RECOVERY_ENABLE != 0)
    if (g_arm_hl04_context.parser.packet_count == 0U && g_arm_hl04_context.uart_recovery_attempted == 0U &&
        (tx_time_get() - g_arm_hl04_context.uart_start_tick) >= ARM_DUAL_IMU_UART_STARTUP_GRACE_MS)
    {
        g_arm_hl04_context.uart_recovery_attempted = 1U;
        uint32_t previous_baud = g_arm_hl04_context.uart_current_baud;
        uint32_t recovery_baud = (previous_baud == ARM_DUAL_IMU_UART_FALLBACK_BAUD)
                                     ? previous_baud
                                     : ARM_DUAL_IMU_UART_FALLBACK_BAUD;
        if (arm_hl04_uart_set_baud(recovery_baud) != 0U)
        {
            LOG_W("HL04 UART recovery: baud %lu -> %lu after %lu ms", (unsigned long)previous_baud,
                  (unsigned long)recovery_baud, (unsigned long)ARM_DUAL_IMU_UART_STARTUP_GRACE_MS);
        }
        else
        {
            LOG_E("HL04 UART recovery failed");
        }
    }
#endif

    arm_hl04_can_poll(&g_arm_hl04_context);
}
