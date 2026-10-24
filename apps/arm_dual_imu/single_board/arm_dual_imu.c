#include "arm_dual_imu.h"

#include "bsp_def.h"
#include "bsp_uart.h"
#include "module_ins.h"
#include "tx_api.h"
#include "usart.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#define LOG_TAG "arm_dual_imu"
#define LOG_LVL LOG_LVL_INFO
#include "ulog_def.h"

#define ARM_DUAL_IMU_PI                  3.14159265358979323846f
#define ARM_DUAL_IMU_TWO_PI              (2.0f * ARM_DUAL_IMU_PI)
#define ARM_DUAL_IMU_UPPER_LENGTH_M      0.204f
#define ARM_DUAL_IMU_FOREARM_LENGTH_M    0.120f
#define ARM_DUAL_IMU_HL04_HEADER_0       0x5AU
#define ARM_DUAL_IMU_HL04_HEADER_1       0xA5U
#define ARM_DUAL_IMU_HL04_TAG            0x91U
#define ARM_DUAL_IMU_HL04_MIN_PAYLOAD    76U
#define ARM_DUAL_IMU_HL04_MAX_PAYLOAD    92U
#define ARM_DUAL_IMU_HL04_FRAME_CAPACITY 100U
#define ARM_DUAL_IMU_UART_BUFFER_SIZE    256U
#define ARM_DUAL_IMU_READ_BUFFER_SIZE    128U
#define ARM_DUAL_IMU_QUAT_NORM_MIN_SQ    0.25f
#define ARM_DUAL_IMU_QUAT_NORM_MAX_SQ    2.25f
#define ARM_DUAL_IMU_SINGULAR_EPSILON    1.0e-4f
#define ARM_DUAL_IMU_REACH_TOLERANCE_M   1.0e-3f

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
    uint8_t                 frame[ARM_DUAL_IMU_HL04_FRAME_CAPACITY];
    uint16_t                index;
    uint16_t                payload_length;
    uint16_t                expected_length;
    uint32_t                packet_count;
    uint32_t                crc_error_count;
    uint32_t                frame_error_count;
} Arm_Hl04_Parser_t;

typedef struct
{
    UART_Device         *uart;
    Arm_Hl04_Parser_t    parser;
    Arm_Dual_Imu_State_t state;
    float                upper_zero_transpose[3][3];
    float                forearm_zero_transpose[3][3];
    float                previous_joint_target[ARM_DUAL_IMU_IK_DOF];
    uint32_t             upper_last_rx_tick;
    uint32_t             both_ready_since_tick;
    uint8_t              have_upper_quaternion;
    uint8_t              have_previous_joint_target;
    uint8_t              calibrated;
    volatile uint8_t     zero_requested;
} Arm_Dual_Imu_Context_t;

static TX_THREAD                  g_arm_dual_imu_thread;
APPS_STACK_SECTION static uint8_t g_arm_dual_imu_stack[ARM_DUAL_IMU_TASK_STACK_SIZE];
BUFFER_SECTION static uint8_t     g_arm_dual_imu_uart_buffer[ARM_DUAL_IMU_UART_BUFFER_SIZE];
static Arm_Dual_Imu_Context_t     g_arm_dual_imu_context;

volatile Arm_Dual_Imu_State_t g_arm_dual_imu_state;

static float arm_dual_imu_clampf(float value, float minimum, float maximum)
{
    if (value < minimum) return minimum;
    if (value > maximum) return maximum;
    return value;
}

static void arm_dual_imu_matrix_identity(float matrix[3][3])
{
    memset(matrix, 0, sizeof(float) * 9U);
    matrix[0][0] = 1.0f;
    matrix[1][1] = 1.0f;
    matrix[2][2] = 1.0f;
}

static void arm_dual_imu_matrix_transpose(const float input[3][3], float output[3][3])
{
    for (uint8_t row = 0U; row < 3U; ++row)
    {
        for (uint8_t column = 0U; column < 3U; ++column) output[row][column] = input[column][row];
    }
}

static void arm_dual_imu_matrix_multiply(const float left[3][3], const float right[3][3], float output[3][3])
{
    float result[3][3] = {0};
    for (uint8_t row = 0U; row < 3U; ++row)
    {
        for (uint8_t column = 0U; column < 3U; ++column)
        {
            for (uint8_t k = 0U; k < 3U; ++k) result[row][column] += left[row][k] * right[k][column];
        }
    }
    memcpy(output, result, sizeof(result));
}

static void arm_dual_imu_matrix_vector(const float matrix[3][3], const float vector[3], float output[3])
{
    float result[3];
    for (uint8_t row = 0U; row < 3U; ++row)
    {
        result[row] = matrix[row][0] * vector[0] + matrix[row][1] * vector[1] + matrix[row][2] * vector[2];
    }
    memcpy(output, result, sizeof(result));
}

static uint8_t arm_dual_imu_normalize_quaternion(const float input[4], float output[4])
{
    if (input == NULL || output == NULL) return 0U;

    float norm_sq = 0.0f;
    for (uint8_t i = 0U; i < 4U; ++i)
    {
        if (!isfinite(input[i])) return 0U;
        norm_sq += input[i] * input[i];
    }

    if (norm_sq < ARM_DUAL_IMU_QUAT_NORM_MIN_SQ || norm_sq > ARM_DUAL_IMU_QUAT_NORM_MAX_SQ) return 0U;

    float inverse_norm = 1.0f / sqrtf(norm_sq);
    for (uint8_t i = 0U; i < 4U; ++i) output[i] = input[i] * inverse_norm;
    return 1U;
}

static void arm_dual_imu_quaternion_to_rotation(const float quaternion[4], float rotation[3][3])
{
    const float w = quaternion[0];
    const float x = quaternion[1];
    const float y = quaternion[2];
    const float z = quaternion[3];

    rotation[0][0] = 1.0f - 2.0f * (y * y + z * z);
    rotation[0][1] = 2.0f * (x * y - w * z);
    rotation[0][2] = 2.0f * (x * z + w * y);
    rotation[1][0] = 2.0f * (x * y + w * z);
    rotation[1][1] = 1.0f - 2.0f * (x * x + z * z);
    rotation[1][2] = 2.0f * (y * z - w * x);
    rotation[2][0] = 2.0f * (x * z - w * y);
    rotation[2][1] = 2.0f * (y * z + w * x);
    rotation[2][2] = 1.0f - 2.0f * (x * x + y * y);
}

static void arm_dual_imu_rotation_z(float angle, float rotation[3][3])
{
    float cosine = cosf(angle);
    float sine   = sinf(angle);
    arm_dual_imu_matrix_identity(rotation);
    rotation[0][0] = cosine;
    rotation[0][1] = -sine;
    rotation[1][0] = sine;
    rotation[1][1] = cosine;
}

static void arm_dual_imu_rotation_y(float angle, float rotation[3][3])
{
    float cosine = cosf(angle);
    float sine   = sinf(angle);
    arm_dual_imu_matrix_identity(rotation);
    rotation[0][0] = cosine;
    rotation[0][2] = sine;
    rotation[2][0] = -sine;
    rotation[2][2] = cosine;
}

static float arm_dual_imu_unwrap_near(float angle, float reference)
{
    while ((angle - reference) > ARM_DUAL_IMU_PI) angle -= ARM_DUAL_IMU_TWO_PI;
    while ((angle - reference) < -ARM_DUAL_IMU_PI) angle += ARM_DUAL_IMU_TWO_PI;
    return angle;
}

static uint8_t arm_dual_imu_solve_legacy_ik(Arm_Dual_Imu_Context_t *context, const float position[3], const float target_rotation[3][3],
                                            float joint_target[ARM_DUAL_IMU_IK_DOF])
{
    const float x         = position[0];
    const float y         = position[1];
    const float z         = position[2];
    const float radius_sq = x * x + y * y + z * z;
    const float radius    = sqrtf(radius_sq);
    const float reach_min = fabsf(ARM_DUAL_IMU_UPPER_LENGTH_M - ARM_DUAL_IMU_FOREARM_LENGTH_M);
    const float reach_max = ARM_DUAL_IMU_UPPER_LENGTH_M + ARM_DUAL_IMU_FOREARM_LENGTH_M;

    if (!isfinite(radius) || radius < (reach_min - ARM_DUAL_IMU_REACH_TOLERANCE_M) || radius > (reach_max + ARM_DUAL_IMU_REACH_TOLERANCE_M) ||
        radius < ARM_DUAL_IMU_SINGULAR_EPSILON)
    {
        return 0U;
    }

    float shoulder_cos =
        (ARM_DUAL_IMU_UPPER_LENGTH_M * ARM_DUAL_IMU_UPPER_LENGTH_M + radius_sq - ARM_DUAL_IMU_FOREARM_LENGTH_M * ARM_DUAL_IMU_FOREARM_LENGTH_M) /
        (2.0f * ARM_DUAL_IMU_UPPER_LENGTH_M * radius);
    float elbow_cos =
        (ARM_DUAL_IMU_UPPER_LENGTH_M * ARM_DUAL_IMU_UPPER_LENGTH_M + ARM_DUAL_IMU_FOREARM_LENGTH_M * ARM_DUAL_IMU_FOREARM_LENGTH_M - radius_sq) /
        (2.0f * ARM_DUAL_IMU_UPPER_LENGTH_M * ARM_DUAL_IMU_FOREARM_LENGTH_M);

    joint_target[0] = atan2f(y, x);
    joint_target[1] = acosf(arm_dual_imu_clampf(shoulder_cos, -1.0f, 1.0f)) + atan2f(z, sqrtf(x * x + y * y)) - ARM_DUAL_IMU_PI / 2.0f;
    joint_target[2] = acosf(arm_dual_imu_clampf(elbow_cos, -1.0f, 1.0f)) - ARM_DUAL_IMU_PI;

    float rotation_1[3][3];
    float rotation_2[3][3];
    float rotation_3[3][3];
    float rotation_12[3][3];
    float rotation_123[3][3];
    float rotation_123_transpose[3][3];
    float rotation_456[3][3];
    arm_dual_imu_rotation_z(joint_target[0], rotation_1);
    arm_dual_imu_rotation_y(joint_target[1], rotation_2);
    arm_dual_imu_rotation_y(joint_target[2], rotation_3);
    arm_dual_imu_matrix_multiply(rotation_1, rotation_2, rotation_12);
    arm_dual_imu_matrix_multiply(rotation_12, rotation_3, rotation_123);
    arm_dual_imu_matrix_transpose(rotation_123, rotation_123_transpose);
    arm_dual_imu_matrix_multiply(rotation_123_transpose, target_rotation, rotation_456);

    joint_target[4]   = atan2f(sqrtf(rotation_456[2][0] * rotation_456[2][0] + rotation_456[2][1] * rotation_456[2][1]), rotation_456[2][2]);
    float middle_sine = sinf(joint_target[4]);
    if (fabsf(middle_sine) < 1.0e-2f)
    {
        joint_target[3] = 0.0f;
        joint_target[5] = (joint_target[4] < ARM_DUAL_IMU_PI / 2.0f) ? atan2f(-rotation_456[0][1], rotation_456[0][0])
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
            joint_target[i] = arm_dual_imu_unwrap_near(joint_target[i], context->previous_joint_target[i]);
        }
    }

    memcpy(context->previous_joint_target, joint_target, sizeof(context->previous_joint_target));
    context->have_previous_joint_target = 1U;
    return 1U;
}

static uint16_t arm_hl04_crc16_update(uint16_t crc, const uint8_t *data, uint16_t length)
{
    for (uint16_t byte_index = 0U; byte_index < length; ++byte_index)
    {
        crc ^= (uint16_t)data[byte_index] << 8U;
        for (uint8_t bit = 0U; bit < 8U; ++bit) crc = (crc & 0x8000U) != 0U ? (uint16_t)((crc << 1U) ^ 0x1021U) : (uint16_t)(crc << 1U);
    }
    return crc;
}

static uint8_t arm_hl04_decode_frame(Arm_Hl04_Parser_t *parser, float quaternion[4])
{
    uint16_t crc        = arm_hl04_crc16_update(0U, parser->frame, 4U);
    crc                 = arm_hl04_crc16_update(crc, &parser->frame[6], parser->payload_length);
    uint16_t packet_crc = (uint16_t)((uint16_t)parser->frame[parser->expected_length - 2U] << 8U) | parser->frame[parser->expected_length - 1U];
    if (crc != packet_crc)
    {
        parser->crc_error_count++;
        return 0U;
    }
    if (parser->frame[6] != ARM_DUAL_IMU_HL04_TAG)
    {
        parser->frame_error_count++;
        return 0U;
    }

    float decoded[4];
    memcpy(&decoded[0], &parser->frame[66], sizeof(float));
    memcpy(&decoded[1], &parser->frame[70], sizeof(float));
    memcpy(&decoded[2], &parser->frame[74], sizeof(float));
    memcpy(&decoded[3], &parser->frame[78], sizeof(float));
    if (arm_dual_imu_normalize_quaternion(decoded, quaternion) == 0U)
    {
        parser->frame_error_count++;
        return 0U;
    }

    parser->packet_count++;
    return 1U;
}

static void arm_hl04_parser_reset(Arm_Hl04_Parser_t *parser)
{
    parser->state           = HL04_WAIT_HEADER_0;
    parser->index           = 0U;
    parser->payload_length  = 0U;
    parser->expected_length = 0U;
}

static uint8_t arm_hl04_parser_feed(Arm_Hl04_Parser_t *parser, uint8_t byte, float quaternion[4])
{
    switch (parser->state)
    {
    case HL04_WAIT_HEADER_0:
        if (byte == ARM_DUAL_IMU_HL04_HEADER_0)
        {
            parser->frame[0] = byte;
            parser->index    = 1U;
            parser->state    = HL04_WAIT_HEADER_1;
        }
        break;

    case HL04_WAIT_HEADER_1:
        if (byte == ARM_DUAL_IMU_HL04_HEADER_1)
        {
            parser->frame[1] = byte;
            parser->index    = 2U;
            parser->state    = HL04_READ_LENGTH_LOW;
        }
        else if (byte != ARM_DUAL_IMU_HL04_HEADER_0)
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
        parser->expected_length = (uint16_t)(parser->payload_length + 8U);
        if (parser->payload_length < ARM_DUAL_IMU_HL04_MIN_PAYLOAD || parser->payload_length > ARM_DUAL_IMU_HL04_MAX_PAYLOAD ||
            parser->expected_length > ARM_DUAL_IMU_HL04_FRAME_CAPACITY)
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
            uint8_t decoded = arm_hl04_decode_frame(parser, quaternion);
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

static void arm_dual_imu_publish(const Arm_Dual_Imu_State_t *state)
{
    const size_t payload_offset   = offsetof(Arm_Dual_Imu_State_t, initialized);
    uint32_t     odd_sequence     = (g_arm_dual_imu_state.sequence + 1U) | 1U;
    g_arm_dual_imu_state.sequence = odd_sequence;
    __DMB();
    memcpy((uint8_t *)(void *)&g_arm_dual_imu_state + payload_offset, (const uint8_t *)state + payload_offset, sizeof(*state) - payload_offset);
    __DMB();
    g_arm_dual_imu_state.sequence = odd_sequence + 1U;
}

static void arm_dual_imu_capture_zero(Arm_Dual_Imu_Context_t *context, const float upper_raw[3][3], const float forearm_raw[3][3])
{
    arm_dual_imu_matrix_transpose(upper_raw, context->upper_zero_transpose);
    arm_dual_imu_matrix_transpose(forearm_raw, context->forearm_zero_transpose);
    context->calibrated                 = 1U;
    context->zero_requested             = 0U;
    context->have_previous_joint_target = 0U;
    LOG_I("dual IMU neutral pose captured");
}

static void arm_dual_imu_calculate(Arm_Dual_Imu_Context_t *context)
{
    Arm_Dual_Imu_State_t *state = &context->state;
    const ULONG           now   = tx_time_get();
    const Ins_t          *ins   = Module_INS_get();
    float                 upper_raw[3][3];
    float                 forearm_raw[3][3];

    state->initialized             = 1U;
    state->upper_last_rx_tick      = context->upper_last_rx_tick;
    state->upper_packet_count      = context->parser.packet_count;
    state->upper_crc_error_count   = context->parser.crc_error_count;
    state->upper_frame_error_count = context->parser.frame_error_count;
    state->upper_imu_valid = (context->have_upper_quaternion != 0U && (now - context->upper_last_rx_tick) <= ARM_DUAL_IMU_UPPER_TIMEOUT_MS) ? 1U : 0U;
    state->forearm_imu_valid =
        (ins != NULL && ins->initialized != 0U && arm_dual_imu_normalize_quaternion(ins->q, state->forearm_quaternion) != 0U) ? 1U : 0U;

    if (state->upper_imu_valid != 0U) arm_dual_imu_quaternion_to_rotation(state->upper_quaternion, upper_raw);
    if (state->forearm_imu_valid != 0U) arm_dual_imu_quaternion_to_rotation(state->forearm_quaternion, forearm_raw);

    if (state->upper_imu_valid != 0U && state->forearm_imu_valid != 0U)
    {
        if (context->both_ready_since_tick == 0U) context->both_ready_since_tick = now;
#if (ARM_DUAL_IMU_AUTO_ZERO_ENABLE != 0)
        if (context->calibrated == 0U && (now - context->both_ready_since_tick) >= ARM_DUAL_IMU_AUTO_ZERO_DELAY_MS)
        {
            context->zero_requested = 1U;
        }
#endif
        if (context->zero_requested != 0U) arm_dual_imu_capture_zero(context, upper_raw, forearm_raw);
    }
    else
    {
        context->both_ready_since_tick = 0U;
    }

    state->calibrated = context->calibrated;
    state->pose_valid = 0U;
    state->ik_valid   = 0U;
    if (state->upper_imu_valid == 0U || state->forearm_imu_valid == 0U || context->calibrated == 0U)
    {
        arm_dual_imu_publish(state);
        return;
    }

    arm_dual_imu_matrix_multiply(context->upper_zero_transpose, upper_raw, state->upper_rotation);
    arm_dual_imu_matrix_multiply(context->forearm_zero_transpose, forearm_raw, state->forearm_rotation);

    const float upper_local[3]   = {ARM_DUAL_IMU_UPPER_LENGTH_M, 0.0f, 0.0f};
    const float forearm_local[3] = {ARM_DUAL_IMU_FOREARM_LENGTH_M, 0.0f, 0.0f};
    float       forearm_vector[3];
    arm_dual_imu_matrix_vector(state->upper_rotation, upper_local, state->elbow_position_m);
    arm_dual_imu_matrix_vector(state->forearm_rotation, forearm_local, forearm_vector);

    /* ponytail: 保留 YUAN 版上臂 Z 镜像；闭环前应替换成实测的完整 3x3 外参。 */
    state->elbow_position_m[2] = -state->elbow_position_m[2];
    for (uint8_t i = 0U; i < 3U; ++i) state->target_position_m[i] = state->elbow_position_m[i] + forearm_vector[i];

    /* YUAN 版末端坐标变换，等价于绕 Y 轴旋转 -90 度。 */
    static const float end_frame_map[3][3] = {
        {0.0f, 0.0f, -1.0f},
        {0.0f, 1.0f, 0.0f},
        {1.0f, 0.0f, 0.0f},
    };
    arm_dual_imu_matrix_multiply(end_frame_map, state->forearm_rotation, state->target_rotation);
    state->pose_valid = 1U;
    state->ik_valid   = arm_dual_imu_solve_legacy_ik(context, state->target_position_m, state->target_rotation, state->legacy_joint_target_rad);
    arm_dual_imu_publish(state);
}

static void arm_dual_imu_task_entry(ULONG thread_input)
{
    (void)thread_input;
    uint8_t read_buffer[ARM_DUAL_IMU_READ_BUFFER_SIZE];

    while (1)
    {
        uint32_t read_length = 0U;
        int read_status = BSP_UART_Read(g_arm_dual_imu_context.uart, read_buffer, sizeof(read_buffer), &read_length, ARM_DUAL_IMU_READ_TIMEOUT_MS);
        if (read_status > 0)
        {
            for (uint32_t i = 0U; i < read_length; ++i)
            {
                float quaternion[4];
                if (arm_hl04_parser_feed(&g_arm_dual_imu_context.parser, read_buffer[i], quaternion) != 0U)
                {
                    memcpy(g_arm_dual_imu_context.state.upper_quaternion, quaternion, sizeof(quaternion));
                    g_arm_dual_imu_context.have_upper_quaternion = 1U;
                    g_arm_dual_imu_context.upper_last_rx_tick    = tx_time_get();
                }
            }
        }
        arm_dual_imu_calculate(&g_arm_dual_imu_context);
    }
}

void arm_dual_imu_init(void)
{
    memset(&g_arm_dual_imu_context, 0, sizeof(g_arm_dual_imu_context));
    memset((void *)&g_arm_dual_imu_state, 0, sizeof(g_arm_dual_imu_state));
    arm_hl04_parser_reset(&g_arm_dual_imu_context.parser);
    arm_dual_imu_matrix_identity(g_arm_dual_imu_context.upper_zero_transpose);
    arm_dual_imu_matrix_identity(g_arm_dual_imu_context.forearm_zero_transpose);

    ARM_DUAL_IMU_UART.Init.BaudRate = ARM_DUAL_IMU_UART_BAUD;
    if (HAL_UART_Init(&ARM_DUAL_IMU_UART) != HAL_OK)
    {
        LOG_E("HL04 UART init failed");
        return;
    }

    UART_Device_init_config uart_config = {
        .huart           = &ARM_DUAL_IMU_UART,
        .rx_buf          = g_arm_dual_imu_uart_buffer,
        .rx_buf_size     = sizeof(g_arm_dual_imu_uart_buffer),
        .expected_rx_len = 0U,
        .rx_mode         = UART_MODE_DMA,
        .tx_mode         = UART_MODE_BLOCKING,
    };
    g_arm_dual_imu_context.uart = BSP_UART_Device_Init(&uart_config);
    if (g_arm_dual_imu_context.uart == NULL)
    {
        LOG_E("HL04 UART device registration failed");
        return;
    }

    UINT status =
        tx_thread_create(&g_arm_dual_imu_thread, "arm_dual_imu", arm_dual_imu_task_entry, 0U, g_arm_dual_imu_stack, sizeof(g_arm_dual_imu_stack),
                         ARM_DUAL_IMU_TASK_PRIORITY, ARM_DUAL_IMU_TASK_PRIORITY, TX_NO_TIME_SLICE, TX_AUTO_START);
    if (status != TX_SUCCESS)
    {
        LOG_E("dual IMU thread create failed: %u", (unsigned)status);
        return;
    }

    LOG_I("dual IMU initialized: HL04 baud=%lu, auto zero=%u", (unsigned long)ARM_DUAL_IMU_UART_BAUD, (unsigned)ARM_DUAL_IMU_AUTO_ZERO_ENABLE);
}

void arm_dual_imu_request_zero(void) { g_arm_dual_imu_context.zero_requested = 1U; }

uint8_t arm_dual_imu_get_state(Arm_Dual_Imu_State_t *state)
{
    if (state == NULL) return 0U;

    for (uint8_t attempt = 0U; attempt < 8U; ++attempt)
    {
        uint32_t sequence_before = g_arm_dual_imu_state.sequence;
        if ((sequence_before & 1U) != 0U) continue;
        __DMB();
        memcpy(state, (const void *)&g_arm_dual_imu_state, sizeof(*state));
        __DMB();
        uint32_t sequence_after = g_arm_dual_imu_state.sequence;
        if (sequence_before == sequence_after && (sequence_after & 1U) == 0U)
        {
            state->sequence = sequence_after;
            return 1U;
        }
    }
    return 0U;
}

void arm_dual_imu_log_status(void)
{
    Arm_Dual_Imu_State_t state;
    if (arm_dual_imu_get_state(&state) == 0U)
    {
        LOG_W("snapshot busy");
        return;
    }

    LOG_I("upper=%u forearm=%u cal=%u pose=%u ik=%u pkt=%lu crc=%lu frame=%lu p=[%.3f %.3f %.3f] q=[%.3f %.3f %.3f %.3f %.3f %.3f]",
          (unsigned)state.upper_imu_valid, (unsigned)state.forearm_imu_valid, (unsigned)state.calibrated, (unsigned)state.pose_valid,
          (unsigned)state.ik_valid, (unsigned long)state.upper_packet_count, (unsigned long)state.upper_crc_error_count,
          (unsigned long)state.upper_frame_error_count, state.target_position_m[0], state.target_position_m[1], state.target_position_m[2],
          state.legacy_joint_target_rad[0], state.legacy_joint_target_rad[1], state.legacy_joint_target_rad[2], state.legacy_joint_target_rad[3],
          state.legacy_joint_target_rad[4], state.legacy_joint_target_rad[5]);
}
