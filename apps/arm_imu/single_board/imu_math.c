#include "imu_math.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#define ARM_IMU_QUAT_NORM_MIN_SQ 0.25f
#define ARM_IMU_QUAT_NORM_MAX_SQ 2.25f

/**
 * @brief 将浮点数限制在指定闭区间内。
 * @param value 输入值。
 * @param minimum 区间下限。
 * @param maximum 区间上限。
 * @return 限幅后的浮点数。
 * @note 调用关系：由解析逆解计算余弦值时调用，避免浮点误差导致 acosf() 域错误。
 */
float arm_imu_math_clampf(float value, float minimum, float maximum)
{
    if (value < minimum) return minimum;
    if (value > maximum) return maximum;
    return value;
}

/**
 * @brief 生成三阶单位旋转矩阵。
 * @param matrix 输出 3x3 单位矩阵。
 * @retval 无。
 * @note 调用关系：由位姿解算初始化及单轴旋转矩阵构造函数调用。
 */
void arm_imu_math_matrix_identity(float matrix[3][3])
{
    memset(matrix, 0, sizeof(float) * 9U);
    matrix[0][0] = 1.0f;
    matrix[1][1] = 1.0f;
    matrix[2][2] = 1.0f;
}

/**
 * @brief 计算三阶矩阵的转置。
 * @param input 输入 3x3 矩阵。
 * @param output 输出转置后的 3x3 矩阵；不得与 input 重叠。
 * @retval 无。
 * @note 调用关系：由零位捕获和腕部相对旋转分解调用。
 */
void arm_imu_math_matrix_transpose(const float input[3][3], float output[3][3])
{
    for (uint8_t row = 0U; row < 3U; ++row)
    {
        for (uint8_t column = 0U; column < 3U; ++column) output[row][column] = input[column][row];
    }
}

/**
 * @brief 计算两个三阶矩阵的乘积。
 * @param left 左乘 3x3 矩阵。
 * @param right 右乘 3x3 矩阵。
 * @param output 输出乘积矩阵，允许与输入矩阵重叠。
 * @retval 无。
 * @note 调用关系：由相对姿态解算、末端坐标变换和解析逆解调用。
 */
void arm_imu_math_matrix_multiply(const float left[3][3], const float right[3][3], float output[3][3])
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

/**
 * @brief 用三阶旋转矩阵变换三维向量。
 * @param matrix 输入 3x3 旋转矩阵。
 * @param vector 输入三维向量。
 * @param output 输出变换后的三维向量，允许与 vector 重叠。
 * @retval 无。
 * @note 调用关系：由三 IMU 位姿解算将本地连杆向量转换到基坐标系。
 */
void arm_imu_math_matrix_vector(const float matrix[3][3], const float vector[3], float output[3])
{
    float result[3];
    for (uint8_t row = 0U; row < 3U; ++row)
    {
        result[row] = matrix[row][0] * vector[0] + matrix[row][1] * vector[1] + matrix[row][2] * vector[2];
    }
    memcpy(output, result, sizeof(result));
}

/**
 * @brief 校验并归一化四元数。
 * @param input 输入四元数，顺序为 w、x、y、z。
 * @param output 输出单位四元数，允许与 input 重叠。
 * @return 1 表示输入有限且模长有效，0 表示参数或数据无效。
 * @note 调用关系：由 UART/CAN 解码和周期位姿计算调用，作为传感器数据有效性门槛。
 */
uint8_t arm_imu_math_normalize_quaternion(const float input[4], float output[4])
{
    if (input == NULL || output == NULL) return 0U;

    float norm_sq = 0.0f;
    for (uint8_t i = 0U; i < 4U; ++i)
    {
        if (!isfinite(input[i])) return 0U;
        norm_sq += input[i] * input[i];
    }
    if (norm_sq < ARM_IMU_QUAT_NORM_MIN_SQ || norm_sq > ARM_IMU_QUAT_NORM_MAX_SQ) return 0U;

    float inverse_norm = 1.0f / sqrtf(norm_sq);
    for (uint8_t i = 0U; i < 4U; ++i) output[i] = input[i] * inverse_norm;
    return 1U;
}

/**
 * @brief 将单位四元数转换为三阶旋转矩阵。
 * @param quaternion 输入单位四元数，顺序为 w、x、y、z。
 * @param rotation 输出 3x3 旋转矩阵。
 * @retval 无。
 * @note 调用关系：由三 IMU 周期计算在传感器数据有效时调用。
 */
void arm_imu_math_quaternion_to_rotation(const float quaternion[4], float rotation[3][3])
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

/**
 * @brief 构造绕 Z 轴旋转的三阶矩阵。
 * @param angle 输入旋转角，单位 rad。
 * @param rotation 输出 3x3 旋转矩阵。
 * @retval 无。
 * @note 调用关系：由解析逆解构造前三个关节的正向旋转链。
 */
void arm_imu_math_rotation_z(float angle, float rotation[3][3])
{
    float cosine = cosf(angle);
    float sine   = sinf(angle);
    arm_imu_math_matrix_identity(rotation);
    rotation[0][0] = cosine;
    rotation[0][1] = -sine;
    rotation[1][0] = sine;
    rotation[1][1] = cosine;
}

/**
 * @brief 构造绕 Y 轴旋转的三阶矩阵。
 * @param angle 输入旋转角，单位 rad。
 * @param rotation 输出 3x3 旋转矩阵。
 * @retval 无。
 * @note 调用关系：由解析逆解构造肩部和肘部旋转矩阵。
 */
void arm_imu_math_rotation_y(float angle, float rotation[3][3])
{
    float cosine = cosf(angle);
    float sine   = sinf(angle);
    arm_imu_math_matrix_identity(rotation);
    rotation[0][0] = cosine;
    rotation[0][2] = sine;
    rotation[2][0] = -sine;
    rotation[2][2] = cosine;
}
