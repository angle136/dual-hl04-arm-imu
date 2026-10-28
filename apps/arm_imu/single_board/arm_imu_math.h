#ifndef _ARM_IMU_MATH_H_
#define _ARM_IMU_MATH_H_

#include <stdint.h>

/** 将浮点数限制在指定闭区间内。 */
float arm_imu_math_clampf(float value, float minimum, float maximum);

/** 生成三阶单位旋转矩阵。 */
void arm_imu_math_matrix_identity(float matrix[3][3]);

/** 计算三阶矩阵的转置；output 不得与 input 重叠。 */
void arm_imu_math_matrix_transpose(const float input[3][3], float output[3][3]);

/** 计算两个三阶矩阵的乘积，output 允许与输入矩阵重叠。 */
void arm_imu_math_matrix_multiply(const float left[3][3], const float right[3][3], float output[3][3]);

/** 用三阶旋转矩阵变换三维向量，output 允许与 vector 重叠。 */
void arm_imu_math_matrix_vector(const float matrix[3][3], const float vector[3], float output[3]);

/** 校验并归一化顺序为 w、x、y、z 的四元数。 */
uint8_t arm_imu_math_normalize_quaternion(const float input[4], float output[4]);

/** 将顺序为 w、x、y、z 的单位四元数转换为三阶旋转矩阵。 */
void arm_imu_math_quaternion_to_rotation(const float quaternion[4], float rotation[3][3]);

/** 构造绕 Z 轴旋转的三阶矩阵。 */
void arm_imu_math_rotation_z(float angle, float rotation[3][3]);

/** 构造绕 Y 轴旋转的三阶矩阵。 */
void arm_imu_math_rotation_y(float angle, float rotation[3][3]);

#endif /* _ARM_IMU_MATH_H_ */
