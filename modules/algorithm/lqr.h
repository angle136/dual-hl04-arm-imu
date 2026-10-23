/*
 * @Author: laladuduqq 2807523947@qq.com
 * @Date: 2026-05-10 17:35:20
 * @LastEditors: laladuduqq 2807523947@qq.com
 * @LastEditTime: 2026-05-23 10:17:18
 * @FilePath: /mas_embedded_threadx/modules/algorithm/lqr.h
 * @Description:
 */
#ifndef __LQR_H
#define __LQR_H
#include "stdint.h"

/* LQR 初始化配置 */
typedef struct
{
    float   K[2];      /* 增益矩阵 */
    uint8_t state_dim; /* 状态维度: 1=角速度控制, 2=角度+角速度 */
} LQR_Init_Config_s;

/* LQR 运行实例 */
typedef struct
{
    float        K[2];          /* 增益矩阵 */
    uint8_t      state_dim;     /* 状态维度 */
    float        ref;           /* 设定值 */
    float        measure;       /* 测量值 */
    float        output;        /* 输出值 */
} LQRInstance;

/**
 * @brief 初始化 LQR 实例
 * @param lqr    LQR 实例指针
 * @param config LQR 初始化配置
 */
void LQRInit(LQRInstance *lqr, LQR_Init_Config_s *config);

/**
 * @brief 计算 LQR 输出
 * @param lqr                   LQR 实例指针
 * @param rad_degree            角度 (rad)
 * @param rad_angular_velocity  角速度 (rad/s)
 * @param rad_ref               参考角度 (rad)
 * @return float                LQR 输出 (Nm)
 */
float LQRCalculate(LQRInstance *lqr, float rad_degree, float rad_angular_velocity, float rad_ref);

#endif
