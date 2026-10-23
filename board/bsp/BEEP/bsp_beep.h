/*
 * @Author: laladuduqq 2807523947@qq.com
 * @Date: 2026-05-07 10:45:05
 * @LastEditors: laladuduqq 2807523947@qq.com
 * @LastEditTime: 2026-05-07 10:45:09
 * @FilePath: /mas_embedded_threadx/board/bsp/BEEP/bsp_beep.h
 * @Description:
 */
#ifndef _BSP_BEEP_H_
#define _BSP_BEEP_H_

#include <stdint.h>

/**
 * @brief 蜂鸣器模块初始化
 */
void BSP_BEEP_Init();
/**
 * @description: 蜂鸣器开启
 * @return {*}
 */
void BSP_BEEP_Start();
/**
 * @description: 蜂鸣器关闭
 * @return {*}
 */
void BSP_BEEP_Stop();
/**
 * @description: 蜂鸣器音调设置
 * @param {uint16_t} psc，音调
 * @param {uint16_t} pwm，音色
 * @return {*}
 */
void BSP_BEEP_Set(uint16_t psc, uint16_t pwm);

#endif // _BSP_BEEP_H_
