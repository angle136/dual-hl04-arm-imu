/*
 * @Author: laladuduqq 2807523947@qq.com
 * @Date: 2026-05-07 10:43:56
 * @LastEditors: laladuduqq 2807523947@qq.com
 * @LastEditTime: 2026-05-07 10:44:50
 * @FilePath: /mas_embedded_threadx/board/bsp/LED/bsp_led.h
 * @Description:
 */
#ifndef _BSP_LED_H_
#define _BSP_LED_H_

#include <stdint.h>

#define LED_Black  0XFF000000
#define LED_White  0XFFFFFFFF
#define LED_Red    0XFFFF0000
#define LED_Green  0XFF00FF00
#define LED_Blue   0XFF0000FF
#define LED_Yellow 0XFFFFFF00


void BSP_LED_Init(void);

void BSP_LED_Show(uint32_t aRGB);

#endif // _BSP_LED_H_