/*
 * @Author: laladuduqq 2807523947@qq.com
 * @Date: 2026-01-14 16:49:17
 * @LastEditors: laladuduqq 2807523947@qq.com
 * @LastEditTime: 2026-05-05 10:00:00
 * @FilePath: /mas_embedded_threadx/board/bsp/PWM/bsp_pwm.h
 * @Description:
 */

#ifndef _BSP_PWM_H_
#define _BSP_PWM_H_

#include <stdint.h>

/* PWM工作模式枚举 */
typedef enum
{
    PWM_MODE_BLOCKING,
    PWM_MODE_IT,
    PWM_MODE_DMA
} PWM_Mode;

/* 对外不透明句柄 */
typedef struct PWM_Device PWM_Device;

/* PWM初始化配置结构体 */
typedef struct
{
    void    *htim;    /* 定时器句柄 (TIM_HandleTypeDef *) */
    uint32_t Channel; /* 通道 */
    uint32_t dutyx10; /* 占空比 */
    PWM_Mode Mode;    /* 工作模式 */
} PWM_Init_Config;

/**
 * @brief  初始化PWM
 * @param  config: PWM初始化配置结构体指针
 * @retval PWM_Device*：初始化成功返回PWM_Device指针，失败返回NULL
 */
PWM_Device *BSP_PWM_Device_Init(PWM_Init_Config *config);
/**
 * @brief  启动PWM输出
 * @param  dev: PWM设备指针
 * @param  data: PWM数据指针（用于DMA模式，可为NULL）
 * @param  len: 数据长度（用于DMA模式，可为0）
 * @retval uint8_t：0表示成功，其他值表示失败
 */
uint8_t BSP_PWM_Start(PWM_Device *dev, uint8_t *data, uint8_t len);
/**
 * @brief  停止PWM输出
 * @param  dev: PWM设备指针
 * @retval uint8_t：0表示成功，其他值表示失败
 */
uint8_t BSP_PWM_Stop(PWM_Device *dev);
/**
 * @brief  设置PWM占空比
 * @param  dev: PWM设备指针
 * @param  duty: 占空比 (0-1000, 对应0.0%-100.0%)
 * @retval uint8_t：0表示成功，其他值表示失败
 * @note duty参数范围0-1000，对应占空比0.0%-100.0%，例如：duty=500表示50.0%，duty=75表示7.5%
 */
uint8_t BSP_PWM_SetDutyCycle(PWM_Device *dev, uint16_t duty);
/**
 * @brief  设置PWM比较值（脉冲宽度）
 * @param  dev: PWM设备指针
 * @param  pulse: 比较值（脉冲宽度）
 * @retval uint8_t：0表示成功，其他值表示失败
 */
uint8_t BSP_PWM_SetPulse(PWM_Device *dev, uint32_t pulse);
/**
 * @brief  设置PWM分频数
 * @param  dev: PWM设备指针
 * @param  psc: 分频数
 * @retval uint8_t：0表示成功，其他值表示失败
 */
uint8_t BSP_PWM_SetPSC(PWM_Device *dev, uint32_t psc);
/**
 * @brief  等待PWM完成事件（用于中断和DMA模式）
 * @param  dev: PWM设备指针
 * @param  wait_option: 等待选项
 * @retval uint8_t：0表示成功，其他值表示失败或超时
 */
uint8_t BSP_PWM_WaitForComplete(PWM_Device *dev, uint32_t wait_option);

#endif /* _BSP_PWM_H_ */
