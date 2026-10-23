/*
 * @Author: laladuduqq 2807523947@qq.com
 * @Date: 2026-05-09 20:31:13
 * @LastEditors: laladuduqq 2807523947@qq.com
 * @LastEditTime: 2026-05-14 19:39:14
 * @FilePath: /mas_embedded_threadx/board/bsp/USB/usbd_cdc_acm_user.h
 * @Description:
 */
#ifndef USBD_CDC_ACM_USER_H
#define USBD_CDC_ACM_USER_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/**
 * @brief  初始化 CDC ACM 设备
 * @param busid    USB 总线 ID（0 起始）
 * @param reg_base USB OTG 外设寄存器基地址
 */
void cdc_acm_init(uint8_t busid, uintptr_t reg_base);

/**
 * @brief  通过 CDC ACM 批量 IN 端点发送数据
 * @param data    发送数据指针
 * @param len     发送字节数
 * @param timeout 发送超时时间
 * @return        true: 成功发送数据  false: 超时或参数无效
 */
int cdc_acm_send(const uint8_t *data, uint32_t len, uint32_t timeout);

/**
 * @brief  通过 CDC ACM 批量 OUT 端点接收数据
 * @param data  接收缓冲区指针
 * @param rx_len 接收字节数指针
 * @param timeout  超时时间
 * @return      true: 成功接收数据  false: 超时或参数无效
 */
int cdc_acm_recv(uint8_t *data, uint32_t buf_size, uint32_t *rx_len, uint32_t timeout);

#endif
