/*
 * @Author: laladuduqq 2807523947@qq.com
 * @Date: 2026-05-10 17:32:15
 * @LastEditors: laladuduqq 2807523947@qq.com
 * @LastEditTime: 2026-05-14 20:11:53
 * @FilePath: /mas_embedded_threadx/modules/REMOTE/SBUS/sbus.h
 * @Description:
 */
#ifndef _SBUS_H_
#define _SBUS_H_

#include "module_remote.h"
#include <stdint.h>
#include "module_offline.h"

/**
 * @brief SBUS 初始化 (配置 UART + 注册 offline)
 * @param out_offline 输出离线设备句柄指针，NULL 则不输出
 * @return 0 成功, -1 失败
 */
int8_t remote_sbus_init(Offline_Device **out_offline);

/**
 * @brief SBUS 单帧解码, 直接写入统一结构体
 * @param data 统一遥控数据结构指针
 * @param offline 离线设备句柄 (用于心跳上报)
 */
void remote_sbus_decode(Remote_Data_t *data, Offline_Device *offline);

#endif // _SBUS_H_