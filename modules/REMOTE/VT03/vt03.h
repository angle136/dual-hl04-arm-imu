/*
 * @Author: laladuduqq 2807523947@qq.com
 * @Date: 2026-05-10 17:32:15
 * @LastEditors: laladuduqq 2807523947@qq.com
 * @LastEditTime: 2026-05-14 20:12:34
 * @FilePath: /mas_embedded_threadx/modules/REMOTE/VT03/vt03.h
 * @Description:
 */
#ifndef _VT03_H_
#define _VT03_H_

#include "module_remote.h"
#include <stdint.h>
#include "module_offline.h"



/**
 * @brief VT03 初始化
 * @param out_offline 输出离线设备句柄指针，NULL 则不输出
 * @return 0 成功, -1 失败
 */
int8_t remote_vt03_init(Offline_Device **out_offline);

/**
 * @brief VT03 单帧解码
 * @param data 统一遥控数据结构指针
 * @param offline 离线设备句柄 (用于心跳上报)
 */
void remote_vt03_decode(Remote_Data_t *data, Offline_Device *offline);

#endif // _VT03_H_