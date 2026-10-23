#ifndef _VT02_H_
#define _VT02_H_

#include "module_remote.h"
#include <stdint.h>
#include "module_offline.h"

/**
 * @brief VT02 初始化
 * @param out_offline 输出离线设备句柄指针，NULL 则不输出
 * @return 0 成功, -1 失败
 */
int8_t remote_vt02_init(Offline_Device **out_offline);

/**
 * @brief VT02 单帧解码, 直接写入统一结构体
 * @param data 统一遥控数据结构指针
 * @param offline 离线设备句柄 (用于心跳上报)
 */
void remote_vt02_decode(Remote_Data_t *data, Offline_Device *offline);

#endif // _VT02_H_