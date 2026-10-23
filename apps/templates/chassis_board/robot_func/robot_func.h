/*
 * @Description: 机器人通用函数模板 (底盘板)
 *
 * 包含:
 *   - handle_referee_data(): 裁判系统数据填充板间通讯结构体
 *
 * 使用说明:
 *   1. 需要 #include "module_referee.h" 和 "referee_protocol.h"
 *   2. 参考 sentry/chassis_board/robot_func/robot_func.c
 */

#ifndef _ROBOT_FUNC_H_
#define _ROBOT_FUNC_H_

#include "<robot>_def.h"   /* TODO: 改为 <你的机器人>_def.h */

/**
 * @brief 从裁判系统获取数据并填充板间通讯结构体
 */
void handle_referee_data(ChassisToGimbal_referee_t *referee_data);

#endif // _ROBOT_FUNC_H_
