/*
 * @Description: 机器人通用函数模板 (云台板)
 *
 * 包含:
 *   - CalcOffsetAngle(): 计算云台与底盘对齐的编码器差值
 *   - RemoteControlSet(): 遥控器输入 → 底盘/云台/发射控制命令
 *
 * 使用说明:
 *   1. RemoteControlSet() 的通道映射需要按实际遥控器配置修改
 *   2. 参考 sentry/gimbal_board/robot_func/robot_func.c
 */

#ifndef _ROBOT_FUNC_H_
#define _ROBOT_FUNC_H_

#include "<robot>_def.h"   /* TODO: 改为 <你的机器人>_def.h */

int16_t CalcOffsetAngle(float getyawangle);

void RemoteControlSet(Chassis_Ctrl_Cmd_t *Chassis_Ctrl,
                      Shoot_Ctrl_Cmd_t  *Shoot_Ctrl,
                      Gimbal_Ctrl_Cmd_t *Gimbal_Ctrl);

#endif // _ROBOT_FUNC_H_
