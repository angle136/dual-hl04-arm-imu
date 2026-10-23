/*
 * @Description: 机器人通用函数实现模板 (云台板)
 *
 * 参考: sentry/gimbal_board/robot_func/robot_func.c
 */

#include "robot_func.h"
#include "module_remote.h"
#include <stdint.h>
#include <string.h>

int16_t CalcOffsetAngle(float getyawangle)
{
    float offset_ecd;
    const float ECD_MAX  = 8191.0f;
    const float ECD_HALF = 4095.5f;

    offset_ecd = getyawangle - (float)YAW_CHASSIS_ALIGN_ECD;
    while (offset_ecd > ECD_HALF) offset_ecd -= ECD_MAX;
    while (offset_ecd < -ECD_HALF) offset_ecd += ECD_MAX;

    return (int16_t)offset_ecd;
}

void RemoteControlSet(Chassis_Ctrl_Cmd_t *Chassis_Ctrl,
                      Shoot_Ctrl_Cmd_t  *Shoot_Ctrl,
                      Gimbal_Ctrl_Cmd_t *Gimbal_Ctrl)
{
    if (!Chassis_Ctrl || !Shoot_Ctrl || !Gimbal_Ctrl) return;

    uint8_t state = Module_Remote_get_offline_status();

    if (state & 0x01)
    {
        Chassis_Ctrl->vx = (float)Module_Remote_get_channel(2) / (float)(SBUS_CHX_DOWN - SBUS_CHX_BIAS);
        Chassis_Ctrl->vy = (float)Module_Remote_get_channel(1) / (float)(SBUS_CHX_DOWN - SBUS_CHX_BIAS);

        /* TODO: 自定义底盘模式切换 (拨杆 CH8) */
        Chassis_Ctrl->chassis_mode = chassis_follow_gimbal_yaw;

        Gimbal_Ctrl->gimbal_mode = gimbal_gyro_mode;
        Gimbal_Ctrl->yaw        -= 0.001f * (float)Module_Remote_get_channel(4);
        Gimbal_Ctrl->pitch      += 0.001f * (float)Module_Remote_get_channel(3);

        /* TODO: 自定义云台/发射模式切换 (拨杆 CH5/CH6/CH7) */
        Shoot_Ctrl->shoot_mode    = shoot_off;
        Shoot_Ctrl->friction_mode = friction_off;
        Shoot_Ctrl->load_mode     = load_stop;
    }
    else
    {
        Gimbal_Ctrl->gimbal_mode   = gimbal_zero_force;
        Chassis_Ctrl->chassis_mode = chassis_zero_force;
        Shoot_Ctrl->shoot_mode     = shoot_off;
        Shoot_Ctrl->friction_mode  = friction_off;
        Shoot_Ctrl->load_mode      = load_stop;
        memset(Chassis_Ctrl, 0, sizeof(Chassis_Ctrl_Cmd_t));
    }
}
