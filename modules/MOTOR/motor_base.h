/*
 * @Author: laladuduqq 2807523947@qq.com
 * @Date: 2026-05-11 17:00:00
 * @FilePath: /mas_embedded_threadx/modules/MOTOR/motor_base.h
 * @Description:
 */
#ifndef _MOTOR_BASE_H_
#define _MOTOR_BASE_H_

#include "module_offline.h"
#include "motor_def.h"
#include <stdint.h>

typedef struct Motor_Base Motor_Base;

struct Motor_Base
{
    Motor_Base       *next;
    Motor_Type_e      type;
    Motor_Transport_e transport;

    Motor_Info_s       info;
    Motor_Setting_s    setting;
    Motor_Controller_s controller;
    Motor_Measure_s    measure;
    Motor_Safety_Limit_s safety;
    Offline_Device    *offline_dev;
    const char        *name;
    volatile uint8_t   feedback_valid;

    void *transport_dev; /* 底层设备句柄 (Can_Device / UART_Device / PWM_Device) */

    /* 原始位置读取接口；由具体电机驱动实现，供底层安全限位使用 */
    int32_t (*GetRawPosition)(Motor_Base *motor);

    /* 输出应用 → 传输层 (协议层实现) */
    void (*Apply)(Motor_Base *motor);
};

#define MOTOR_GET_DERIVED(base_ptr, derived_type) ((derived_type *)(base_ptr))

/**
 * @brief 注册电机对象到全局链表。
 *
 * @param motor 输入，电机实例基类指针。
 *
 * @retval 无。
 *
 * @note 调用关系：由各电机驱动 Init 函数调用；注册后由 Motor_ControlAll() 和 Motor_ApplyAll() 统一调度。
 */
void Motor_Register(Motor_Base *motor);

/**
 * @brief 控制计算阶段：统一计算所有已注册电机的输出扭矩。
 *
 * @param 无。
 *
 * @retval 无。
 *
 * @note 调用关系：由 motor task 周期调用；本函数不发送 CAN，只更新 controller.output_torque。
 * @note 行为说明：离线/禁用电机清零，开环电机跳过闭环算法，闭环电机调用统一算法计算。
 */
void Motor_ControlAll(void);

/**
 * @brief 输出应用阶段：各电机把输出应用到传输层。
 *
 * @param 无。
 *
 * @retval 无。
 *
 * @note 调用关系：由 motor task 在 Motor_ControlAll() 和 PowerControl_Update() 后调用。
 * @note 调度策略：内部按 CAN 槽位轮询调用各电机 Apply()，保证同一条 CAN 内部相邻控制帧之间留出等待。
 */
void Motor_ApplyAll(void);
/**
 * @brief 启动电机
 */
void Motor_Start(Motor_Base *motor);
/**
 * @brief 停止电机
 */
void Motor_Stop(Motor_Base *motor);
/**
 * @brief 设置电机参考值
 */
void Motor_SetRef(Motor_Base *motor, float ref);
/**
 * @brief 改变电机反馈源
 */
void Motor_ChangeFeed(Motor_Base *motor, Closeloop_Type_e loop, uint8_t feedback_source);
/**
 * @brief 设置电机外环控制模式
 */
void Motor_OuterLoop(Motor_Base *motor, Closeloop_Type_e outer_loop);
/**
 * @brief 设置电机前馈扭矩
 */
void Motor_SetForwardTorque(Motor_Base *motor, float torque);
/**
 * @brief 设置电机输出扭矩
 */
void Motor_SetOutputTorque(Motor_Base *motor, float torque);

/**
 * @brief 查询电机底层安全故障锁死状态。
 *
 * @param 无。
 *
 * @return uint8_t 返回 1 表示已触发安全故障，返回 0 表示未触发。
 *
 * @note 调用关系：供 APP 或调试代码只读查询；故障触发后不在运行期自动清除。
 */
uint8_t Motor_SafetyFaultActive(void);

/**
 * @brief 执行单台电机的最终发送前安全校验。
 *
 * @param motor 输入，需要执行最终校验的电机基类指针。
 *
 * @return uint8_t 返回 1 表示允许当前电机继续执行 Apply；返回 0 表示本台电机已被安全失能，
 *                 驱动层应立即走零输出分支。
 *
 * @note 调用关系：由具体电机驱动的 Apply() 入口调用，位置应在调用 MIT/位置/速度/电流
 *       协议编码函数之前。全局预扫描仍由 Motor_ApplyAll() 负责。
 * @note 安全策略：本接口只处理当前电机的最终拦截，不替代全局预扫描；下一周期预扫描仍会负责全局急停。
 */
uint8_t Motor_SafetyCheckBeforeApply(Motor_Base *motor);

#endif /* _MOTOR_BASE_H_ */
