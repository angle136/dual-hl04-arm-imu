#include "motor_base.h"
#include "bsp_can.h"
#include "bsp_dwt.h"
#include "motor_algorithm.h"

#define LOG_TAG "motor_base"
#define LOG_LVL LOG_LVL_INFO
#include "ulog_def.h"

#define MOTOR_CAN_SLOT_GAP_S 0.0002f

#ifndef ARM_SOFTWARE_LIMIT_ENABLE
#define ARM_SOFTWARE_LIMIT_ENABLE 1U
#endif

static Motor_Base *g_motor_list = NULL;
static volatile uint8_t g_motor_safety_fault = 0U;

/**
 * @brief 清零单个电机的控制输出和积分状态。
 *
 * @param motor 输入，需要清零的电机基类指针。
 *
 * @retval 无。
 *
 * @note 调用关系：由 Motor_ControlAll()、motor_emergency_stop_all() 和安全故障路径调用。
 */
static void motor_clear_output(Motor_Base *motor)
{
    if (motor == NULL) return;

    motor->controller.output           = 0.0f;
    motor->controller.output_torque    = 0.0f;
    motor->controller.feedforward_torque = 0.0f;
    motor->controller.speed_PID.Output = 0.0f;
    motor->controller.speed_PID.Iout   = 0.0f;
    motor->controller.angle_PID.Output = 0.0f;
    motor->controller.angle_PID.Iout   = 0.0f;
}

/**
 * @brief 根据电机配置的最大力矩限制当前输出力矩。
 *
 * @param motor 输入，需要限幅的电机基类指针。
 *
 * @retval 无。
 *
 * @note 调用关系：由 Motor_SetOutputTorque() 和 Motor_ApplyAll() 调用。
 * @note 安全策略：当 info.max_torque > 0 时，将 controller.output_torque
 *       硬限制到 [-max_torque, max_torque]；max_torque <= 0 表示保持旧行为，不启用该限幅。
 */
static void motor_limit_output_torque(Motor_Base *motor)
{
    if (motor == NULL) return;
    if (motor->info.max_torque <= 0.0f) return;

    if (motor->controller.output_torque > motor->info.max_torque)
    {
        motor->controller.output_torque = motor->info.max_torque;
    }
    else if (motor->controller.output_torque < -motor->info.max_torque)
    {
        motor->controller.output_torque = -motor->info.max_torque;
    }
}

/**
 * @brief 急停并失能全部已注册电机。
 *
 * @param fault_motor 输入，触发故障的电机，可为 NULL。
 * @param reason      输入，故障原因字符串。
 *
 * @retval 无。
 *
 * @note 调用关系：由 motor_safety_check() 在底层限位触发时调用。
 * @note 安全策略：锁存全局安全故障，清零全部输出，置全部 enableflag=0，并交给 Offline 模块统一蜂鸣报警。
 */
static void motor_emergency_stop_all(Motor_Base *fault_motor, const char *reason)
{
    const char *name = (fault_motor != NULL && fault_motor->name != NULL) ? fault_motor->name : "unknown_motor";
    if (reason == NULL) reason = "motor safety fault";

    if (g_motor_safety_fault == 0U)
    {
        LOG_E("MOTOR SAFETY FAULT: motor=%s reason=%s", name, reason);
        Module_Offline_SetFatalFault(name, reason);
    }
    g_motor_safety_fault = 1U;

    for (Motor_Base *motor = g_motor_list; motor; motor = motor->next)
    {
        motor->setting.enableflag = 0U;
        motor_clear_output(motor);
    }
}

/**
 * @brief 获取当前电机用于角度限位的反馈角度。
 *
 * @param motor 输入，电机基类指针。
 *
 * @return float 返回单圈/协议角度或软件累计总角度；未启用角度限位时返回 0。
 *
 * @note 调用关系：由全局限位检查和具体驱动 Apply 前的最终检查调用。
 */
static float motor_get_safety_angle(const Motor_Base *motor)
{
    if (motor == NULL) return 0.0f;

    switch (motor->safety.angle_source)
    {
    case MOTOR_SAFETY_ANGLE_SINGLE:
        return motor->measure.single_round_angle;
    case MOTOR_SAFETY_ANGLE_TOTAL:
        return motor->measure.total_angle;
    case MOTOR_SAFETY_ANGLE_DISABLED:
    default:
        return 0.0f;
    }
}

/**
 * @brief 检查单个电机是否处于配置的底层安全限位内。
 *
 * @param motor 输入，需要检查的电机基类指针。
 *
 * @return uint8_t 返回 1 表示允许继续 Apply；返回 0 表示电机指针无效。
 *
 * @note 调用关系：由 Motor_ApplyAll() 在调用具体驱动 Apply() 前执行，是下发 CAN/PWM/UART 前的最终安全闸门。
 */
static uint8_t motor_safety_check(Motor_Base *motor)
{
#if !ARM_SOFTWARE_LIMIT_ENABLE
    (void)motor;
    return 1U;
#else
    if (motor == NULL) return 0U;
    if (g_motor_safety_fault != 0U)
    {
        motor->setting.enableflag = 0U;
        motor_clear_output(motor);
        return 1U;
    }

    if (motor->safety.enable == 0U) return 1U;

    /*
     * 限位必须等到驱动层收到过至少一帧有效反馈后才启用。
     * 上电初始 raw/angle 通常为 0，不能把“尚未收到反馈”误判为真实位置超限。
     */
    if (motor->feedback_valid == 0U) return 1U;

    if (motor->offline_dev != NULL && Module_Offline_get_device_status(motor->offline_dev) == STATE_OFFLINE)
    {
        return 1U;
    }

    if (motor->safety.use_raw_position != 0U && motor->GetRawPosition != NULL)
    {
        int32_t raw = motor->GetRawPosition(motor);
        /*
         * 支持跨越编码器回零点的合法区间：
         * min <= max 表示普通区间 [min,max]；
         * min > max 表示跨零区间 [min,raw_max] ∪ [raw_min,max]。
         */
        uint8_t raw_in_range = (motor->safety.raw_min <= motor->safety.raw_max)
                                   ? (raw >= motor->safety.raw_min && raw <= motor->safety.raw_max)
                                   : (raw >= motor->safety.raw_min || raw <= motor->safety.raw_max);
        if (raw_in_range == 0U)
        {
            LOG_E("MOTOR LIMIT RAW: motor=%s raw=%ld limit=[%ld,%ld] angle=%.3f",
                  motor->name ? motor->name : "unknown_motor",
                  (long)raw,
                  (long)motor->safety.raw_min,
                  (long)motor->safety.raw_max,
                  motor->measure.total_angle);
            motor_emergency_stop_all(motor, "raw position limit");
            return 1U;
        }
    }

    if (motor->safety.angle_source != MOTOR_SAFETY_ANGLE_DISABLED)
    {
        float angle = motor_get_safety_angle(motor);
        if (angle < motor->safety.angle_min_rad || angle > motor->safety.angle_max_rad)
        {
            LOG_E("MOTOR LIMIT ANGLE: motor=%s angle=%.3f limit=[%.3f,%.3f]",
                  motor->name ? motor->name : "unknown_motor",
                  angle,
                  motor->safety.angle_min_rad,
                  motor->safety.angle_max_rad);
            motor_emergency_stop_all(motor, "angle limit");
            return 1U;
        }
    }

    return 1U;
#endif
}

/**
 * @brief 判断单台电机当前反馈是否仍在安全限位内。
 *
 * @param motor 输入，需要检查的电机基类指针。
 *
 * @return uint8_t 返回 1 表示位置有效或未启用限位；返回 0 表示已超限。
 *
 * @note 调用关系：由全局限位检查和具体驱动 Apply() 的最终校验共同调用。
 * @note 本函数只读取状态，不修改 enableflag、不触发报警，便于在发送前重复检查。
 */
static uint8_t motor_limit_is_safe(const Motor_Base *motor)
{
#if !ARM_SOFTWARE_LIMIT_ENABLE
    (void)motor;
    return 1U;
#else
    if (motor == NULL || motor->safety.enable == 0U) return 1U;
    if (motor->feedback_valid == 0U) return 1U;
    if (motor->offline_dev != NULL && Module_Offline_get_device_status(motor->offline_dev) == STATE_OFFLINE) return 1U;

    if (motor->safety.use_raw_position != 0U && motor->GetRawPosition != NULL)
    {
        int32_t raw = motor->GetRawPosition((Motor_Base *)motor);
        uint8_t raw_in_range = (motor->safety.raw_min <= motor->safety.raw_max)
                                   ? (raw >= motor->safety.raw_min && raw <= motor->safety.raw_max)
                                   : (raw >= motor->safety.raw_min || raw <= motor->safety.raw_max);
        if (raw_in_range == 0U) return 0U;
    }

    if (motor->safety.angle_source != MOTOR_SAFETY_ANGLE_DISABLED)
    {
        float angle = motor_get_safety_angle(motor);
        if (angle < motor->safety.angle_min_rad || angle > motor->safety.angle_max_rad) return 0U;
    }

    return 1U;
#endif
}

/**
 * @brief 判断电机是否属于指定 CAN 总线分组。
 *
 * @param motor     输入，电机基类指针。
 * @param bus_index 输入，CAN 总线序号，当前约定 0=CAN1，1=CAN2，2=CAN3。
 *
 * @return uint8_t 返回 1 表示该电机使用指定 CAN 总线；返回 0 表示不属于该组或不是 CAN 电机。
 *
 * @note 调用关系：由 motor_find_next_can() 调用；本函数只读取 transport_dev，不发送报文。
 */
static uint8_t motor_is_can_bus(const Motor_Base *motor, uint8_t bus_index)
{
    if (motor == NULL || motor->transport != MOTOR_TRANSPORT_CAN || motor->transport_dev == NULL) return 0U;

    const Can_Device      *can_dev = (const Can_Device *)motor->transport_dev;
    const CAN_Bus_Manager *bus     = (const CAN_Bus_Manager *)can_dev->_bus;
    if (bus == NULL) return 0U;

    switch (bus_index)
    {
    case 0:
        return BSP_CAN_IS_HANDLE1(bus->hcan) ? 1U : 0U;
    case 1:
        return BSP_CAN_IS_HANDLE2(bus->hcan) ? 1U : 0U;
    case 2:
        return BSP_CAN_IS_HANDLE3(bus->hcan) ? 1U : 0U;
    default:
        return 0U;
    }
}

/**
 * @brief 从链表指定位置开始查找下一台属于指定 CAN 总线的电机。
 *
 * @param start     输入，查找起点指针。
 * @param bus_index 输入，CAN 总线序号，当前约定 0=CAN1，1=CAN2，2=CAN3。
 *
 * @return Motor_Base* 找到则返回电机基类指针；未找到则返回 NULL。
 *
 * @note 调用关系：由 Motor_ApplyAll() 的 CAN 槽位调度调用；本函数不发送报文。
 */
static Motor_Base *motor_find_next_can(Motor_Base *start, uint8_t bus_index)
{
    for (Motor_Base *motor = start; motor; motor = motor->next)
    {
        if (motor_is_can_bus(motor, bus_index)) return motor;
    }
    return NULL;
}

/**
 * @brief 注册电机对象到全局链表。
 *
 * @param motor 输入，需要注册的电机基类指针。
 *
 * @retval 无。
 *
 * @note 调用关系：由各电机驱动 Init 函数调用；注册后由 Motor_ControlAll() 和 Motor_ApplyAll() 统一调度。
 */
void Motor_Register(Motor_Base *motor)
{
    if (motor == NULL) return;
    motor->next  = g_motor_list;
    g_motor_list = motor;
}

/**
 * @brief 执行所有已注册电机的控制计算。
 *
 * @param 无。
 *
 * @retval 无。
 *
 * @note 调用关系：由 module_motor.c 中的 motor_task_entry() 周期调用；本阶段只计算输出，不发送报文。
 */
void Motor_ControlAll(void)
{
    for (Motor_Base *motor = g_motor_list; motor; motor = motor->next)
    {
        uint8_t offline = (motor->offline_dev != NULL && Module_Offline_get_device_status(motor->offline_dev) == STATE_OFFLINE);

        if (offline || motor->setting.enableflag == 0)
        {
            motor_clear_output(motor);
            continue;
        }

        /* 开环电机: 无控制计算 (Apply 阶段直通 ref) */
        if (motor->setting.loop_type == OPEN_LOOP) continue;

        motor->controller.output_torque = Motor_CalculateTorque(motor);
    }
}

/**
 * @brief 应用所有已注册电机的输出并触发传输层发送。
 *
 * @param 无。
 *
 * @retval 无。
 *
 * @note 调用关系：由 module_motor.c 中的 motor_task_entry() 在功率限制后调用。
 * @note 调度策略：按 CAN 槽位轮询发送；每个槽位内每条 CAN 总线最多应用一台电机，
 *       槽位之间等待 MOTOR_CAN_SLOT_GAP_S，复刻旧工程“CAN1/CAN2 可同片发送，同一 CAN 内部要隔开”的节奏。
 */
void Motor_ApplyAll(void)
{
    /*
     * 先对全部已注册电机做一次安全预扫描，再开始任何一条控制报文的 Apply。
     * 这样不会出现“前面的电机已经发出力矩，后面的电机才发现超限”的半周期风险。
     */
    for (Motor_Base *motor = g_motor_list; motor; motor = motor->next)
    {
        if (motor_safety_check(motor) == 0U) return;
    }

    Motor_Base *can_cursor[BSP_CAN_BUS_NUM];
    for (uint8_t bus_index = 0; bus_index < BSP_CAN_BUS_NUM; ++bus_index)
    {
        can_cursor[bus_index] = g_motor_list;
    }

    while (1)
    {
        uint8_t sent_in_slot = 0U;

        for (uint8_t bus_index = 0; bus_index < BSP_CAN_BUS_NUM; ++bus_index)
        {
            Motor_Base *motor = motor_find_next_can(can_cursor[bus_index], bus_index);
            if (motor != NULL)
            {
                motor_limit_output_torque(motor);
                if (motor->Apply != NULL) motor->Apply(motor);
                can_cursor[bus_index] = motor->next;
                sent_in_slot          = 1U;
            }
        }

        if (sent_in_slot == 0U) break;
        BSP_DWT_Delay(MOTOR_CAN_SLOT_GAP_S);
    }

    for (Motor_Base *motor = g_motor_list; motor; motor = motor->next)
    {
        if (motor->transport != MOTOR_TRANSPORT_CAN && motor->Apply != NULL)
        {
            motor_limit_output_torque(motor);
            motor->Apply(motor);
        }
    }
}

/**
 * @brief 使能单个电机。
 *
 * @param m 输入，需要使能的电机基类指针。
 *
 * @retval 无。
 *
 * @note 调用关系：由 APP 或上层控制逻辑调用。
 * @note 安全策略：如果底层安全限位已经触发锁死，本函数拒绝重新置 enableflag=1。
 */
void Motor_Start(Motor_Base *m)
{
    if (m == NULL) return;
    if (g_motor_safety_fault != 0U)
    {
        LOG_E("reject Motor_Start after safety fault: motor=%s", m->name ? m->name : "unknown_motor");
        m->setting.enableflag = 0U;
        return;
    }
    m->setting.enableflag = 1;
}

/**
 * @brief 停止并失能单个电机。
 *
 * @param m 输入，需要失能的电机基类指针。
 *
 * @retval 无。
 *
 * @note 调用关系：由 APP 或安全逻辑调用；真正零输出报文由后续 Motor_ApplyAll() 调用具体驱动 Apply() 完成。
 */
void Motor_Stop(Motor_Base *m)
{
    if (m == NULL) return;
    m->setting.enableflag = 0;
}

/**
 * @brief 执行单台电机的最终发送前安全校验。
 *
 * @param motor 输入，需要执行最终校验的电机基类指针。
 *
 * @return uint8_t 返回 1 表示允许当前电机继续执行 Apply；返回 0 表示本台电机已被安全失能。
 *
 * @note 调用关系：由具体电机驱动 Apply() 入口调用，位于协议编码和 CAN 发送之前。
 * @note 安全策略：本接口只处理当前电机；发现刚刚越限时清零本台输出并拒绝正常输出，
 *       下一周期的 Motor_ApplyAll() 全局预扫描仍会负责全局急停。
 */
uint8_t Motor_SafetyCheckBeforeApply(Motor_Base *motor)
{
#if !ARM_SOFTWARE_LIMIT_ENABLE
    (void)motor;
    return 1U;
#else
    if (motor == NULL) return 0U;

    /* 当前电机本来就失能时，不需要重复做位置拦截。 */
    if (motor->setting.enableflag == 0U) return 1U;

    /* 已经发生全局安全故障时，本台只允许走驱动层零输出分支。 */
    if (g_motor_safety_fault != 0U)
    {
        motor->setting.enableflag = 0U;
        motor_clear_output(motor);
        return 0U;
    }

    if (motor_limit_is_safe(motor) != 0U) return 1U;

    LOG_E("MOTOR FINAL SAFETY BLOCK: motor=%s raw=%ld angle=%.3f",
          motor->name ? motor->name : "unknown_motor",
          (long)((motor->GetRawPosition != NULL) ? motor->GetRawPosition(motor) : 0L),
          motor->measure.total_angle);
    motor->setting.enableflag = 0U;
    motor_clear_output(motor);
    return 0U;
#endif
}

/**
 * @brief 设置电机控制参考值。
 *
 * @param m   输入，需要设置参考值的电机基类指针。
 * @param ref 输入，参考值，单位由当前控制模式决定。
 *
 * @retval 无。
 */
void Motor_SetRef(Motor_Base *m, float ref)
{
    if (m == NULL) return;
    m->controller.ref = ref;
}

/**
 * @brief 修改电机闭环反馈来源。
 *
 * @param m               输入，需要修改反馈源的电机基类指针。
 * @param loop            输入，指定角度环或速度环。
 * @param feedback_source 输入，反馈来源编号。
 *
 * @retval 无。
 */
void Motor_ChangeFeed(Motor_Base *m, Closeloop_Type_e loop, uint8_t feedback_source)
{
    if (m == NULL) return;
    if (loop == ANGLE_LOOP)
        m->setting.angle_feedback_source = feedback_source;
    else if (loop == SPEED_LOOP)
        m->setting.speed_feedback_source = feedback_source;
}

/**
 * @brief 设置电机外环控制模式。
 *
 * @param m          输入，需要设置外环模式的电机基类指针。
 * @param outer_loop 输入，目标闭环类型。
 *
 * @retval 无。
 */
void Motor_OuterLoop(Motor_Base *m, Closeloop_Type_e outer_loop)
{
    if (m == NULL) return;
    m->setting.loop_type = outer_loop;
}

/**
 * @brief 设置电机前馈力矩。
 *
 * @param m      输入，需要设置前馈的电机基类指针。
 * @param torque 输入，前馈力矩，单位 N·m。
 *
 * @retval 无。
 */
void Motor_SetForwardTorque(Motor_Base *m, float torque)
{
    if (m == NULL) return;
    m->controller.feedforward_torque = torque;
}

/**
 * @brief 设置电机输出力矩。
 *
 * @param m      输入，需要设置输出力矩的电机基类指针。
 * @param torque 输入，输出力矩，单位 N·m。
 *
 * @retval 无。
 *
 * @note 安全策略：如果底层安全限位已经触发锁死，本函数只清零输出，不再接受新的力矩值。
 */
void Motor_SetOutputTorque(Motor_Base *m, float torque)
{
    if (m == NULL) return;
    if (g_motor_safety_fault != 0U)
    {
        motor_clear_output(m);
        return;
    }
    m->controller.output_torque = torque;
    motor_limit_output_torque(m);
}

/**
 * @brief 查询电机底层安全故障锁死状态。
 *
 * @param 无。
 *
 * @return uint8_t 返回 1 表示已触发安全故障，返回 0 表示未触发。
 */
uint8_t Motor_SafetyFaultActive(void)
{
    return g_motor_safety_fault;
}
