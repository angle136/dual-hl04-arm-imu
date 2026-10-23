#include "motor_damiao.h"
#include "bsp_can.h"
#include "bsp_dwt.h"
#include "bsp_def.h"
#include "module_offline.h"
#include "user_lib.h"
#include <stdint.h>
#include <string.h>

#define LOG_TAG "motor_dm"
#define LOG_LVL LOG_LVL_DBG
#include "ulog_def.h"

/* 达妙电机参数配置表 */
static const DM_Motor_Params_t dm_motor_params[] = {
    [DM4310] = {-12.5f, 12.5f, -30.0f, 30.0f, 0.0f, 500.0f, 0.0f, 5.0f, -10.0f, 10.0f},
    /* DM4340 位置/速度/力矩量程参照旧机械臂工程；Kp/Kd 下限保留 0，保证 Kp=0/Kd=0 时 MIT 原始增益也为 0。 */
    [DM4340] = {-12.5f, 12.5f, -10.0f, 10.0f, 0.0f, 500.0f, 0.0f, 5.0f, -28.0f, 28.0f},
    [DM6220] = {-12.5f, 12.5f, -45.0f, 45.0f, 0.0f, 54.0f, 0.0f, 5.0f, -10.0f, 10.0f},
    [DM3507] = {-12.566f, 12.566f, -50.0f, 50.0f, 0.0f, 54.0f, 0.0f, 5.0f, -5.0f, 5.0f},
};

/**
 * @brief 根据电机型号获取达妙编解码参数。
 *
 * @param type 输入，电机型号枚举。
 *
 * @return const DM_Motor_Params_t* 返回对应型号的参数表指针；型号未配置时返回 DM4310 参数。
 *
 * @note 调用关系：由 Motor_DM_Init() 调用；参数用于反馈解码和 MIT 控制帧编码。
 */
static const DM_Motor_Params_t *dm_get_params(Motor_Type_e type)
{
    if (type >= DM4310 && type <= DM3519 && dm_motor_params[type].p_max > dm_motor_params[type].p_min)
        return &dm_motor_params[type];
    return &dm_motor_params[DM4310];
}

/*  CAN 接收回调 */
static void dm_can_rx_callback(Can_Device *dev, const uint8_t *data, uint8_t len)
{
    if (len < 8 || !dev->user_arg) return;
    DM_Motor_t              *motor = (DM_Motor_t *)dev->user_arg;
    const DM_Motor_Params_t *param = motor->params;

    uint8_t error_code        = (data[0] >> 4) & 0x0F;
    motor->measure.id         = data[0] & 0x0F;
    motor->measure.Error_Code = (error_code >= 0x08 && error_code <= 0x0E) ? (DMMotorError_t)error_code : DM_NO_ERROR;

    uint16_t tmp;
    tmp                                    = (uint16_t)((data[1] << 8) | data[2]);
    motor->measure.raw_position           = tmp;
    motor->base.measure.single_round_angle = uint_to_float(tmp, param->p_min, param->p_max, 16);
    tmp                                    = (uint16_t)((data[3] << 4) | data[4] >> 4);
    motor->base.measure.speed_rad          = uint_to_float(tmp, param->v_min, param->v_max, 12);
    tmp                                    = (uint16_t)(((data[4] & 0x0f) << 8) | data[5]);
    motor->measure.torque                  = uint_to_float(tmp, param->t_min, param->t_max, 12);
    motor->measure.T_Mos                   = (float)data[6];
    motor->measure.T_Rotor                 = (float)data[7];

    float current_angle = motor->base.measure.single_round_angle;
    if (motor->base.feedback_valid == 0U)
    {
        /*
         * 首帧只建立累计角度基线，不根据初始值判断跨圈。
         * 这样 J6 上电时处于任意合法姿态都不会被误计成已经跨过一圈。
         */
        motor->measure.last_single_round_angle = current_angle;
        motor->measure.total_round             = 0;
        motor->base.measure.total_angle       = current_angle;
    }
    else
    {
        float diff       = current_angle - motor->measure.last_single_round_angle;
        float range      = param->p_max - param->p_min;
        float half_range = range * 0.5f;
        if (diff < -half_range)
        {
            diff += range;
            motor->measure.total_round++;
        }
        else if (diff > half_range)
        {
            diff -= range;
            motor->measure.total_round--;
        }

        motor->base.measure.total_angle += diff;
        motor->measure.last_single_round_angle = current_angle;
    }

    motor->base.measure.torque_nm          = motor->measure.torque;
    /* 只有收到完整有效反馈后，底层 raw/角度限位才允许参与判定。 */
    motor->base.feedback_valid = 1U;

    Module_Offline_device_update(motor->base.offline_dev);
}

/**
 * @brief 读取达妙电机原始位置反馈。
 *
 * @param base 输入，电机基类指针，函数内部转换为 DM_Motor_t。
 *
 * @return int32_t 返回最近一次反馈报文中的 16bit 原始位置值。
 *
 * @note 调用关系：由 Motor_ApplyAll() 的底层安全限位检查调用；本函数不访问外设、不发送 CAN。
 */
static int32_t dm_get_raw_position(Motor_Base *base)
{
    DM_Motor_t *motor = MOTOR_GET_DERIVED(base, DM_Motor_t);
    return (motor != NULL) ? (int32_t)motor->measure.raw_position : 0;
}

/**
 * @brief 发送达妙电机 MIT 模式控制帧。
 *
 * @param motor 输入，达妙电机对象指针，提供 CAN 设备、模式和型号限幅参数。
 * @param pos   输入，目标位置，单位 rad。
 * @param vel   输入，目标速度，单位 rad/s。
 * @param kp    输入，位置刚度系数。
 * @param kd    输入，速度阻尼系数。
 * @param torq  输入，前馈力矩，单位 N·m。
 *
 * @retval 无。
 *
 * @note 调用关系：由 dm_apply() 按电机模式调用；内部调用 BSP_CAN_SendMessage() 将报文压入 CAN TX FIFO。
 * @note 时序要求：运行期同一 CAN 内部的帧间隔由 Motor_ApplyAll() 的槽位调度统一保证。
 */
static void mit_ctrl(DM_Motor_t *motor, float pos, float vel, float kp, float kd, float torq)
{
    Can_Device              *can_dev = (Can_Device *)motor->base.transport_dev;
    BSP_CanMsg_t             msg;
    const DM_Motor_Params_t *param = motor->params;

    /* 从 Can_Device 获取 CAN 句柄 */
    CAN_Bus_Manager *bus = (CAN_Bus_Manager *)can_dev->_bus;
    msg.hcan             = bus->hcan;
    msg.id               = can_dev->tx_id + DM_MIT_MODE;
    msg.len              = 8;

    uint16_t pos_tmp = float_to_uint(pos, param->p_min, param->p_max, 16);
    uint16_t vel_tmp = float_to_uint(vel, param->v_min, param->v_max, 12);
    uint16_t kp_tmp  = float_to_uint(kp, param->kp_min, param->kp_max, 12);
    uint16_t kd_tmp  = float_to_uint(kd, param->kd_min, param->kd_max, 12);
    uint16_t tor_tmp = float_to_uint(torq, param->t_min, param->t_max, 12);

    msg.data[0] = (pos_tmp >> 8);
    msg.data[1] = pos_tmp;
    msg.data[2] = (vel_tmp >> 4);
    msg.data[3] = ((vel_tmp & 0xF) << 4) | (kp_tmp >> 8);
    msg.data[4] = kp_tmp;
    msg.data[5] = (kd_tmp >> 4);
    msg.data[6] = ((kd_tmp & 0xF) << 4) | (tor_tmp >> 8);
    msg.data[7] = tor_tmp;

    BSP_CAN_SendMessage(&msg);
}

/**
 * @brief 发送达妙电机位置-速度模式控制帧。
 *
 * @param motor      输入，达妙电机对象指针，提供 CAN 设备和型号限幅参数。
 * @param pos_degree 输入，目标位置，单位 degree，函数内部转换为 rad 后按电机参数限幅。
 * @param vel        输入，目标速度，单位 rad/s。
 *
 * @retval 无。
 *
 * @note 调用关系：由 dm_apply() 按电机模式调用；内部调用 BSP_CAN_SendMessage() 将报文压入 CAN TX FIFO。
 * @note 时序要求：运行期同一 CAN 内部的帧间隔由 Motor_ApplyAll() 的槽位调度统一保证。
 */
static void pos_speed_ctrl(DM_Motor_t *motor, float pos_degree, float vel)
{
    Can_Device              *can_dev = (Can_Device *)motor->base.transport_dev;
    BSP_CanMsg_t             msg;
    const DM_Motor_Params_t *param = motor->params;

    CAN_Bus_Manager *bus = (CAN_Bus_Manager *)can_dev->_bus;
    msg.hcan             = bus->hcan;
    msg.id               = can_dev->tx_id + DM_POS_MODE;
    msg.len              = 8;

    float    pos_rad = deg_to_rad(pos_degree);
    uint8_t *pbuf    = (uint8_t *)&pos_rad;
    uint8_t *vbuf    = (uint8_t *)&vel;

    VAL_LIMIT(pos_rad, param->p_min, param->p_max);
    msg.data[0] = pbuf[0];
    msg.data[1] = pbuf[1];
    msg.data[2] = pbuf[2];
    msg.data[3] = pbuf[3];
    msg.data[4] = vbuf[0];
    msg.data[5] = vbuf[1];
    msg.data[6] = vbuf[2];
    msg.data[7] = vbuf[3];

    BSP_CAN_SendMessage(&msg);
}

/**
 * @brief 发送达妙电机速度模式控制帧。
 *
 * @param motor 输入，达妙电机对象指针，提供 CAN 设备和当前模式。
 * @param vel   输入，目标速度，单位 rad/s。
 *
 * @retval 无。
 *
 * @note 调用关系：由 dm_apply() 按电机模式调用；内部调用 BSP_CAN_SendMessage() 将报文压入 CAN TX FIFO。
 * @note 时序要求：运行期同一 CAN 内部的帧间隔由 Motor_ApplyAll() 的槽位调度统一保证。
 */
static void speed_ctrl(DM_Motor_t *motor, float vel)
{
    Can_Device  *can_dev = (Can_Device *)motor->base.transport_dev;
    BSP_CanMsg_t msg;

    CAN_Bus_Manager *bus = (CAN_Bus_Manager *)can_dev->_bus;
    msg.hcan             = bus->hcan;
    msg.id               = can_dev->tx_id + DM_SPD_MODE;
    msg.len              = 4;

    uint8_t *vbuf = (uint8_t *)&vel;
    msg.data[0]   = vbuf[0];
    msg.data[1]   = vbuf[1];
    msg.data[2]   = vbuf[2];
    msg.data[3]   = vbuf[3];

    BSP_CAN_SendMessage(&msg);
}

/**
 * @brief 发送达妙电机 PSI 模式控制帧（位置-速度-电流）。
 *
 * @param motor   输入，达妙电机对象指针，提供 CAN 设备和当前模式。
 * @param pos     输入，目标位置，单位 rad。
 * @param vel     输入，目标速度，单位 rad/s。
 * @param current 输入，目标电流，单位 A。
 *
 * @retval 无。
 *
 * @note 调用关系：由 dm_apply() 按电机模式调用；内部调用 BSP_CAN_SendMessage() 将报文压入 CAN TX FIFO。
 * @note 时序要求：运行期同一 CAN 内部的帧间隔由 Motor_ApplyAll() 的槽位调度统一保证。
 */
static void psi_ctrl(DM_Motor_t *motor, float pos, float vel, float current)
{
    Can_Device  *can_dev = (Can_Device *)motor->base.transport_dev;
    BSP_CanMsg_t msg;

    CAN_Bus_Manager *bus = (CAN_Bus_Manager *)can_dev->_bus;
    msg.hcan             = bus->hcan;
    msg.id               = can_dev->tx_id + DM_PSI_MODE;
    msg.len              = 8;

    uint16_t u16_vel = vel * 100;
    uint16_t u16_cur = current * 10000;

    uint8_t *pbuf = (uint8_t *)&pos;
    uint8_t *vbuf = (uint8_t *)&u16_vel;
    uint8_t *ibuf = (uint8_t *)&u16_cur;

    msg.data[0] = pbuf[0];
    msg.data[1] = pbuf[1];
    msg.data[2] = pbuf[2];
    msg.data[3] = pbuf[3];
    msg.data[4] = vbuf[0];
    msg.data[5] = vbuf[1];
    msg.data[6] = ibuf[0];
    msg.data[7] = ibuf[1];

    BSP_CAN_SendMessage(&msg);
}

/**
 * @brief 达妙电机输出应用函数。
 *
 * @param base 输入，电机基类指针，函数内部通过 MOTOR_GET_DERIVED() 转换为 DM_Motor_t。
 *
 * @retval 无。
 *
 * @note 调用关系：由 Motor_ApplyAll() 槽位调度调用；内部先同步达妙本体启停状态，
 *       再根据 mode_type 调用 mit_ctrl()、pos_speed_ctrl()、speed_ctrl() 或 psi_ctrl()。
 * @note 启停策略：enableflag 只表示软件请求；当本电机无错误码、反馈有效、在线且通过安全检查时，
 *       仅在 hardware_start_sent 为 0 时发送一次 MOTOR_START，并让本周期结束，下一周期再发送控制帧。
 *       软件失能、错误码、掉线、反馈无效或安全检查失败时，仅在 hardware_start_sent 为 1 时发送一次 MOTOR_STOP。
 * @note 隔离性：hardware_start_sent 属于当前 DM_Motor_t 对象，Motor_DM_Cmd() 使用该对象自己的
 *       CAN 设备和发送 ID，不会启动链表中的其它电机。
 */
static void dm_apply(Motor_Base *base)
{
    DM_Motor_t *motor = MOTOR_GET_DERIVED(base, DM_Motor_t);
    uint8_t     offline =
        (base->offline_dev != NULL && Module_Offline_get_device_status(base->offline_dev) == STATE_OFFLINE);
    uint8_t     safety_ok = Motor_SafetyCheckBeforeApply(base);
    uint8_t     output_allowed =
        (offline == 0U && motor->measure.Error_Code == DM_NO_ERROR && base->feedback_valid != 0U && base->setting.enableflag != 0U &&
         safety_ok != 0U)
            ? 1U
            : 0U;

    if (output_allowed == 0U)
    {
        /*
         * 先发送一次协议层 STOP，再发送零控制帧。
         * hardware_start_sent 置零后，后续周期不会重复发送 STOP。
         */
        if (motor->hardware_start_sent != 0U)
        {
            Motor_DM_Cmd(motor, DM_CMD_MOTOR_STOP);
            motor->hardware_start_sent = 0U;
            LOG_I("DM hardware stop: %s", base->name ? base->name : "unknown_motor");
        }

        switch (motor->mode_type)
        {
        case DM_MIT_MODE:
            mit_ctrl(motor, 0, 0, 0, 0, 0);
            break;
        case DM_POS_MODE:
            pos_speed_ctrl(motor, 0, 0);
            break;
        case DM_SPD_MODE:
            speed_ctrl(motor, 0);
            break;
        case DM_PSI_MODE:
            psi_ctrl(motor, 0, 0, 0);
            break;
        default:
            break;
        }
        return;
    }

    /*
     * 达妙硬件启动命令必须在正常控制帧之前单独发送。
     * 本周期只发 START，不同时发送 MIT 力矩，避免启动命令和控制帧混在同一状态切换周期。
     */
    if (motor->hardware_start_sent == 0U)
    {
        Motor_DM_Cmd(motor, DM_CMD_MOTOR_START);
        motor->hardware_start_sent = 1U;
        LOG_I("DM hardware start: %s", base->name ? base->name : "unknown_motor");
        return;
    }

    switch (motor->mode_type)
    {
    case DM_MIT_MODE:
        mit_ctrl(motor, 0, 0, 0, 0, base->controller.output_torque);
        break;
    case DM_POS_MODE:
        pos_speed_ctrl(motor, base->controller.ref, PI);
        break;
    case DM_SPD_MODE:
        speed_ctrl(motor, base->controller.ref);
        break;
    case DM_PSI_MODE:
        psi_ctrl(motor, 0, 0, base->controller.ref);
        break;
    default:
        break;
    }
}

/**
 * @brief 向达妙电机发送特殊命令帧。
 *
 * @param motor 输入，达妙电机对象指针，提供 CAN 设备和当前控制模式。
 * @param cmd   输入，达妙特殊命令枚举，例如清错、使能或失能命令。
 *
 * @retval 无。
 *
 * @note 调用关系：由 Motor_DM_Init() 或上层调试代码调用；内部调用 BSP_CAN_SendMessage() 发送 0xFF 填充的命令帧。
 * @note 时序要求：发送后调用 BSP_DWT_Delay() 留出命令帧间隔，避免上电初期连续特殊命令过密。
 */
void Motor_DM_Cmd(DM_Motor_t *motor, DMMotor_Mode_e cmd)
{
    Can_Device  *can_dev = (Can_Device *)motor->base.transport_dev;
    BSP_CanMsg_t msg;

    CAN_Bus_Manager *bus = (CAN_Bus_Manager *)can_dev->_bus;
    msg.hcan             = bus->hcan;
    msg.id               = can_dev->tx_id + motor->mode_type;
    msg.len              = 8;

    msg.data[0] = 0xff;
    msg.data[1] = 0xff;
    msg.data[2] = 0xff;
    msg.data[3] = 0xff;
    msg.data[4] = 0xff;
    msg.data[5] = 0xff;
    msg.data[6] = 0xff;
    msg.data[7] = (uint8_t)cmd;

    BSP_CAN_SendMessage(&msg);
    BSP_DWT_Delay(0.0004f); /* 400us 间隔 */
}

/**
 * @brief 初始化并注册达妙电机对象。
 *
 * @param config       输入，通用电机初始化配置，包含电机型号、CAN 设备、离线检测和控制参数。
 * @param DM_Mode_type 输入，达妙控制模式偏移值，例如 DM_MIT_MODE、DM_POS_MODE、DM_SPD_MODE。
 *
 * @return DM_Motor_t* 初始化成功返回达妙电机对象指针；内存申请、参数检查或 CAN 注册失败时返回 NULL。
 *
 * @note 调用关系：由 APP 层电机注册函数调用；内部完成内存申请、基类字段初始化、CAN 接收注册、
 *       达妙清错/失能命令发送，并通过 Motor_Register() 注册到电机调度层。
 * @note 时序要求：初始化阶段清错和失能命令各重复发送一次，Motor_DM_Cmd() 内部负责命令帧间隔。
 * @note 安全策略：第一阶段只读取反馈和验证限位，不在初始化时发送 MOTOR_START，避免上电后电机进入
 *       内部使能状态并产生阻尼；运行期由 dm_apply() 根据单台电机的 enableflag 和反馈状态同步硬件启停。
 */
DM_Motor_t *Motor_DM_Init(Motor_Init_Config_s *config, uint32_t DM_Mode_type)
{
    DM_Motor_t *motor = NULL;
    BSP_MEM_ALLOC_WAIT(motor, sizeof(DM_Motor_t), TX_NO_WAIT);
    if (motor == NULL)
    {
        LOG_E("Failed to allocate memory for DM motor");
        return NULL;
    }
    memset(motor, 0, sizeof(DM_Motor_t));

    /* 初始化基类字段 */
    motor->base.type      = config->motor_init_info.motor_type;
    motor->base.transport = MOTOR_TRANSPORT_CAN;
    motor->base.info      = config->motor_init_info;
    motor->base.setting   = config->setting_init_config;
    motor->base.safety    = config->safety_limit_config;
    motor->base.name      = config->offline_init_config.name;
    motor->mode_type      = DM_Mode_type;
    motor->params         = dm_get_params(config->motor_init_info.motor_type);

    /* Master ID (rx_id) 必须大于 CAN ID (tx_id) 且各不相同 */
    {
        uint32_t can_id    = config->transport_config.can.tx_id;
        uint32_t master_id = config->transport_config.can.rx_id;
        if (master_id <= can_id)
        {
            LOG_E("DM motor init failed: Master ID (rx_id=0x%03X) must be greater than CAN ID (tx_id=0x%03X)", master_id, can_id);
            BSP_MEM_FREE(motor);
            return NULL;
        }
    }

    /* 注册 CAN 设备 */
    Can_Device *can_dev = BSP_CAN_Device_Init(&config->transport_config.can);
    if (can_dev == NULL)
    {
        LOG_E("Failed to initialize CAN device for DM motor");
        BSP_MEM_FREE(motor);
        return NULL;
    }
    motor->base.transport_dev = can_dev;

    /* 设置 CAN 接收回调 */
    can_dev->rx_callback = dm_can_rx_callback;
    can_dev->user_arg    = motor;

    /* 初始化控制器 */
    if (motor->base.setting.algorithm_type == CONTROL_PID)
    {
        PIDInit(&motor->base.controller.speed_PID, &config->controller_init_config.speed_PID);
        PIDInit(&motor->base.controller.angle_PID, &config->controller_init_config.angle_PID);
    }
    else if (motor->base.setting.algorithm_type == CONTROL_LQR)
    {
        LQRInit(&motor->base.controller.lqr, &config->controller_init_config.lqr_init);
    }
    motor->base.controller.other_angle_feedback_ptr = config->controller_init_config.other_angle_feedback_ptr;
    motor->base.controller.other_speed_feedback_ptr = config->controller_init_config.other_speed_feedback_ptr;

    /* 离线检测 */
    motor->base.offline_dev = Module_Offline_register(&config->offline_init_config);

    /*
     * 第一阶段只做反馈读取：清除错误后显式保持失能。
     * 不在 Init 中发送 MOTOR_START，避免软件 enableflag=0 但电机本体已进入使能态，
     * 造成不同电机上电后阻尼/灯色不一致。
     */
    motor->measure.Error_Code = DM_NO_ERROR;
    Motor_DM_Cmd(motor, DM_CMD_CLEAR_ERROR);
    Motor_DM_Cmd(motor, DM_CMD_CLEAR_ERROR);
    Motor_DM_Cmd(motor, DM_CMD_MOTOR_STOP);
    Motor_DM_Cmd(motor, DM_CMD_MOTOR_STOP);

    /* 注册到全局链表 */
    motor->base.GetRawPosition = dm_get_raw_position;
    motor->base.Apply          = dm_apply;
    Motor_Register(&motor->base);

    LOG_I("DM motor initialized (type=%d, mode=0x%03X)", motor->base.info.motor_type, DM_Mode_type);

    return motor;
}
