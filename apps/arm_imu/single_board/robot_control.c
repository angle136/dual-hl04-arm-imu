#include "robot_control.h"

#include "arm_dual_imu.h"
#include "bsp_def.h"
#include "bsp_gpio.h"
#include "main.h"

#if (ARM_IMU_DEBUG_ENABLE != 0)
#include "arm_imu_debug.h"
#endif

#define LOG_TAG "app_arm_imu"
#define LOG_LVL LOG_LVL_INFO
#include "ulog_def.h"

#if (ARM_DUAL_IMU_ENABLE != 0)
#define ARM_IMU_ZERO_BUTTON_DEBOUNCE_MS 200U

/**
 * @brief 处理用户按键触发的双 IMU 手动置零请求。
 * @param 无。
 * @retval 无。
 * @note 调用关系：由 KEY 外部中断经 BSP_GPIO 回调；执行 200 ms 软件消抖后只置零请求标志，不在中断中计算矩阵。
 */
static void arm_imu_zero_button_callback(void)
{
    static uint32_t last_press_tick;
    static uint8_t  have_last_press;
    uint32_t        now = HAL_GetTick();

    if (have_last_press != 0U && (now - last_press_tick) < ARM_IMU_ZERO_BUTTON_DEBOUNCE_MS) return;

    last_press_tick = now;
    have_last_press = 1U;
    arm_dual_imu_request_zero();
}

/**
 * @brief 初始化启用状态下的双 IMU 手臂映射 APP。
 * @param 无。
 * @retval 无。
 * @note 调用关系：由应用初始化框架调用；依次启动双 IMU 模块、注册手动置零按键，并按配置启动调试线程。
 */
void robot_control_init(void)
{
    arm_dual_imu_init();

    if (BSP_GPIO_EXTI_Register(KEY_Pin, arm_imu_zero_button_callback) == 0xFFU)
    {
        LOG_E("manual zero button registration failed");
    }

#if (ARM_IMU_DEBUG_ENABLE != 0)
    arm_imu_debug_init();
#endif

    LOG_I("dual HL04 arm application initialized");
}
#else
/**
 * @brief 处理双 IMU 功能在编译期关闭时的 APP 初始化入口。
 * @param 无。
 * @retval 无。
 * @note 调用关系：由应用初始化框架调用；仅记录禁用状态，不创建线程或初始化外设。
 */
void robot_control_init(void) { LOG_W("dual IMU arm application disabled by configuration"); }
#endif
