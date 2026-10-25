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
void robot_control_init(void) { LOG_W("dual IMU arm application disabled by configuration"); }
#endif
