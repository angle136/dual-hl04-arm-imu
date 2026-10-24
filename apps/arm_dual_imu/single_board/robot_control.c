#include "robot_control.h"

#include "arm_dual_imu.h"
#include "bsp_def.h"
#include "tx_api.h"

#define LOG_TAG "app_arm_dual_imu"
#define LOG_LVL LOG_LVL_INFO
#include "ulog_def.h"

#if (ARM_DUAL_IMU_ENABLE != 0)
static TX_THREAD                  g_arm_dual_imu_status_thread;
APPS_STACK_SECTION static uint8_t g_arm_dual_imu_status_stack[1024];

static void arm_dual_imu_status_task(ULONG thread_input)
{
    (void)thread_input;

    while (1)
    {
        arm_dual_imu_log_status();
        tx_thread_sleep(1000U);
    }
}

void robot_control_init(void)
{
    arm_dual_imu_init();

    UINT status = tx_thread_create(&g_arm_dual_imu_status_thread, "arm_dual_imu_status", arm_dual_imu_status_task, 0U, g_arm_dual_imu_status_stack,
                                   sizeof(g_arm_dual_imu_status_stack), 20U, 20U, TX_NO_TIME_SLICE, TX_AUTO_START);
    if (status != TX_SUCCESS)
    {
        LOG_E("dual IMU status thread create failed: %u", (unsigned)status);
        return;
    }

    LOG_I("dual IMU arm application initialized");
}
#else
void robot_control_init(void) { LOG_W("dual IMU arm application disabled by configuration"); }
#endif
