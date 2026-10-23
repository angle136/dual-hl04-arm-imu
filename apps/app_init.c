#include "app_init.h"
#include "tx_api.h"

#define LOG_LVL LOG_LVL_INFO
#define LOG_TAG "APP_Init"
#include "ulog_def.h"

#include "robot_control.h"

void APP_Init(void)
{
    robot_control_init();

    LOG_I("APP init finished");
}