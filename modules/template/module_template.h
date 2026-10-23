/*
 * @Author: <your-name> <your-email>
 * @Date: <date>
 * @Description: <模块名> 对外接口头文件
 *
 * 使用说明:
 * 1. 复制 template/ 文件夹到 modules/<MODULE_NAME>/
 * 2. 全局替换 "TEMPLATE" / "template" 为你的模块名
 * 3. 根据实际需求修改默认参数和对外接口
 */
#ifndef _MODULE_TEMPLATE_H_
#define _MODULE_TEMPLATE_H_

#include <stdint.h>
#include <stdbool.h>

/* 默认参数 (可被 module_config.h / robot.cmake 覆盖) */

#ifndef TEMPLATE_TASK_STACK_SIZE
#define TEMPLATE_TASK_STACK_SIZE 512 /* 任务栈大小 (字节) */
#endif

#ifndef TEMPLATE_TASK_PRIORITY
#define TEMPLATE_TASK_PRIORITY 10 /* 任务优先级 (1~31, 越小优先级越高) */
#endif

#ifndef TEMPLATE_TASK_INTERVAL_MS
#define TEMPLATE_TASK_INTERVAL_MS 10 /* 任务循环间隔 (ms) */
#endif

/* 模块数据结构 */

typedef struct
{
    uint8_t initialized; /* 0=未初始化, 1=已初始化 */
    /* TODO: 在此添加模块私有数据 */
    float example_value;
} Module_Template_Device_t;

/**
 * @brief 模块初始化
 * @return 0 成功, 非 0 失败
 */
int Module_Template_Init(void);

/**
 * @brief 获取模块设备句柄 (用于读取数据或状态)
 * @return 模块设备指针, 未初始化时返回 NULL
 */
const Module_Template_Device_t *Module_Template_Get(void);

/**
 * @brief 示例: 设置某个参数
 * @param value 参数值
 */
void Module_Template_SetExample(float value);

#endif /* _MODULE_TEMPLATE_H_ */
