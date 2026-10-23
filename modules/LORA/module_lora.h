/*
 * @brief 塔石 L33 LoRa 透传模块。
 *        两板点对点透传，应用层帧：0xAA 0x55 + LEN + SEQ + 数据 + CRC8。
 * @note  参数由塔石上位机预先配置，本模块只负责运行期收发。
 */
#ifndef _MODULE_LORA_H_
#define _MODULE_LORA_H_

#include "bsp_uart.h"
#include <stdint.h>

typedef enum
{
    LORA_TYPE_INT8,
    LORA_TYPE_INT16,
    LORA_TYPE_INT32,
    LORA_TYPE_UINT8,
    LORA_TYPE_UINT16,
    LORA_TYPE_UINT32,
    LORA_TYPE_FLOAT,
    LORA_TYPE_DOUBLE,
} Lora_Data_Type;

/* 注册项只保存用户变量指针，不复制变量内容。 */
int Lora_Register(const char *name, void *value, Lora_Data_Type type);

void    Module_Lora_Init(void);
int     Lora_Start(void);
uint8_t Module_Lora_get_offline_status(void);

#endif /* _MODULE_LORA_H_ */
