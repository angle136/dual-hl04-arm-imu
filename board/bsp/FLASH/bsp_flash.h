#ifndef _BSP_FLASH_H_
#define _BSP_FLASH_H_

#include <stdint.h>

/**
 * @description: 写入数据到Flash存储区
 * @details      F4写入内部Flash指定扇区(0x080E0000, 128KB)，
 *               H7写入外部W25Q64(从地址0开始)
 * @param {uint8_t*} buffer - 要写入的数据指针
 * @param {uint32_t} length - 数据大小(字节)
 * @return {uint8_t} 0表示成功，其他值表示失败
 */
uint8_t BSP_FLASH_Write_Buffer(uint8_t *buffer, uint32_t length);

/**
 * @description: 从Flash存储区读取数据
 * @details      F4从内部Flash指定扇区读取，
 *               H7从外部W25Q64读取(从地址0开始)
 * @param {uint8_t*} buffer - 接收数据缓冲区指针
 * @param {uint32_t} length - 数据大小(字节)
 * @return {uint8_t} 0表示成功，其他值表示失败
 */
uint8_t BSP_FLASH_Read_Buffer(uint8_t *buffer, uint32_t length);

#endif // _BSP_FLASH_H_
