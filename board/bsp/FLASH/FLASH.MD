# BSP FLASH 驱动文档

## 概述

BSP FLASH 是平台抽象的非易失存储驱动层，提供统一的读写接口，底层根据芯片自动切换：

- **STM32F407xx**：操作内部 MCU Flash（扇区 11，`0x080E0000`，128 KB），通过 HAL_FLASH 实现。
- **STM32H723xx**：操作外部 W25Q64 QSPI Flash（8 MB），通过 OSPI 外设以 Quad I/O 模式访问。

所有操作均为同步阻塞模式。

## 架构设计

### 平台抽象模型

```
BSP_FLASH_Write_Buffer() / BSP_FLASH_Read_Buffer()
    │
    ├── STM32F407xx
    │   ├── HAL_FLASH_Unlock()
    │   ├── BSP_FLASH_Erase_Sector()    ── 扇区11 整扇区擦除
    │   ├── HAL_FLASH_Program(HALFWORD) ── 半字写入（STM32 内部 Flash 要求）
    │   └── HAL_FLASH_Lock()
    │
    └── STM32H723xx
        ├── W25Qxx_Init()               ── 首次调用时自动初始化（读 JEDEC ID 验证）
        ├── W25Qxx_BlockErase_64K()     ── 按需擦除 64KB 块
        │   ├── W25Qxx_WriteEnable()    ── 写使能 + WEL 轮询
        │   ├── HAL_OSPI_Command()      ── 发送擦除命令
        │   └── OSPI_W25Qxx_AutoPollingMemReady() ── 等待 BUSY 清零
        ├── W25Qxx_WritePage()          ── 按页写入（256B/页，Quad 模式）
        │   ├── W25Qxx_WriteEnable()
        │   ├── HAL_OSPI_Command()      ── 发送页编程命令
        │   ├── HAL_OSPI_Transmit()     ── 4 线传输数据
        │   └── OSPI_W25Qxx_AutoPollingMemReady()
        └── HAL_OSPI_Receive()          ── 4 线 Fast Read Quad I/O 读取
```

### F4 内部 Flash 参数

| 参数 | 值 | 说明 |
|------|-----|------|
| `USER_FLASH_SECTOR` | `FLASH_SECTOR_11` | 使用的扇区编号 |
| `USER_FLASH_SECTOR_ADDR` | `0x080E0000` | 扇区起始地址 |
| `USER_FLASH_SECTOR_SIZE` | `128 KB` | 扇区大小 |
| 写入粒度 | 半字（16-bit） | STM32F4 内部 Flash 硬件要求 |
| 擦除粒度 | 整扇区（128 KB） | 写前整扇区擦除 |

### H7 外部 W25Q64 参数

| 参数 | 值 | 说明 |
|------|-----|------|
| `W25Qxx_FlashSize` | `0x800000`（8 MB） | Flash 总容量 |
| `W25Qxx_PageSize` | `256` 字节 | 页编程最大粒度 |
| `W25Qxx_FLASH_ID` | `0xEF4017` | JEDEC ID（Manufacturer: Winbond, Type: W25Q64） |
| 擦除粒度 | `64 KB` 块 | 写前按需擦除所需块 |
| 数据线模式 | Quad I/O（4 线） | 地址 + 数据均为 4 线 |
| OSPI 句柄 | `hospi2` | 外部 `OSPI_HandleTypeDef` |

### W25Q64 关键命令

| 命令 | 宏 | 说明 |
|------|-----|------|
| `0x9F` | `W25Qxx_CMD_JedecID` | 读取 JEDEC ID，用于验证芯片 |
| `0x06` | `W25Qxx_CMD_WriteEnable` | 写使能（擦除/编程前置） |
| `0xD8` | `W25Qxx_CMD_BlockErase_64K` | 64KB 块擦除 |
| `0x32` | `W25Qxx_CMD_QuadInputPageProgram` | 4 线页编程（≤256 字节） |
| `0xEB` | `W25Qxx_CMD_FastReadQuad_IO` | 4 线快速读取 |
| `0x05` | `W25Qxx_CMD_ReadStatus_REG1` | 读状态寄存器 1（BUSY/WEL） |

## API 接口说明

### 写入

```c
uint8_t BSP_FLASH_Write_Buffer(uint8_t *buffer, uint32_t length);
```

```c
uint8_t BSP_FLASH_Read_Buffer(uint8_t *buffer, uint32_t length);
```


## 使用示例

### 写入并回读验证

```c
#include "bsp_flash.h"
#include "bsp_def.h"

static uint8_t g_flash_buf[256] BUFFER_SECTION;

void flash_write_read_demo(void)
{
    /* 准备数据 */
    for (int i = 0; i < 256; i++)
    {
        g_flash_buf[i] = (uint8_t)i;
    }

    /* 写入 Flash */
    if (BSP_FLASH_Write_Buffer(g_flash_buf, sizeof(g_flash_buf)) != 0)
    {
        /* 写入失败 */
        return;
    }

    /* 清除缓冲区后回读 */
    memset(g_flash_buf, 0, sizeof(g_flash_buf));

    if (BSP_FLASH_Read_Buffer(g_flash_buf, sizeof(g_flash_buf)) != 0)
    {
        /* 读取失败 */
        return;
    }

    /* 验证数据 */
    for (int i = 0; i < 256; i++)
    {
        if (g_flash_buf[i] != (uint8_t)i)
        {
            /* 数据不匹配 */
            return;
        }
    }
}
```

## 内部实现细节

### F4 

- STM32F4 内部 Flash 以半字（16-bit）为单位编程。`BSP_FLASH_Write_Buffer` 将字节流两两组合为半字写入，末尾奇数字节单独按字节写入：
- 写前无条件擦除整个扇区 11（128KB）。这意味着即使只写入少量数据，扇区内原有数据也会全部丢失。如需增量更新，应先读取 → 修改 → 整体写回。

### H7
- `BSP_FLASH_Write_Buffer` 和 `BSP_FLASH_Read_Buffer` 首次调用时自动通过 `w25qxx_inited` 标志触发 `W25Qxx_Init()`，读取 JEDEC ID 并验证是否为 `0xEF4017`。验证失败则返回错误。
- 每次擦除/编程操作前调用 `W25Qxx_WriteEnable()`，发送 `0x06` 命令后通过 OSPI AutoPolling 等待状态寄存器 1 的 WEL 位（bit1）置位。操作完成后通过 `OSPI_W25Qxx_AutoPollingMemReady()` 轮询 BUSY 位（bit0）清零，确保操作完成。
- `BSP_FLASH_Write_Buffer` 根据写入长度计算需要擦除的 64KB 块数，从地址 0 开始依次擦除。不需要全片擦除。

## 注意事项

- **F4 写前整扇区擦除** — 扇区 11 原有数据在写入时会被完全清除。需要保留的数据应在写前读取并合并。
- **无锁保护** — 本驱动不提供互斥锁。多任务并发访问需上层保证互斥。
- **写入不是原子操作** — 写入过程（尤其是 H7 多页写入）中掉电会导致数据不完整。

## 版本历史

| 版本 | 日期 | 说明 |
|------|------|------|
| v1.0 | 2026-05-05 | 初始版本 |
