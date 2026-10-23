#include "bsp_pwm.h"
#include "bsp_def.h"
#include "tim.h"
#include "tx_api.h"
#include <string.h>

#define LOG_TAG "bsp_pwm"
#define LOG_LVL LOG_LVL_WARNING
#include "ulog_def.h"

/* ── 事件标志位 ── */
#define BSP_PWM_EVENT_COMPLETE ((ULONG)0x01)

/* 全局设备链表 */
static PWM_Device *devices_list = NULL;

/* PWM 设备 */
typedef struct PWM_Device
{
    PWM_Device          *next;        /* 指向下一个设备指针 */
    TIM_HandleTypeDef   *htim;        /* 定时器句柄 */
    uint32_t             Channel;     /* 通道 */
    uint32_t             Period;      /* 自动重装载值 */
    uint32_t             dutyx10;     /* 占空比 (0-1000) */
    PWM_Mode             Mode;        /* 工作模式 */
    TX_EVENT_FLAGS_GROUP event_flags; /* PWM 事件组 */
} PWM_Device;

/* 内部辅助函数 */

static uint32_t PWM_Convert_Dutyx10_To_Pulse(PWM_Device *dev) { return (dev->Period * dev->dutyx10) / 1000; }

/* 对外函数 */

PWM_Device *BSP_PWM_Device_Init(PWM_Init_Config *config)
{
    if (config == NULL || config->htim == NULL)
    {
        LOG_E("Invalid config");
        return NULL;
    }

    TIM_HandleTypeDef *htim = (TIM_HandleTypeDef *)config->htim;

    /* 创建设备 */
    PWM_Device *dev = NULL;
    BSP_MEM_ALLOC_WAIT(dev, sizeof(PWM_Device), TX_NO_WAIT);
    if (dev == NULL)
    {
        LOG_E("Failed to allocate device");
        return NULL;
    }

    memset(dev, 0, sizeof(PWM_Device));
    dev->htim    = htim;
    dev->Channel = config->Channel;
    dev->Period  = htim->Init.Period; /* 从 CubeMX 配置读取周期 */
    dev->dutyx10 = config->dutyx10;
    dev->Mode    = config->Mode;

    /* 创建事件标志组 */
    if (tx_event_flags_create(&dev->event_flags, "pwm_evt") != TX_SUCCESS)
    {
        LOG_E("Event flags create failed");
        BSP_MEM_FREE(dev);
        return NULL;
    }

    /* 挂载到全局设备链表 */
    dev->next    = devices_list;
    devices_list = dev;

    LOG_I("Device init OK: ch=%lu", dev->Channel);
    return dev;
}

uint8_t BSP_PWM_Start(PWM_Device *dev, uint8_t *data, uint8_t len)
{
    HAL_StatusTypeDef hal_status;
    uint32_t          hal_channel;
    uint32_t         *dma_buffer;
    uint16_t          dma_length;

    if (dev == NULL || dev->htim == NULL) return 1;

    hal_channel = dev->Channel;

    /* 清除残留事件标志 */
    {
        ULONG dummy;
        tx_event_flags_get(&dev->event_flags, BSP_PWM_EVENT_COMPLETE, TX_OR_CLEAR, &dummy, TX_NO_WAIT);
    }

    switch (dev->Mode)
    {
    case PWM_MODE_BLOCKING:
        hal_status = HAL_TIM_PWM_Start(dev->htim, hal_channel);
        break;

    case PWM_MODE_IT:
        hal_status = HAL_TIM_PWM_Start_IT(dev->htim, hal_channel);
        break;

    case PWM_MODE_DMA:
        if (data == NULL || len == 0)
        {
            LOG_E("DMA mode requires valid data buffer and length");
            return 1;
        }
        dma_buffer = (uint32_t *)data;
        dma_length = len;

        if (BSP_IS_DMA_INACCESSIBLE(dma_buffer))
        {
            LOG_E("DMA buffer in inaccessible memory");
            return 1;
        }
        BSP_CACHE_CLEAN(dma_buffer, dma_length * sizeof(uint32_t));

        hal_status = HAL_TIM_PWM_Start_DMA(dev->htim, hal_channel, dma_buffer, dma_length);
        break;

    default:
        LOG_E("Invalid PWM mode: %d", dev->Mode);
        return 1;
    }

    if (hal_status != HAL_OK)
    {
        LOG_E("Failed to start PWM: HAL status=%d", hal_status);
        return 1;
    }

    return 0;
}

uint8_t BSP_PWM_Stop(PWM_Device *dev)
{
    HAL_StatusTypeDef hal_status;
    uint32_t          hal_channel;

    if (dev == NULL || dev->htim == NULL) return 1;

    hal_channel = dev->Channel;

    switch (dev->Mode)
    {
    case PWM_MODE_BLOCKING:
        hal_status = HAL_TIM_PWM_Stop(dev->htim, hal_channel);
        break;
    case PWM_MODE_IT:
        hal_status = HAL_TIM_PWM_Stop_IT(dev->htim, hal_channel);
        break;
    case PWM_MODE_DMA:
        hal_status = HAL_TIM_PWM_Stop_DMA(dev->htim, hal_channel);
        break;
    default:
        return 1;
    }

    return (hal_status == HAL_OK) ? 0 : 1;
}

uint8_t BSP_PWM_SetDutyCycle(PWM_Device *dev, uint16_t duty)
{
    uint32_t hal_channel;
    uint32_t pulse;

    if (dev == NULL) return 1;

    if (duty > 1000) duty = 1000;

    dev->dutyx10 = duty;
    pulse        = PWM_Convert_Dutyx10_To_Pulse(dev);
    hal_channel  = dev->Channel;

    __HAL_TIM_SET_COMPARE(dev->htim, hal_channel, pulse);

    return 0;
}

uint8_t BSP_PWM_SetPulse(PWM_Device *dev, uint32_t pulse)
{
    uint32_t hal_channel;

    if (dev == NULL) return 1;

    if (pulse > dev->Period) pulse = dev->Period;

    dev->dutyx10 = (pulse * 1000) / dev->Period;
    hal_channel  = dev->Channel;

    __HAL_TIM_SET_COMPARE(dev->htim, hal_channel, pulse);

    return 0;
}

uint8_t BSP_PWM_SetPSC(PWM_Device *dev, uint32_t psc)
{
    if (dev == NULL || dev->htim == NULL) return 1;

    __HAL_TIM_SET_PRESCALER(dev->htim, psc);

    return 0;
}

uint8_t BSP_PWM_WaitForComplete(PWM_Device *dev, uint32_t wait_option)
{
    if (dev == NULL) return 1;

    ULONG actual = 0;
    UINT  status = tx_event_flags_get(&dev->event_flags, BSP_PWM_EVENT_COMPLETE, TX_OR_CLEAR, &actual, wait_option);

    return (status == TX_SUCCESS) ? 0 : 1;
}

/* HAL 回调 */

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
    for (PWM_Device *dev = devices_list; dev ; dev = dev->next)
    {
        if (dev->htim == htim)
        {
            tx_event_flags_set(&dev->event_flags, BSP_PWM_EVENT_COMPLETE, TX_OR);
        }
    }
}
