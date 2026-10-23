#include "bsp_dwt.h"

typedef struct
{
    uint32_t s;
    uint16_t ms;
    uint16_t us;
} DWT_Time_t;

static DWT_Time_t SysTime;
static uint32_t   CPU_FREQ_Hz, CPU_FREQ_Hz_ms, CPU_FREQ_Hz_us;
static uint32_t   CYCCNT_RountCount;
static uint32_t   CYCCNT_LAST;
static uint64_t   CYCCNT64;

// 重入保护标志:0-未持有, 1-已持有
static volatile uint8_t dwt_lock_state = 0;

/**
 * @brief 检查并更新DWT计数器溢出状态
 * @note 使用简单的状态量判断避免重入问题
 */
static inline void DWT_CNT_Update(void)
{
    // 如果已经持有锁,直接返回(重入情况)
    if (dwt_lock_state)
    {
        return;
    }
    
    // 获取
    dwt_lock_state = 1;
    
    volatile uint32_t cnt_now = DWT->CYCCNT;
    
    // 检测溢出:如果当前值小于上次值,说明发生了溢出
    if (cnt_now < CYCCNT_LAST)
    {
        CYCCNT_RountCount++;
    }
    
    CYCCNT_LAST = cnt_now;
    
    // 释放
    dwt_lock_state = 0;
}

void DWT_SysTimeUpdate(void)
{
    volatile uint32_t cnt_now = DWT->CYCCNT;

    // 更新溢出状态
    DWT_CNT_Update();

    // 计算64位计数器值:溢出次数 * 2^32 + 当前计数值
    CYCCNT64 = (uint64_t)CYCCNT_RountCount * 0x100000000ULL + (uint64_t)cnt_now;
    
    uint64_t total_us = CYCCNT64 / CPU_FREQ_Hz_us;
    SysTime.s  = (uint32_t)(total_us / 1000000ULL);
    uint64_t remaining_us = total_us % 1000000ULL;
    SysTime.ms = (uint16_t)(remaining_us / 1000ULL);
    SysTime.us = (uint16_t)(remaining_us % 1000ULL);
}

void BSP_DWT_Init(uint32_t CPU_Freq_mHz)
{
    /* 使能DWT外设 */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

    /* DWT CYCCNT寄存器计数清0 */
    DWT->CYCCNT = (uint32_t)0u;

    /* 使能Cortex-M DWT CYCCNT寄存器 */
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    CPU_FREQ_Hz       = CPU_Freq_mHz * 1000000;
    CPU_FREQ_Hz_ms    = CPU_FREQ_Hz / 1000;
    CPU_FREQ_Hz_us    = CPU_FREQ_Hz / 1000000;
    CYCCNT_RountCount = 0;
    dwt_lock_state    = 0;

    CYCCNT_LAST = DWT->CYCCNT;
}

float BSP_DWT_GetDeltaT(uint32_t *cnt_last)
{
    volatile uint32_t cnt_now = DWT->CYCCNT;
    float             dt      = ((uint32_t)(cnt_now - *cnt_last)) / ((float)(CPU_FREQ_Hz));
    *cnt_last                 = cnt_now;

    // 更新溢出状态
    DWT_CNT_Update();

    return dt;
}

double BSP_DWT_GetDeltaT64(uint32_t *cnt_last)
{
    volatile uint32_t cnt_now = DWT->CYCCNT;
    double            dt      = ((uint32_t)(cnt_now - *cnt_last)) / ((double)(CPU_FREQ_Hz));
    *cnt_last                 = cnt_now;

    // 更新溢出状态
    DWT_CNT_Update();

    return dt;
}

float BSP_DWT_GetTimeline_s(void)
{
    DWT_SysTimeUpdate();

    return (float)SysTime.s + ((float)SysTime.ms) * 0.001f + ((float)SysTime.us) * 0.000001f;
}

float BSP_DWT_GetTimeline_ms(void)
{
    DWT_SysTimeUpdate();

    return ((float)SysTime.s) * 1000.0f + (float)SysTime.ms + ((float)SysTime.us) * 0.001f;
}

uint64_t BSP_DWT_GetTimeline_us(void)
{
    DWT_SysTimeUpdate();

    return (uint64_t)SysTime.s * 1000000ULL + (uint64_t)SysTime.ms * 1000ULL + SysTime.us;
}

void BSP_DWT_Delay(float Delay)
{

    uint32_t tickstart = DWT->CYCCNT;
    uint32_t wait_cycles = (uint32_t)(Delay * (float)CPU_FREQ_Hz);

    // 避免溢出导致的死循环
    while ((DWT->CYCCNT - tickstart) < wait_cycles)
    {
        // 添加编译器屏障防止循环被优化掉
        __asm__ volatile("" ::: "memory");
    }
}
