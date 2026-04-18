#include "bsp_dwt.h"

static DWT_Typedef SysTime;
static uint32_t CPU_FREQ_Hz, CPU_FREQ_Hz_ms, CPU_FREQ_Hz_us;
static uint32_t CYCCNT_RountCount;
static uint32_t CYCCNT_LAST;
static uint64_t CYCCNT64;

/**
 * @brief static 函数,用于检查DWT CYCCNT寄存器是否溢出,并更新CYCCNT_RountCount
 * @attention 此函数假设两次调用之间的时间间隔不超过一次溢出
 */
static void DWT_CNT_Update(void)
{
    // 添加位锁,用于互斥访问
    static volatile uint8_t bit_locker = 0;
    if (!bit_locker)
    {
        /* 上锁 */
        bit_locker = 1;
        /* 读取DWT->CYCCNT寄存器的当前值 */
        volatile uint32_t cnt_now = DWT->CYCCNT;
        /* 溢出判断 */
        if (cnt_now < CYCCNT_LAST)
            CYCCNT_RountCount++;

        CYCCNT_LAST = DWT->CYCCNT;
        /* 解锁 */
        bit_locker = 0;
    }
}

// 启用 DWT 周期计数器
void DWT_Init(uint32_t CPU_Freq_mHz)
{


    // 对于 Cortex-M7，有时需要先解锁（部分情况下需要）
    if ((DWT->CTRL & DWT_CTRL_NOCYCCNT_Msk) == 0)   // 检查是否支持
    {
        // 解锁 DWT（Cortex-M7 通常需要这一步）
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;   // 启用 Trace
        DWT->LAR = 0xC5ACCE55;                            // 解锁（如果有锁）

        DWT->CYCCNT = 0;                                  // 清零计数器
        DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;              // 使能 CYCCNT
    }
    CPU_FREQ_Hz = CPU_Freq_mHz * 1000000;
    CPU_FREQ_Hz_ms = CPU_FREQ_Hz / 1000;
    CPU_FREQ_Hz_us = CPU_FREQ_Hz / 1000000;
    DWT_CNT_Update();
}

float DWT_GetDeltaT(uint32_t *cnt_last)
{
    uint32_t cnt_now = DWT->CYCCNT;
    /* 返回32位浮点数 */
    float dt = ((uint32_t)(cnt_now - *cnt_last)) / ((float)(CPU_FREQ_Hz));
    *cnt_last = cnt_now;
    DWT_CNT_Update();

    return dt;
}

double DWT_GetDeltaT64(uint32_t *cnt_last)
{
    volatile uint32_t cnt_now = DWT->CYCCNT;
    /* 返回64位浮点数 */
    double dt = ((uint32_t)(cnt_now - *cnt_last)) / ((double)(CPU_FREQ_Hz));
    *cnt_last = cnt_now;

    DWT_CNT_Update();

    return dt;
}

void DWT_SysTimeUpdate(void)
{
    /* 更新系统时间 */
    volatile uint32_t cnt_now = DWT->CYCCNT;
    static uint64_t CNT_TEMP1, CNT_TEMP2, CNT_TEMP3;

    /*  */
    DWT_CNT_Update();

    /*  */
    CYCCNT64 = (uint64_t)CYCCNT_RountCount * (uint64_t)UINT32_MAX + (uint64_t)cnt_now;
    CNT_TEMP1 = CYCCNT64 / CPU_FREQ_Hz;
    CNT_TEMP2 = CYCCNT64 - CNT_TEMP1 * CPU_FREQ_Hz;
    SysTime.s = CNT_TEMP1;
    SysTime.ms = CNT_TEMP2 / CPU_FREQ_Hz_ms;
    CNT_TEMP3 = CNT_TEMP2 - SysTime.ms * CPU_FREQ_Hz_ms;
    SysTime.us = CNT_TEMP3 / CPU_FREQ_Hz_us;
}

float DWT_GetTimeline_s(void)
{
    DWT_SysTimeUpdate();

    float DWT_Timelinef32 = SysTime.s + SysTime.ms * 0.001f + SysTime.us * 0.000001f;

    return DWT_Timelinef32;
}


float DWT_GetTimeline_ms(void)
{
    DWT_SysTimeUpdate();

    float DWT_Timelinef32 = SysTime.s * 1000 + SysTime.ms + SysTime.us * 0.001f;

    return DWT_Timelinef32;
}


uint64_t DWT_GetTimeline_us(void)
{
    DWT_SysTimeUpdate();

    uint64_t DWT_Timelinef32 = SysTime.s * 1000000 + SysTime.ms * 1000 + SysTime.us;

    return DWT_Timelinef32;
}


void DWT_Delay(float Delay)
{
    uint32_t tickstart = DWT->CYCCNT;
    float wait = Delay;

    while ((DWT->CYCCNT - tickstart) < wait * (float)CPU_FREQ_Hz)
        ;
}