#ifndef BSP_DWT_H
#define BSP_DWT_H

#ifdef __cplusplus
extern "C"{
#endif

#include "main.h"
#include <stdint.h>
#include "core_cm7.h"   // 必须包含这个头文件

typedef struct
{
    uint32_t s;
    uint16_t ms;
    uint16_t us;
}DWT_Typedef;

void DWT_Init(uint32_t CPU_Freq_mHz);
float DWT_GetDeltaT(uint32_t *cnt_last);
double DWT_GetDeltaT64(uint32_t *cnt_last);
void DWT_SysTimeUpdate(void);
float DWT_GetTimeline_s(void);
float DWT_GetTimeline_ms(void);
uint64_t DWT_GetTimeline_us(void);
void DWT_Delay(float Delay);

#ifdef __cplusplus
}
#endif

#endif /* BSP_DWT_H */
