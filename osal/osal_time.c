/*****************************************************************************
 * osal_time.c —— 时间与延时抽象实现（CMSIS-RTOS v2 版）
 *
 * 依赖方向：osal -> bsp（bsp_dwt 提供时间戳）、osal -> cmsis_os2（提供 tick）
 *
 * 注意：本工程 CubeMX 生成的是 CMSIS-RTOS v2（cmsis_os2.h / cmsis_os2.c），
 *   v1 的 osKernelSysTick / osKernelSysTickFrequency 在 v2 中已改名为
 *   osKernelGetTickCount / osKernelGetTickFreq。
 *****************************************************************************/
#include "osal_time.h"

#include "bsp_dwt.h"     /* DWT_Init / DWT_GetDeltaT：bsp 层时间源 */
#include "cmsis_os2.h"   /* v2：osKernelGetTickCount / osKernelGetTickFreq / osDelay */

int32_t OSAL_Time_Init(uint32_t cpu_mhz)
{
    if (cpu_mhz == 0u) { return SER_ERR_PARAM; }
    DWT_Init(cpu_mhz);
    return SER_OK;
}

float OSAL_Time_Dt(uint32_t *last_cnt)
{
    if (last_cnt == 0) { return 0.0f; }
    return DWT_GetDeltaT(last_cnt);
}

void OSAL_Time_ResetDt(uint32_t *last_cnt)
{
    if (last_cnt == 0) { return; }
    (void)DWT_GetDeltaT(last_cnt);
}

uint32_t OSAL_Time_GetTick(void)
{
    return osKernelGetTickCount();
}

uint32_t OSAL_Time_GetMs(void)
{
    uint32_t tick = osKernelGetTickCount();
    uint32_t freq = (uint32_t)osKernelGetTickFreq();

    if (freq == 0u) { return tick; }
    /* 先除后乘，避免长时间运行后溢出（1ms 精度下余数可忽略） */
    return (tick / freq) * 1000u + ((tick % freq) * 1000u) / freq;
}

void OSAL_Time_DelayMs(uint32_t ms)
{
    (void)osDelay(ms);
}
