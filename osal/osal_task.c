/*****************************************************************************
 * osal_task.c —— 任务与节拍抽象实现（CMSIS-RTOS v2 版）
 *
 * 依赖方向：osal -> cmsis_os2（RTOS 原语）
 *
 * 注意：本工程 CubeMX 生成的是 CMSIS-RTOS v2（cmsis_os2.h / cmsis_os2.c）。
 *   Middlewares 下的 cmsis_os.h 只是一个兼容壳（内部只 #include "cmsis_os2.h"），
 *   并不提供 v1 的 osThreadDef_t / osThreadCreate / osKernelSysTick 等接口，
 *   所以这里必须用 v2 的 osThreadNew / osThreadTerminate / osDelayUntil。
 *****************************************************************************/
#include "osal_task.h"

#include "cmsis_os2.h"

/* OSAL_PRIO_* 的数值与 CMSIS-RTOS2 的 osPriority 枚举一致
 * （Low=8 / Normal=24 / High=40 / Realtime=48），仍做一次显式映射，
 * 避免以后两边数值被改动后静默错位。 */
static osPriority_t osal_prio_to_cmsis(uint32_t prio)
{
    switch (prio) {
        case OSAL_PRIO_LOW:      return osPriorityLow;
        case OSAL_PRIO_NORMAL:   return osPriorityNormal;
        case OSAL_PRIO_HIGH:     return osPriorityHigh;
        case OSAL_PRIO_REALTIME: return osPriorityRealtime;
        default:                 return osPriorityNormal;
    }
}

osal_task_t OSAL_Task_Create(const char *name, osal_task_entry_t entry, void *arg,
                             uint32_t stack_bytes, uint32_t prio)
{
    osThreadAttr_t attr;

    if ((name == 0) || (entry == 0) || (stack_bytes < 4u)) { return 0; }

    attr.name       = name;
    attr.attr_bits  = osThreadDetached;
    attr.cb_mem     = 0;
    attr.cb_size    = 0u;
    attr.stack_mem  = 0;
    /* v2 的 stack_size 单位就是字节（v1 才需要 /4 换算成 word） */
    attr.stack_size = stack_bytes;
    attr.priority   = osal_prio_to_cmsis(prio);
    attr.tz_module  = 0;
    attr.reserved   = 0u;

    return (osal_task_t)osThreadNew((osThreadFunc_t)entry, arg, &attr);
}

int32_t OSAL_Task_Destroy(osal_task_t task)
{
    if (task == 0) { return SER_ERR_PARAM; }
    return (osThreadTerminate((osThreadId_t)task) == osOK) ? SER_OK : SER_ERR_STATE;
}

int32_t OSAL_Task_DelayUntil(uint32_t *tick, uint32_t inc)
{
    if ((tick == 0) || (inc == 0u)) { return SER_ERR_PARAM; }

    /* v2 的 osDelayUntil(ticks) 只接受一个“绝对唤醒时刻”参数，
     * 周期推进要由调用方维护：
     *   首次使用前 *tick 必须先初始化为当前 osKernelGetTickCount()，
     *   为 0 时这里自动取一次，避免首个周期直接睡满一整圈。 */
    if (*tick == 0u) { *tick = osKernelGetTickCount(); }
    *tick += inc;

    return (osDelayUntil(*tick) == osOK) ? SER_OK : SER_ERR_STATE;
}
