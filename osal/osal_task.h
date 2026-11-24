/*****************************************************************************
 * osal_task.h —— 任务与节拍抽象（osal 层）
 *
 * 分层位置：osal —— 上层只用本文件的接口，不直接 include cmsis_os2 头文件，
 *   这样换 RTOS 时只改 osal 层。
 *
 * 说明：当前工程的 Leg_Task 由 CubeMX 在 freertos.c 里创建（osThreadNew），
 *   应用层只用到 OSAL_Task_DelayUntil() 做固定周期节拍。
 *   OSAL_Task_Create/Destroy 供后续新增任务使用。
 *****************************************************************************/
#ifndef __OSAL_TASK_H
#define __OSAL_TASK_H

#include <stdint.h>
#include "return_typedef.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef void *osal_task_t;
typedef void (*osal_task_entry_t)(void *arg);

/* 优先级：数值与 CMSIS-RTOS2 的 osPriority* 一致，此处只暴露中立名字 */
#define OSAL_PRIO_LOW       8u
#define OSAL_PRIO_NORMAL   24u
#define OSAL_PRIO_HIGH     40u
#define OSAL_PRIO_REALTIME 48u

/* 创建 / 删除任务；stack_bytes 为字节数（非 word 数）。失败返回 0。 */
osal_task_t OSAL_Task_Create(const char *name, osal_task_entry_t entry, void *arg,
                             uint32_t stack_bytes, uint32_t prio);
int32_t OSAL_Task_Destroy(osal_task_t task);

/* 固定周期节拍：*tick 是上一次唤醒时刻（首次使用前初始化为当前 tick，
 * 为 0 时由实现自动取一次），每次调用阻塞到下一个 inc 毫秒周期。
 * 对应 CMSIS-RTOS2 的 osDelayUntil(绝对 tick)。 */
int32_t OSAL_Task_DelayUntil(uint32_t *tick, uint32_t inc);

#ifdef __cplusplus
}
#endif

#endif /* __OSAL_TASK_H */
