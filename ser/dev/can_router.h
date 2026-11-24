/*****************************************************************************
 * can_router.h —— CAN 帧路由（ser/dev）
 *
 * 分层位置：ser/dev —— 把 bsp 的中断上行"翻译"成具体设备的分发。
 *   允许依赖：bsp（注册接收者）、ser/dev 的各驱动、common
 *   禁止依赖：任何 app 全局变量
 *
 * 这是本次分层最关键的一处改动：
 *   旧工程的 Mycan.c 属于 bsp 层，却 extern 引用 app 层（Task/leg_task.c）的
 *   电机句柄数组，形成 bsp -> app 的反向依赖与环状依赖。
 *   现在改为【依赖注入 + 注册回调】：
 *     1) CAN_Router_Init()        把自己注册为 bsp 的接收者；
 *     2) CAN_Router_BindAk/El05() 由 app 在初始化时把句柄数组交进来；
 *   路由器从此不认识任何全局变量，bsp 也不再认识上层。
 *****************************************************************************/
#ifndef __CAN_ROUTER_H
#define __CAN_ROUTER_H

#include <stdint.h>
#include "bsp_can.h"
#include "M2006_motor.h"
#include "vesc_motor.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 初始化：注册为 bsp_can 的接收者（应在 BSP_CAN_Init 之后调用）。
 * 同时清空之前的 M2006 绑定，方便重复初始化。 */
int32_t M2006_CAN_Router_Init(bsp_can_bus_t bus);//M2006

/* 绑定一组 M2006 句柄（依赖注入）。可以【多次调用】把多组数组绑到同一条总线，
 * 一帧反馈会依次在每一组里按 ID 找对应电机：
 *     CAN_Router_M2006(chassis_motor, 4);   // ID 1~4
 *     CAN_Router_M2006(guide_motor,   1);   // ID 5
 * 最多 M2006_ROUTER_MAX_SEG 组（can_router.c 里配）。
 * 返回 SER_OK / SER_ERR_PARAM（空指针）/ SER_ERR_LIMIT（组数超上限）。 */
int32_t CAN_Router_M2006(M2006_Motor *motors, uint8_t count);//M2006

/* 清空所有 M2006 绑定（一般不需要，Init 里会自动清）。 */
void CAN_Router_M2006_Clear(void);



int32_t VESC_CAN_Router_Init(bsp_can_bus_t bus);//VESC

/* 绑定电机句柄数组（依赖注入）。绑定之前收到的帧会被丢弃，仅计数。 */
int32_t CAN_Router_VESC(VESC_Motor *motors, uint8_t count);//VESC

/* 获取收到并成功解析的帧总数（上电等待驱动板反馈时用它判断"是否已有通信"） */

void M2006_CAN_Router_OnFrame(bsp_can_bus_t bus, uint32_t id, uint32_t id_type,
                        const uint8_t *data, uint8_t len);
        
void VESC_CAN_Router_OnFrame(bsp_can_bus_t bus, uint32_t id, uint32_t id_type,
                             const uint8_t *data, uint8_t len);

#ifdef __cplusplus
}
#endif

#endif /* __CAN_ROUTER_H */