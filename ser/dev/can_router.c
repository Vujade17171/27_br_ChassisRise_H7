/*****************************************************************************
 * can_router.c —— CAN 帧路由实现
 *
 * 数据流（上行）：bsp 中断取帧 -> 本文件按帧格式分流 -> 各驱动解析到句柄
 * 依赖方向：can_router -> bsp_can（注册）、can_router -> ak/el05 驱动
 * 本文件不引用任何 app 层符号。
 *****************************************************************************/
#include "can_router.h"

/* 绑定的 M2006 句柄数组：支持"多段"绑定，
 * 一次 CAN 帧会按顺序在所有段里找 ID 匹配的电机。
 * 例如底盘 4 个 + 导向 1 个：
 *     CAN_Router_M2006(chassis_motor, 4);
 *     CAN_Router_M2006(guide_motor,   1);
 * 每段只存指针 + 个数，本模块不定义任何电机实例。 */
#define M2006_ROUTER_MAX_SEG   4u

typedef struct
{
    M2006_Motor *motors;
    uint8_t      count;
} m2006_router_seg_t;

static m2006_router_seg_t s_m2006_seg[M2006_ROUTER_MAX_SEG];
static uint8_t s_m2006_seg_num;

static VESC_Motor *s_vesc;
static uint8_t s_vesc_num;


int32_t M2006_CAN_Router_Init(bsp_can_bus_t bus)
{
    s_m2006_seg_num = 0u;       /* 重新初始化时清掉旧的绑定 */
    return BSP_CAN_RegisterRx(bus, M2006_CAN_Router_OnFrame); // M2006回调接口
}

int32_t CAN_Router_M2006(M2006_Motor *motors, uint8_t count)
{
    if ((motors == 0) || (count == 0u))
    {
        return SER_ERR_PARAM;
    }
    if (s_m2006_seg_num >= (uint8_t)M2006_ROUTER_MAX_SEG)
    {
        return SER_ERR_LIMIT;   /* 段数上限，见 M2006_ROUTER_MAX_SEG */
    }

    s_m2006_seg[s_m2006_seg_num].motors = motors;
    s_m2006_seg[s_m2006_seg_num].count  = count;
    s_m2006_seg_num++;

    return SER_OK;
}

void CAN_Router_M2006_Clear(void)
{
    uint8_t i;

    for (i = 0u; i < (uint8_t)M2006_ROUTER_MAX_SEG; i++)
    {
        s_m2006_seg[i].motors = 0;
        s_m2006_seg[i].count  = 0u;
    }
    s_m2006_seg_num = 0u;
}

uint32_t CAN_Router_GetFrameCount(bsp_can_bus_t bus)
{
    return BSP_CAN_GetRxFrameCount(bus);
}

void M2006_CAN_Router_OnFrame(bsp_can_bus_t bus, uint32_t id, uint32_t id_type,
                             const uint8_t *data, uint8_t len)
{
    uint8_t i;

    (void)bus;
    (void)id_type;

    if ((data == 0) || (len < 8u))
    {
        return;
    }

    /* 依次在每个已绑定的段里找 ID 匹配的电机，找到就结束 */
    for (i = 0u; i < s_m2006_seg_num; i++)
    {
        if (M2006_Motor_RxDispatch(s_m2006_seg[i].motors,
                                   s_m2006_seg[i].count,
                                   id, data, len) == SER_OK)
        {
            return;
        }
    }
}



int32_t VESC_CAN_Router_Init(bsp_can_bus_t bus)
{
    return BSP_CAN_RegisterRx(bus, VESC_CAN_Router_OnFrame); // VESC回调接口
}

int32_t CAN_Router_VESC(VESC_Motor *motors, uint8_t count)
{
    if ((motors == 0) || (count == 0u))
    {
        return SER_ERR_PARAM;
    }
    s_vesc = motors;
    s_vesc_num = count;
    return SER_OK;
}


void VESC_CAN_Router_OnFrame(bsp_can_bus_t bus, uint32_t id, uint32_t id_type,
                             const uint8_t *data, uint8_t len)
{
    (void)bus;
    (void)id_type;

    if ((data == 0) || (len < 8u))
    {
        return;
    }

    if (s_vesc != 0)
    {
        (void)VESC_Motor_RxDispatch(s_vesc, s_vesc_num, id, data, len);
    }
}