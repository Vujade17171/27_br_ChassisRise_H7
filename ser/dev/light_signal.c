/*****************************************************************************
 * light_signal.c —— 4 路光电信号服务实现（ser/dev）
 *
 * 数据流：PG7~PG10 --BSP_GPIO_Refresh()--> s_raw[] --逐路消抖--> s_level[] --Get()--> app
 * 本文件不 include 任何 HAL 头，也不认识 app 的任何符号。
 *
 * 每路各自独立计数：一路抖动不会影响另外三路的消抖进度。
 *****************************************************************************/
#include "light_signal.h"

#if (LIGHT_SIGNAL_DEBOUNCE_N == 0u) || (LIGHT_SIGNAL_DEBOUNCE_N > 255u)
#error "LIGHT_SIGNAL_DEBOUNCE_N must be 1..255"
#endif

#if (LIGHT_SIGNAL_PIN1 != LIGHT_SIGNAL_CH1) || (LIGHT_SIGNAL_PIN2 != LIGHT_SIGNAL_CH2) || \
    (LIGHT_SIGNAL_PIN3 != LIGHT_SIGNAL_CH3) || (LIGHT_SIGNAL_PIN4 != LIGHT_SIGNAL_CH4)
#error "LIGHT_SIGNAL_PINx and LIGHT_SIGNAL_CHx must have the same numbering"
#endif

/* 通道 -> bsp 逻辑引脚（换引脚只改 light_signal.h 的 4 个宏） */
static const bsp_gpio_pin_t s_pin_map[LIGHT_SIGNAL_NUM] =
{
    LIGHT_SIGNAL_PIN1,   /* CH1 -> PG7  */
    LIGHT_SIGNAL_PIN2,   /* CH2 -> PG4  */
    LIGHT_SIGNAL_PIN3,   /* CH3 -> PD15 */
    LIGHT_SIGNAL_PIN4    /* CH4 -> PD12 */
};

static uint8_t s_raw[LIGHT_SIGNAL_NUM];       /* bsp 写进来的原始电平（绑定地址指向它） */
static uint8_t s_level[LIGHT_SIGNAL_NUM];     /* 消抖后的稳定电平，对外只认这个         */
static uint8_t s_diff_cnt[LIGHT_SIGNAL_NUM];  /* 与稳定电平不一致的连续拍数             */
static uint8_t s_inited;                      /* 0 = 还没 Init，Update 直接返回         */

/* ---------------------------------------------------------------------------
 * 初始化 / 周期更新
 * ------------------------------------------------------------------------- */
int32_t Light_Signal_Init(void)
{
    uint8_t i;

    s_inited = 0u;

    for (i = 0u; i < (uint8_t)LIGHT_SIGNAL_NUM; i++)
    {
        int32_t ret = BSP_GPIO_Bind(s_pin_map[i], &s_raw[i]);

        if (ret != SER_OK) { return ret; }

        s_level[i]    = s_raw[i];   /* 以上电第一拍的实际电平作为初值 */
        s_diff_cnt[i] = 0u;
    }

    s_inited = 1u;
    return SER_OK;
}

void Light_Signal_Update(void)
{
    uint8_t i;

    if (s_inited == 0u) { return; }

    (void)BSP_GPIO_Refresh();   /* 采样：bsp 把 4 路引脚电平写进 s_raw[] */

    for (i = 0u; i < (uint8_t)LIGHT_SIGNAL_NUM; i++)
    {
        uint8_t raw = s_raw[i];

        if (raw == s_level[i])
        {
            s_diff_cnt[i] = 0u;     /* 一致：清掉计数，稳定电平不动 */
            continue;
        }

        s_diff_cnt[i]++;
        if (s_diff_cnt[i] >= (uint8_t)LIGHT_SIGNAL_DEBOUNCE_N)
        {
            s_level[i]    = raw;    /* 连续 N 拍都一样，才认这次翻转 */
            s_diff_cnt[i] = 0u;
        }
    }
}

/* ---------------------------------------------------------------------------
 * 取值
 * ------------------------------------------------------------------------- */
uint8_t Light_Signal_Get(uint8_t ch)
{
    return (ch < (uint8_t)LIGHT_SIGNAL_NUM) ? s_level[ch] : 0u;
}

uint8_t Light_Signal_GetRaw(uint8_t ch)
{
    return (ch < (uint8_t)LIGHT_SIGNAL_NUM) ? s_raw[ch] : 0u;
}

uint8_t Light_Signal_GetAll(void)
{
    uint8_t i;
    uint8_t mask = 0u;

    for (i = 0u; i < (uint8_t)LIGHT_SIGNAL_NUM; i++)
    {
        if (s_level[i] != 0u) { mask |= (uint8_t)(1u << i); }
    }

    return mask;
}
