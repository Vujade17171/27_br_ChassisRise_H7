/*****************************************************************************
 * bsp_gpio.c —— 数字输入引脚读取实现（bsp 层）
 *
 * 只做两件事：HAL_GPIO_ReadPin 取电平、写进上层绑定的地址。
 * 引脚的时钟 / 方向 / 上下拉由 MX_GPIO_Init()（CubeMX）负责，这里一概不管。
 *
 * 全文件只有一处和"物理引脚"有关：g_gpio_map[]。
 * 要加一个输入脚：g_gpio_map[] 加一行 + bsp_gpio.h 加一个编号，别处不动。
 *
 * 数据流：引脚电平 --Refresh()--> 上层绑定的那个 1 字节变量
 *****************************************************************************/
#include "bsp_gpio.h"
#include "main.h"   /* GPIOx / GPIO_PIN_x / HAL_GPIO_ReadPin */

/* 引脚描述：本文件是"逻辑编号 -> 物理引脚"的唯一真源 */
typedef struct
{
    GPIO_TypeDef *port;     /* 端口，如 GPIOG */
    uint16_t      pin;      /* 引脚号，如 GPIO_PIN_7 */
} bsp_gpio_map_t;

static const bsp_gpio_map_t g_gpio_map[BSP_GPIO_PIN_NUM] =
{
    /* BSP_GPIO_PIN_1 -> PG7（方向 / 上下拉由 MX_GPIO_Init 配置） */
    { GPIOG, GPIO_PIN_7 },
    /* BSP_GPIO_PIN_2 -> PG4 */
    { GPIOG, GPIO_PIN_4 },
    /* BSP_GPIO_PIN_3 -> PD15 */
    { GPIOD, GPIO_PIN_15 },
    /* BSP_GPIO_PIN_4 -> PD12 */
    { GPIOD, GPIO_PIN_12 },
};

/* 每个引脚绑定的目标地址（由上层在 Bind 时注入，0 = 未绑定） */
static uint8_t *g_signal_addr[BSP_GPIO_PIN_NUM];

/* 采一次电平：0 = 低，1 = 高 */
static uint8_t read_level(bsp_gpio_pin_t pin)
{
    return (HAL_GPIO_ReadPin(g_gpio_map[pin].port, g_gpio_map[pin].pin)
            == GPIO_PIN_SET) ? 1u : 0u;
}

/* ---------------------------------------------------------------------------
 * 绑定地址
 * ------------------------------------------------------------------------- */
int32_t BSP_GPIO_Bind(bsp_gpio_pin_t pin, uint8_t *signal_addr)
{
    if (pin >= (bsp_gpio_pin_t)BSP_GPIO_PIN_NUM) { return SER_ERR_PARAM; }

    g_signal_addr[pin] = signal_addr;

    /* 绑定瞬间先采一次，避免上层在第一次 Refresh 之前读到 0（脏值） */
    if (signal_addr != 0)
    {
        *signal_addr = read_level(pin);
    }

    return SER_OK;
}

/* ---------------------------------------------------------------------------
 * 刷新
 * ------------------------------------------------------------------------- */
int32_t BSP_GPIO_Refresh(void)
{
    uint8_t i;

    for (i = 0u; i < (uint8_t)BSP_GPIO_PIN_NUM; i++)
    {
        uint8_t *dst = g_signal_addr[i];

        if (dst == 0) { continue; }   /* 未绑定：跳过 */

        *dst = read_level((bsp_gpio_pin_t)i);
    }

    return SER_OK;
}
