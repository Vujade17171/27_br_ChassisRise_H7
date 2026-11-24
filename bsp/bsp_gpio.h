/*****************************************************************************
 * bsp_gpio.h —— 数字输入引脚读取（bsp 层）
 *
 * 分工：引脚的时钟 / 方向 / 上下拉由 CubeMX 的 MX_GPIO_Init() 负责
 *   （PD12~PD15 已在 Core/Src/gpio.c 里配成 GPIO_Input / NOPULL），
 *   本模块【只读不配】：不碰 RCC、不改引脚模式、不开中断。
 *
 * 分层位置：bsp —— 只做"取电平"这一件事，不含业务语义，也不认识上层对象。
 *   允许依赖：HAL / CMSIS / common 公共头
 *   禁止依赖：FreeRTOS、ser 与 app 目录
 *
 * 解耦约定（地址注入，与 bsp_can 的注册回调同一套思路）：
 *   1) 上层只做两件事：
 *        a. 初始化时 BSP_GPIO_Bind(pin, &我的变量)   —— 把地址交给 bsp；
 *        b. 之后直接用那个变量读 0/1              —— 不再调任何读接口。
 *      上层自始至终看不到 GPIO_TypeDef / GPIO_PinState。
 *   2) "编号 -> 端口 + 引脚" 只写在 bsp_gpio.c 的引脚表里，
 *      换引脚（甚至换板子）只改那一行，app / ser 一行都不用动。
 *   3) 本头文件不 include 任何 HAL 头，杜绝 HAL 类型渗透到上层。
 *
 * 典型用法（app 层）：
 *      static uint8_t g_pg7;                       // 0/1 信号落地用的变量
 *      BSP_GPIO_Bind(BSP_GPIO_PIN_1, &g_pg7);      // 绑定地址（绑定瞬间就会采一次）
 *      for (;;) {
 *          BSP_GPIO_Refresh();                     // 每周期刷一次
 *          if (g_pg7 != 0u) { ... }                // 只用地址里的 0/1
 *          osDelay(1);
 *      }
 *
 * 当前引脚表（详见 bsp_gpio.c 的 g_gpio_map[]）：
 *   BSP_GPIO_PIN_1  ->  PG7      BSP_GPIO_PIN_2  ->  PG8
 *   BSP_GPIO_PIN_3  ->  PG9      BSP_GPIO_PIN_4  ->  PG10
 *   四个脚都必须先在 CubeMX 里配成 GPIO_Input / NOPULL。
 *****************************************************************************/
#ifndef __BSP_GPIO_H
#define __BSP_GPIO_H

#include <stdint.h>
#include "return_typedef.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 逻辑编号：只在【绑定】时用一次，之后上层只认自己那个变量的地址。
 * 编号即引脚表下标，越界一律返回 SER_ERR_PARAM。 */
typedef uint8_t bsp_gpio_pin_t;

#define BSP_GPIO_PIN_1      0u
#define BSP_GPIO_PIN_2      1u
#define BSP_GPIO_PIN_3      2u
#define BSP_GPIO_PIN_4      3u
#define BSP_GPIO_PIN_NUM    4u

/* 绑定地址：把 pin 的 0/1 电平绑定到 signal_addr 指向的 1 字节变量。
 *   - signal_addr 由【上层】提供，生命周期必须长于绑定（一般为 static / 全局）；
 *   - 绑定成功立即采样一次，保证第一次 Refresh 之前读到的就是真实电平；
 *   - signal_addr 传 0 表示解绑，该引脚不再被刷新；
 *   - 同一引脚重复绑定会覆盖旧地址。
 * 返回 SER_OK / SER_ERR_PARAM（pin 越界）。 */
int32_t BSP_GPIO_Bind(bsp_gpio_pin_t pin, uint8_t *signal_addr);

/* 刷新所有已绑定引脚：读一次 IDR，把 0（低）/1（高）写进各自绑定的地址。
 * 只写 1 个字节、不加锁、不回调、不调用 OS API，任务或中断里都能调。
 * 未绑定的引脚直接跳过。返回 SER_OK。
 * 注意：这是唯一的采样时机，上层没调 Refresh 时地址里的值保持不变。 */
int32_t BSP_GPIO_Refresh(void);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_GPIO_H */
