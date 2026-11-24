/*****************************************************************************
 * light_signal.h —— 光电信号服务（ser/dev）
 *
 * 分层位置：服务层 —— 坐在 bsp_gpio 之上、app 之下。
 *   允许依赖：bsp_gpio（0/1 信号）、common 公共头
 *   禁止依赖：HAL 头、app 全局变量、CAN 相关模块
 *
 * 职责只有两件：
 *   1) 每个周期采一次 4 路引脚的原始电平（内部调 BSP_GPIO_Refresh）；
 *   2) 消抖：每路连续 N 拍一致才认账，对外只暴露稳定的 0/1。
 * 极性判断、边沿检测、业务含义都不在这里 —— 那是 app 的事。
 *
 * 用法（app 层）：
 *      Light_Signal_Init();                             // 初始化时一次，内部完成 4 路绑定
 *      for (;;) {
 *          Light_Signal_Update();                       // 每个周期一次（1 kHz → 消抖 5 ms）
 *          if (Light_Signal_Get(LIGHT_SIGNAL_CH1) != 0u) { ... }
 *          uint8_t all = Light_Signal_GetAll();         // bit0~bit3 一次取 4 路
 *          osDelay(1);
 *      }
 *****************************************************************************/
#ifndef __LIGHT_SIGNAL_H
#define __LIGHT_SIGNAL_H

#include <stdint.h>
#include "return_typedef.h"
#include "bsp_gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 本服务占用哪 4 个逻辑引脚（换引脚只改这一处，物理端口在 bsp_gpio.c 的引脚表里） */
#define LIGHT_SIGNAL_PIN1            BSP_GPIO_PIN_1     /* -> PG7  */
#define LIGHT_SIGNAL_PIN2            BSP_GPIO_PIN_2     /* -> PG8  */
#define LIGHT_SIGNAL_PIN3            BSP_GPIO_PIN_3     /* -> PG9  */
#define LIGHT_SIGNAL_PIN4            BSP_GPIO_PIN_4     /* -> PG10 */

/* 对外用的通道号：数值与上面的逻辑引脚编号一致（0~3），
 * 这样 Get(LIGHT_SIGNAL_CHx) 与 Init 里绑定的 PINx 一定是同一路。 */
#define LIGHT_SIGNAL_CH1             0u
#define LIGHT_SIGNAL_CH2             1u
#define LIGHT_SIGNAL_CH3             2u
#define LIGHT_SIGNAL_CH4             3u

#define LIGHT_SIGNAL_NUM             4u

/* 消抖拍数：连续多少拍不一致才算真的翻转。
 * 本模块按 1 kHz 调用时，5 拍 ≈ 5 ms，足够滤掉光电管的毛刺。 */
#ifndef LIGHT_SIGNAL_DEBOUNCE_N
#define LIGHT_SIGNAL_DEBOUNCE_N     5u
#endif

/* 初始化：把 4 路 bsp 引脚电平绑定到本模块内部，清零消抖状态，
 * 并以第一拍的实际电平作为初值（避免上电瞬间产生一次假跳变）。
 * 返回 SER_OK / SER_ERR_PARAM（引脚编号越界）。 */
int32_t Light_Signal_Init(void);

/* 采样 + 消抖，每个控制周期调用一次（与调用周期强相关，见 LIGHT_SIGNAL_DEBOUNCE_N）。
 * 内部已经调过 BSP_GPIO_Refresh()，上层不需要再单独刷 bsp。
 * 未初始化时直接返回。 */
void Light_Signal_Update(void);

/* 指定通道消抖后的稳定电平：0 或 1（原样，不做极性翻转）。
 * ch 越界返回 0。ch 取 LIGHT_SIGNAL_CH1 ~ LIGHT_SIGNAL_CH4。 */
uint8_t Light_Signal_Get(uint8_t ch);

/* 指定通道上一拍的原始电平（未经消抖），只在调 DEBOUNCE_N 或排查毛刺时用。
 * ch 越界返回 0。 */
uint8_t Light_Signal_GetRaw(uint8_t ch);

/* 4 路稳定电平打包：bit0 = CH1 … bit3 = CH4，1 表示该路为高电平。 */
uint8_t Light_Signal_GetAll(void);

#ifdef __cplusplus
}
#endif

#endif /* __LIGHT_SIGNAL_H */
