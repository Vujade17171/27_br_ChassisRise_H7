/*****************************************************************************
 * vofaprint.h —— VOFA+ 上位机观测输出（ser/dev）
 *
 * 作用：把单片机里的变量推到 VOFA+ 上位机，用波形 / 控件观察。
 * 底层：默认 USART10（PE2/PE3）+ DMA1_Stream1 TX，115200 8N1（CubeMX 已配好）。
 *       用哪个串口由本文件下面的 VOFA_PRINT_UART* 四个宏决定，换串口只改这四行，
 *       vofaprint.c 里不出现任何具体串口名。
 *       本模块只"写数据"，不碰引脚 / 时钟 / 波特率。
 *
 * 上层只用这三个接口（看不到任何 HAL 类型）：
 *     VOFA_Print_Floats(v, n);            // JustFloat 二进制（推荐：省带宽、波形顺）
 *     VOFA_Print_Csv(v, n);               // FireWater 文本 "v0,v1,...,vn\n"（可读、好接控件）
 *     VOFA_Print_Text("hello\n");         // 原样文本（自己拼任意格式）
 *
 * 特点：
 *   1) 非阻塞：数据先进 512 字节环形缓冲，DMA 在后台慢慢发；缓冲满时整条丢弃并计数，
 *      【不会发出半条帧】——半条会让 VOFA+ 解析错位，比丢一整条更糟。
 *   2) 不依赖 printf：文本模式的浮点格式化自己算（固定 3 位小数），
 *      所以不需要 MicroLIB，也不怕 %f 没开。
 *   3) 单生产者：约定只从同一个任务里调（一般是控制任务），不要多任务并发调用。
 *
 * 用法示例（任务里）：
 *     (void)VOFA_Print_Init();                       // 上电调用一次（也可以不调，首次发送会自动初始化）
 *     for (;;) {
 *         float v[3] = { speed, target, (float)err };
 *         VOFA_Print_Floats(v, 3);                   // VOFA+ 选 JustFloat 协议
 *         // 或者 VOFA_Print_Csv(v, 3);              // VOFA+ 选 FireWater 协议
 *         OSAL_Time_DelayMs(10);
 *     }
 *
 * 注意：vofaprint.c 里实现了两个全局符号（供 HAL 的 DMA 发送完成链路使用）：
 *     VOFA_PRINT_UART_IRQH      —— 默认 USART10_IRQHandler，转给 HAL_UART_IRQHandler()
 *     HAL_UART_TxCpltCallback   —— DMA 发完一段后推进环形缓冲
 *   如果以后在 CubeMX 里勾选了该串口的中断，it.c 也会生成同名中断函数，
 *   那时会和这里的强定义重名，删掉其中一处即可。
 *****************************************************************************/
#ifndef __VOFA_PRINT_H
#define __VOFA_PRINT_H

#include <stdint.h>
#include "return_typedef.h"

/* ---------------------------------------------------------------------------
 * 串口选择：换串口【只改这四条】，vofaprint.c / 上层代码都不用动。
 *
 *   VOFA_PRINT_UART        CubeMX 生成的 UART 句柄（usart.c 里的全局变量）
 *   VOFA_PRINT_UART_INST   外设基地址，TxCplt 回调里用它判断"是不是我这一路"
 *   VOFA_PRINT_UART_IRQN   该串口的中断号，Init 里要使能（HAL 的 DMA 发送收尾靠它）
 *   VOFA_PRINT_UART_IRQH   中断服务函数名，必须和 startup 文件里该串口的向量名一致
 *
 * 换串口时要确认三件事（都在 CubeMX 侧，本文件管不到）：
 *   1) 新串口已 MX_xxx_UART_Init()，且 __HAL_LINKDMA(uart, hdmatx, ...) 配了 TX DMA；
 *   2) DMA 的 IRQ 在 NVIC 里使能（it.c 里有对应的 DMAx_StreamY_IRQHandler）；
 *   3) 该串口的全局中断没有被别处占用（本模块会自己使能它）。
 *
 * 例：换成 USART1：
 *   #define VOFA_PRINT_UART        huart1
 *   #define VOFA_PRINT_UART_INST   USART1
 *   #define VOFA_PRINT_UART_IRQN   USART1_IRQn
 *   #define VOFA_PRINT_UART_IRQH   USART1_IRQHandler
 *   #define VOFA_PRINT_UART_IRQH_OWNED  0   // 见下：it.c 里已经有这个函数了
 * ------------------------------------------------------------------------- */
#ifndef VOFA_PRINT_UART
#define VOFA_PRINT_UART         huart10
#endif
#ifndef VOFA_PRINT_UART_INST
#define VOFA_PRINT_UART_INST    USART10
#endif
#ifndef VOFA_PRINT_UART_IRQN
#define VOFA_PRINT_UART_IRQN    USART10_IRQn
#endif
#ifndef VOFA_PRINT_UART_IRQH
#define VOFA_PRINT_UART_IRQH    USART10_IRQHandler
#endif

/* 1 = 由 vofaprint.c 定义这个串口的中断函数（默认，适用于 CubeMX 没勾该串口 NVIC 的情况）
 * 0 = 该串口的中断函数已经存在于 it.c，这里就不要再定义，否则重复符号；
 *     前提是 it.c 里那个函数体就是 HAL_UART_IRQHandler(&该串口句柄)。
 *     换到 USART1（it.c 已有 USART1_IRQHandler）时选 0。 */
#ifndef VOFA_PRINT_UART_IRQH_OWNED
#define VOFA_PRINT_UART_IRQH_OWNED  1
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* 环形缓冲大小（字节）。一帧 4 通道 JustFloat 是 20 字节，512 字节约 25 帧。 */
#ifndef VOFA_PRINT_BUF_SIZE
#define VOFA_PRINT_BUF_SIZE     512u
#endif

/* 单次最多送多少个通道（JustFloat 一帧 = 4*n+4 字节；CSV 一行约 12*n 字符） */
#ifndef VOFA_PRINT_MAX_CH
#define VOFA_PRINT_MAX_CH       17u
#endif

/* 初始化：清空缓冲 / 计数，并使能 VOFA_PRINT_UART_IRQN 中断
 * （DMA 发完后 HAL 要靠该串口的 TC 中断收尾）。
 * 可重复调用；也可以完全不管它 —— 第一次发送时会自动初始化。
 * 返回 SER_OK。 */
int32_t VOFA_Print_Init(void);

/* JustFloat 协议：n 个 float（小端，4 字节）+ 帧尾 00 00 80 7F。
 * VOFA+ 里选 "JustFloat"，不需要再设帧尾。
 * count：1 ~ VOFA_PRINT_MAX_CH。
 * 返回 SER_OK / SER_ERR_PARAM（空指针、通道数越界）/ SER_ERR_BUS（缓冲满，本帧被丢弃）。 */
int32_t VOFA_Print_Floats(const float *values, uint8_t count);

/* FireWater 协议：把 n 个 float 格式化成 "v0,v1,...,vn\n"（固定 3 位小数）。
 * VOFA+ 里选 "FireWater"，分隔符填 ","，帧尾 "\n"。
 * 返回 SER_OK / SER_ERR_PARAM / SER_ERR_BUS（同上）。 */
int32_t VOFA_Print_Csv(const float *values, uint8_t count);

/* 原样发一段文本（不自动补换行）。长度必须 < VOFA_PRINT_BUF_SIZE。
 * 返回 SER_OK / SER_ERR_PARAM / SER_ERR_BUS。 */
int32_t VOFA_Print_Text(const char *text);

/* 观测用计数：成功入队的条数 / 因缓冲满丢弃的条数 / 串口发送出错的次数。
 * 丢包只看 GetDropCount 是否在涨；涨了就说明发得比 115200 能承载的快。 */
uint32_t VOFA_Print_GetSentCount(void);
uint32_t VOFA_Print_GetDropCount(void);
uint32_t VOFA_Print_GetErrorCount(void);

#ifdef __cplusplus
}
#endif

#endif /* __VOFA_PRINT_H */
