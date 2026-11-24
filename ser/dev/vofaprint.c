/*****************************************************************************
 * vofaprint.c —— VOFA+ 观测输出实现（ser/dev）
 *
 * 数据流：
 *   app --VOFA_Print_Floats/Csv/Text--> 组帧 --> 环形缓冲 s_buf[]
 *        --> TX DMA --> 串口 --> VOFA+
 *   发送完成：DMA TC 中断 -> HAL 使能该串口的 TC 中断 -> VOFA_PRINT_UART_IRQH
 *        -> HAL_UART_TxCpltCallback -> 推进 s_tail -> 继续发下一段
 *
 * 用哪个串口由 vofaprint.h 里的 VOFA_PRINT_UART / _INST / _IRQN / _IRQH 四个宏决定，
 * 本文件不出现任何具体串口名，换串口只改那四行。
 *
 * 为什么要自己开串口中断：
 *   CubeMX 只配了 TX 的 DMA，没勾串口本身的全局中断。而 HAL 的
 *   HAL_UART_Transmit_DMA() 在 NORMAL 模式下是"等最后一个字节真正移出移位寄存器"
 *   才算完成：DMA 传完后它会打开 TC 中断，靠 VOFA_PRINT_UART_IRQH 里调
 *   HAL_UART_IRQHandler() 来收尾并回调 HAL_UART_TxCpltCallback()。
 *   不打开这个中断，gState 会永远停在 BUSY_TX，第二次发送就再也发不出去。
 *   所以本文件在 Init 里使能 VOFA_PRINT_UART_IRQN，并提供 VOFA_PRINT_UART_IRQH。
 *
 * 本文件不做任何 printf 格式化（文本模式自己算 3 位小数），也不碰引脚/时钟。
 *****************************************************************************/
#include "vofaprint.h"

#include "main.h"     /* HAL / NVIC                          */
#include "usart.h"    /* VOFA_PRINT_UART 指向的 UART 句柄     */

#include <string.h>

/* JustFloat 帧尾：0x00 0x00 0x80 0x7F（VOFA+ 约定，小端 float 流） */
#define VOFA_TAIL_BYTE0     0x00u
#define VOFA_TAIL_BYTE1     0x00u
#define VOFA_TAIL_BYTE2     0x80u
#define VOFA_TAIL_BYTE3     0x7Fu

/* 环形缓冲：s_head 只由生产者（任务）写，s_tail 只由消费者（DMA 完成中断）写，
 * 单生产者单消费者，16 位下标对齐访问是原子的，所以不需要加锁。 */
static uint8_t s_buf[VOFA_PRINT_BUF_SIZE];
static volatile uint16_t s_head;
static volatile uint16_t s_tail;
static volatile uint16_t s_tx_len;      /* 当前这一小段 DMA 发了多少字节 */
static volatile uint8_t  s_tx_busy;     /* 1 = DMA 正在发 */
static volatile uint8_t  s_inited;
static volatile uint32_t s_sent;
static volatile uint32_t s_drop;
static volatile uint32_t s_error;

static void vofa_kick(void);
static int32_t vofa_push(const uint8_t *data, uint16_t len);
static uint16_t vofa_fmt_float(char *dst, float value);

/* ---------------------------------------------------------------------------
 * 初始化
 * ------------------------------------------------------------------------- */
int32_t VOFA_Print_Init(void)
{
    uint32_t primask;

    /* 先关中断再清状态，避免 DMA 完成中断此时改 s_tail */
    primask = __get_PRIMASK();
    __disable_irq();

    s_head    = 0u;
    s_tail    = 0u;
    s_tx_len  = 0u;
    s_tx_busy = 0u;
    s_sent    = 0u;
    s_drop    = 0u;
    s_error   = 0u;
    s_inited  = 1u;

    if (primask == 0u) { __enable_irq(); }

    /* HAL 的 DMA 发送收尾依赖该串口的 TC 中断（见文件头说明）。
     * 优先级 6：低于 FDCAN（5），高于 SysTick（15），不调用任何 OS API。 */
    HAL_NVIC_SetPriority(VOFA_PRINT_UART_IRQN, 6u, 0u);
    HAL_NVIC_EnableIRQ(VOFA_PRINT_UART_IRQN);

    return SER_OK;
}

/* ---------------------------------------------------------------------------
 * 对外接口
 * ------------------------------------------------------------------------- */
int32_t VOFA_Print_Floats(const float *values, uint8_t count)
{
    uint8_t  frame[VOFA_PRINT_MAX_CH * 4u + 4u];
    uint16_t n = 0u;
    uint8_t  i;

    if ((values == 0) || (count == 0u) || (count > (uint8_t)VOFA_PRINT_MAX_CH))
    {
        return SER_ERR_PARAM;
    }

    /* Cortex-M 是小端，直接按 4 字节拷贝就是 JustFloat 要的字节序 */
    for (i = 0u; i < count; i++)
    {
        memcpy(&frame[n], &values[i], 4u);
        n = (uint16_t)(n + 4u);
    }

    frame[n++] = VOFA_TAIL_BYTE0;
    frame[n++] = VOFA_TAIL_BYTE1;
    frame[n++] = VOFA_TAIL_BYTE2;
    frame[n++] = VOFA_TAIL_BYTE3;

    return vofa_push(frame, n);
}

int32_t VOFA_Print_Csv(const float *values, uint8_t count)
{
    char     line[VOFA_PRINT_MAX_CH * 12u + 2u];   /* 每通道最多 11 字符 + 分隔符 */
    uint16_t n = 0u;
    uint8_t  i;

    if ((values == 0) || (count == 0u) || (count > (uint8_t)VOFA_PRINT_MAX_CH))
    {
        return SER_ERR_PARAM;
    }

    for (i = 0u; i < count; i++)
    {
        n = (uint16_t)(n + vofa_fmt_float(&line[n], values[i]));
        line[n] = (i + 1u < count) ? ',' : '\n';
        n = (uint16_t)(n + 1u);
    }

    return vofa_push((const uint8_t *)line, n);
}

int32_t VOFA_Print_Text(const char *text)
{
    uint16_t len = 0u;

    if (text == 0) { return SER_ERR_PARAM; }

    while ((text[len] != '\0') && (len < (uint16_t)VOFA_PRINT_BUF_SIZE))
    {
        len++;
    }

    if (len == 0u) { return SER_OK; }

    /* 超过一个缓冲区的文本直接丢，避免发出半条导致 VOFA+ 错位 */
    if (len >= (uint16_t)VOFA_PRINT_BUF_SIZE)
    {
        s_drop++;
        return SER_ERR_PARAM;
    }

    return vofa_push((const uint8_t *)text, len);
}

uint32_t VOFA_Print_GetSentCount(void)  { return s_sent; }
uint32_t VOFA_Print_GetDropCount(void)  { return s_drop; }
uint32_t VOFA_Print_GetErrorCount(void) { return s_error; }

/* ---------------------------------------------------------------------------
 * 环形缓冲：入队（整条进或整条丢）
 * ------------------------------------------------------------------------- */
static int32_t vofa_push(const uint8_t *data, uint16_t len)
{
    uint16_t used;
    uint16_t free_len;
    uint16_t first;

    if ((data == 0) || (len == 0u)) { return SER_ERR_PARAM; }
    if (len >= (uint16_t)VOFA_PRINT_BUF_SIZE) { s_drop++; return SER_ERR_PARAM; }

    /* 第一次发送自动初始化，省得上层忘了调 Init */
    if (s_inited == 0u) { (void)VOFA_Print_Init(); }

    used     = (uint16_t)((s_head + VOFA_PRINT_BUF_SIZE - s_tail) % VOFA_PRINT_BUF_SIZE);
    free_len = (uint16_t)(VOFA_PRINT_BUF_SIZE - 1u - used);

    if (free_len < len)
    {
        s_drop++;               /* 放不下：整条丢，绝不写半条 */
        return SER_ERR_BUS;
    }

    first = (uint16_t)(VOFA_PRINT_BUF_SIZE - s_head);
    if (first > len) { first = len; }

    memcpy(&s_buf[s_head], data, first);
    if (first < len)
    {
        memcpy(&s_buf[0], &data[first], (uint16_t)(len - first));
    }

    s_head = (uint16_t)((s_head + len) % VOFA_PRINT_BUF_SIZE);
    s_sent++;

    vofa_kick();
    return SER_OK;
}

/* ---------------------------------------------------------------------------
 * 调度：DMA 空闲时，从 s_tail 起送一段连续数据（最多到环尾）
 * ------------------------------------------------------------------------- */
static void vofa_kick(void)
{
    uint16_t chunk;
    uint16_t tail;
    uint32_t primask;

    primask = __get_PRIMASK();
    __disable_irq();

    if ((s_tx_busy != 0u) || (s_head == s_tail))
    {
        if (primask == 0u) { __enable_irq(); }
        return;
    }

    tail  = s_tail;
    chunk = (s_head > tail) ? (uint16_t)(s_head - tail)
                            : (uint16_t)(VOFA_PRINT_BUF_SIZE - tail);

    s_tx_busy = 1u;             /* 先占位，防止任务与中断同时启动 DMA */
    s_tx_len  = chunk;

    if (primask == 0u) { __enable_irq(); }

    if (HAL_UART_Transmit_DMA(&VOFA_PRINT_UART, &s_buf[tail], chunk) != HAL_OK)
    {
        /* 串口状态异常（上一次错误没清干净等）：强制复位一次，等下次调用再试 */
        (void)HAL_UART_AbortTransmit(&VOFA_PRINT_UART);

        primask = __get_PRIMASK();
        __disable_irq();
        s_tx_busy = 0u;
        s_tx_len  = 0u;
        if (primask == 0u) { __enable_irq(); }

        s_error++;
    }
}

/* ---------------------------------------------------------------------------
 * 发送完成：推进 s_tail，接着发下一段
 * ------------------------------------------------------------------------- */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance != VOFA_PRINT_UART_INST) { return; }

    s_tail   = (uint16_t)((s_tail + s_tx_len) % VOFA_PRINT_BUF_SIZE);
    s_tx_len = 0u;
    s_tx_busy = 0u;

    vofa_kick();
}

#if (VOFA_PRINT_UART_IRQH_OWNED != 0)
void VOFA_PRINT_UART_IRQH(void)
{
    HAL_UART_IRQHandler(&VOFA_PRINT_UART);
}
#endif

/* ---------------------------------------------------------------------------
 * 浮点转文本：固定 3 位小数，不依赖 printf
 *   例： 1.234 -> "1.234"    -0.05 -> "-0.050"    12 -> "12.000"
 * 返回写入的字符数（最长 11：符号 1 + 整数 6 + 小数点 1 + 3 位小数）。
 * ------------------------------------------------------------------------- */
static uint16_t vofa_fmt_float(char *dst, float value)
{
    char    tmp[8];
    int32_t scaled;
    int32_t ip;
    int32_t fp;
    uint8_t k = 0u;
    uint16_t n = 0u;

    /* 限幅，避免整数部分太长把行缓冲写穿（±999999.999 已经远大于实际变量） */
    if (value > 999999.0f)  { value = 999999.0f; }
    if (value < -999999.0f) { value = -999999.0f; }

    /* 四舍五入到 0.001 */
    scaled = (int32_t)((value >= 0.0f) ? (value * 1000.0f + 0.5f)
                                       : (value * 1000.0f - 0.5f));

    if (scaled < 0)
    {
        dst[n++] = '-';
        scaled = -scaled;
    }

    ip = scaled / 1000;
    fp = scaled % 1000;

    if (ip == 0)
    {
        tmp[k++] = '0';
    }
    else
    {
        while (ip > 0)
        {
            tmp[k++] = (char)('0' + (ip % 10));
            ip /= 10;
        }
    }
    while (k > 0u) { dst[n++] = tmp[--k]; }

    dst[n++] = '.';
    dst[n++] = (char)('0' + (fp / 100));
    dst[n++] = (char)('0' + ((fp / 10) % 10));
    dst[n++] = (char)('0' + (fp % 10));

    return n;
}
