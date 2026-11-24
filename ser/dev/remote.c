/*****************************************************************************
 * remote.c —— 遥控器接收服务实现（ser/dev）
 *
 * 数据流：
 *   USART1(PB15) --DMA1_Stream0--> s_dma_buf[] --IDLE/半满/满--> RxEventCallback
 *      --逐字节状态机--> 12 字节完整帧 --> 校验(帧头/版本/CRC/范围) --> g_remote_control
 *
 * 与 F407 原版的区别只有"取字节"这一层：
 *   原版是 HAL_UART_Receive_IT() 每字节一次中断；
 *   本工程底层是 DMA，所以改成 HAL_UARTEx_ReceiveToIdle_DMA()，
 *   每次由 HAL 告诉我们"这一段有几个字节"，再喂给同一套状态机。
 *   帧格式、CRC、校验规则、对外 API 都与原版一致。
 *
 * 本文件不 include 任何 app 头，也不认识 CAN / 电机。
 *****************************************************************************/
#include "remote.h"

#include <string.h>

#define FRAME_HEADER_1 0xAAU
#define FRAME_HEADER_2 0x55U

/* 本工程的串口句柄（CubeMX 生成在 Core/Src/usart.c） */
extern UART_HandleTypeDef huart1;

volatile RemoteControlData_t g_remote_control = {0};

static UART_HandleTypeDef *s_uart = NULL;                        /* 绑定的串口       */
static uint8_t  s_dma_buf[REMOTE_CONTROL_DMA_BUF_SIZE] = {0};    /* DMA 落地区       */
static uint8_t  s_frame[REMOTE_CONTROL_FRAME_SIZE] = {0};        /* 正在拼的帧       */
static uint8_t  s_frame_index = 0U;                              /* 当前拼到第几字节 */
static volatile uint8_t s_started = 0U;                          /* 0 = 未启动       */
static volatile uint32_t s_uart_error_count = 0U;

static uint8_t  RemoteControl_Crc8(const uint8_t *data, uint8_t length);
static bool     RemoteControl_DecodeFrame(const uint8_t *frame);
static void     RemoteControl_ProcessByte(uint8_t byte);
static void     RemoteControl_ProcessBuffer(const uint8_t *buf, uint16_t length);
static HAL_StatusTypeDef RemoteControl_ArmReceive(void);

/* ---------------------------------------------------------------------------
 * 启动 / 停止
 * ------------------------------------------------------------------------- */
HAL_StatusTypeDef RemoteControl_Start(UART_HandleTypeDef *huart)
{
    HAL_StatusTypeDef status;

    if (huart == NULL) { return HAL_ERROR; }

    /* 重复调用时先把手上的接收彻底停掉，避免两路 DMA 抢同一个句柄 */
    if (s_started != 0U) { (void)HAL_UART_AbortReceive(huart); }

    s_uart        = huart;
    s_frame_index = 0U;
    memset(s_frame, 0, sizeof(s_frame));
    memset((void *)&g_remote_control, 0, sizeof(g_remote_control));

    s_started = 1U;
    status = RemoteControl_ArmReceive();

    if (status != HAL_OK)
    {
        s_started = 0U;
        s_uart    = NULL;
    }

    return status;
}

HAL_StatusTypeDef RemoteControl_Init(void)
{
    return RemoteControl_Start(&huart1);
}

void RemoteControl_Stop(void)
{
    UART_HandleTypeDef *huart = s_uart;

    s_started     = 0U;
    s_uart        = NULL;
    s_frame_index = 0U;

    if (huart != NULL) { (void)HAL_UART_AbortReceive(huart); }
}

/* ---------------------------------------------------------------------------
 * 取快照 / 在线判断
 * ------------------------------------------------------------------------- */
bool RemoteControl_GetSnapshot(RemoteControlData_t *data)
{
    uint32_t primask;

    if (data == NULL) { return false; }

    primask = __get_PRIMASK();
    __disable_irq();
    memcpy(data, (const void *)&g_remote_control, sizeof(*data));
    if (primask == 0U) { __enable_irq(); }

    return (data->valid_frame_count > 0U);
}

bool RemoteControl_IsOnline(uint32_t timeout_ms)
{
    RemoteControlData_t snapshot;

    if (!RemoteControl_GetSnapshot(&snapshot)) { return false; }

    return ((uint32_t)(HAL_GetTick() - snapshot.last_update_ms) <= timeout_ms);
}

uint32_t RemoteControl_GetValidFrameCount(void)
{
    return g_remote_control.valid_frame_count;
}

uint32_t RemoteControl_GetCrcErrorCount(void)
{
    return g_remote_control.crc_error_count;
}

uint32_t RemoteControl_GetUartErrorCount(void)
{
    return s_uart_error_count;
}

/* ---------------------------------------------------------------------------
 * 挂上 DMA 接收（IDLE 或缓冲满都会回调）
 * ------------------------------------------------------------------------- */
static HAL_StatusTypeDef RemoteControl_ArmReceive(void)
{
    if ((s_uart == NULL) || (s_started == 0U)) { return HAL_ERROR; }

    return HAL_UARTEx_ReceiveToIdle_DMA(s_uart, s_dma_buf,
                                        (uint16_t)REMOTE_CONTROL_DMA_BUF_SIZE);
}

/* ---------------------------------------------------------------------------
 * HAL 回调（中断上下文）
 *
 * RxEventCallback 的 Size 是"本次这段缓冲里实际收到的字节数"：
 *   - IDLE 触发：一帧（或一串帧）结束；
 *   - 缓冲收满：这是被 32 字节切开的一段，状态机跨段继续拼帧。
 * ------------------------------------------------------------------------- */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if ((huart == NULL) || (huart != s_uart) || (s_started == 0U)) { return; }

    if (Size > (uint16_t)REMOTE_CONTROL_DMA_BUF_SIZE)
    {
        Size = (uint16_t)REMOTE_CONTROL_DMA_BUF_SIZE;
    }

    RemoteControl_ProcessBuffer(s_dma_buf, Size);

    (void)RemoteControl_ArmReceive();   /* 立刻重新挂上，减少丢字节窗口 */
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if ((huart == NULL) || (huart != s_uart)) { return; }

    s_uart_error_count++;
    g_remote_control.uart_error_count = s_uart_error_count;

    /* 半帧作废，重新找帧头；清溢出标志后再挂接收 */
    s_frame_index = 0U;
    __HAL_UART_CLEAR_OREFLAG(huart);
    (void)RemoteControl_ArmReceive();
}

/* ---------------------------------------------------------------------------
 * 字节状态机：找 AA 55 -> 收满 12 字节 -> 校验
 * ------------------------------------------------------------------------- */
static void RemoteControl_ProcessBuffer(const uint8_t *buf, uint16_t length)
{
    uint16_t i;

    if (buf == NULL) { return; }

    for (i = 0U; i < length; i++)
    {
        RemoteControl_ProcessByte(buf[i]);
    }
}

static void RemoteControl_ProcessByte(uint8_t byte)
{
    switch (s_frame_index)
    {
        case 0U:
            if (byte == FRAME_HEADER_1)
            {
                s_frame[0]    = byte;
                s_frame_index = 1U;
            }
            break;

        case 1U:
            if (byte == FRAME_HEADER_2)
            {
                s_frame[1]    = byte;
                s_frame_index = 2U;
            }
            else if (byte == FRAME_HEADER_1)
            {
                s_frame[0] = byte;     /* AA AA 55：重新算帧头 */
            }
            else
            {
                s_frame_index = 0U;
            }
            break;

        default:
            s_frame[s_frame_index] = byte;
            s_frame_index++;
            if (s_frame_index >= (uint8_t)REMOTE_CONTROL_FRAME_SIZE)
            {
                (void)RemoteControl_DecodeFrame(s_frame);
                s_frame_index = 0U;
            }
            break;
    }
}

/* ---------------------------------------------------------------------------
 * 帧校验 + 解析
 * ------------------------------------------------------------------------- */
static bool RemoteControl_DecodeFrame(const uint8_t *frame)
{
    uint8_t calculated_crc;
    bool    power_on;
    int8_t  left_x;
    int8_t  left_y;
    int8_t  right_x;
    int8_t  right_y;

    if ((frame[0] != FRAME_HEADER_1) ||
        (frame[1] != FRAME_HEADER_2) ||
        (frame[2] != REMOTE_CONTROL_PROTOCOL_VERSION))
    {
        return false;
    }

    calculated_crc = RemoteControl_Crc8(&frame[2], 9U);
    if (calculated_crc != frame[11])
    {
        g_remote_control.crc_error_count++;
        return false;
    }

    if (frame[4] > 1U) { return false; }

    power_on = (frame[4] == 1U);
    left_x   = (int8_t)frame[5];
    left_y   = (int8_t)frame[6];
    right_x  = (int8_t)frame[7];
    right_y  = (int8_t)frame[8];

    if ((left_x  < -100) || (left_x  > 100) ||
        (left_y  < -100) || (left_y  > 100) ||
        (right_x < -100) || (right_x > 100) ||
        (right_y < -100) || (right_y > 100))
    {
        return false;
    }

    if (power_on)
    {
        if ((frame[9] < 1U) || (frame[9] > 2U) ||
            (frame[10] < 1U) || (frame[10] > 2U))
        {
            return false;
        }
    }
    else if ((left_x != 0) || (left_y != 0) ||
             (right_x != 0) || (right_y != 0) ||
             (frame[9] != 0U) || (frame[10] != 0U))
    {
        return false;
    }

    g_remote_control.sequence      = frame[3];
    g_remote_control.power_on      = power_on;
    g_remote_control.left_x        = left_x;
    g_remote_control.left_y        = left_y;
    g_remote_control.right_x       = right_x;
    g_remote_control.right_y       = right_y;
    g_remote_control.left_gear     = frame[9];
    g_remote_control.right_gear    = frame[10];
    g_remote_control.last_update_ms = HAL_GetTick();
    g_remote_control.valid_frame_count++;

    return true;
}

/* CRC8：多项式 0x07，初值 0，不反转 */
static uint8_t RemoteControl_Crc8(const uint8_t *data, uint8_t length)
{
    uint8_t crc = 0U;
    uint8_t index;
    uint8_t bit;

    for (index = 0U; index < length; index++)
    {
        crc ^= data[index];
        for (bit = 0U; bit < 8U; bit++)
        {
            if ((crc & 0x80U) != 0U)
            {
                crc = (uint8_t)((crc << 1U) ^ 0x07U);
            }
            else
            {
                crc <<= 1U;
            }
        }
    }

    return crc;
}
