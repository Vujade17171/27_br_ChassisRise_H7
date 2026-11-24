/*****************************************************************************
 * remote.h —— 遥控器接收服务（ser/dev）
 *
 * 帧格式（定长 12 字节，AA 55 帧头，大端无关、逐字节）：
 *   [0]  0xAA
 *   [1]  0x55
 *   [2]  协议版本，固定 0x01
 *   [3]  序号，每帧 +1（仅用于观测丢帧，不参与校验）
 *   [4]  上电标志：0 = 未上电，1 = 已上电
 *   [5]  左摇杆 X，int8，-100 ~ +100
 *   [6]  左摇杆 Y，int8，-100 ~ +100
 *   [7]  右摇杆 X，int8，-100 ~ +100
 *   [8]  右摇杆 Y，int8，-100 ~ +100
 *   [9]  左挡位，1 ~ 2（未上电时必须为 0）
 *   [10] 右挡位，1 ~ 2（未上电时必须为 0）
 *   [11] CRC8：覆盖 [2]~[10] 共 9 字节，多项式 0x07，初值 0
 *
 * 接收方式：
 *   本工程底层是 USART1 + DMA1_Stream0（DMA_NORMAL）+ IDLE 中断，
 *   所以这里用 HAL_UARTEx_ReceiveToIdle_DMA()：
 *     每次 IDLE（一帧或一串帧结束）或 DMA 缓冲满，HAL 回调
 *     HAL_UARTEx_RxEventCallback(huart, Size) 告诉我们这一段有多少字节，
 *     交给字节状态机逐字节跑。帧头 / 版本 / CRC / 数值范围任何一步不对
 *     都丢弃并重新同步，不需要上层关心分包边界。
 *   换波特率 / 换串口只影响 CubeMX 那边，本文件不用改。
 *
 * 分层位置：服务层（ser/dev）
 *   允许依赖：HAL（UART 句柄）、common 公共头
 *   禁止依赖：app 全局变量、can / 电机相关模块
 *
 * 用法（app / 任务里）：
 *      (void)RemoteControl_Init();          // 上电后调用一次，内部用本工程的 huart1
 *      for (;;) {
 *          if (RemoteControl_IsOnline(REMOTE_CONTROL_TIMEOUT_MS)) {
 *              RemoteControlData_t rc;
 *              if (RemoteControl_GetSnapshot(&rc)) { ... }
 *          }
 *          osDelay(1);
 *      }
 *****************************************************************************/
#ifndef __REMOTE_H
#define __REMOTE_H

#include <stdbool.h>
#include <stdint.h>
#include "main.h"   /* UART_HandleTypeDef / HAL 状态码 / HAL_GetTick */

#ifdef __cplusplus
extern "C" {
#endif

#define REMOTE_CONTROL_FRAME_SIZE        12U     /* 一帧固定 12 字节                 */
#define REMOTE_CONTROL_PROTOCOL_VERSION  0x01U   /* 只认这个版本                     */
#define REMOTE_CONTROL_TIMEOUT_MS        300U    /* 判失联的默认超时                 */
#define REMOTE_CONTROL_DMA_BUF_SIZE      32U     /* DMA 一段最多收多少字节（≥ 2 帧） */

/* 解析结果（每次收到合法帧刷新一次，由中断写入） */
typedef struct
{
    uint8_t  sequence;            /* 帧序号                          */
    bool     power_on;            /* 遥控器是否上电                  */
    int16_t   left_x;              /* 左摇杆 X，-100~100              */
    int16_t   left_y;              /* 左摇杆 Y，-100~100              */
    int16_t   right_x;             /* 右摇杆 X，-100~100              */
    int16_t   right_y;             /* 右摇杆 Y，-100~100              */
    uint8_t  left_gear;           /* 左挡位，1~2（未上电为 0）        */
    uint8_t  right_gear;          /* 右挡位，1~2（未上电为 0）        */
    uint32_t last_update_ms;      /* 最后一帧合法帧的时刻（HAL_GetTick） */
    uint32_t valid_frame_count;   /* 累计合法帧数                    */
    uint32_t crc_error_count;     /* 累计 CRC 错误帧数               */
    uint32_t uart_error_count;    /* 累计 UART 错误（ORE/FE/NE/PE）次数 */
} RemoteControlData_t;

extern volatile RemoteControlData_t g_remote_control;

/* 启动：清空状态并挂上 DMA 接收。同一次上电只调一次。
 * huart 必须是已 MX_xxx_UART_Init() 过、且已经 __HAL_LINKDMA 过 RX DMA 的句柄。
 * 返回 HAL_OK / HAL_ERROR / HAL_BUSY。 */
HAL_StatusTypeDef RemoteControl_Start(UART_HandleTypeDef *huart);

/* 本工程专用简写：内部直接 extern huart1（USART1 / PB15），
 * 省得 app 再去 include usart.h。等价于 RemoteControl_Start(&huart1)。 */
HAL_StatusTypeDef RemoteControl_Init(void);

/* 停止接收（调试/重配串口时用）。返回后不会再解析任何字节。 */
void RemoteControl_Stop(void);

/* 取一份快照（关中断拷贝，避免读到半帧数据）。
 * 返回 false 表示还没收到过任何合法帧（此时 data 已被清零）。 */
bool RemoteControl_GetSnapshot(RemoteControlData_t *data);

/* 是否在线：最近一帧合法帧距今不超过 timeout_ms 且至少收到过一帧。 */
bool RemoteControl_IsOnline(uint32_t timeout_ms);

/* 观测用：累计合法帧 / CRC 错误 / UART 错误 */
uint32_t RemoteControl_GetValidFrameCount(void);
uint32_t RemoteControl_GetCrcErrorCount(void);
uint32_t RemoteControl_GetUartErrorCount(void);

/* 下面两个是 HAL 弱回调的实现（DMA + IDLE 路径）。
 * 仅供 HAL 调用，app 不要直接调。 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size);
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart);

#ifdef __cplusplus
}
#endif

#endif /* __REMOTE_H */
