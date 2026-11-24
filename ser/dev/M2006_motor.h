/*****************************************************************************
 * M2006_motor.h —— RoboMaster M2006 P36 电机 + C610 电调驱动（ser/dev）
 *
 * 分层位置：ser/dev —— 只做 C610 协议的打包 / 解析，不直接碰 HAL。
 *   允许依赖：bsp_can（通用标准帧收发）、common
 *   禁止依赖：app 全局变量、CAN 句柄、HAL 头文件
 *
 * C610 协议（《RM C610 无刷电机调速器使用说明》）：
 *   - 控制：标准帧 DATA，DLC = 8
 *       ID 0x200 控制电调 ID 1~4，ID 0x1FF 控制电调 ID 5~8；
 *       每路一个 int16 大端电流值，范围 -10000~10000，
 *       对应输出转矩电流 -10~10 A（1 LSB = 1 mA）。
 *   - 反馈：标准帧 DATA，DLC = 8，ID = 0x200 + 电调 ID（0x201~0x208）
 *       DATA[0:2] 转子机械角度 0~8191（对应 0~360°）
 *       DATA[2:4] 转子转速 int16，单位 rpm
 *       DATA[4:6] 实际转矩电流 int16，单位 mA
 *       DATA[6:8] 保留
 *   - M2006 P36 减速比 36:1，反馈的转速 / 角度都是【转子侧】。
 *
 * 解耦方式与 vesc_motor 一致：
 *   - 本模块只认 bsp_can 的【总线号 + 标准帧 ID + 数据】，不认识 CAN 句柄；
 *   - C610 的控制帧由 4 个电调共享，所以 SetCurrent 只更新目标值，
 *     由上层每个控制周期调用一次 M2006_Motor_SendCurrents() 统一打包发送；
 *   - 上行由 can_router 把帧交给 M2006_Motor_RxDispatch()，本模块不注册中断。
 *****************************************************************************/
#ifndef __M2006_MOTOR_H
#define __M2006_MOTOR_H

#include <stdint.h>
#include "return_typedef.h"
#include "bsp_can.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ---------------- C610 协议常量 ---------------- */
#define M2006_MOTOR_ID_MIN      1u
#define M2006_MOTOR_ID_MAX      8u

#define M2006_CTRL_ID_1_4       0x200u   /* 控制电调 ID 1~4 */
#define M2006_CTRL_ID_5_8       0x1FFu   /* 控制电调 ID 5~8 */
#define M2006_FB_ID_BASE        0x200u   /* 反馈 ID = 0x200 + 电调 ID */

#define M2006_CURRENT_MIN       (-10000) /* -10 A，单位 mA */
#define M2006_CURRENT_MAX       (10000)  /* +10 A，单位 mA */

#define M2006_ANGLE_MAX         8191u    /* 单圈转子角度原始值上限 */
#define M2006_ANGLE_RES         8192.0f  /* 每圈 8192 个计数 */
#define M2006_GEAR_RATIO        36.0f    /* M2006 P36 减速比 */

/* ---------------- 反馈数据（由中断回调写入） ---------------- */
typedef struct
{
    uint16_t Angle_Raw;             /* 0~8191，转子机械角度原始值              */
    int16_t  Speed_Rpm;             /* 转子转速，rpm（有符号）                 */
    int16_t  Current_mA;            /* 实际转矩电流，mA（有符号）              */

    float    Angle_Deg;             /* 0~360°，单圈转子机械角度               */
    float    Output_Speed_Rpm;      /* 输出轴转速 = Speed_Rpm / 36             */
    float    Output_Current_A;      /* 实际转矩电流 = Current_mA / 1000        */
    float    Total_Rotor_Angle_Deg; /* 累计转子角度，跨 0/360 与多圈都连续     */
    float    Total_Output_Angle_Deg;/* 累计输出轴角度 = 转子累计 / 36          */

    uint32_t RxCount;               /* 收到的反馈帧数，用于判断通信是否正常    */
} M2006_RxData;

/* ---------------- 基础参数 ---------------- */
typedef struct
{
    bsp_can_bus_t bus;              /* 所在 CAN 总线                           */
    uint8_t       ID;               /* 电调 ID，1~8                            */
} M2006_Basic_Parameters;

/* ---------------- 电机句柄 ---------------- */
typedef struct M2006_Motor
{
    M2006_RxData           RxData;              /* 运行反馈                    */
    M2006_Basic_Parameters Basic_Parameters;    /* 总线 / ID                   */

    int16_t Target_Current;                     /* 目标转矩电流，mA，已限幅    */

    /* 函数指针：与 vesc_motor 一样，把控制入口挂到句柄上。
     * 注意：SetCurrent 只更新目标值，不会立刻发 CAN；
     *       发送由 M2006_Motor_SendCurrents() 统一完成。 */
    void (*SetCurrent)(struct M2006_Motor *motor, int32_t current_mA);
    void (*Stop)(struct M2006_Motor *motor);

    /* 内部状态：上一帧角度，0xFFFF 表示还没收到过反馈 */
    uint16_t Last_Angle_Raw;
} M2006_Motor;

/* 初始化句柄：绑定总线 / ID / 控制函数，清空反馈数据。
 * 返回 SER_OK / SER_ERR_PARAM（空指针、ID 越界、总线越界）。 */
int32_t M2006_Motor_Struct_Init(M2006_Motor *motor, bsp_can_bus_t bus, uint8_t id);

/* 设置目标转矩电流（mA），内部限幅到 -10000~10000。
 * 只写句柄，不发送；每个控制周期末尾调用一次 SendCurrents。 */
void M2006_Motor_SetCurrent(M2006_Motor *motor, int32_t current_mA);

/* 目标电流清零（同样只写句柄，不发送） */
void M2006_Motor_Stop(M2006_Motor *motor);

/* 把一个电机数组的目标电流打包成 0x200 / 0x1FF 标准帧发出去。
 *   - motors 必须包含【同一条总线上需要一起控制的所有电机】，
 *     数组里没有的 ID 会按 0 电流填充（避免残留上一拍输出）。
 *   - 支持 ID 1~8；ID 1~4 走 0x200，ID 5~8 走 0x1FF；
 *   - 每帧 DLC 固定 8，大端 int16，单位 mA。
 * 返回 SER_OK；任一帧发送失败或参数非法返回 SER_ERR_*。 */
int32_t M2006_Motor_SendCurrents(M2006_Motor *motors, uint8_t count);

/* 接收分发：把一帧标准帧按 ID 交给数组里对应的电机。
 * std_id 范围 0x201~0x208，data/len 为 C610 反馈帧。
 * 返回 SER_OK / SER_ERR_PARAM / SER_ERR_NOENT。 */
int32_t M2006_Motor_RxDispatch(M2006_Motor *motors, uint8_t count,
                               uint32_t std_id, const uint8_t *data, uint8_t len);

/* 单电机反馈解析：把一帧 C610 反馈写入指定句柄。
 * 返回 SER_OK / SER_ERR_PARAM / SER_ERR_NOENT（ID 不匹配）。 */
int32_t M2006_Motor_OnCanRx(M2006_Motor *motor, uint32_t std_id,
                            const uint8_t *data, uint8_t len);

#ifdef __cplusplus
}
#endif

#endif /* __M2006_MOTOR_H */
