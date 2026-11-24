/*****************************************************************************
 * M2006_motor.c —— RoboMaster M2006 P36 + C610 电调协议实现（ser/dev）
 *
 * 只负责 C610 的报文打包 / 解析，收发原语统一走 bsp_can：
 *   下行：BSP_CAN_SendStd()          （0x200 / 0x1FF 标准帧）
 *   上行：M2006_Motor_RxDispatch()   （由 can_router 调用，不注册中断）
 *****************************************************************************/
#include "M2006_motor.h"

/* 把目标电流限制在 C610 允许的 -10000~10000 mA（对应 -10~10 A） */
static int16_t clamp_current_mA(int32_t current_mA)
{
    if (current_mA > M2006_CURRENT_MAX) {
        current_mA = M2006_CURRENT_MAX;
    } else if (current_mA < M2006_CURRENT_MIN) {
        current_mA = M2006_CURRENT_MIN;
    }
    return (int16_t)current_mA;
}

/* int16 大端写入：C610 的控制电流和数据域都是高字节在前 */
static void put_i16_be(uint8_t *dst, int16_t value)
{
    uint16_t u = (uint16_t)value;

    dst[0] = (uint8_t)(u >> 8);
    dst[1] = (uint8_t)(u & 0xFFu);
}

/* ---------------------------------------------------------------------------
 * 初始化 / 控制入口
 * ------------------------------------------------------------------------- */
int32_t M2006_Motor_Struct_Init(M2006_Motor *motor, bsp_can_bus_t bus, uint8_t id)
{
    M2006_RxData zero = {0};

    if (motor == 0) { return SER_ERR_PARAM; }
    if ((id < M2006_MOTOR_ID_MIN) || (id > M2006_MOTOR_ID_MAX)) { return SER_ERR_PARAM; }
    if (bus >= (bsp_can_bus_t)BSP_CAN_BUS_NUM) { return SER_ERR_PARAM; }

    motor->Basic_Parameters.bus = bus;
    motor->Basic_Parameters.ID  = id;

    motor->RxData = zero;
    motor->Target_Current = 0;
    motor->Last_Angle_Raw = 0xFFFFu;   /* 还没收到过反馈 */

    motor->SetCurrent = M2006_Motor_SetCurrent;
    motor->Stop       = M2006_Motor_Stop;

    return SER_OK;
}

void M2006_Motor_SetCurrent(M2006_Motor *motor, int32_t current_mA)
{
    if (motor == 0) { return; }
    motor->Target_Current = clamp_current_mA(current_mA);
}

void M2006_Motor_Stop(M2006_Motor *motor)
{
    if (motor == 0) { return; }
    motor->Target_Current = 0;
}

/* ---------------------------------------------------------------------------
 * 下行：把数组里所有电机的目标电流打包成 0x200 / 0x1FF
 *
 * C610 一帧控制 4 个电调，所以必须整组打包；数组里没有的 ID 填 0，
 * 防止上一拍残留的输出继续作用在没参与本拍计算的电机上。
 * ------------------------------------------------------------------------- */
int32_t M2006_Motor_SendCurrents(M2006_Motor *motors, uint8_t count)
{
    uint8_t i;
    uint8_t bus;
    int32_t ret = SER_OK;

    if ((motors == 0) || (count == 0u) || (count > M2006_MOTOR_ID_MAX)) {
        return SER_ERR_PARAM;
    }

    /* 先整体校验，避免发了一半才发现某个句柄非法 */
    for (i = 0u; i < count; i++) {
        uint8_t id = motors[i].Basic_Parameters.ID;

        if ((id < M2006_MOTOR_ID_MIN) || (id > M2006_MOTOR_ID_MAX)) {
            return SER_ERR_PARAM;
        }
        if (motors[i].Basic_Parameters.bus >= (bsp_can_bus_t)BSP_CAN_BUS_NUM) {
            return SER_ERR_PARAM;
        }
    }

    for (bus = 0u; bus < (uint8_t)BSP_CAN_BUS_NUM; bus++) {
        uint8_t data_1_4[8] = {0};
        uint8_t data_5_8[8] = {0};
        uint8_t has_1_4 = 0u;
        uint8_t has_5_8 = 0u;

        for (i = 0u; i < count; i++) {
            uint8_t id;
            uint8_t slot;
            int16_t current;

            if (motors[i].Basic_Parameters.bus != (bsp_can_bus_t)bus) {
                continue;
            }

            id      = motors[i].Basic_Parameters.ID;
            current = clamp_current_mA(motors[i].Target_Current);

            if (id <= 4u) {
                slot = (uint8_t)(id - 1u);
                put_i16_be(&data_1_4[slot * 2u], current);
                has_1_4 = 1u;
            } else {
                slot = (uint8_t)(id - 5u);
                put_i16_be(&data_5_8[slot * 2u], current);
                has_5_8 = 1u;
            }
        }

        if (has_1_4 != 0u) {
            int32_t r = BSP_CAN_SendStd((bsp_can_bus_t)bus, M2006_CTRL_ID_1_4,
                                        data_1_4, 8u);
            if ((r != SER_OK) && (ret == SER_OK)) { ret = r; }
        }
        if (has_5_8 != 0u) {
            int32_t r = BSP_CAN_SendStd((bsp_can_bus_t)bus, M2006_CTRL_ID_5_8,
                                        data_5_8, 8u);
            if ((r != SER_OK) && (ret == SER_OK)) { ret = r; }
        }
    }

    return ret;
}

/* ---------------------------------------------------------------------------
 * 上行：反馈解析
 *
 * C610 反馈帧（标准帧，DLC=8）：
 *   DATA[0:2] 转子机械角度 0~8191
 *   DATA[2:4] 转子转速 int16（rpm）
 *   DATA[4:6] 实际转矩电流 int16（mA）
 *   DATA[6:8] 保留
 * ------------------------------------------------------------------------- */
int32_t M2006_Motor_OnCanRx(M2006_Motor *motor, uint32_t std_id,
                            const uint8_t *data, uint8_t len)
{
    uint16_t angle_raw;
    int16_t  speed_rpm;
    int16_t  current_mA;
    float    angle_deg;

    if ((motor == 0) || (data == 0) || (len < 6u)) { return SER_ERR_PARAM; }
    if (std_id != (uint32_t)(M2006_FB_ID_BASE + motor->Basic_Parameters.ID)) {
        return SER_ERR_NOENT;
    }

    angle_raw = (uint16_t)((((uint16_t)data[0] << 8) | (uint16_t)data[1]) & 0x1FFFu);
    speed_rpm = (int16_t)(((uint16_t)data[2] << 8) | (uint16_t)data[3]);
    current_mA = (int16_t)(((uint16_t)data[4] << 8) | (uint16_t)data[5]);

    angle_deg = (float)angle_raw * (360.0f / M2006_ANGLE_RES);

    motor->RxData.Angle_Raw      = angle_raw;
    motor->RxData.Speed_Rpm      = speed_rpm;
    motor->RxData.Current_mA     = current_mA;
    motor->RxData.Angle_Deg      = angle_deg;
    motor->RxData.Output_Speed_Rpm =
        (float)speed_rpm / M2006_GEAR_RATIO;
    motor->RxData.Output_Current_A =
        (float)current_mA / 1000.0f;

    /* 角度跨 0/8191 处理：差值按半圈（4096）判方向，再累加到总角度。
     * C610 默认 1 kHz 反馈，1 ms 内转子转不到半圈，4096 的阈值足够。 */
    if (motor->Last_Angle_Raw > M2006_ANGLE_MAX) {
        /* 第一帧：以【上电时的位置】作为 0 点（相对基准）。
         * 不能拿 angle_deg 当起点：那是转子 0~360° 的绝对角，四个电机的绝对角
         * 互不相同，用它当 0 点会让同一个目标（如 ±360°）对四个电机变成不同行程，
         * 平台会被顶歪。置 0 之后，"抬 10°"对四路就是同一个行程。 */
        motor->RxData.Total_Rotor_Angle_Deg = 0.0f;
    } else {
        int16_t delta = (int16_t)((int32_t)angle_raw -
                                  (int32_t)motor->Last_Angle_Raw);

        if (delta > 4096) {
            delta = (int16_t)(delta - 8192);
        } else if (delta < -4096) {
            delta = (int16_t)(delta + 8192);
        }
        motor->RxData.Total_Rotor_Angle_Deg +=
            (float)delta * (360.0f / M2006_ANGLE_RES);
    }
    motor->Last_Angle_Raw = angle_raw;

    motor->RxData.Total_Output_Angle_Deg =
        motor->RxData.Total_Rotor_Angle_Deg / M2006_GEAR_RATIO;
    motor->RxData.RxCount++;

    return SER_OK;
}

int32_t M2006_Motor_RxDispatch(M2006_Motor *motors, uint8_t count,
                               uint32_t std_id, const uint8_t *data, uint8_t len)
{
    uint8_t id;
    uint8_t i;

    if ((motors == 0) || (data == 0) || (len < 6u)) { return SER_ERR_PARAM; }
    if ((std_id < (M2006_FB_ID_BASE + M2006_MOTOR_ID_MIN)) ||
        (std_id > (M2006_FB_ID_BASE + M2006_MOTOR_ID_MAX))) {
        return SER_ERR_NOENT;
    }

    id = (uint8_t)(std_id - M2006_FB_ID_BASE);

    for (i = 0u; i < count; i++) {
        if (motors[i].Basic_Parameters.ID == id) {
            return M2006_Motor_OnCanRx(&motors[i], std_id, data, len);
        }
    }

    return SER_ERR_NOENT;
}
