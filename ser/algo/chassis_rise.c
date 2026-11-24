#include "chassis_rise.h"
#include "can_router.h"
#include "chassis_alg.h"

#define APP_M2006_NUM         4u        //电机数量
#define APP_VESC_NUM          4u        //VESC电机数量

#define RATIO 36.0f
#define GEAR_D 48.0f



#define RISE_ANGLE1  (310.0f / (3.141592f * GEAR_D) * 360.0f)
#define RISE_ANGLE2  (310.0f / (3.141592f * GEAR_D) * 360.0f)
#define RISE_ANGLE3  (310.0f / (3.141592f * GEAR_D) * 360.0f)
#define RISE_ANGLE4  (310.0f / (3.141592f * GEAR_D) * 360.0f)

#define DOWN_ANGLE1  (4.0f / (3.141592f * GEAR_D) * 360.0f)
#define DOWN_ANGLE2  (4.0f / (3.141592f * GEAR_D) * 360.0f)
#define DOWN_ANGLE3  (4.0f / (3.141592f * GEAR_D) * 360.0f)
#define DOWN_ANGLE4  (4.0f / (3.141592f * GEAR_D) * 360.0f)

#define RESET_ANGLE  (20.0f / (3.141592f * GEAR_D) * 360.0f)

#define GUIDE_FORWARD_ANGLE -85.0f / (3.141592f * GEAR_D) * 360.0f
#define GUIDE_BACKWARD_ANGLE 75.0f / (3.141592f * GEAR_D) * 360.0f

/* 电流指令变化率限制：每周期（1 ms）最多变化多少 mA，上升 / 下降分开给。
 *   上升 500 mA/ms ：0 → 7 A 约 14 ms，起动力矩是斜着加上去的，不顿；
 *   下降 1500 mA/ms：7 A → 0 约 4.7 ms，松手 / 换向时电流退得快，安全优先。
 * 调参：想更"软"就调小，想更"跟手"就调大；
 *       把 CURRENT_SLEW_DOWN_MA 设成很大的值（如 100000）等于不限制下降。 */
#define CURRENT_SLEW_UP_MA     2000
#define CURRENT_SLEW_DOWN_MA   1500

/* 目标角度分段逼近（防止 up/down 一键把目标从 0 跳到 ±两万多转子度）：
 *   POS_SLEW_FULL_STROKE_MS —— 走完整段行程（RISE_ANGLE = 300 mm）用多少 ms；
 *   POS_SLEW_STEP_DEG       —— 换算成"每 1 ms 目标最多前进多少转子度"。
 * 四个电机每拍用同一个步长，所以四路目标的绝对值始终相等，
 * 只有方向按安装镜像（1、4 反向），四条腿就能一直保持同一角度。
 * 想整体快/慢，只改 POS_SLEW_FULL_STROKE_MS 一个数即可。 */
#define POS_SLEW_FULL_STROKE_MS   500.0f
#define POS_SLEW_STEP_DEG         ((float)(RISE_ANGLE1 * RATIO) / POS_SLEW_FULL_STROKE_MS)

M2006_Motor chassis_motor[APP_M2006_NUM]={0};
VESC_Motor vesc_motor[APP_VESC_NUM]={0};
M2006_Motor guide_motor[1]={0};
extern volatile float g_chassis_dt_s;   /* 周期，单位 秒   */

static pid_type chassis_out_pid[APP_M2006_NUM]={0};
static pid_type chassis_in_pid[APP_M2006_NUM]={0};

static pid_type guide_out_pid[1]={0};
static pid_type guide_in_pid[1]={0};

/* 上一拍真正下发的电流指令（mA），变化率限制的基准 */
static int32_t chassis_current_cmd[APP_M2006_NUM]={0};

/* 目标角度：req = 上层请求的最终目标，cmd = 本拍实际下发的目标（分段逼近 req） */
static float chassis_target_req[APP_M2006_NUM]={0};
static float chassis_target_cmd[APP_M2006_NUM]={0};

void chassis_rise_init(){
    BSP_CAN_Init(BSP_CAN_BUS_1);
    BSP_CAN_Init(BSP_CAN_BUS_2);
	M2006_Motor_Struct_Init(&chassis_motor[0], BSP_CAN_BUS_1, 1);
    M2006_Motor_Struct_Init(&chassis_motor[1], BSP_CAN_BUS_1, 2);
	M2006_Motor_Struct_Init(&chassis_motor[2], BSP_CAN_BUS_1, 3);
	M2006_Motor_Struct_Init(&chassis_motor[3], BSP_CAN_BUS_1, 4);
    M2006_Motor_Struct_Init(&guide_motor[0], BSP_CAN_BUS_1, 5);

    VESC_Motor_Struct_Init(&vesc_motor[0], BSP_CAN_BUS_2, 91);   //右上
	VESC_Motor_Struct_Init(&vesc_motor[1], BSP_CAN_BUS_2, 121);  //左上
    VESC_Motor_Struct_Init(&vesc_motor[2], BSP_CAN_BUS_2, 80);  //左下
    VESC_Motor_Struct_Init(&vesc_motor[3], BSP_CAN_BUS_2, 103);   //右下	


    PID_init(&chassis_out_pid[0], 10.55f,0.0f, 0.0f);
    PID_init(&chassis_in_pid[0], 3.0f, 0.20f, 0.0f);

    PID_init(&chassis_out_pid[1], 10.55f,0.0f, 0.0f);
    PID_init(&chassis_in_pid[1], 3.0f, 0.20f, 0.0f);

    PID_init(&chassis_out_pid[2], 10.55f,0.0f, 0.0f);
    PID_init(&chassis_in_pid[2], 3.0f, 0.20f, 0.0f);

	PID_init(&chassis_out_pid[3], 10.55f,0.0f, 0.0f);
    PID_init(&chassis_in_pid[3], 4.0f, 0.30f, 0.0f);
//----------------------------------------------------------
    PID_init(&guide_out_pid[0], 10.0f,0.0f, 0.0f);
    PID_init(&guide_in_pid[0], 4.0f, 0.5f, 0.0f);

    /* 上电 / 重新初始化：电流与目标基准清零，从当前位置平滑起步 */
    for (uint8_t i = 0u; i < (uint8_t)APP_M2006_NUM; i++)
    {
        chassis_current_cmd[i] = 0;
        chassis_target_req[i]  = 0.0f;
        chassis_target_cmd[i]  = 0.0f;
    }

    M2006_CAN_Router_Init(BSP_CAN_BUS_1);
    CAN_Router_M2006(chassis_motor, APP_M2006_NUM);   /* 底盘 4 个：ID 1~4 */
    CAN_Router_M2006(guide_motor,   1u);              /* 导向 1 个：ID 5   */

    VESC_CAN_Router_Init(BSP_CAN_BUS_2);
    CAN_Router_VESC(vesc_motor, APP_VESC_NUM);
}

/* 上层只"请求"目标：写进 chassis_target_req[]，
 * 真正送给 PID 的目标由 chassis_target_cmd[] 分段逼近（见 M2006_angle_set），
 * 这样 up/down 一键不会把目标从 0 直接跳到两万多转子度。 */
static void set_angle(uint8_t idx, float target_angle){
    if (idx >= (uint8_t)APP_M2006_NUM) { return; }
    chassis_target_req[idx] = target_angle;
}

static void up_one(){
   set_angle(0u, -RISE_ANGLE1 * RATIO);
}

static void up_two(){
   set_angle(1u, RISE_ANGLE2 * RATIO);
}

static void up_three(){
   set_angle(2u, RISE_ANGLE3 * RATIO);
}

static void up_four(){
   set_angle(3u, -RISE_ANGLE4 * RATIO);
}

static void down_one(){
   set_angle(0u, -DOWN_ANGLE1 * RATIO);
}

static void down_two(){
   set_angle(1u, DOWN_ANGLE2 * RATIO);
}

static void down_three(){
   set_angle(2u, DOWN_ANGLE3 * RATIO);
}

static void down_four(){
   set_angle(3u, -DOWN_ANGLE4 * RATIO);
}

static void reset_one(){
   set_angle(0u, -RESET_ANGLE * RATIO);
}

static void reset_two(){
   set_angle(1u, RESET_ANGLE * RATIO);
}

static void reset_three(){
   set_angle(2u, RESET_ANGLE * RATIO);
}

static void reset_four(){
   set_angle(3u, -RESET_ANGLE * RATIO);
}

static void M2006_angle_set(uint8_t idx, M2006_Motor *motor, pid_type *out_pid, pid_type *in_pid)
{
    int32_t target;
    int32_t cmd;
    float   req;
    float   now;

    if ((motor == 0) || (out_pid == 0) || (in_pid == 0) || (idx >= (uint8_t)APP_M2006_NUM)) { return; }

    /* 目标分段逼近：本拍目标最多朝 req 走 POS_SLEW_STEP_DEG，
     * 四路用同一个步长 => 目标绝对值一直相等，四条腿同步升降。 */
    now = chassis_target_cmd[idx];
    req = chassis_target_req[idx];

    if (req > (now + POS_SLEW_STEP_DEG))
    {
        now += POS_SLEW_STEP_DEG;
    }
    else if (req < (now - POS_SLEW_STEP_DEG))
    {
        now -= POS_SLEW_STEP_DEG;
    }
    else
    {
        now = req;                      /* 一步之内：直接到位 */
    }

    chassis_target_cmd[idx] = now;
    out_pid->target_val     = now;

    /* 外环：转子角度 -> 速度指令；内环：转子转速 -> 电流指令 */
    in_pid->target_val = PID_realize(out_pid, motor->RxData.Total_Rotor_Angle_Deg);
    in_pid->pidoutput = PID_realize(in_pid, (float)motor->RxData.Speed_Rpm);

    target = (int32_t)in_pid->pidoutput;

    /* 变化率限制：上升最多 +CURRENT_SLEW_UP_MA，下降最多 -CURRENT_SLEW_DOWN_MA */
    cmd = chassis_current_cmd[idx];

    if (target > cmd)
    {
        /* 要往上：离得远就迈一步，一步之内直接到位（避免永远差一点） */
        cmd = ((target - cmd) > (int32_t)CURRENT_SLEW_UP_MA)
              ? (cmd + (int32_t)CURRENT_SLEW_UP_MA)
              : target;
    }
    else if (target < cmd)
    {
        /* 要往下：步长单独给，通常比上升大，退电流更快更安全 */
        cmd = ((cmd - target) > (int32_t)CURRENT_SLEW_DOWN_MA)
              ? (cmd - (int32_t)CURRENT_SLEW_DOWN_MA)
              : target;
    }
    else
    {
        cmd = target;                   /* 目标没变 */
    }

    chassis_current_cmd[idx] = cmd;
    M2006_Motor_SetCurrent(motor, cmd);
}

void chassis_up_all(){
    up_one();
    up_two();
    up_three();
    up_four();

}

void chassis_up_1_2(){
    up_two();
    up_one();
}

void chassis_up_3_4(){      
    up_three();
    up_four();
}


void chassis_down_all(){
    down_one();
    down_two();
    down_three();
    down_four();
}

void chassis_down_1_2(){
    down_two();
    down_one();
}

void chassis_down_3_4(){      
    down_three();
    down_four();
}

void chassis_reset_all(){
    reset_one();
    reset_two();
    reset_three();
    reset_four();
}



void chassis_pid(){
    float values[17]={0};                                   /* 16 路电机 + 1 路 dt */
    for(uint8_t i=0;i<APP_M2006_NUM;i++){
        M2006_angle_set(i, &chassis_motor[i], &chassis_out_pid[i], &chassis_in_pid[i]);
        values[i] = chassis_motor[i].RxData.Current_mA/1000.0f;
        values[i+4] = chassis_in_pid[i].pidoutput/1000.0f;   /* 变化率限制后的实际指令 */
        values[i+8] = chassis_out_pid[i].target_val/RATIO;
        values[i+12] = chassis_motor[i].RxData.Total_Output_Angle_Deg;
    }
    values[16] = g_chassis_dt_s;
    VOFA_Print_Csv(values, 17);
    M2006_Motor_SendCurrents(chassis_motor, APP_M2006_NUM);
}

static void guide_angle_set(float target_angle){
    guide_out_pid[0].target_val     = target_angle * RATIO;
}

void guide_forward(){
    guide_angle_set(GUIDE_FORWARD_ANGLE);
}

void guide_backward(){
    guide_angle_set(GUIDE_BACKWARD_ANGLE);
}

void guide_reset(){
    guide_angle_set(0.0f);
}

void guide_pid(){
    guide_in_pid[0].target_val = PID_realize(&guide_out_pid[0], guide_motor[0].RxData.Total_Rotor_Angle_Deg);
    guide_in_pid[0].pidoutput = PID_realize(&guide_in_pid[0], (float)guide_motor[0].RxData.Speed_Rpm);

    int32_t target = (int32_t)guide_in_pid[0].pidoutput;
    M2006_Motor_SetCurrent(&guide_motor[0], target);

    /* 导向电机是 ID 5，属于 0x1FF 那一帧（ID 5~8）；
     * 底盘 1~4 走的是 0x200，所以这里必须单独发一次，否则它的电流永远出不去 */
    (void)M2006_Motor_SendCurrents(guide_motor, 1u);
}
