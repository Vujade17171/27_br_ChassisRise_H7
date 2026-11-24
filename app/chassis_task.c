#include "chassis_task.h"
#include "main.h"        /* SystemCoreClock（测周期用）        */
#include "osal_time.h"
#include "osal_task.h"   /* OSAL_Task_DelayUntil：严格周期节拍 */
#include "M2006_motor.h"
#include "can_router.h"
#include "chassis_rise.h"
#include "chassis_alg.h"
#include "remote.h"
#include "light_signal.h"

#define CHASSIS_TASK_PERIOD_MS   1u     /* 控制周期：严格 1 ms */

#define FORWARD_SPEED  0.30f

uint8_t task_sign=0;

ChassisVelocity chassis_velocity = {0.0f, 0.0f, 0.0f}; //x向前，y向左，yaw逆时针
extern M2006_Motor chassis_motor[4];
extern M2006_Motor guide_motor[1];

uint8_t ch1=3;
uint8_t ch2=3;
uint8_t ch3=3;
uint8_t ch4=3;

RemoteControlData_t rc;

uint8_t a=0;

/* 上一拍 → 这一拍的实际周期（由 DWT 周期计数器测，分辨率 1/CPU主频 ≈ 3.6 ns @275MHz）。
 * 放在文件作用域，方便调试器 Watch，或直接塞进 VOFA 看节拍稳不稳：
 *   float v[2] = { g_chassis_dt_s, (float)g_chassis_dt_us };
 * 正常应稳定在 1.000 ms；偶尔 2 ms 说明漏拍，长期偏大说明任务超时。 */
volatile float    g_chassis_dt_s  = 0.0f;   /* 周期，单位 秒   */

static void chassis_speedset(float x,float y,float yaw){
        chassis_velocity.x = x;
        chassis_velocity.y = y;
        chassis_velocity.yaw = yaw;
}

/* 台阶状态机里的每一层都是"死等"式循环：里面没有任何阻塞调用，
 * 如果不主动让出 CPU，空闲任务永远跑不到（vTaskDelay 一次都不调），
 * 轻则看门狗/低优先级任务饿死，重则调度器表现异常。
 * 所以下面每个 while 循环体末尾都补一次 1 ms 延时。 */
#define STEP_LOOP_DELAY()   OSAL_Time_DelayMs(1u)

void chassis_task(void *argument)
{
    uint32_t period_tick = 0u;      /* OSAL_Task_DelayUntil 的周期锚点 */
    uint32_t dt_last_cnt = 0u;      /* DWT 计数缓存（OSAL_Time_Dt 用）  */

    chassis_rise_init();
    Light_Signal_Init();
    (void)RemoteControl_Init();

    /* 周期测量的计时基准：DWT 按 CPU 主频计数。
     * 用 SystemCoreClock 推导，以后改主频这里不用跟着改。 */
    (void)OSAL_Time_Init((uint32_t)(SystemCoreClock / 1000000u));
    OSAL_Time_ResetDt(&dt_last_cnt);    /* 预置计数，避免第一拍算出巨大的 dt */

    for (;;)
    {
        /* 放在循环最前面：量的是"上一拍进入循环 → 这一拍进入循环"，
         * 正好是任务周期（含 1 ms 睡眠 + 本轮执行时间）。 */
        g_chassis_dt_s  = OSAL_Time_Dt(&dt_last_cnt);



        //调试距离使用
//        if(rc.right_gear==1){
//            guide_forward();
//        }
//        else if(rc.right_gear==2){
//            guide_backward();
//        }

        //上台阶任务
        if(rc.left_gear==2 && rc.right_gear==1){
            task_sign=1;
        }
        else if(rc.left_gear==2 && rc.right_gear==2){
            if(task_sign==1)
            {
                while(1){
                    chassis_up_all();
                    chassis_pid();
                    guide_pid();
                    STEP_LOOP_DELAY();          /* 让出 CPU */
                    if(chassis_motor[0].RxData.Total_Output_Angle_Deg < -700.0f){
                        break;
                    }
                }
                while(1){
                    guide_forward();
                    guide_pid();
                    chassis_pid();
                    Light_Signal_Update();
                    chassis_speedset(FORWARD_SPEED,0.0f,0.0f);
                    chassis_speedsend(chassis_velocity);
                    while(Light_Signal_Get(LIGHT_SIGNAL_CH1)==0 && guide_motor[0].RxData.Total_Output_Angle_Deg <-180.0f){//待调
                        chassis_down_3_4();
                        guide_pid();
                        chassis_pid();
                        chassis_speedset(0.8f,0.0f,0.0f);
                        chassis_speedsend(chassis_velocity);
                        Light_Signal_Update();
                        STEP_LOOP_DELAY();      /* 让出 CPU */
                        while(Light_Signal_Get(LIGHT_SIGNAL_CH2)==0){
                            guide_backward();
                            guide_pid();
                            chassis_speedset(FORWARD_SPEED,0.0f,0.0f);
                            chassis_pid();
                            chassis_speedsend(chassis_velocity);
                            Light_Signal_Update();
                            STEP_LOOP_DELAY();  /* 让出 CPU */
                            while(Light_Signal_Get(LIGHT_SIGNAL_CH3)==0){
                                chassis_down_1_2();
                                chassis_speedset(0.4f,0.0f,0.0f);
                                guide_pid();
                                chassis_pid();
                                chassis_speedsend(chassis_velocity);
                                Light_Signal_Update();
                                STEP_LOOP_DELAY();  /* 让出 CPU */
                                while (Light_Signal_Get(LIGHT_SIGNAL_CH4)==0)
                                {
                                    guide_reset();
                                    guide_pid();
                                    chassis_pid();
                                    chassis_speedset(0.0f,0.0f,0.0f);
                                    chassis_speedsend(chassis_velocity);
                                    STEP_LOOP_DELAY();  /* 让出 CPU */
                                    task_sign=0;
                                    break;
                                }
                                if(task_sign==0){
                                    chassis_speedsend(chassis_velocity);
                                    break;
                                }
                            }
                            if(task_sign==0){
                                chassis_speedsend(chassis_velocity);
                                break;
                            }
                        }
                        if(task_sign==0){
                            chassis_speedsend(chassis_velocity);
                            break;
                        }
                    }
                    if(task_sign==0){
                        chassis_speedsend(chassis_velocity);
                        break;
                    }
                    STEP_LOOP_DELAY();          /* 让出 CPU：CH1 已满足时这层也会空转 */
                }            
            }
        }
        chassis_reset_all();
        chassis_speedsend(chassis_velocity);
        guide_pid();
        chassis_pid();

        /* 长时间停在台阶状态机里之后，period_tick 会远远落后于当前 tick。
         * 这里先重新对齐一次，否则回到主循环会连续"追赶"几十上百拍、
         * 每拍都不睡，反而把 CPU 占满（等于又卡了一次）。 */
        if ((OSAL_Time_GetTick() - period_tick) > (2u * CHASSIS_TASK_PERIOD_MS))
        {
            period_tick = 0u;       /* 0 = 下一次 DelayUntil 自动取当前 tick */
        }

        /* 严格 1 ms 周期：按【绝对唤醒时刻】睡，不把本轮执行时间累加进去。
         * 原来的 OSAL_Time_DelayMs(1) 是"相对延时"，周期 = 执行时间 + 1 ms，会慢慢漂。
         * 若某轮执行超过 1 ms，会立刻返回并尽快追上节拍（不补睡）。 */
        (void)OSAL_Task_DelayUntil(&period_tick, CHASSIS_TASK_PERIOD_MS);
    }
}
