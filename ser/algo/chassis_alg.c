#include "chassis_alg.h"
#include "can_router.h"
#include "chassis_config.h"

extern VESC_Motor vesc_motor[4];

static WheelVelocity chassis_inverse_kinematics(ChassisVelocity target_speed)
{
    WheelVelocity wheel_speed;

    // 计算每个轮子的速度
    wheel_speed.speed[0] =  target_speed.x + target_speed.y + target_speed.yaw * CHASSIS_R; //右上
    wheel_speed.speed[1] =  target_speed.x - target_speed.y - target_speed.yaw * CHASSIS_R; //左上
    wheel_speed.speed[2] =  target_speed.x + target_speed.y - target_speed.yaw * CHASSIS_R; //左下
    wheel_speed.speed[3] =  target_speed.x - target_speed.y + target_speed.yaw * CHASSIS_R; //右下	

    return wheel_speed;
}

static WheelVelocity speed_to_rpm(WheelVelocity wheel_speed)
{
    WheelVelocity rpm_speed;

    for (uint8_t i = 0; i < 4; i++)
    {
        rpm_speed.speed[i] = wheel_speed.speed[i] * 60.0f/(PI * WHEEL_D);
    }

    return rpm_speed;
}


static void motor_speed_control(WheelVelocity wheel_speed)
{
    wheel_speed = speed_to_rpm(wheel_speed);
    for (uint8_t i = 0; i < 4; i++)
    {
        VESC_Set_Rpm_Struct(&vesc_motor[i], (int32_t)(wheel_speed.speed[i]));
    }
}

void chassis_speedsend(ChassisVelocity target_speed){
    WheelVelocity motor_speed;
    motor_speed = chassis_inverse_kinematics(target_speed);
    motor_speed_control(motor_speed);
}

