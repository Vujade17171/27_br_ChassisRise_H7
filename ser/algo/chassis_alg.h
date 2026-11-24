#ifndef __CHASSIS_ALG_H
#define __CHASSIS_ALG_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float x;
    float y;
    float yaw;
} ChassisVelocity;

typedef struct {
    float speed[4];  /* 轮速 m/s */
} WheelVelocity;

void chassis_speedsend(ChassisVelocity target_speed);

#ifdef __cplusplus
}
#endif

#endif /* __CHASSIS_ALG_H */