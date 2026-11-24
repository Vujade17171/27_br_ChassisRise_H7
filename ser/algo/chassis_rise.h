#ifndef __chassis_RISE_H
#define __chassis_RISE_H

#include <stdint.h>
#include "M2006_motor.h"
#include "pid.h"
#include "vofaprint.h"

#ifdef __cplusplus
extern "C" {
#endif
void chassis_rise_init();

void chassis_up_all();
void chassis_up_1_2();
void chassis_up_3_4();

void chassis_down_all();
void chassis_down_1_2();
void chassis_down_3_4();

void chassis_pid();


void guide_forward();

void guide_backward();

void guide_reset();

void guide_pid();

#ifdef __cplusplus
}
#endif

#endif /* __chassis_RISE_H */