//
// Created by Forest on 2026-02-12.
//

#ifndef F446_FOC_FOC_H
#define F446_FOC_FOC_H

#define SQRT3_F 1.7320508075688772f
#define PI_F 3.141592653589793f

#include "main.h"
#include "tim.h"

extern float ua;
extern float ub;

extern uint16_t angle_raw;
extern uint16_t angle_offset;
extern float angle_el;

extern uint8_t voltage_power_supply;

void FOC_Init(void);
void set_pwm_compare(int16_t Ua, int16_t Ub, int16_t Uc, uint16_t half_period);
void sin_cos_theta(float angle, float *sin_theta, float *cos_theta);
void inverse_park_transform(float Uq, float Ud, float sin_theta, float cos_theta, float *u_alpha, float *u_beta);
void inverse_clarke_transform(float u_alpha, float u_beta, float *a, float *b, float *c);
void open_loop_voltage(float Uq, float Ud, float angle);
void setPhaseVoltage(float Uq, float Ud, float angle_el);
float raw_to_electrical_angle(uint16_t raw_angle, uint16_t raw_offset);
#endif // F446_FOC_FOC_H
