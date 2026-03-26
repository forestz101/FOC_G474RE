//
// Created by Forest on 2026-02-12.
//

#include "foc.h"

float ua, ub;
float angle_el;
float sint, cost;
float a, b, c;

float raw_to_electrical_angle(const uint16_t raw_angle, const uint16_t raw_offset) {
    const float mech = (2.0f * PI_F) * ((float)(raw_angle - raw_offset) / 16384.0f);
    const float elec = mech * 7;

    return fmodf(elec, 2.0f * PI_F);
}

void set_pwm_compare(const int16_t Ua, const int16_t Ub, const int16_t Uc, const uint16_t half_period) {

    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, Ua + half_period);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, Ub + half_period);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, Uc + half_period);
}

void sin_cos_theta(const float angle, float *sin_theta, float *cos_theta) {
    *sin_theta = sinf(angle);
    *cos_theta = cosf(angle);
}

void inverse_park_transform(const float Uq, const float Ud,
    const float sin_theta, const float cos_theta, float *u_alpha, float *u_beta) {
    *u_alpha = Ud*cos_theta - Uq*sin_theta;
    *u_beta = Uq*cos_theta - Ud*sin_theta;
}

void inverse_clarke_transform(const float u_alpha, const float u_beta, float *a, float *b, float *c) {
    *a = u_alpha;
    *b = (SQRT3_F * u_beta - u_alpha)/2;
    *c = (-u_alpha - SQRT3_F * u_beta)/2;
    // setPwm(Ua, Ub, Uc);
}

void open_loop_voltage(const float Uq, const float Ud, const float angle)
{
    sin_cos_theta(angle, &sint, &cost);
    inverse_park_transform(Uq, Ud, sint, cost, &ua, &ub);
    inverse_clarke_transform(ua, ub, &a, &b, &c);
    set_pwm_compare(a, b, c, pwm_period/2);
}

void setPhaseVoltage(const float Uq, const float Ud, const float angle_el) {
    float u_alpha = Ud*cos(angle_el) - Uq*sin(angle_el);
    float u_beta = Uq*cos(angle_el) - Ud*sin(angle_el);

    const uint32_t Ua = u_alpha + Uq/2;
    const uint32_t Ub = (sqrt(3) * u_beta-u_alpha)/2 + Uq/2;
    const uint32_t Uc = (-u_alpha - sqrt(3) * u_beta)/2 + Uq/2;
    set_pwm_compare(Ua, Ub, Uc, 0);
}
