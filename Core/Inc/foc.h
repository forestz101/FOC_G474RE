//
// Created by Forest on 2026-02-12.
//

// Math Constants
#define SQRT3_F 1.732050807568877f
#define HALF_SQRT3_F 0.8660254037844386f
#define INVERSE_SQRT3_F 0.5773502691896258f
#define INVERSE_2SQRT3_F 1.154700538379252f

#define PI_F 3.141592653589793f
#define PI2_F 6.283185307179586f

// ADC reference voltage
#define ADC_VREF     3.3f
#define ADC_SCALE    (ADC_VREF / 4095.0f)

// CC6922SG sensitivity
#define CC6922SG_SENS 0.0132f   // 13.2 mV/A

// Control loop period length
#define CONTROL_DT (1.0f / 20753.0f)
#define PWM_PERIOD 65534    // 0xFFFF - 1

#ifndef FOC_H
#define FOC_H

#include "main.h"
#include <stdint.h>

typedef struct {
    float i_a;
    float i_b;
    float i_c;
} phase_currents_t;

typedef struct {
    float i_alpha;
    float i_beta;
} alpha_beta_t;

typedef struct {
    float v_alpha;
    float v_beta;
} volt_alpha_beta_t;

typedef struct {
    float id;
    float iq;
} direct_quadrature_t;

typedef struct {
    float sin_theta;
    float cos_theta;
} foc_trig_t;

typedef struct {
    float v_a;
    float v_b;
    float v_c;
} v_abc_t;

typedef struct {
    float d_a;
    float d_b;
    float d_c;
} duty_t;

typedef struct {
    float kp;
    float ki;
    float integrator;
    float out_min;
    float out_max;
} pi_t;

typedef struct {
    float id_ref;
    float iq_ref;
    float ud;
    float uq;
} foc_commands_t;

extern volatile uint16_t current_buf[2]; // [CSA, CSC]
extern volatile uint16_t current_ref_buf[2]; // [CSA REF, CSC REF]
extern phase_currents_t phase_currents;
extern float vbus;
extern pi_t pi_d;
extern pi_t pi_q;
extern foc_commands_t foc_cmd;
extern direct_quadrature_t dq;
extern float el_angle;
extern uint8_t foc_enable;

// Public API
void foc_init();
void GetPhaseCurrents();
void ClarkeTransform();
void ParkTransform();
void InversePark();
float PI_Run(pi_t *pi, float error, float dt);
void FOC_Step(float dt);
void OpenLoopStep();

#endif // FOC_H
