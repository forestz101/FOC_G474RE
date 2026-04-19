//
// Created by Forest on 2026-02-12.
//

#ifndef FOC_H
#define FOC_H

#include "main.h"
#include <stdint.h>
#include "math_defs.h"
#include "motor_config.h"
#include "observer.h"

// ADC / current sense
#define ADC_VREF      3.3f
#define ADC_SCALE     (ADC_VREF / 4095.0f)
#define CC6922SG_SENS 0.0132f

// Control loop
#define CONTROL_DT (1.0f / 20753.0f)
#define PWM_PERIOD 65534

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

typedef enum {
    MC_DISABLED  = 0,
    MC_FOC       = 1,
    MC_OPEN_LOOP = 2,
} mc_mode_t;

typedef struct {
    float vbus;
    mc_mode_t mode;
    float el_angle;
    float current_scale;

    phase_currents_t    phase_currents;
    foc_trig_t          trig;
    alpha_beta_t        alpha_beta;
    direct_quadrature_t dq;
    volt_alpha_beta_t   volt_ab;
    v_abc_t             v_abc;
    duty_t              duty;

    foc_commands_t cmd;
    pi_t pi_d;
    pi_t pi_q;

    observer_state_t observer;
} motor_controller_t;

extern motor_controller_t mc;

extern volatile uint16_t current_buf[2];
extern volatile uint16_t current_ref_buf[2];

// Public API
void mc_init(void);
void foc_step(float dt, float encoder_el_angle);
void open_loop_step(void);
float pi_run(pi_t *pi, float error, float dt);

#endif // FOC_H
