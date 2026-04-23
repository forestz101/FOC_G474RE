#ifndef OBSERVER_H
#define OBSERVER_H

#include "smo.h"

// Fusion of sensorless and sensored
// encoder correction gain vs electrical speed (rad/s)
#define FUSION_SPEED_LOW   50.0f
#define FUSION_SPEED_HIGH  300.0f
#define FUSION_GAIN_MAX    0.8f
#define FUSION_GAIN_MIN    0.0f

typedef struct {
    // Output
    float theta_est;
    float omega_est;

    // Motor params
    float Rs;
    float Ls;
    float flux_linkage;

    // Previous-cycle voltage
    float v_alpha_prev;
    float v_beta_prev;

    // SMO backend
    smo_state_t smo;

    // Fusion
    float correction_gain;

    // Startup: skip SMO until voltage history is valid
    uint8_t warmup_cycles;
} observer_state_t;

void observer_init(observer_state_t *obs,
                   float Rs, float Ls, float flux_linkage);

void observer_step(observer_state_t *obs,
                   float i_alpha, float i_beta,
                   float encoder_el_angle, float dt);

void observer_update_voltage(observer_state_t *obs,
                             float v_alpha, float v_beta);

#endif // OBSERVER_H
