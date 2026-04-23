#ifndef SMO_H
#define SMO_H

#include <stdint.h>

typedef struct {
    // Current observer
    float i_alpha_hat;
    float i_beta_hat;
    // Estimated BEMF
    float e_alpha_hat;
    float e_beta_hat;
    // Tuning
    float gain;
    float lpf_alpha;
    float sigmoid_gain;
    // PLL
    float pll_kp;
    float pll_ki;
    float pll_integrator;
    // PLL angle state
    float theta;
} smo_state_t;

void smo_init(smo_state_t *s, float flux_linkage);

void smo_step(smo_state_t *s,
              float v_alpha_prev, float v_beta_prev,
              float i_alpha, float i_beta,
              float Rs, float Ls,
              float dt,
              float *theta_out, float *omega_out);

#endif // SMO_H
