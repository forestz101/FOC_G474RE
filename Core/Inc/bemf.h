#ifndef BEMF_H
#define BEMF_H

#include <stdint.h>

typedef struct {
    float i_alpha_prev;
    float i_beta_prev;
    float e_alpha_lpf;
    float e_beta_lpf;
    float lpf_alpha;
} bemf_state_t;

void bemf_init(bemf_state_t *b);

void bemf_step(bemf_state_t *b,
               float v_alpha_prev, float v_beta_prev,
               float i_alpha, float i_beta,
               float Rs, float Ls, float flux_linkage,
               float dt,
               float *theta_out, float *omega_out);

#endif // BEMF_H
