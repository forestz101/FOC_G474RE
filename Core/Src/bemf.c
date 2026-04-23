#include "bemf.h"
#include "math_defs.h"
#include <math.h>

void bemf_init(bemf_state_t *b)
{
    b->i_alpha_prev = 0.0f;
    b->i_beta_prev  = 0.0f;
    b->e_alpha_lpf  = 0.0f;
    b->e_beta_lpf   = 0.0f;
    b->lpf_alpha    = 0.05f;
}

void bemf_step(bemf_state_t *b,
               float v_alpha_prev, float v_beta_prev,
               float i_alpha, float i_beta,
               float Rs, float Ls, float flux_linkage,
               float dt,
               float *theta_out, float *omega_out)
{
    // Finite-difference current derivative
    float di_alpha = (i_alpha - b->i_alpha_prev) / dt;
    float di_beta  = (i_beta  - b->i_beta_prev)  / dt;

    // Voltage equation: e = v - R*i - L*di/dt
    float e_alpha_raw = v_alpha_prev - Rs * i_alpha - Ls * di_alpha;
    float e_beta_raw  = v_beta_prev  - Rs * i_beta  - Ls * di_beta;

    // Low-pass filter
    b->e_alpha_lpf = b->lpf_alpha * e_alpha_raw + (1.0f - b->lpf_alpha) * b->e_alpha_lpf;
    b->e_beta_lpf  = b->lpf_alpha * e_beta_raw  + (1.0f - b->lpf_alpha) * b->e_beta_lpf;

    // Angle: e_alpha = -flux * omega * sin(theta), e_beta = flux * omega * cos(theta)
    float theta = atan2f(-b->e_alpha_lpf, b->e_beta_lpf);
    if (theta < 0.0f)
        theta += PI2_F;
    *theta_out = theta;

    // Speed from BEMF magnitude
    float e_mag = sqrtf(b->e_alpha_lpf * b->e_alpha_lpf +
                        b->e_beta_lpf  * b->e_beta_lpf);
    *omega_out = e_mag / flux_linkage;

    // Store for next cycle
    b->i_alpha_prev = i_alpha;
    b->i_beta_prev  = i_beta;
}
