// Estimate current vector, compare to real current vector,
// the correction needed to keep them matched IS the BEMF
// LPF that, feed into PLL. Then get the angle

#include "smo.h"
#include "math_defs.h"
#include <math.h>

static inline float sigmoid(float x, float gain)
{
    return x / (fabsf(x) + (1.0f / gain));
}

void smo_init(smo_state_t *s, float flux_linkage)
{
    s->i_alpha_hat   = 0.0f;
    s->i_beta_hat    = 0.0f;
    s->e_alpha_hat   = 0.0f;
    s->e_beta_hat    = 0.0f;
    s->gain          = 20.0f * flux_linkage;
    s->lpf_alpha     = 0.02f;
    s->sigmoid_gain  = 100.0f;
    s->pll_kp        = 400.0f;
    s->pll_ki        = 40000.0f;
    s->pll_integrator = 0.0f;
    s->theta         = 0.0f;
}

void smo_step(smo_state_t *s,
              float v_alpha_prev, float v_beta_prev,
              float i_alpha, float i_beta,
              float Rs, float Ls,
              float dt,
              float *theta_out, float *omega_out)
{
    // Current estimation error
    float err_alpha = s->i_alpha_hat - i_alpha;
    float err_beta  = s->i_beta_hat  - i_beta;

    // Sliding mode switching signals
    float z_alpha = s->gain * sigmoid(err_alpha, s->sigmoid_gain);
    float z_beta  = s->gain * sigmoid(err_beta,  s->sigmoid_gain);

    // Current observer (Euler integration)
    float di_alpha_hat = (v_alpha_prev - Rs * s->i_alpha_hat
                          - s->e_alpha_hat - z_alpha) / Ls;
    float di_beta_hat  = (v_beta_prev  - Rs * s->i_beta_hat
                          - s->e_beta_hat  - z_beta)  / Ls;

    s->i_alpha_hat += di_alpha_hat * dt;
    s->i_beta_hat  += di_beta_hat  * dt;
    s->i_alpha_hat = clampf(s->i_alpha_hat, -200.0f, 200.0f);
    s->i_beta_hat  = clampf(s->i_beta_hat,  -200.0f, 200.0f);

    // Extract BEMF from sliding signals via LPF
    s->e_alpha_hat = s->lpf_alpha * z_alpha + (1.0f - s->lpf_alpha) * s->e_alpha_hat;
    s->e_beta_hat  = s->lpf_alpha * z_beta  + (1.0f - s->lpf_alpha) * s->e_beta_hat;

    // PLL angle tracking
    float sin_th = sinf(s->theta);
    float cos_th = cosf(s->theta);
    float pll_error = -s->e_alpha_hat * cos_th - s->e_beta_hat * sin_th;

    s->pll_integrator += s->pll_ki * pll_error * dt;
    s->pll_integrator = clampf(s->pll_integrator, -5000.0f, 5000.0f);
    float omega = s->pll_integrator + s->pll_kp * pll_error;
    s->theta += omega * dt;
    s->theta = wrap_angle(s->theta);

    *theta_out = s->theta;
    *omega_out = omega;
}
