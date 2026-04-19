#include "observer.h"
#include "math_defs.h"
#include <math.h>

void observer_init(observer_state_t *obs,
                   float Rs, float Ls, float flux_linkage)
{
    obs->theta_est       = 0.0f;
    obs->omega_est       = 0.0f;
    obs->Rs              = Rs;
    obs->Ls              = Ls;
    obs->flux_linkage    = flux_linkage;
    obs->v_alpha_prev    = 0.0f;
    obs->v_beta_prev     = 0.0f;
    obs->correction_gain = FUSION_GAIN_MAX;
    obs->warmup_cycles   = 0;

    smo_init(&obs->smo, flux_linkage);
}

void observer_step(observer_state_t *obs,
                   float i_alpha, float i_beta,
                   float encoder_el_angle, float dt)
{
    // Pure encoder until voltage history is valid
    if (obs->warmup_cycles < 2) {
        obs->warmup_cycles++;
        obs->theta_est = encoder_el_angle;
        obs->omega_est = 0.0f;
        return;
    }

    float raw_theta = 0.0f;
    float raw_omega = 0.0f;

    smo_step(&obs->smo,
             obs->v_alpha_prev, obs->v_beta_prev,
             i_alpha, i_beta,
             obs->Rs, obs->Ls,
             dt,
             &raw_theta, &raw_omega);

    obs->theta_est = raw_theta;
    obs->omega_est = raw_omega;

    // Speed-scheduled encoder correction gain
    float speed_abs = fabsf(obs->omega_est);
    if (speed_abs < FUSION_SPEED_LOW) {
        obs->correction_gain = FUSION_GAIN_MAX;
    } else if (speed_abs > FUSION_SPEED_HIGH) {
        obs->correction_gain = FUSION_GAIN_MIN;
    } else {
        float t = (speed_abs - FUSION_SPEED_LOW)
                / (FUSION_SPEED_HIGH - FUSION_SPEED_LOW);
        obs->correction_gain = FUSION_GAIN_MAX
                             + t * (FUSION_GAIN_MIN - FUSION_GAIN_MAX);
    }

    // Apply encoder correction (reject glitches > 30 deg)
    float error = wrap_to_pm_pi(encoder_el_angle - obs->theta_est);
    error = clampf(error, -0.52f, 0.52f);

    obs->theta_est += obs->correction_gain * error;
    obs->theta_est = wrap_angle(obs->theta_est);
}

void observer_update_voltage(observer_state_t *obs,
                             float v_alpha, float v_beta)
{
    obs->v_alpha_prev = v_alpha;
    obs->v_beta_prev  = v_beta;
}
