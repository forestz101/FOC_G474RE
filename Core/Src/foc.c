#include "foc.h"
#include <math.h>
#include "hrtim.h"

float vbus = 20;
float el_angle = 0;
float current_scale = ADC_SCALE / CC6922SG_SENS;
uint8_t foc_enable = 0;

foc_commands_t foc_cmd = {
    .id_ref = 0.0f,
    .iq_ref = 0.0f,
    .ud = 0.0f,
    .uq = 0.0f
};

pi_t pi_d = {
    .kp = 3.0f,
    .ki = 0.3f,
    .integrator = 0.0f,
    .out_min = 0.0f,    // set in init
    .out_max = 0.0f     // set in init
};

pi_t pi_q = {
    .kp = 3.0f,
    .ki = 0.3f,
    .integrator = 0.0f,
    .out_min = 0.0f,    // set in init
    .out_max = 0.0f     // set in init
};

phase_currents_t phase_currents = {
    .i_a = 0.0f,
    .i_b = 0.0f,
    .i_c = 0.0f
};

foc_trig_t foc_trig = {
    .sin_theta = 0.0f,
    .cos_theta = 0.0f
};

volt_alpha_beta_t volt_ab = {
    .v_alpha = 0.0f,
    .v_beta = 0.0f
};

alpha_beta_t alpha_beta = {
    .i_alpha = 0.0f,
    .i_beta = 0.0f
};

direct_quadrature_t dq = {
    .id = 0.0f,
    .iq = 0.0f
};

v_abc_t v_abc = {
    .v_a = 0.0f,
    .v_b = 0.0f,
    .v_c = 0.0f
};

duty_t duty = {
    .d_a = 0.0f,
    .d_b = 0.0f,
    .d_c = 0.0f
};


// PhaseCurrents_t phase_currents;
volatile uint16_t current_buf[2] = {0}; // [CSA, CSC]
volatile uint16_t current_ref_buf[2] = {0}; // [CSA REF, CSC REF]

void foc_init()
{
    const float vmax = vbus / 2;
    pi_d.out_min = -vmax;
    pi_d.out_max =  vmax;

    pi_q.out_min = -vmax;
    pi_q.out_max =  vmax;

    foc_cmd.id_ref = 0.0f;
    foc_cmd.iq_ref = .0f;
}

void GetPhaseCurrents()
{
    const int16_t a_diff = (int16_t)(current_buf[0] - current_ref_buf[0]);
    const int16_t c_diff = (int16_t)(current_buf[1] - current_ref_buf[1]);

    phase_currents.i_a = (float)a_diff * current_scale;
    phase_currents.i_c = (float)c_diff * current_scale * -1;    // C-phase sensor polarity reversed
    phase_currents.i_b = -(phase_currents.i_a + phase_currents.i_c);
}

void CalculateFOCTrig()
{
    foc_trig.sin_theta = sinf(el_angle);
    foc_trig.cos_theta = cosf(el_angle);
}

void ClarkeTransform()
{
    alpha_beta.i_alpha = phase_currents.i_a;
    alpha_beta.i_beta  = phase_currents.i_a * INVERSE_SQRT3_F + phase_currents.i_b * INVERSE_2SQRT3_F;
}

void ParkTransform()
{
    dq.id =  alpha_beta.i_alpha * foc_trig.cos_theta + alpha_beta.i_beta * foc_trig.sin_theta;
    dq.iq = -alpha_beta.i_alpha * foc_trig.sin_theta + alpha_beta.i_beta * foc_trig.cos_theta;
}

void InversePark()
{
    volt_ab.v_alpha = foc_cmd.ud * foc_trig.cos_theta - foc_cmd.uq * foc_trig.sin_theta;
    volt_ab.v_beta  = foc_cmd.ud * foc_trig.sin_theta + foc_cmd.uq * foc_trig.cos_theta;
}

void InverseClarke()
{
    v_abc.v_a = volt_ab.v_alpha;
    v_abc.v_b = (-volt_ab.v_alpha + SQRT3_F * volt_ab.v_beta) / 2;
    v_abc.v_c = (-volt_ab.v_alpha - SQRT3_F * volt_ab.v_beta) / 2;
}

void ComputePWMDuty()
{
    duty.d_a = v_abc.v_a / vbus + 0.5f;
    duty.d_b = v_abc.v_b / vbus + 0.5f;
    duty.d_c = v_abc.v_c / vbus + 0.5f;

    if (duty.d_a < 0.03f) duty.d_a = 0.03f;
    if (duty.d_b < 0.03f) duty.d_b = 0.03f;
    if (duty.d_c < 0.03f) duty.d_c = 0.03f;

    if (duty.d_a > 0.97f) duty.d_a = 0.97f;
    if (duty.d_b > 0.97f) duty.d_b = 0.97f;
    if (duty.d_c > 0.97f) duty.d_c = 0.97f;

}

void WriteDuty()
{
    const uint16_t a_counts = (uint16_t)(duty.d_a * PWM_PERIOD);
    const uint16_t b_counts = (uint16_t)(duty.d_b * PWM_PERIOD);
    const uint16_t c_counts = (uint16_t)(duty.d_c * PWM_PERIOD);

    __HAL_HRTIM_SETCOMPARE(&hhrtim1,
                      HRTIM_TIMERINDEX_TIMER_B,
                      HRTIM_COMPAREUNIT_1,
                      a_counts);
    __HAL_HRTIM_SETCOMPARE(&hhrtim1,
                      HRTIM_TIMERINDEX_TIMER_A,
                      HRTIM_COMPAREUNIT_1,
                      b_counts);
    __HAL_HRTIM_SETCOMPARE(&hhrtim1,
                      HRTIM_TIMERINDEX_TIMER_E,
                      HRTIM_COMPAREUNIT_1,
                      c_counts);
}

float PI_Run(pi_t *pi, float error, float dt)
{
    pi->integrator += pi->ki * error * dt;

    if (pi->integrator > pi->out_max) pi->integrator = pi->out_max;
    if (pi->integrator < pi->out_min) pi->integrator = pi->out_min;

    float out = pi->kp * error + pi->integrator;

    if (out > pi->out_max) out = pi->out_max;
    if (out < pi->out_min) out = pi->out_min;

    return out;
}

void FOC_Step(const float dt)
{
    // float angle = getAngle();
    GetPhaseCurrents();
    CalculateFOCTrig();
    ClarkeTransform();
    ParkTransform();

    const float err_d = foc_cmd.id_ref - dq.id;
    const float err_q = foc_cmd.iq_ref - dq.iq;

    foc_cmd.ud = PI_Run(&pi_d, err_d, dt);
    foc_cmd.uq = PI_Run(&pi_q, err_q, dt);

    InversePark();
    InverseClarke();
    ComputePWMDuty();
    WriteDuty();
}

void OpenLoopStep()
{
    CalculateFOCTrig();
    InversePark();
    InverseClarke();
    ComputePWMDuty();
    WriteDuty();
}
