#include "foc.h"
#include <math.h>
#include "hrtim.h"

float vbus = 30;
float vmax;
float el_angle = 0;
float current_scale = ADC_SCALE / CC6922SG_SENS;
float svpwm_coeffs[6][4] = {0}; // SVPWM coeffs, computed at init
uint8_t foc_enable = 0;

foc_commands_t foc_cmd = {
    .id_ref = 0.0f,
    .iq_ref = 0.0f,
    .ud = 0.0f,
    .uq = 0.0f
};

pi_t pi_d = {
    .kp = 1.0f,
    .ki = 200.0f,
    .integrator = 0.0f,
    .out_min = 0.0f,    // set in init
    .out_max = 0.0f     // set in init
};

pi_t pi_q = {
    .kp = 1.0f,
    .ki = 200.0f,
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

void FOCInit()
{
    vmax = vbus * INVERSE_SQRT3_F;

    pi_d.out_min = -vmax;
    pi_d.out_max =  vmax;

    pi_q.out_min = -vmax;
    pi_q.out_max =  vmax;

    foc_cmd.id_ref = 0.0f;
    foc_cmd.iq_ref = 0.0f;

    ComputeSVPWMCoeffs();
}

void ComputeSVPWMCoeffs(void)
{
    // SVPWM: compute coeffs variable coeffs[6][4]
    // Each coeffs[s][0..3] = { A, B, C, D } where
    // A = 0.5 * cos(phi1), B = 0.5 * sin(phi1)
    // C = 0.5 * cos(phi2), D = 0.5 * sin(phi2)
    // phi1,phi2 are the two active vector angles for sector s (s = 0..5 -> sectors 1..6)

    // Call once at startup (or when Ts changes). coeffs_out must be a 6x4 float array.
    // Stored as fractions of Ts (half-angle factors).

    // cos and sin for the six active vector angles: 0, 60, 120, 180, 240, 300 degrees
    const float cosv[6] = {  1.0f,  0.5f, -0.5f, -1.0f, -0.5f,  0.5f };
    const float sinv[6] = {  0.0f,  HALF_SQRT3_F, HALF_SQRT3_F, 0.0f, -HALF_SQRT3_F, -HALF_SQRT3_F };

    for (int s = 0; s < 6; ++s) {
        const float A = 0.5f * cosv[s];
        const float B = 0.5f * sinv[s];
        const float C = 0.5f * cosv[(s + 1) % 6];
        const float D = 0.5f * sinv[(s + 1) % 6];

        svpwm_coeffs[s][0] = A;
        svpwm_coeffs[s][1] = B;
        svpwm_coeffs[s][2] = C;
        svpwm_coeffs[s][3] = D;
    }
}

void GetPhaseCurrents()
{
    const int16_t a_diff = (int16_t)(current_buf[0] - current_ref_buf[0]);
    const int16_t c_diff = (int16_t)(current_buf[1] - current_ref_buf[1]);

    const float alpha = 0.5f;
    phase_currents.i_a += alpha * ((float)a_diff * current_scale - phase_currents.i_a);
    phase_currents.i_c += alpha * ((float)c_diff * current_scale * -1 - phase_currents.i_c);    // C-phase sensor polarity reversed

    // phase_currents.i_a = (float)a_diff * current_scale;
    // phase_currents.i_c = (float)c_diff * current_scale * -1;    // C-phase sensor polarity reversed

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
    dq.id = alpha_beta.i_alpha * foc_trig.cos_theta + alpha_beta.i_beta * foc_trig.sin_theta;
    dq.iq = -alpha_beta.i_alpha * foc_trig.sin_theta + alpha_beta.i_beta * foc_trig.cos_theta;
}

void InversePark()
{
    volt_ab.v_alpha = foc_cmd.ud * foc_trig.cos_theta - foc_cmd.uq * foc_trig.sin_theta;
    volt_ab.v_beta  = foc_cmd.ud * foc_trig.sin_theta + foc_cmd.uq * foc_trig.cos_theta;

    // Clamp to circle: |V| <= Vbus/sqrt(3)
    const float mag_sq = volt_ab.v_alpha * volt_ab.v_alpha + volt_ab.v_beta * volt_ab.v_beta;
    const float vmax_sq = vmax * vmax;

    if (mag_sq > vmax_sq) {
        // Fast inverse sqrt approximation (Quake algorithm)
        float inv_mag = 1.0f / sqrtf(mag_sq);
        volt_ab.v_alpha *= vmax * inv_mag;
        volt_ab.v_beta *= vmax * inv_mag;
    }
}

void InverseClarke()
{
    v_abc.v_a = volt_ab.v_alpha;
    v_abc.v_b = (-volt_ab.v_alpha + SQRT3_F * volt_ab.v_beta) / 2;
    v_abc.v_c = (-volt_ab.v_alpha - SQRT3_F * volt_ab.v_beta) / 2;
}

void SVPWM()
{
    // Convert alpha-beta to normalized reference vector (relative to Vdc/2)
    const float vbus_half = vbus * 0.5f;
    const float a = volt_ab.v_alpha / vbus_half;
    const float b = volt_ab.v_beta  / vbus_half;

    // sector detection (projections)
    float wX = b;
    float wY = 0.5f * (b + a);
    float wZ = 0.5f * (b - a);

    uint8_t sector;
    if (wY < 0) {
        if (wZ < 0) sector = 5;
        else if (wX <= 0) sector = 4;
        else sector = 3;
    } else {
        if (wZ >= 0) sector = 2;
        else if (wX <= 0) sector = 6;
        else sector = 1;
    }

    // T1, T2 as fractions of the PWM period using precomputed coeffs
    const float *c = svpwm_coeffs[sector - 1]; // c[0]=A_f, c[1]=B_f, c[2]=C_f, c[3]=D_f
    float T1_frac = c[0] * a + c[1] * b;
    float T2_frac = c[2] * a + c[3] * b;

    // clamp and ensure T0 >= 0
    if (T1_frac < 0.0f) T1_frac = 0.0f;
    if (T2_frac < 0.0f) T2_frac = 0.0f;
    if (T1_frac + T2_frac > 1.0f) {
        float s = 1.0f / (T1_frac + T2_frac);
        T1_frac *= s;
        T2_frac *= s;
    }

    float T0_frac = 1.0f - T1_frac - T2_frac;
    if (T0_frac < 0.0f) T0_frac = 0.0f;
    const float halfT0 = 0.5f * T0_frac;

    // per-sector mapping to Ta,Tb,Tc (fractions of full period)
    float Ta_frac, Tb_frac, Tc_frac;
    switch (sector) {
    case 1:
        Ta_frac = halfT0 + T1_frac + T2_frac;
        Tb_frac = halfT0 + T2_frac;
        Tc_frac = halfT0;
        break;
    case 2:
        Ta_frac = halfT0 + T1_frac;
        Tb_frac = halfT0 + T1_frac + T2_frac;
        Tc_frac = halfT0;
        break;
    case 3:
        Ta_frac = halfT0;
        Tb_frac = halfT0 + T1_frac + T2_frac;
        Tc_frac = halfT0 + T2_frac;
        break;
    case 4:
        Ta_frac = halfT0;
        Tb_frac = halfT0 + T1_frac;
        Tc_frac = halfT0 + T1_frac + T2_frac;
        break;
    case 5:
        Ta_frac = halfT0 + T2_frac;
        Tb_frac = halfT0;
        Tc_frac = halfT0 + T1_frac + T2_frac;
        break;
    default: // sector 6
        Ta_frac = halfT0 + T1_frac + T2_frac;
        Tb_frac = halfT0;
        Tc_frac = halfT0 + T1_frac;
        break;
    }

    duty.d_a = Ta_frac;
    duty.d_b = Tb_frac;
    duty.d_c = Tc_frac;
}

void ComputePWMDuty()
{
    duty.d_a = v_abc.v_a / vbus + 0.5f;
    duty.d_b = v_abc.v_b / vbus + 0.5f;
    duty.d_c = v_abc.v_c / vbus + 0.5f;

    if (duty.d_a < 0.01f) duty.d_a = 0.005f;
    if (duty.d_b < 0.01f) duty.d_b = 0.005f;
    if (duty.d_c < 0.01f) duty.d_c = 0.005f;

    if (duty.d_a > 0.99f) duty.d_a = 0.995f;
    if (duty.d_b > 0.99f) duty.d_b = 0.995f;
    if (duty.d_c > 0.99f) duty.d_c = 0.995f;
}

void WriteDuty()
{
    uint16_t a_counts = (uint16_t)(duty.d_a * PWM_PERIOD);
    uint16_t b_counts = (uint16_t)(duty.d_b * PWM_PERIOD);
    uint16_t c_counts = (uint16_t)(duty.d_c * PWM_PERIOD);

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

float ComputePI(pi_t *pi, float error, float dt)
{
    const float P = pi->kp * error;
    const float ki_dt = pi->ki * dt;

    // dynamic integrator bounds so P + I stays within output limits
    const float I_max = pi->out_max - P;
    const float I_min = pi->out_min - P;

    // integrate then clamp to dynamic bounds (branchless clamp)
    float I = pi->integrator + ki_dt * error;
    if (I > I_max) I = I_max;
    else if (I < I_min) I = I_min;

    pi->integrator = I;

    // output (guaranteed within bounds by dynamic integrator clamp)
    float out = P + I;

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

    foc_cmd.ud = ComputePI(&pi_d, err_d, dt);
    foc_cmd.uq = ComputePI(&pi_q, err_q, dt);

    InversePark();
    // InverseClarke();
    // ComputePWMDuty();
    SVPWM();
    WriteDuty();
}

void OpenLoopStep()
{
    CalculateFOCTrig();
    InversePark();
    // InverseClarke();
    // ComputePWMDuty();
    SVPWM();
    WriteDuty();
}
