#include "foc.h"
#include <math.h>
#include "hrtim.h"

motor_controller_t mc = {
    .vbus          = 30.0f,
    .mode          = MC_DISABLED,
    .el_angle      = 0.0f,
    .current_scale = ADC_SCALE / CC6922SG_SENS,
};

static float svpwm_coeffs[6][4] = {0}; // SVPWM coeffs, computed at init

// PhaseCurrents_t phase_currents;
volatile uint16_t current_buf[2] = {0}; // [CSA, CSC]
volatile uint16_t current_ref_buf[2] = {0}; // [CSA REF, CSC REF]

static void compute_svpwm_coeffs(void)
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

void mc_init(void)
{
    const float vmax = mc.vbus * INVERSE_SQRT3_F;

    mc.pi_d.kp      = 1.0f;
    mc.pi_d.ki      = 200.0f;
    mc.pi_d.out_min = -vmax;
    mc.pi_d.out_max =  vmax;

    mc.pi_q.kp      = 1.0f;
    mc.pi_q.ki      = 200.0f;
    mc.pi_q.out_min = -vmax;
    mc.pi_q.out_max =  vmax;

    mc.cmd.id_ref = 0.0f;
    mc.cmd.iq_ref = 0.0f;

    compute_svpwm_coeffs();
    observer_init(&mc.observer, MOTOR_RS, MOTOR_LS, MOTOR_FLUX_LINKAGE);
}

static void GetPhaseCurrents(void)
{
    const int16_t a_diff = (int16_t)(current_buf[0] - current_ref_buf[0]);
    const int16_t c_diff = (int16_t)(current_buf[1] - current_ref_buf[1]);

    const float alpha = 0.5f;
    mc.phase_currents.i_a += alpha * ((float)a_diff * mc.current_scale - mc.phase_currents.i_a);
    mc.phase_currents.i_c += alpha * ((float)c_diff * mc.current_scale * -1.0f - mc.phase_currents.i_c);    // C-phase sensor polarity reversed

    // mc.phase_currents.i_a = (float)a_diff * mc.current_scale;
    // mc.phase_currents.i_c = (float)c_diff * mc.current_scale * -1.0f;    // C-phase sensor polarity reversed

    mc.phase_currents.i_b = -(mc.phase_currents.i_a + mc.phase_currents.i_c);
}

static void CalculateFOCTrig(void)
{
    mc.trig.sin_theta = sinf(mc.el_angle);
    mc.trig.cos_theta = cosf(mc.el_angle);
}

static void ClarkeTransform(void)
{
    mc.alpha_beta.i_alpha = mc.phase_currents.i_a;
    mc.alpha_beta.i_beta  = mc.phase_currents.i_a * INVERSE_SQRT3_F
                          + mc.phase_currents.i_b * INVERSE_2SQRT3_F;
}

static void ParkTransform(void)
{
    mc.dq.id =  mc.alpha_beta.i_alpha * mc.trig.cos_theta
              + mc.alpha_beta.i_beta  * mc.trig.sin_theta;
    mc.dq.iq = -mc.alpha_beta.i_alpha * mc.trig.sin_theta
              + mc.alpha_beta.i_beta  * mc.trig.cos_theta;
}

static void InversePark(void)
{
    mc.volt_ab.v_alpha = mc.cmd.ud * mc.trig.cos_theta
                       - mc.cmd.uq * mc.trig.sin_theta;
    mc.volt_ab.v_beta  = mc.cmd.ud * mc.trig.sin_theta
                       + mc.cmd.uq * mc.trig.cos_theta;

    // Clamp to circle: |V| <= Vbus/sqrt(3)
    const float vmax = mc.vbus * INVERSE_SQRT3_F;
    const float mag_sq = mc.volt_ab.v_alpha * mc.volt_ab.v_alpha
                       + mc.volt_ab.v_beta  * mc.volt_ab.v_beta;
    const float vmax_sq = vmax * vmax;

    if (mag_sq > vmax_sq) {
        // Fast inverse sqrt approximation (Quake algorithm)
        float inv_mag = 1.0f / sqrtf(mag_sq);
        mc.volt_ab.v_alpha *= vmax * inv_mag;
        mc.volt_ab.v_beta  *= vmax * inv_mag;
    }
}

static void InverseClarke(void)
{
    mc.v_abc.v_a = mc.volt_ab.v_alpha;
    mc.v_abc.v_b = (-mc.volt_ab.v_alpha + SQRT3_F * mc.volt_ab.v_beta) / 2.0f;
    mc.v_abc.v_c = (-mc.volt_ab.v_alpha - SQRT3_F * mc.volt_ab.v_beta) / 2.0f;
}

static void svpwm(void)
{
    // Convert alpha-beta to normalized reference vector (relative to Vdc/2)
    const float vbus_half = mc.vbus * 0.5f;
    const float a = mc.volt_ab.v_alpha / vbus_half;
    const float b = mc.volt_ab.v_beta  / vbus_half;

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

    mc.duty.d_a = Ta_frac;
    mc.duty.d_b = Tb_frac;
    mc.duty.d_c = Tc_frac;
}

static void ComputePWMDuty(void)
{
    mc.duty.d_a = mc.v_abc.v_a / mc.vbus + 0.5f;
    mc.duty.d_b = mc.v_abc.v_b / mc.vbus + 0.5f;
    mc.duty.d_c = mc.v_abc.v_c / mc.vbus + 0.5f;

    if (mc.duty.d_a < 0.01f) mc.duty.d_a = 0.005f;
    if (mc.duty.d_b < 0.01f) mc.duty.d_b = 0.005f;
    if (mc.duty.d_c < 0.01f) mc.duty.d_c = 0.005f;

    if (mc.duty.d_a > 0.99f) mc.duty.d_a = 0.995f;
    if (mc.duty.d_b > 0.99f) mc.duty.d_b = 0.995f;
    if (mc.duty.d_c > 0.99f) mc.duty.d_c = 0.995f;
}

static void WriteDuty(void)
{
    uint16_t a_counts = (uint16_t)(mc.duty.d_a * PWM_PERIOD);
    uint16_t b_counts = (uint16_t)(mc.duty.d_b * PWM_PERIOD);
    uint16_t c_counts = (uint16_t)(mc.duty.d_c * PWM_PERIOD);

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

float pi_run(pi_t *pi, float error, float dt)
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

void foc_step(const float dt, float encoder_el_angle)
{
    GetPhaseCurrents();
    ClarkeTransform();

    observer_step(&mc.observer,
                  mc.alpha_beta.i_alpha, mc.alpha_beta.i_beta,
                  encoder_el_angle, dt);
    mc.el_angle = mc.observer.theta_est;

    CalculateFOCTrig();
    ParkTransform();

    const float err_d = mc.cmd.id_ref - mc.dq.id;
    const float err_q = mc.cmd.iq_ref - mc.dq.iq;

    mc.cmd.ud = pi_run(&mc.pi_d, err_d, dt);
    mc.cmd.uq = pi_run(&mc.pi_q, err_q, dt);

    InversePark();

    observer_update_voltage(&mc.observer,
                            mc.volt_ab.v_alpha, mc.volt_ab.v_beta);

    // InverseClarke();
    // ComputePWMDuty();
    svpwm();
    WriteDuty();
}

void open_loop_step(void)
{
    CalculateFOCTrig();
    InversePark();
    // InverseClarke();
    // ComputePWMDuty();
    svpwm();
    WriteDuty();
}
