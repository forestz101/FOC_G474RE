#include "foc.h"
#include <math.h>

#include "hrtim.h"

void GetPhaseCurrents(PhaseCurrents_t *I, uint32_t current_data[2], uint32_t reference_data[2])
{
    // Convert ADC readings to voltages
    float VrefA   = reference_data[0] * ADC_SCALE;
    float VrefC   = reference_data[1] * ADC_SCALE;

    float VsenseA = current_data[0] * ADC_SCALE;
    float VsenseC = current_data[1] * ADC_SCALE;

    // Compute currents
    float Ia = (VsenseA - VrefA) / CC6922SG_SENS;
    float Ic = (VsenseC - VrefC) / CC6922SG_SENS;

    // Fix sign based on wiring orientation
    Ia = -Ia;   // IP− → motor
    // Ic stays positive (IP+ → motor)

    // Clarke transform uses Ia and Ib, so compute Ib
    float Ib = -(Ia + Ic);

    I->Ia = Ia;
    I->Ib = Ib;
}

// -------------------------
// Clarke Transform
// -------------------------
AlphaBeta_t Clarke(const PhaseCurrents_t I)
{
    AlphaBeta_t out;
    out.Ialpha = I.Ia;
    out.Ibeta  = (I.Ia + 2.0f * I.Ib) * 0.57735026919f; // 1/sqrt(3)
    return out;
}

// -------------------------
// Park Transform
// -------------------------
void Park(AlphaBeta_t Iab, float angle, float *Id, float *Iq)
{
    float s = sinf(angle);
    float c = cosf(angle);

    *Id =  Iab.Ialpha * c + Iab.Ibeta * s;
    *Iq = -Iab.Ialpha * s + Iab.Ibeta * c;
}

// -------------------------
// Inverse Park
// -------------------------
VoltAlphaBeta_t InvPark(float Ud, float Uq, float angle)
{
    VoltAlphaBeta_t out;

    float s = sinf(angle);
    float c = cosf(angle);

    out.Valpha = Ud * c - Uq * s;
    out.Vbeta  = Ud * s + Uq * c;

    return out;
}

// -------------------------
// SVPWM
// -------------------------
DutyABC_t SVPWM(VoltAlphaBeta_t v, float Vbus)
{
    DutyABC_t d;

    float X = v.Vbeta;
    float Y = (0.8660254f * v.Valpha - 0.5f * v.Vbeta);
    float Z = (-0.8660254f * v.Valpha - 0.5f * v.Vbeta);

    float Tmax = fmaxf(fmaxf(X, Y), Z);
    float Tmin = fminf(fminf(X, Y), Z);

    float offset = 0.5f * (Tmax + Tmin);

    d.Ta = (X - offset) / Vbus + 0.5f;
    d.Tb = (Y - offset) / Vbus + 0.5f;
    d.Tc = (Z - offset) / Vbus + 0.5f;

    return d;
}

// -------------------------
// PI Controller
// -------------------------
float PI_Run(PI_t *pi, float error, float dt)
{
    pi->integrator += pi->Ki * error * dt;

    if (pi->integrator > pi->out_max) pi->integrator = pi->out_max;
    if (pi->integrator < pi->out_min) pi->integrator = pi->out_min;

    float out = pi->Kp * error + pi->integrator;

    if (out > pi->out_max) out = pi->out_max;
    if (out < pi->out_min) out = pi->out_min;

    return out;
}

// -------------------------
// Main FOC Step
// -------------------------
void FOC_Step(
    PhaseCurrents_t I,
    float Vbus,
    AngleProvider_t getAngle,
    PI_t *pi_d,
    PI_t *pi_q,
    FOC_Commands_t *cmd,
    float dt)
{
    float angle = getAngle();

    AlphaBeta_t Iab = Clarke(I);

    float Id, Iq;
    Park(Iab, angle, &Id, &Iq);

    float err_d = cmd->Id_ref - Id;
    float err_q = cmd->Iq_ref - Iq;

    cmd->Ud = PI_Run(pi_d, err_d, dt);
    cmd->Uq = PI_Run(pi_q, err_q, dt);

    VoltAlphaBeta_t Vab = InvPark(cmd->Ud, cmd->Uq, angle);

    DutyABC_t d = SVPWM(Vab, Vbus);

    // Hardware write is done outside this module
    // (HRTIM, TIM1, etc.)
}

DutyABC_t open_loop_step(float Ud, float Uq, float angle, float Vbus)
{
    VoltAlphaBeta_t Vab = InvPark(Ud, Uq, angle);

    DutyABC_t d = SVPWM(Vab, Vbus);

    return d;
}

void write_duty(const DutyABC_t duty)
{
    uint16_t counta = duty.Ta * 59999;
    uint16_t countb = duty.Tb * 59999;
    uint16_t countc = duty.Tc * 59999;

    __HAL_HRTIM_SETCOMPARE(&hhrtim1,
                      HRTIM_TIMERINDEX_TIMER_B,
                      HRTIM_COMPAREUNIT_1,
                      counta);
    __HAL_HRTIM_SETCOMPARE(&hhrtim1,
                      HRTIM_TIMERINDEX_TIMER_A,
                      HRTIM_COMPAREUNIT_1,
                      countb);
    __HAL_HRTIM_SETCOMPARE(&hhrtim1,
                      HRTIM_TIMERINDEX_TIMER_E,
                      HRTIM_COMPAREUNIT_1,
                      countc);
}
