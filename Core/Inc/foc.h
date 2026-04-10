//
// Created by Forest on 2026-02-12.
//

#define SQRT3_F 1.7320508075688772f
#define PI_F 3.141592653589793f

#define TIMERID_PHA HRTIM_TIMERID_TIMER_B
#define TIMERID_PHB HRTIM_TIMERID_TIMER_A
#define TIMERID_PHC HRTIM_TIMERID_TIMER_E

#define TIMEROUT_PHA HRTIM_OUTPUT_TB1
#define TIMEROUT_PHB HRTIM_OUTPUT_TA1
#define TIMEROUT_PHC HRTIM_OUTPUT_TE1

// ADC reference voltage
#define ADC_VREF     3.3f
#define ADC_SCALE    (ADC_VREF / 4095.0f)

// CC6922SG sensitivity
#define CC6922SG_SENS 0.0132f   // 13.2 mV/A

#include "main.h"
// #include "tim.h"
#ifndef FOC_H
#define FOC_H

#include <stdint.h>

typedef struct {
    float Ia;
    float Ib;
    float Ic;
} PhaseCurrents_t;

typedef struct {
    float Ialpha;
    float Ibeta;
} AlphaBeta_t;

typedef struct {
    float Valpha;
    float Vbeta;
} VoltAlphaBeta_t;

typedef struct {
    float Ta;
    float Tb;
    float Tc;
} DutyABC_t;

typedef struct {
    float Kp;
    float Ki;
    float integrator;
    float out_min;
    float out_max;
} PI_t;

typedef struct {
    float Id_ref;
    float Iq_ref;
    float Ud;
    float Uq;
} FOC_Commands_t;

// Angle provider function pointer
typedef float (*AngleProvider_t)(void);

// Public API
AlphaBeta_t Clarke(PhaseCurrents_t I);
void Park(AlphaBeta_t Iab, float angle, float *Id, float *Iq);
VoltAlphaBeta_t InvPark(float Ud, float Uq, float angle);
DutyABC_t SVPWM(VoltAlphaBeta_t v, float Vbus);
float getAngle();

float PI_Run(PI_t *pi, float error, float dt);

void FOC_Step(
    PhaseCurrents_t I,
    float Vbus,
    AngleProvider_t getAngle,
    PI_t *pi_d,
    PI_t *pi_q,
    FOC_Commands_t *cmd,
    float dt);

DutyABC_t open_loop_step(float Ud, float Uq, float angle, float Vbus);
void write_duty(const DutyABC_t duty);
#endif // F446_FOC_FOC_H
