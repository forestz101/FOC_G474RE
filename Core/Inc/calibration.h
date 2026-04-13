//
// Created by Forest on 2026-04-09.
//

#ifndef FOC_G474RE_CALIBRATION_H
#define FOC_G474RE_CALIBRATION_H

#pragma once
#include <stdint.h>

void calibrate(float Ud, float Vbus);
void calibrate_offset(float Ud, uint16_t steps, uint16_t delay);
void calibrate_clear(void);

#endif //FOC_G474RE_CALIBRATION_H
