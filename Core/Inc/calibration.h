//
// Created by Forest on 2026-04-09.
//

#ifndef FOC_G474RE_CALIBRATION_H
#define FOC_G474RE_CALIBRATION_H

#pragma once
#include <stdint.h>

uint16_t calibrate_single_electrical_rev(float Ud, float Vbus, int steps);

#endif //FOC_G474RE_CALIBRATION_H
