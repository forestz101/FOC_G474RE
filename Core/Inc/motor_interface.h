/**
  ******************************************************************************
  * @file           : motor_interface.h
  * @brief          : Header for motor interface utilities
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

#ifndef __MOTOR_INTERFACE_H__
#define __MOTOR_INTERFACE_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usart.h"

#define ENCODER_BITS     14
#define ENCODER_COUNTS   (1U << ENCODER_BITS)   // 8192
#define ENCODER_MASK     (ENCODER_COUNTS - 1U)  // 0x1FFF
#define WS22_DMA_BUF_SIZE 128  // power of 2 (e.g., 64, 128, 256)
#define LUT_SIZE 64
#define POLE_PAIRS 1
#define CAL_SAMPLES (LUT_SIZE * POLE_PAIRS)
#define CALIB_FLASH_ADDR  0x0807F800  // last flash page G474RE

typedef struct {
  uint16_t position;
  uint16_t temperature;
  uint8_t  device_type;
  uint8_t  valid;
} MotorSensorData;

extern volatile float commanded_position_deg;

void motor_interface_init(void);
uint16_t motor_interface_get_position(void);
uint16_t motor_interface_get_position_raw(void);
MotorSensorData motor_interface_get_data(void);
void motor_interface_set_offset(uint16_t off);
void motor_interface_set_reverse(uint8_t rev);
void motor_interface_set_lut(const int16_t *lut);
void motor_interface_load_calibration(void);
void motor_interface_save_calibration(void);

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_INTERFACE_H__ */
