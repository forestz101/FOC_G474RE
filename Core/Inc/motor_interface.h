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

// typedef struct {
//   uint16_t position;
//   uint16_t temperature;
//   uint8_t device_type;
//   uint8_t message_type;  // 0=device type packet, 1=temp+position packet
//   uint8_t valid;         // 1 if data is valid, 0 if not
// } MotorSensorData;
//
// void motor_interface_init(void);
// uint8_t *get_raw_uart(void);
// MotorSensorData motor_interface_get_data(void);
// void motor_interface_process_rx_callback(UART_HandleTypeDef *huart);
#define ENCODER_BITS     13
#define ENCODER_COUNTS   (1U << ENCODER_BITS)   // 8192
#define ENCODER_MASK     (ENCODER_COUNTS - 1U)  // 0x1FFF

typedef struct {
  uint16_t position;
  uint16_t temperature;
  uint8_t  device_type;
  uint8_t  valid;
} MotorSensorData;

void motor_interface_init(void);
uint16_t motor_interface_get_position(void);
MotorSensorData motor_interface_get_data(void);
void motor_interface_set_offset(uint16_t off);
void motor_interface_set_reverse(uint8_t rev);

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_INTERFACE_H__ */
