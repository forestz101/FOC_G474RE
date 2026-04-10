/**
  ******************************************************************************
  * @file           : motor_interface.c
  * @brief          : Motor interface utilities for WaveSculptor communication
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

/* Includes ------------------------------------------------------------------*/
#include "motor_interface.h"
#include <stdio.h>
#include "usart.h"
#include "dma.h"
#include "stm32g4xx_ll_usart.h"
#include "stm32g4xx_ll_dma.h"

// Bit masks and constants for WaveSculptor protocol
#define MASK_7B           0x7F  // 7-bit mask for data bytes (bits 0-6)
#define MASK_6B           0x3F  // 6-bit mask for device type/temperature MSB
#define SYNC_BIT_MASK     0x80  // Bit 7 set indicates start of packet
#define MSG_TYPE_MASK     0xC0  // Bits 7-6 contain message type
#define MSG_TYPE_DEVICE   0x80  // 0b10 << 6: Device type packet
#define MSG_TYPE_TEMP_POS 0xC0  // 0b11 << 6: Temperature + position packet

// // Private variables for UART handling
// static uint8_t rxBuf[4];
// static uint8_t rxByte;
// static uint32_t frameIndex = 0;  // Position in frame (0-3)
// static uint32_t syncErrors = 0;
// static uint32_t validPackets = 0;
// static uint32_t debugMode = 0;  // Debug mode flag - enabled by default
// static MotorSensorData currentData = {0};
#define WS22_DMA_BUF_SIZE 128  // power of 2 (e.g., 64, 128, 256)

static uint8_t  ws22_dma_buf[WS22_DMA_BUF_SIZE];
static MotorSensorData currentData = {0};
static uint16_t encoder_offset = 0;
static uint8_t reverse_direction = 0;

/**
  * @brief  Initialize motor interface
  * @retval None
  */
void motor_interface_init(void)
{
  // Start UART5 RX DMA in circular mode
  HAL_UART_Receive_DMA(&huart5, ws22_dma_buf, WS22_DMA_BUF_SIZE);

  // Disable DMA interrupts for this channel (adjust DMAx/CHy if needed)
  LL_DMA_DisableIT_HT(DMA1, LL_DMA_CHANNEL_3);
  LL_DMA_DisableIT_TC(DMA1, LL_DMA_CHANNEL_3);
  LL_DMA_DisableIT_TE(DMA1, LL_DMA_CHANNEL_3);
}

static inline uint32_t ws22_dma_write_index(void)
{
  uint32_t remaining = LL_DMA_GetDataLength(DMA1, LL_DMA_CHANNEL_2);
  return (WS22_DMA_BUF_SIZE - remaining) & (WS22_DMA_BUF_SIZE - 1);
}

static int find_last_sync(void)
{
  uint32_t w = ws22_dma_write_index();

  for (int i = 0; i < WS22_DMA_BUF_SIZE; i++)
  {
    uint32_t idx = (w + WS22_DMA_BUF_SIZE - 1 - i) & (WS22_DMA_BUF_SIZE - 1);

    uint8_t b0 = ws22_dma_buf[idx];

    // Sync byte: bit7 = 1
    if (b0 & 0x80)
    {
      uint8_t b1 = ws22_dma_buf[(idx + 1) & (WS22_DMA_BUF_SIZE - 1)];
      uint8_t b2 = ws22_dma_buf[(idx + 2) & (WS22_DMA_BUF_SIZE - 1)];
      uint8_t b3 = ws22_dma_buf[(idx + 3) & (WS22_DMA_BUF_SIZE - 1)];

      // Data bytes: bit7 = 0
      if (!(b1 & 0x80) && !(b2 & 0x80) && !(b3 & 0x80))
        return (int)idx;  // frame start
    }
  }

  return -1; // no valid frame found
}

static void ws22_parse_latest_frame(void)
{
  int idx = find_last_sync();
  if (idx < 0)
    return;

  uint8_t b0 = ws22_dma_buf[idx];
  uint8_t b1 = ws22_dma_buf[(idx + 1) & (WS22_DMA_BUF_SIZE - 1)];
  uint8_t b2 = ws22_dma_buf[(idx + 2) & (WS22_DMA_BUF_SIZE - 1)];
  uint8_t b3 = ws22_dma_buf[(idx + 3) & (WS22_DMA_BUF_SIZE - 1)];

  uint8_t msgType = (b0 >> 6) & 0x03;

  if (msgType == 0b10) {
    // Device type packet
    currentData.device_type = b0 & 0x3F;
    currentData.valid = 1;
  }
  else if (msgType == 0b11) {
    // Temperature + position packet
    currentData.temperature =
        ((uint16_t)(b0 & 0x3F) << 7) | (b1 & 0x7F);
    currentData.position =
        ((uint16_t)(b2 & 0x7F) << 7) | (b3 & 0x7F);
    currentData.valid = 1;
  }
}

void motor_interface_set_offset(uint16_t off)
{
  encoder_offset = off & ENCODER_MASK;
}

void motor_interface_set_reverse(uint8_t rev)
{
  reverse_direction = rev ? 1 : 0;
}


uint16_t motor_interface_get_position(void)
{
  ws22_parse_latest_frame();

  uint16_t raw = currentData.position & ENCODER_MASK;

  // 1. Apply direction reversal
  if (reverse_direction)
    raw = (ENCODER_COUNTS - raw) & ENCODER_MASK;

  // 2. Apply calibration offset
  raw = (raw - encoder_offset) & ENCODER_MASK;

  return raw;   // PROCESSED
}


MotorSensorData motor_interface_get_data(void)
{
  ws22_parse_latest_frame();
  return currentData;
}

//
// /**
//   * @brief  Get current motor sensor data
//   * @retval MotorSensorData structure with current readings
//   */
// MotorSensorData motor_interface_get_data(void)
// {
//   return currentData;
// }
//
// uint8_t *get_raw_uart(void) {
//   return rxBuf;
// }
//
// /**
//   * @brief  Process UART receive callback for motor interface
//   * @param  huart: UART handle
//   * @retval None
//   */
// void motor_interface_process_rx_callback(UART_HandleTypeDef *huart)
// {
//   // HAL_UART_Receive_IT(&huart5, rxBuf, 4);
//   //
//   // return ;
//
//   LL_GPIO_SetOutputPin(GPO1_GPIO_Port, GPO1_Pin);
//
//   if (huart->Instance == UART5)
//   {
//     if (frameIndex == 0)
//     {
//       // Looking for sync byte (bit 7 must be 1)
//       if ((rxByte & SYNC_BIT_MASK) != 0)
//       {
//         // Found sync byte - start of packet
//         rxBuf[0] = rxByte;
//         frameIndex = 1;
//       }
//       else
//       {
//         // Not a sync byte, keep looking
//         syncErrors++;
//       }
//     }
//     else
//     {
//       // We're in the middle of a frame
//       // Bytes 2-4 should have bit 7 clear (0x00-0x7F)
//       if ((rxByte & SYNC_BIT_MASK) != 0)
//       {
//         // Found another sync byte - frame was incomplete, restart
//         syncErrors++;
//         rxBuf[0] = rxByte;
//         frameIndex = 1;
//       }
//       else
//       {
//         // Valid data byte
//         rxBuf[frameIndex] = rxByte;
//         frameIndex++;
//
//         if (frameIndex == 4)
//         {
//           // Complete frame received - parse it
//           uint8_t startByte = rxBuf[0];
//           uint8_t msgType = (startByte >> 6) & 0x03;  // Bits 7-6
//
//           if (msgType == 0b10)
//           {
//             // Device type packet
//             currentData.device_type = startByte & MASK_6B;
//             currentData.message_type = 0;
//             currentData.valid = 1;
//           }
//           else if (msgType == 0b11)
//           {
//             // Temperature + position packet
//             currentData.temperature = ((uint16_t)(rxBuf[0] & MASK_6B) << 7) | (rxBuf[1] & MASK_7B);
//             currentData.position = ((uint16_t)(rxBuf[2] & MASK_7B) << 7) | (rxBuf[3] & MASK_7B);
//             currentData.message_type = 1;
//             currentData.valid = 1;
//             // if (debugMode)
//             // {
//             //   printf("  -> TEMP+POS: Temp=%u | Pos=%u\r\n", currentData.temperature, currentData.position);
//             // }
//           }
//
//           validPackets++;
//           frameIndex = 0;  // Reset for next frame
//         }
//       }
//     }
//
//     // Continue receiving
//     HAL_UART_Receive_IT(&huart5, &rxByte, 1);
//   }
//   LL_GPIO_ResetOutputPin(GPO1_GPIO_Port, GPO1_Pin);
// }

