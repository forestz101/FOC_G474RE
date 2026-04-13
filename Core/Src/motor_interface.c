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
#include "foc.h"
#include "stm32g4xx_ll_usart.h"
#include "stm32g4xx_ll_dma.h"

static uint8_t  ws22_dma_buf[WS22_DMA_BUF_SIZE];
static MotorSensorData currentData = {0};
static uint16_t encoder_offset = 0;
static uint8_t reverse_direction = 0;
static int16_t encoder_lut[LUT_SIZE] = {0};
static uint8_t lut_valid = 0;

#include "stm32g4xx_hal.h"


typedef struct {
  uint16_t offset;
  uint8_t  reverse;
  uint8_t  lut_valid;
  int16_t  lut[LUT_SIZE];
} EncoderCalData;

static int16_t encoder_lut[LUT_SIZE];

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

  motor_interface_load_calibration();
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

static void apply_calibration(const EncoderCalData *cal)
{
  encoder_offset   = cal->offset & ENCODER_MASK;
  reverse_direction = cal->reverse ? 1 : 0;
  lut_valid        = cal->lut_valid ? 1 : 0;

  // if (lut_valid) {
  //   for (int i = 0; i < LUT_SIZE; i++)
  //     encoder_lut[i] = cal->lut[i];
  // }
}

void motor_interface_set_offset(uint16_t off)
{
  encoder_offset = off & ENCODER_MASK;
}

void motor_interface_set_reverse(uint8_t rev)
{
  reverse_direction = rev ? 1 : 0;
}

void motor_interface_set_lut(const int16_t *lut)
{
  for (int i = 0; i < LUT_SIZE; i++)
    encoder_lut[i] = lut[i];

  lut_valid = 1;
}

void motor_interface_load_calibration(void)
{
  const EncoderCalData *cal = (const EncoderCalData *)CALIB_FLASH_ADDR;

  // Very simple validity check: lut_valid must be 0 or 1, reverse 0 or 1
  if (cal->lut_valid <= 1 && cal->reverse <= 1) {
    apply_calibration(cal);
  }
}

void motor_interface_save_calibration(void)
{
  EncoderCalData cal;

  if (encoder_offset != cal.offset)
  {
    printf("Saving calibration: offset = %u counts\r\n", encoder_offset);
    cal.offset = encoder_offset;
  }

  if (reverse_direction != cal.reverse)
  {
    printf("Saving calibration: reverse = %u\r\n", reverse_direction);
    cal.reverse = reverse_direction;
  }

  if (lut_valid)
  {
    printf("Saving calibration: LUT:\r\n");
    cal.lut_valid = lut_valid;

    for (int i = 0; i < LUT_SIZE; i++)
    {
      printf("  LUT[%d] = %+6d\r\n", i, encoder_lut[i]);
      cal.lut[i] = encoder_lut[i];
    }
  } else
  {
    cal.lut_valid = 0;
    printf("Saving calibration: lut_valid = %u\r\n", lut_valid);
  }

  HAL_FLASH_Unlock();

  // Erase one page
  FLASH_EraseInitTypeDef erase = {0};
  uint32_t pageError = 0;

  erase.TypeErase   = FLASH_TYPEERASE_PAGES;
  erase.Page        = (CALIB_FLASH_ADDR - FLASH_BASE) / FLASH_PAGE_SIZE;
  erase.NbPages     = 1;

  if (HAL_FLASHEx_Erase(&erase, &pageError) != HAL_OK) {
    HAL_FLASH_Lock();
    return;
  }

  // Program as 64-bit double words
  uint64_t *src = (uint64_t *)&cal;
  uint32_t addr = CALIB_FLASH_ADDR;
  uint32_t bytes = sizeof(EncoderCalData);

  for (uint32_t i = 0; i < (bytes + 7) / 8; i++) {
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,
                          addr,
                          src[i]) != HAL_OK) {
      break;
                          }
    addr += 8;
  }

  HAL_FLASH_Lock();
}

uint16_t motor_interface_get_position(void)
{
  ws22_parse_latest_frame();
  uint16_t raw = currentData.position & ENCODER_MASK;

  if (reverse_direction)
    raw = (ENCODER_COUNTS - raw) & ENCODER_MASK;

  raw = (raw - encoder_offset) & ENCODER_MASK;

  if (lut_valid)
  {
    uint32_t idx  = raw >> 5;     // 256-entry LUT
    uint32_t frac = raw & 0x1F;

    int16_t o0 = encoder_lut[idx];
    int16_t o1 = encoder_lut[(idx + 1) & 0xFF];

    int32_t interp = o0 + (((int32_t)(o1 - o0) * frac) >> 5);

    raw = (raw + interp) & ENCODER_MASK;
  }

  return raw;
}

float motor_interface_get_position_rad(void)
{
  return (float)motor_interface_get_position() / ENCODER_COUNTS * PI2_F;
}

uint16_t motor_interface_get_position_raw(void)
{
  ws22_parse_latest_frame();

  uint16_t raw = currentData.position & ENCODER_MASK;

  // Apply direction reversal ONLY
  if (reverse_direction)
    raw = (ENCODER_COUNTS - raw) & ENCODER_MASK;

  return raw;
}

MotorSensorData motor_interface_get_data(void)
{
  ws22_parse_latest_frame();
  return currentData;
}

