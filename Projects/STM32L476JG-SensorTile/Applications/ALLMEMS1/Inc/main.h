/**
  ******************************************************************************
  * @file    main.h
  * @author  System Research & Applications Team - Catania Lab.
  * @version 4.2.0
  * @date    07-Feb-2022
  * @brief   Header for main.c module
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

#ifndef __MAIN_H
#define __MAIN_H

#include "ALLMEMS1_config.h"
#include <stdio.h>

#define MCR_BLUEMS_F2I_1D(in, out_int, out_dec) {out_int = (int32_t)in; out_dec= (int32_t)((in-out_int)*10);};
#define MCR_BLUEMS_F2I_2D(in, out_int, out_dec) {out_int = (int32_t)in; out_dec= (int32_t)((in-out_int)*100);};

extern void Error_Handler(void);

#ifdef ALLMEMS1_ENABLE_SD_CARD_LOGGING
  extern void RTC_DataConfig(uint8_t WeekDay, uint8_t Date, uint8_t Month, uint8_t Year);
  extern void RTC_TimeConfig(uint8_t Hours, uint8_t Minutes, uint8_t Seconds);
  extern void RTC_AlarmConfig(uint8_t StepHour, uint8_t StepMin, uint8_t StepSec);

  extern void RTC_GetCurrentDateTime(void);
#endif

extern uint8_t BufferToWrite[256];
extern int32_t BytesToWrite;

#define ANGLE_MODE MODE_PITCH_ROLL_GRAVITY_INCLINATION

#define ALGO_FREQ_FX            100U

#define ALGO_PERIOD_FX          (1000U / ALGO_FREQ_FX)

#define DEFAULT_uhCCR1_Val      (10000U / ALGO_FREQ_FX)

#define ALGO_FREQ_CP_GR_SD_TL_VC        50U

#define ALGO_PERIOD_CP_GR_SD_TL_VC      (1000U / ALGO_FREQ_CP_GR_SD_TL_VC)

#define DEFAULT_uhCCR2_Val      (10000U / ALGO_FREQ_CP_GR_SD_TL_VC)

#define ALGO_FREQ_AR_ID_PE      16U

#define ALGO_PERIOD_AR_ID_PE    (1000U / ALGO_FREQ_AR_ID_PE)

#define DEFAULT_uhCCR3_Val      (10000U / ALGO_FREQ_AR_ID_PE)

#define FREQ_ACC_GYRO_MAG               20U

#define ALGO_PERIOD_ACC_GYRO_MAG        (1000U / FREQ_ACC_GYRO_MAG)

#define DEFAULT_uhCCR4_Val              (10000U / FREQ_ACC_GYRO_MAG)

#define RANGE_TIME_WITHOUT_CONNECTED  20000

typedef struct  {
	 uint32_t x;
	 uint32_t y;
	 uint32_t z;

}BSP_MOTION_SENSOR_Axes_t;

void UART5_Transmit(uint8_t*, uint16_t);

#endif
