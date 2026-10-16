/**
  ******************************************************************************
  * @file    ALLMEMS1_config.h
  * @author  System Research & Applications Team - Catania Lab.
  * @version 4.2.0
  * @date    07-Feb-2022
  * @brief   FP-SNS-ALLMEMS1 configuration
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

#ifndef __ALLMEMS1_CONFIG_H
#define __ALLMEMS1_CONFIG_H

#ifndef ALLMEMS1_MOTIONFA
  #define ALLMEMS1_MOTIONCP
#endif

#define QUAT_UPDATE_MUL_10MS 3

#define SEND_N_QUATERNIONS 3

#ifndef ALLMEMS1_ENABLE_SD_CARD_LOGGING

#endif

#define ALLMEMS1_VERSION_MAJOR '4'
#define ALLMEMS1_VERSION_MINOR '2'
#define ALLMEMS1_VERSION_PATCH '0'

#define NAME_BLUEMS 'A','M','1','V',ALLMEMS1_VERSION_MAJOR,ALLMEMS1_VERSION_MINOR,ALLMEMS1_VERSION_PATCH

#define ALLMEMS1_PACKAGENAME "FP-SNS-ALLMEMS1"

#ifdef ALLMEMS1_ENABLE_SD_CARD_LOGGING

  #define RTC_CLOCK_SOURCE_LSE

  #ifdef RTC_CLOCK_SOURCE_LSI
    #define RTC_ASYNCH_PREDIV    0x7F
    #define RTC_SYNCH_PREDIV     0xF9
  #endif

  #ifdef RTC_CLOCK_SOURCE_LSE
    #define RTC_ASYNCH_PREDIV  0x7F
    #define RTC_SYNCH_PREDIV   0x00FF
  #endif
#endif

#ifdef ALLMEMS1_ENABLE_PRINTF
    #include "usbd_cdc_interface.h"
    #define XPRINTF(...) {\
      char TmpBufferToWrite[256];\
      int32_t TmpBytesToWrite;\
      TmpBytesToWrite = sprintf( TmpBufferToWrite, __VA_ARGS__);\
      CDC_Fill_Buffer(( uint8_t * )TmpBufferToWrite, TmpBytesToWrite);\
    }
#else
	#include <stdio.h>
	#include <stdint.h>
    void UART5_Transmit(uint8_t *buffer, uint16_t length);
    #include "p2_config.h"
    #define XPRINTF(...) {\
	  char TmpBufferToWrite[P2_LOG_LINE_BYTES];\
      int32_t TmpBytesToWrite;\
      TmpBytesToWrite = (int32_t) snprintf(TmpBufferToWrite, sizeof(TmpBufferToWrite), __VA_ARGS__);\
      if (TmpBytesToWrite < 0) TmpBytesToWrite = 0;\
      if (TmpBytesToWrite >= (int32_t)sizeof(TmpBufferToWrite)) TmpBytesToWrite = 0;\
      UART5_Transmit((uint8_t*)TmpBufferToWrite, TmpBytesToWrite); \
    }

	#define ALLMEMS1_PRINTF(...) {\
    }
#endif

#ifdef USE_STM32F4XX_NUCLEO
#define STM32_UUID ((uint32_t *)0x1FFF7A10)
#endif

#ifdef USE_STM32L4XX_NUCLEO
#define STM32_UUID ((uint32_t *)0x1FFF7590)
#endif

#define STM32_MCU_ID ((uint32_t *)0xE0042000)

#if ((SEND_N_QUATERNIONS<1) || (SEND_N_QUATERNIONS>3))
  #error "SEND_N_QUATERNIONS could be only 1,2 or 3"
#endif

#if ((QUAT_UPDATE_MUL_10MS!=3) && (SEND_N_QUATERNIONS!=1))
  #error "If QUAT_UPDATE_MUL_10MS!=3 then SEND_N_QUATERNIONS must be = 1"
#endif

#endif
