/**
  ******************************************************************************
  * @file    MotionID_Manager.c
  * @author  System Research & Applications Team - Catania Lab.
  * @version 4.2.0
  * @date    07-Feb-2022
  * @brief   This file includes Motion Intensity Detection interface functions
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

#include <stdio.h>
#include "TargetFeatures.h"

extern float sensitivity_Mul;

MID_output_t MIDCode = MID_ON_DESK;

void MotionID_manager_run(BSP_MOTION_SENSOR_AxesRaw_t ACC_Value_Raw)
{
  MID_input_t iDataIN;

  iDataIN.AccX = ACC_Value_Raw.x * sensitivity_Mul;
  iDataIN.AccY = ACC_Value_Raw.y * sensitivity_Mul;
  iDataIN.AccZ = ACC_Value_Raw.z * sensitivity_Mul;

  MotionID_Update(&iDataIN, &MIDCode);
}

void MotionID_manager_init(void)
{
  char LibVersion[36];

  MotionID_Initialize(MID_MCU_STM32);
  MotionID_GetLibVersion(LibVersion);

  TargetBoardFeatures.MotionIDIsInitalized=1;
  ALLMEMS1_PRINTF("Initialized %s\r\n", LibVersion);
}
