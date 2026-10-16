/**
  ******************************************************************************
  * @file    MotionGR_Manager.c
  * @author  System Research & Applications Team - Catania Lab.
  * @version 4.2.0
  * @date    07-Feb-2022
  * @brief   This file includes gesture recognition interface functions
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

MGR_output_t GestureRecognitionCode = MGR_NOGESTURE;

void MotionGR_manager_run(BSP_MOTION_SENSOR_AxesRaw_t ACC_Value_Raw)
{
  MGR_input_t iDataIN;

  iDataIN.AccX = ACC_Value_Raw.x * sensitivity_Mul;
  iDataIN.AccY = ACC_Value_Raw.y * sensitivity_Mul;
  iDataIN.AccZ = ACC_Value_Raw.z * sensitivity_Mul;

  MotionGR_Update(&iDataIN, &GestureRecognitionCode);
}

void MotionGR_manager_init(void)
{
  char LibVersion[36];
  char acc_orientation[3];

  MotionGR_Initialize();
  MotionGR_GetLibVersion(LibVersion);

  acc_orientation[0] ='w';
  acc_orientation[1] ='s';
  acc_orientation[2] ='u';

  MotionGR_SetOrientation_Acc(acc_orientation);

  TargetBoardFeatures.MotionGRIsInitalized=1;
  ALLMEMS1_PRINTF("Initialized %s\r\n", LibVersion);
}
