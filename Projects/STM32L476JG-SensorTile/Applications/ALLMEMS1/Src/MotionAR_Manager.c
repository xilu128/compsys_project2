/**
  ******************************************************************************
  * @file    MotionAR_Manager.c
  * @author  System Research & Applications Team - Catania Lab.
  * @version 4.2.0
  * @date    07-Feb-2022
  * @brief   This file includes activity recognition interface functions
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
#include <limits.h>
#include "TargetFeatures.h"

extern float sensitivity_Mul;

MAR_output_t ActivityCode = MAR_NOACTIVITY;

void MotionAR_manager_run(BSP_MOTION_SENSOR_AxesRaw_t ACC_Value_Raw, uint32_t TimeStamp)
{
  MAR_input_t iDataIN;

  iDataIN.acc_x = ACC_Value_Raw.x * sensitivity_Mul;
  iDataIN.acc_y = ACC_Value_Raw.y * sensitivity_Mul;
  iDataIN.acc_z = ACC_Value_Raw.z * sensitivity_Mul;

  MotionAR_Update(&iDataIN, &ActivityCode, (long int)(TimeStamp & LONG_MAX));
}

void MotionAR_manager_init(void)
{
  char LibVersion[36];
  char acc_orientation[3];

  MotionAR_Initialize();
  MotionAR_GetLibVersion(LibVersion);

  acc_orientation[0] ='w';
  acc_orientation[1] ='s';
  acc_orientation[2] ='u';

  MotionAR_SetOrientation_Acc(acc_orientation);

  TargetBoardFeatures.MotionARIsInitalized=1;
  ALLMEMS1_PRINTF("Initialized %s\r\n", LibVersion);
}
