/**
  ******************************************************************************
  * @file    MotionSD_Manager.c
  * @author  MEMS Software Solutions Team
  * @version 4.2.0
  * @date    07-Feb-2022
  * @brief   This file contains Standing vs Sitting Desk Detection interface functions
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

#include "TargetFeatures.h"

MSD_output_t StandingSittingDeskCode;

void MotionSD_manager_init(void)
{
  char LibVersion[36];
  char acc_orientation[3];

  MotionSD_Initialize();
  MotionSD_GetLibVersion(LibVersion);

  acc_orientation[0] ='w';
  acc_orientation[1] ='s';
  acc_orientation[2] ='u';

  MotionSD_SetOrientation_Acc(acc_orientation);

  TargetBoardFeatures.MotionSDIsInitalized=1;
  ALLMEMS1_PRINTF("Initialized %s\r\n", LibVersion);
}

void MotionSD_manager_run(MSD_input_t *data_in, MSD_output_t *data_out)
{
  MotionSD_Update(data_in, data_out);
}

void MotionSD_manager_reset(void)
{
  MotionSD_Reset();
}

void MotionSD_manager_get_version(char *version, int *length)
{
  *length = (int)MotionSD_GetLibVersion(version);
}
