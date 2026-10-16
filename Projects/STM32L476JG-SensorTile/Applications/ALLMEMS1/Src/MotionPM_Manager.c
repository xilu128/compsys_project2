/**
  ******************************************************************************
  * @file    MotionPM_Manager.c
  * @author  MEMS Software Solutions Team
  * @version 4.2.0
  * @date    07-Feb-2022
  * @brief   This file contains Pedo Meter interface functions
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

MPM_output_t PedometerCode;

void MotionPM_manager_init(void)
{
  char LibVersion[36];

  MotionPM_Initialize();
  MotionPM_GetLibVersion(LibVersion);

  TargetBoardFeatures.MotionPMIsInitalized=1;
  ALLMEMS1_PRINTF("Initialized %s\r\n", LibVersion);
}

void MotionPM_manager_run(MPM_input_t *data_in, MPM_output_t *data_out)
{
	MotionPM_Update(data_in, data_out);
}

void MotionPM_manager_reset(void)
{
	MotionPM_ResetStepCount();
}

void MotionPM_manager_get_version(char *version, int *length)
{
  *length = (int)MotionPM_GetLibVersion(version);
}
