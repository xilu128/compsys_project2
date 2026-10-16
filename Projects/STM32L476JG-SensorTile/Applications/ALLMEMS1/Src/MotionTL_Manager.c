/**
  ******************************************************************************
  * @file    MotionTL_Manager.c
  * @author  MEMS Software Solutions Team
  * @version 4.2.0
  * @date    07-Feb-2022
  * @brief   This file contains Tilt Sensing interface functions
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

#include <math.h>
#include "TargetFeatures.h"

#define CAL_DATA_NUM_RECORDS  100

#define CAL_DATA_NUM_RECORDS  100

void MotionTL_manager_init(void)
{
  char LibVersion[36];
  char acc_orientation[4];
  MTL_acc_cal_t acc_cal;

  acc_orientation[0] ='w';
  acc_orientation[1] ='s';
  acc_orientation[2] ='u';

  MotionTL_Initialize(MTL_MCU_STM32, acc_orientation);

  MotionTL_GetLibVersion(LibVersion);
  TargetBoardFeatures.MotionTLIsInitalized=1;
  ALLMEMS1_PRINTF("Initialized %s\r\n", LibVersion);

  MotionTL_manager_LoadCalValuesFromNVM(&acc_cal);

  if(MotionTL_SetCalValues(&acc_cal) != CAL_PASS)
  {
    ALLMEMS1_PRINTF("\t--> Set Accelerometer Calibration Failed\r\n");
    ALLMEMS1_PRINTF("\t--> ");
    ResetAccellerometerCalibrationInMemory();
  }
}

void MotionTL_manager_run(MTL_input_t *data_in, uint64_t timestamp_ms, MTL_output_t *data_out)
{
  MotionTL_Update(data_in, timestamp_ms, data_out);
}

void MotionTL_manager_getAngleMode(MTL_angle_mode_t *mode)
{
  MTL_knobs_t knobs;
  MotionTL_GetKnobs(&knobs);
  *mode = knobs.mode;
}

void MotionTL_manager_setAngleMode(MTL_angle_mode_t mode)
{
  MTL_knobs_t knobs;
  MotionTL_GetKnobs(&knobs);
  knobs.mode = mode;
  MotionTL_SetKnobs(&knobs);
}

void MotionTL_manager_get_version(char *version, int *length)
{
  *length = (int)MotionTL_GetLibVersion(version);
}

void MotionTL_manager_calibratePosition(MTL_cal_position_t cal_position)
{
  float cal_data[CAL_DATA_NUM_RECORDS][3];

  if (CollectData(cal_data, CAL_DATA_NUM_RECORDS) == 0)
  {
    MotionTL_CalibratePosition(cal_data, CAL_DATA_NUM_RECORDS, cal_position);
  }
}

MTL_cal_result_t MotionTL_manager_getCalibrationValues(MTL_acc_cal_t *acc_cal)
{
  return MotionTL_GetCalValues(acc_cal);
}

void MotionTL_manager_setCalibrationValues(MTL_acc_cal_t *acc_cal)
{
  MotionTL_SetCalValues(acc_cal);
}

void MotionTL_manager_getEstimatedMeasTime(float *time_s)
{
  GetEstimatedMeasTime(time_s, CAL_DATA_NUM_RECORDS);
}

uint8_t MotionTL_manager_LoadCalValuesFromNVM(MTL_acc_cal_t *acc_cal)
{
  char Success=1;
  float data[6] = {0.0f, 0.0f, 0.0f, 1.0f, 1.0f, 1.0f};

  Success= ReCallAccellerometerCalibrationFromMemory(6, (uint32_t *)data);

  for (int i = 0; i < 3; i++)
  {
    acc_cal->offset[i] = data[i];
    acc_cal->gain[i] = data[i + 3];
  }

  return Success;
}

uint8_t MotionTL_manager_SaveCalValuesInNVM(MTL_acc_cal_t *acc_cal)
{
  char Success=1;
  float data[6] = {0.0f, 0.0f, 0.0f, 1.0f, 1.0f, 1.0f};

  for (int i = 0; i < 3; i++)
  {
    data[i] = acc_cal->offset[i];
    data[i + 3] = acc_cal->gain[i];
  }

  Success= SaveAccellerometerCalibrationToMemory(6, (uint32_t *)data);

  return Success;
}
