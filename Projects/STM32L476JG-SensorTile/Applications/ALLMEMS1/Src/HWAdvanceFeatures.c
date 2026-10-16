/**
  ******************************************************************************
  * @file    HWAdvanceFeatures.c
  * @author  System Research & Applications Team - Catania Lab.
  * @version 4.2.0
  * @date    07-Feb-2022
  * @brief   DS3/DSM HW Advance Features API
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
#include "HWAdvanceFeatures.h"
#include "TargetFeatures.h"
#include "sensor_service.h"

extern uint16_t PedometerStepCount;

uint32_t HWAdvanceFeaturesStatus=0;

static float DefaultAccODR;
static uint8_t MultipleAccEventEnabled= 0;

void InitHWFeatures(void){

  BSP_MOTION_SENSOR_GetOutputDataRate(ACCELERO_INSTANCE,MOTION_ACCELERO,&DefaultAccODR);
}

void DisableHWFeatures(void)
{

}

void EnableHWOrientation6D(void)
{

  if(!MultipleAccEventEnabled)
    DisableHWFeatures();

  if(BSP_MOTION_SENSOR_Enable_6D_Orientation(ACCELERO_INSTANCE, BSP_MOTION_SENSOR_INT2_PIN)!=BSP_ERROR_NONE) {
    ALLMEMS1_PRINTF("Error Enabling 6D Orientation\r\n");
  } else {
    ALLMEMS1_PRINTF("Enabled 6D Orientation\r\n");
    W2ST_ON_HW_FEATURE(W2ST_HWF_6DORIENTATION);
  }
}

void DisableHWOrientation6D(void)
{

  if(BSP_MOTION_SENSOR_Disable_6D_Orientation(ACCELERO_INSTANCE)!=BSP_ERROR_NONE) {
    ALLMEMS1_PRINTF("Error Disabling 6D Orientation\r\n");
  } else {
    ALLMEMS1_PRINTF("Disabled 6D Orientation\r\n");
    W2ST_OFF_HW_FEATURE(W2ST_HWF_6DORIENTATION);
  }

  BSP_MOTION_SENSOR_SetOutputDataRate(ACCELERO_INSTANCE,MOTION_ACCELERO, DefaultAccODR);
}

AccEventType GetHWOrientation6D(void)
{
  AccEventType OrientationResult = ACC_NOT_USED;

  uint8_t xl = 0;
  uint8_t xh = 0;
  uint8_t yl = 0;
  uint8_t yh = 0;
  uint8_t zl = 0;
  uint8_t zh = 0;

  if (BSP_MOTION_SENSOR_Get_6D_Orientation_XL( ACCELERO_INSTANCE, &xl ) != BSP_ERROR_NONE ){
    ALLMEMS1_PRINTF("Error getting 6D orientation XL axis from LSM6DS3\r\n");
  }

  if (BSP_MOTION_SENSOR_Get_6D_Orientation_XH( ACCELERO_INSTANCE, &xh ) != BSP_ERROR_NONE ){
    ALLMEMS1_PRINTF("Error getting 6D orientation XH axis from LSM6DS3\r\n");
  }

  if (BSP_MOTION_SENSOR_Get_6D_Orientation_YL( ACCELERO_INSTANCE, &yl ) != BSP_ERROR_NONE ){
    ALLMEMS1_PRINTF("Error getting 6D orientation YL axis from LSM6DS3\r\n");
  }

  if (BSP_MOTION_SENSOR_Get_6D_Orientation_YH( ACCELERO_INSTANCE, &yh ) != BSP_ERROR_NONE ){
    ALLMEMS1_PRINTF("Error getting 6D orientation YH axis from LSM6DS3\r\n");
  }

  if (BSP_MOTION_SENSOR_Get_6D_Orientation_ZL( ACCELERO_INSTANCE, &zl ) != BSP_ERROR_NONE ){
    ALLMEMS1_PRINTF("Error getting 6D orientation ZL axis from LSM6DS3\r\n");
  }

  if (BSP_MOTION_SENSOR_Get_6D_Orientation_ZH( ACCELERO_INSTANCE, &zh ) != BSP_ERROR_NONE ){
    ALLMEMS1_PRINTF("Error getting 6D orientation ZH axis from LSM6DS3\r\n");
  }

  if ( xl == 0 && yl == 0 && zl == 0 && xh == 0 && yh == 1 && zh == 0 ) {
    OrientationResult = ACC_6D_OR_RIGTH;
  } else if ( xl == 1 && yl == 0 && zl == 0 && xh == 0 && yh == 0 && zh == 0 ) {
    OrientationResult = ACC_6D_OR_TOP;
  } else if ( xl == 0 && yl == 0 && zl == 0 && xh == 1 && yh == 0 && zh == 0 ) {
    OrientationResult = ACC_6D_OR_BOTTOM;
  } else if ( xl == 0 && yl == 1 && zl == 0 && xh == 0 && yh == 0 && zh == 0 ) {
    OrientationResult = ACC_6D_OR_LEFT;
  } else if ( xl == 0 && yl == 0 && zl == 0 && xh == 0 && yh == 0 && zh == 1 ) {
    OrientationResult = ACC_6D_OR_UP;
  } else if ( xl == 0 && yl == 0 && zl == 1 && xh == 0 && yh == 0 && zh == 0 ){
    OrientationResult = ACC_6D_OR_DOWN;
  } else {
    ALLMEMS1_PRINTF("None of the 6D orientation axes is set in LSM6DS3\r\n");
  }

  return OrientationResult;
}

void EnableHWTilt(void)
{

  if(!MultipleAccEventEnabled)
    DisableHWFeatures();

  if(BSP_MOTION_SENSOR_Enable_Tilt_Detection(ACCELERO_INSTANCE, BSP_MOTION_SENSOR_INT2_PIN)!=BSP_ERROR_NONE) {
    ALLMEMS1_PRINTF("Error Enabling Tilt Detection\r\n");
  } else {
    ALLMEMS1_PRINTF("Enabled Tilt\r\n");
    W2ST_ON_HW_FEATURE(W2ST_HWF_TILT);
  }
}

void DisableHWTilt(void)
{

  if(BSP_MOTION_SENSOR_Disable_Tilt_Detection(ACCELERO_INSTANCE)!=BSP_ERROR_NONE) {
    ALLMEMS1_PRINTF("Error Disabling Tilt Detection\r\n");
  } else {
    ALLMEMS1_PRINTF("Disabled Tilt\r\n");
    W2ST_OFF_HW_FEATURE(W2ST_HWF_TILT);
  }

  BSP_MOTION_SENSOR_SetOutputDataRate(ACCELERO_INSTANCE,MOTION_ACCELERO,DefaultAccODR);
}

void EnableHWWakeUp(void)
{

  if(!MultipleAccEventEnabled)
    DisableHWFeatures();

  if(BSP_MOTION_SENSOR_Enable_Wake_Up_Detection(ACCELERO_INSTANCE, BSP_MOTION_SENSOR_INT2_PIN)!= BSP_ERROR_NONE) {
    ALLMEMS1_PRINTF("Error Enabling Wake Up Detection\r\n");
  } else {
    ALLMEMS1_PRINTF("Enabled Wake Up\r\n");
    W2ST_ON_HW_FEATURE(W2ST_HWF_WAKE_UP);
  }
}

void DisableHWWakeUp(void)
{

  if(BSP_MOTION_SENSOR_Disable_Wake_Up_Detection(ACCELERO_INSTANCE)!= BSP_ERROR_NONE) {
    ALLMEMS1_PRINTF("Error Disabling Wake Up Detection\r\n");
  } else {
    ALLMEMS1_PRINTF("Disabled Wake Up\r\n");
    W2ST_OFF_HW_FEATURE(W2ST_HWF_WAKE_UP);
  }

  BSP_MOTION_SENSOR_SetOutputDataRate(ACCELERO_INSTANCE,MOTION_ACCELERO,DefaultAccODR);
}

void DisableHWFreeFall(void)
{

  if(BSP_MOTION_SENSOR_Disable_Free_Fall_Detection(ACCELERO_INSTANCE)!= BSP_ERROR_NONE) {
    ALLMEMS1_PRINTF("Error Disabling Free Fall Detection\r\n");
  } else {
    ALLMEMS1_PRINTF("Disabled Free Fall\r\n");
    W2ST_OFF_HW_FEATURE(W2ST_HWF_FREE_FALL);
  }

  BSP_MOTION_SENSOR_SetOutputDataRate(ACCELERO_INSTANCE,MOTION_ACCELERO,DefaultAccODR);
}

void EnableHWDoubleTap(void)
{

  if(!MultipleAccEventEnabled)
    DisableHWFeatures();

  if(BSP_MOTION_SENSOR_Enable_Double_Tap_Detection(ACCELERO_INSTANCE, BSP_MOTION_SENSOR_INT2_PIN)!= BSP_ERROR_NONE) {
    ALLMEMS1_PRINTF("Error Enabling Double Tap Detection\r\n");
  } else {
    ALLMEMS1_PRINTF("Enabled Double Tap\r\n");
    W2ST_ON_HW_FEATURE(W2ST_HWF_DOUBLE_TAP);
  }

}

void DisableHWDoubleTap(void)
{

  if(BSP_MOTION_SENSOR_Disable_Double_Tap_Detection (ACCELERO_INSTANCE)!= BSP_ERROR_NONE) {
    ALLMEMS1_PRINTF("Error Disabling Double Tap Detection\r\n");
  } else {
    ALLMEMS1_PRINTF("Disabled Double Tap\r\n");
    W2ST_OFF_HW_FEATURE(W2ST_HWF_DOUBLE_TAP);
  }

  BSP_MOTION_SENSOR_SetOutputDataRate(ACCELERO_INSTANCE,MOTION_ACCELERO,DefaultAccODR);
}

void EnableHWSingleTap(void)
{

  if(!MultipleAccEventEnabled)
    DisableHWFeatures();

  if(BSP_MOTION_SENSOR_Enable_Single_Tap_Detection(ACCELERO_INSTANCE, BSP_MOTION_SENSOR_INT2_PIN)!= BSP_ERROR_NONE) {
    ALLMEMS1_PRINTF("Error Enabling Single Tap Detection\r\n");
  } else {
    ALLMEMS1_PRINTF("Enabled Sigle Tap\r\n");
    W2ST_ON_HW_FEATURE(W2ST_HWF_SINGLE_TAP);
  }
}

void DisableHWSingleTap(void)
{

  if(BSP_MOTION_SENSOR_Disable_Single_Tap_Detection(ACCELERO_INSTANCE)!= BSP_ERROR_NONE) {
    ALLMEMS1_PRINTF("Error Disabling Single Tap Detection\r\n");
  } else {
    ALLMEMS1_PRINTF("Disabled Sigle Tap\r\n");
    W2ST_OFF_HW_FEATURE(W2ST_HWF_SINGLE_TAP);
  }

  BSP_MOTION_SENSOR_SetOutputDataRate(ACCELERO_INSTANCE,MOTION_ACCELERO,DefaultAccODR);
}

void EnableHWMultipleEvents(void)
{
  ALLMEMS1_PRINTF("EnableHWMultipleEvents\r\n");
  DisableHWFeatures();

  MultipleAccEventEnabled= 1;

  EnableHWPedometer();
  EnableHWTilt();
  EnableHWFreeFall();
  EnableHWSingleTap();
  EnableHWDoubleTap();
  EnableHWOrientation6D();

  PedometerStepCount=0;
  AccEvent_Notify(PedometerStepCount, 3);

  W2ST_ON_HW_FEATURE(W2ST_HWF_MULTIPLE_EVENTS);
}

void DisableHWMultipleEvents(void)
{
  ALLMEMS1_PRINTF("DisableHWMultipleEvents\r\n");
  DisableHWFeatures();
  MultipleAccEventEnabled= 0;
  W2ST_OFF_HW_FEATURE(W2ST_HWF_MULTIPLE_EVENTS);
}
