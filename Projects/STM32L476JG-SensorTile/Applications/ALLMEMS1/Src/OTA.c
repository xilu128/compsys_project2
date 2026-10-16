/**
  ******************************************************************************
  * @file    OTA.c
  * @author  System Research & Applications Team - Catania Lab.
  * @version 4.2.0
  * @date    07-Feb-2022
  * @brief   Over-the-Air Update API implementation
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
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "TargetFeatures.h"

#include "OTA.h"

typedef struct
{
  uint32_t Version;
  uint32_t MagicNum;
  uint32_t OTAStartAdd;
  uint32_t OTADoneAdd;
  uint32_t OTAMaxSize;
  uint32_t ProgStartAdd;
} BootLoaderFeatures_t;

#define BL_VERSION_MAJOR 2
#define BL_VERSION_MINOR 0
#define BL_VERSION_PATCH 0

#ifndef STM32_SENSORTILEBOX

  #define OTA_ADDRESS_START  0x08080010

  #define OTA_MAGIC_NUM_POS  0x08080000

  #define OTA_MAGIC_DONE_NUM_POS  0x08080008

#else

  #define OTA_ADDRESS_START  0x08100010

  #define OTA_MAGIC_NUM_POS  0x08100000

  #define OTA_MAGIC_DONE_NUM_POS  0x08100008

#endif

#define OTA_MAGIC_NUM 0xDEADBEEF

#define OTA_ENABLE_PRINTF

#ifdef OTA_ENABLE_PRINTF
  #define OTA_PRINTF ALLMEMS1_PRINTF
#else
  #define OTA_PRINTF(...)
#endif

#define OTA_ERROR_FUNCTION() { while(1);}

static uint32_t SizeOfUpdateBlueFW=0;
static uint32_t AspecteduwCRCValue=0;
static uint32_t WritingAddress = OTA_ADDRESS_START;

static BootLoaderFeatures_t *BootLoaderFeatures = (BootLoaderFeatures_t *)0x08003F00;

int8_t CheckBootLoaderCompliance(void)
{
  OTA_PRINTF("Testing BootLoaderCompliance:\r\n");
  OTA_PRINTF("\tVersion  %ld.%ld.%ld\r\n",
              BootLoaderFeatures->Version>>16     ,
             (BootLoaderFeatures->Version>>8)&0xFF,
              BootLoaderFeatures->Version    &0xFF);

  if((( BootLoaderFeatures->Version>>16      )!=BL_VERSION_MAJOR) |
     (((BootLoaderFeatures->Version>>8 )&0xFF)!=BL_VERSION_MINOR) |
      ((BootLoaderFeatures->Version     &0xFF)!=BL_VERSION_PATCH)) {
    OTA_PRINTF("\tBL Version  Ko\r\n");
    return 0;
  } else {
    OTA_PRINTF("\tBL Version  Ok\r\n");
  }

  if(BootLoaderFeatures->MagicNum==OTA_MAGIC_NUM) {
    OTA_PRINTF("\tMagicNum    OK\r\n");
  } else {
    OTA_PRINTF("\tMagicNum    KO\r\n");
    return 0;
  }

  OTA_PRINTF("\tMaxSize =%lx\r\n",BootLoaderFeatures->OTAMaxSize);

  if(BootLoaderFeatures->OTAStartAdd==(OTA_ADDRESS_START-16)) {
    OTA_PRINTF("\tOTAStartAdd OK\r\n");
  } else {
    OTA_PRINTF("\tOTAStartAdd KO\r\n");
    return 0;
  }

  if(BootLoaderFeatures->OTADoneAdd==OTA_MAGIC_DONE_NUM_POS) {
    OTA_PRINTF("\tOTADoneAdd  OK\r\n");
  } else {
    OTA_PRINTF("\tOTADoneAdd  KO\r\n");
    return 0;
  }

  return 1;
}

int8_t UpdateFWBlueMS(uint32_t *SizeOfUpdate,uint8_t * att_data, int32_t data_length,uint8_t WriteMagicNum)
{
  int8_t ReturnValue=0;

  if(data_length>(*SizeOfUpdate)){

    OTA_PRINTF("OTA something wrong data_length=%ld RemSizeOfUpdate=%ld....\r\nPlease Try again\r\n",data_length,(*SizeOfUpdate));
    ReturnValue = -1;

    *SizeOfUpdate=0;
  } else {
    uint64_t ValueToWrite;
    int32_t Counter;

    HAL_FLASH_Unlock();

    for(Counter=0;Counter<data_length;Counter+=8) {
      memcpy((uint8_t*) &ValueToWrite,att_data+Counter,data_length-Counter+1);

      if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, WritingAddress,ValueToWrite)==HAL_OK) {
       WritingAddress+=8;
      } else {

        OTA_ERROR_FUNCTION();
      }
    }

    *SizeOfUpdate -= data_length;

    if(*SizeOfUpdate==0) {

      OTA_PRINTF("OTA Update saved\r\n");

      if(WriteMagicNum) {
        uint32_t uwCRCValue = 0;

        if(AspecteduwCRCValue) {

          CRC_HandleTypeDef   CrcHandle;

          CrcHandle.Instance = CRC;

          CrcHandle.Init.DefaultPolynomialUse    = DEFAULT_POLYNOMIAL_ENABLE;

          CrcHandle.Init.DefaultInitValueUse     = DEFAULT_INIT_VALUE_ENABLE;

          CrcHandle.Init.InputDataInversionMode  = CRC_INPUTDATA_INVERSION_NONE;

          CrcHandle.Init.OutputDataInversionMode = CRC_OUTPUTDATA_INVERSION_DISABLE;

          CrcHandle.InputDataFormat              = CRC_INPUTDATA_FORMAT_WORDS;

          if(HAL_CRC_GetState(&CrcHandle) != HAL_CRC_STATE_RESET) {
            HAL_CRC_DeInit(&CrcHandle);
          }

          if (HAL_CRC_Init(&CrcHandle) != HAL_OK) {

            OTA_ERROR_FUNCTION();
          } else {
            OTA_PRINTF("CRC  Initialized\n\r");
          }

          uwCRCValue = HAL_CRC_Calculate(&CrcHandle, (uint32_t *)OTA_ADDRESS_START, SizeOfUpdateBlueFW>>2);

          if(uwCRCValue==AspecteduwCRCValue) {
            ReturnValue=1;
            OTA_PRINTF("OTA CRC-checked\r\n");
          } else {
            OTA_PRINTF("OTA Error CRC-checking\r\n");
          }
        } else {
          ReturnValue=1;

        }
        if(ReturnValue==1) {

          WritingAddress = OTA_MAGIC_NUM_POS;
          ValueToWrite=(((uint64_t)SizeOfUpdateBlueFW)<<32)| (OTA_MAGIC_NUM);

          if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, WritingAddress,ValueToWrite)!=HAL_OK) {

            OTA_ERROR_FUNCTION();
          } else {
            WritingAddress = OTA_MAGIC_NUM_POS+8;

            ValueToWrite=((((uint64_t)(BootLoaderFeatures->ProgStartAdd))<<32)| (0x00));

            if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, WritingAddress,ValueToWrite)!=HAL_OK) {

              OTA_ERROR_FUNCTION();
            } else {
              OTA_PRINTF("OTA will be installed at next board reset\r\n");
            }
          }
        } else {
          ReturnValue=-1;
          if(AspecteduwCRCValue) {
            OTA_PRINTF("Wrong CRC! Computed=%lx  aspected=%lx ... Try again\r\n",uwCRCValue,AspecteduwCRCValue);
          }
        }
      }
    }

    HAL_FLASH_Lock();
  }
  return ReturnValue;
}

void StartUpdateFWBlueMS(uint32_t SizeOfUpdate, uint32_t uwCRCValue)
{
  FLASH_EraseInitTypeDef EraseInitStruct;
  uint32_t SectorError = 0;
  OTA_PRINTF("Start FLASH Erase\r\n");

  SizeOfUpdateBlueFW = SizeOfUpdate;
  AspecteduwCRCValue = uwCRCValue;
  WritingAddress = OTA_ADDRESS_START;

  EraseInitStruct.TypeErase   = FLASH_TYPEERASE_PAGES;
  EraseInitStruct.Banks       = GetBank(OTA_MAGIC_NUM_POS);
  EraseInitStruct.Page        = GetPage(OTA_MAGIC_NUM_POS);
  EraseInitStruct.NbPages     = (SizeOfUpdate+16+FLASH_PAGE_SIZE-1)/FLASH_PAGE_SIZE;

  HAL_FLASH_Unlock();

#ifdef STM32L4R9xx

  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_OPTVERR);

  if (__HAL_FLASH_GET_FLAG(FLASH_FLAG_PEMPTY) != 0) {
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_PEMPTY);
  }
#endif

  if(HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError) != HAL_OK){

    OTA_ERROR_FUNCTION();
  } else {
    OTA_PRINTF("End FLASH Erase %ld Pages of 4KB\r\n",EraseInitStruct.NbPages);
  }

  HAL_FLASH_Lock();
}
