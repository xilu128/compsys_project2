/**
  ******************************************************************************
  * @file    usbd_cdc_interface.h
  * @author  MCD Application Team
  * @version 4.2.0
  * @date    07-Feb-2022
  * @brief   Header for usbd_cdc_interface.c file.
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

#ifndef __USBD_CDC_IF_H
#define __USBD_CDC_IF_H

#include "usbd_cdc.h"

#define USB_RxBufferDim                         2048

#define TIMx                             TIM8
#define TIMx_CLK_ENABLE                  __HAL_RCC_TIM8_CLK_ENABLE
#define TIMx_FORCE_RESET()               __HAL_RCC_USART3_FORCE_RESET()
#define TIMx_RELEASE_RESET()             __HAL_RCC_USART3_RELEASE_RESET()

#define TIMx_IRQn                        TIM8_IRQn
#define TIMx_IRQHandler                  TIM8_IRQHandler

#define CDC_POLLING_INTERVAL             5

extern USBD_CDC_ItfTypeDef  USBD_CDC_fops;

uint8_t CDC_Fill_Buffer(uint8_t* Buf, uint32_t TotalLen);

#endif
