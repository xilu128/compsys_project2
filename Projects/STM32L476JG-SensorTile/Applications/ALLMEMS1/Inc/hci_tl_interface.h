/**
  ******************************************************************************
  * @file  : hci_tl_interface.h
  * @author  System Research & Applications Team - Catania Lab.
  * @version 4.2.0
  * @date    07-Feb-2022
  * @brief : This file contains all the functions prototypes for the STM32
  *          BlueNRG-MS HCI Transport Layer interface
  *		     This file must be moved in the application folder by the user
  *		     and renamed 'hci_tl_interface'
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

#ifndef __SENSORTILE_BLUENRG_H
#define __SENSORTILE_BLUENRG_H

#ifdef __cplusplus
 extern "C" {
#endif

#include "SensorTile_conf.h"

#define HCI_TL_SPI_EXTI_PORT  GPIOC
#define HCI_TL_SPI_EXTI_PIN   GPIO_PIN_5
#define HCI_TL_SPI_EXTI_IRQn  EXTI9_5_IRQn

#define HCI_TL_SPI_IRQ_PORT   GPIOC
#define HCI_TL_SPI_IRQ_PIN    GPIO_PIN_5

#define HCI_TL_SPI_CS_PORT    GPIOB
#define HCI_TL_SPI_CS_PIN     GPIO_PIN_2

#define HCI_TL_RST_PORT       GPIOH
#define HCI_TL_RST_PIN        GPIO_PIN_0

int32_t HCI_TL_SPI_Init    (void* pConf);
int32_t HCI_TL_SPI_DeInit  (void);
int32_t HCI_TL_SPI_Receive (uint8_t* buffer, uint16_t size);
int32_t HCI_TL_SPI_Send    (uint8_t* buffer, uint16_t size);
int32_t HCI_TL_SPI_Reset   (void);

void hci_tl_lowlevel_init(void);

void hci_tl_lowlevel_isr(void);

#ifdef __cplusplus
}
#endif
#endif
