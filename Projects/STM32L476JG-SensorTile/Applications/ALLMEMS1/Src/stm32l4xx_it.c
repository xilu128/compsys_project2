/**
  ******************************************************************************
  * @file    stm32l4xx_it.c
  * @author  System Research & Applications Team - Catania Lab.
  * @version 4.2.0
  * @date    07-Feb-2022
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and
  *          peripherals interrupt service routine.
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

#include "main.h"
#include "stm32l4xx_it.h"
#include "hci_tl_interface.h"

extern TIM_HandleTypeDef    TimEnvHandle;
extern TIM_HandleTypeDef    TimCCHandle;
extern TIM_HandleTypeDef    TimAudioDataHandle;

extern UART_HandleTypeDef UartHandle;

#ifdef ALLMEMS1_MOTIONFA
extern TIM_HandleTypeDef    TimFitnessActivitiesHandle;
#endif

#ifdef ALLMEMS1_ENABLE_SD_CARD_LOGGING
extern SPI_HandleTypeDef SPI_SD_Handle;
extern TIM_HandleTypeDef    TimSdRecordingHandle;

extern RTC_HandleTypeDef RtcHandle;
#endif

#ifdef ALLMEMS1_ENABLE_PRINTF
  extern PCD_HandleTypeDef hpcd;
  extern TIM_HandleTypeDef  TimHandle;
#endif

void NMI_Handler(void)
{
}

void HardFault_Handler(void)
{

  while (1)
  {
  }
}

void MemManage_Handler(void)
{

  while (1)
  {
  }
}

void BusFault_Handler(void)
{

  while (1)
  {
  }
}

void UsageFault_Handler(void)
{

  while (1)
  {
  }
}

void SVC_Handler(void)
{
}

void DebugMon_Handler(void)
{
}

void PendSV_Handler(void)
{
}

void SysTick_Handler(void)
{
  HAL_IncTick();
}

#ifdef ALLMEMS1_MOTIONFA

void TIM3_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&TimFitnessActivitiesHandle);
}
#endif

void TIM4_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&TimEnvHandle);
}

void TIM5_IRQHandler(void)
{

}

void TIM1_CC_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&TimCCHandle);
}

#ifdef ALLMEMS1_ENABLE_SD_CARD_LOGGING

void TIM2_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&TimSdRecordingHandle);
}

void DMA2_Channel2_IRQHandler(void)
{
  HAL_DMA_IRQHandler(SPI_SD_Handle.hdmatx);
}

void RTC_Alarm_IRQHandler(void)
{
  HAL_RTC_AlarmIRQHandler(&RtcHandle);
}
#endif

void EXTI3_IRQHandler(void)
{
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_3);

}

void EXTI9_5_IRQHandler(void)
{

  HAL_GPIO_EXTI_IRQHandler(HCI_TL_SPI_EXTI_PIN);
}

void EXTI2_IRQHandler( void )
{

}

#ifdef ALLMEMS1_ENABLE_PRINTF

void OTG_FS_IRQHandler(void)
{
  HAL_PCD_IRQHandler(&hpcd);
}

void TIM8_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&TimHandle);
}
#endif

void UART5_IRQHandler(void)
{
  HAL_UART_IRQHandler(&UartHandle);
}
