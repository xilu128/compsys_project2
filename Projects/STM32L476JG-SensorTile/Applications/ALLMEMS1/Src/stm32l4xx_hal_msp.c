/**
  ******************************************************************************
  * @file    stm32l4xx_hal_msp.c
  * @author  System Research & Applications Team - Catania Lab.
  * @version 4.2.0
  * @date    07-Feb-2022
  * @brief   HAL MSP module.
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

#include "stm32l4xx_hal.h"
#include "ALLMEMS1_config.h"
#include "SensorTile.h"

void HAL_PCD_MspInit(PCD_HandleTypeDef *hpcd)
{

}

void HAL_PCD_MspDeInit(PCD_HandleTypeDef *hpcd)
{

}

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM4) {

    __HAL_RCC_TIM4_CLK_ENABLE();

    HAL_NVIC_SetPriority(TIM4_IRQn, 0xF, 0);

    HAL_NVIC_EnableIRQ(TIM4_IRQn);
  #ifdef ALLMEMS1_MOTIONFA
  } else if(htim->Instance == TIM3) {

    __HAL_RCC_TIM3_CLK_ENABLE();

    HAL_NVIC_SetPriority(TIM3_IRQn, 0xF, 0);

    HAL_NVIC_EnableIRQ(TIM3_IRQn);
  #endif
  } else if (htim->Instance == TIM5) {

    __HAL_RCC_TIM5_CLK_ENABLE();

    HAL_NVIC_SetPriority(TIM5_IRQn, 0xF, 0);

    HAL_NVIC_EnableIRQ(TIM5_IRQn);
  }

}

void HAL_TIM_OC_MspInit(TIM_HandleTypeDef *htim)
{

  __HAL_RCC_TIM1_CLK_ENABLE();

  HAL_NVIC_SetPriority(TIM1_CC_IRQn, 0x8, 0);
  HAL_NVIC_EnableIRQ(TIM1_CC_IRQn);
}

void HAL_CRC_MspInit(CRC_HandleTypeDef *hcrc)
{

  __HAL_RCC_CRC_CLK_ENABLE();
}

void HAL_CRC_MspDeInit(CRC_HandleTypeDef *hcrc)
{

  __HAL_RCC_CRC_FORCE_RESET();

  __HAL_RCC_CRC_RELEASE_RESET();
}

void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{
	if (huart->Instance == UART5) {

		  GPIO_InitTypeDef  GPIO_InitStruct;

		  __HAL_RCC_GPIOC_CLK_ENABLE();

		  __HAL_RCC_GPIOD_CLK_ENABLE();

		  __HAL_RCC_UART5_CLK_ENABLE();

		  GPIO_InitStruct.Pin       = GPIO_PIN_12;
		  GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
		  GPIO_InitStruct.Pull      = GPIO_NOPULL;
		  GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;
		  GPIO_InitStruct.Alternate = GPIO_AF8_UART5;

		  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

		  GPIO_InitStruct.Pin = GPIO_PIN_2;
		  GPIO_InitStruct.Alternate = GPIO_AF8_UART5;

		  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

		  HAL_NVIC_SetPriority(UART5_IRQn, 0, 1);
		  HAL_NVIC_EnableIRQ(UART5_IRQn);

	}

	if (huart->Instance == USART1) {

		  GPIO_InitTypeDef  GPIO_InitStruct;

		  __HAL_RCC_GPIOG_CLK_ENABLE();

		  __HAL_RCC_USART1_CLK_ENABLE();

		  GPIO_InitStruct.Pin       = GPIO_PIN_9;
		  GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
		  GPIO_InitStruct.Pull      = GPIO_NOPULL;
		  GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;
		  GPIO_InitStruct.Alternate = GPIO_AF7_USART1;

		  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

		  GPIO_InitStruct.Pin = GPIO_PIN_10;
		  GPIO_InitStruct.Alternate = GPIO_AF7_USART1;

		  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

	}

}

void HAL_UART_MspDeInit(UART_HandleTypeDef *huart)
{

  __HAL_RCC_UART5_FORCE_RESET();
  __HAL_RCC_UART5_RELEASE_RESET();

  HAL_GPIO_DeInit(GPIOC, GPIO_PIN_12);

  HAL_GPIO_DeInit(GPIOD, GPIO_PIN_2);

  HAL_NVIC_DisableIRQ(UART5_IRQn);
}
