/**
  ******************************************************************************
  * @file    stm32l4xx_hal_exti.c
  * @author  MCD Application Team
  * @brief   EXTI HAL module driver.
  *          This file provides firmware functions to manage the following
  *          functionalities of the General Purpose Input/Output (EXTI) peripheral:
  *           + Initialization and de-initialization functions
  *           + IO operation functions
  *
  @verbatim
  ==============================================================================
                    ##### EXTI Peripheral features #####
  ==============================================================================
  [..]
    (+) Each Exti line can be configured within this driver.

    (+) Exti line can be configured in 3 different modes
        (++) Interrupt
        (++) Event
        (++) Both of them

    (+) Configurable Exti lines can be configured with 3 different triggers
        (++) Rising
        (++) Falling
        (++) Both of them

    (+) When set in interrupt mode, configurable Exti lines have two diffenrents
        interrupt pending registers which allow to distinguish which transition
        occurs:
        (++) Rising edge pending interrupt
        (++) Falling

    (+) Exti lines 0 to 15 are linked to gpio pin number 0 to 15. Gpio port can
        be selected throught multiplexer.

                     ##### How to use this driver #####
  ==============================================================================
  [..]

    (#) Configure the EXTI line using HAL_EXTI_SetConfigLine().
        (++) Choose the interrupt line number by setting "Line" member from
             EXTI_ConfigTypeDef structure.
        (++) Configure the interrupt and/or event mode using "Mode" member from
             EXTI_ConfigTypeDef structure.
        (++) For configurable lines, configure rising and/or falling trigger
             "Trigger" member from EXTI_ConfigTypeDef structure.
        (++) For Exti lines linked to gpio, choose gpio port using "GPIOSel"
             member from GPIO_InitTypeDef structure.

    (#) Get current Exti configuration of a dedicated line using
        HAL_EXTI_GetConfigLine().
        (++) Provide exiting handle as parameter.
        (++) Provide pointer on EXTI_ConfigTypeDef structure as second parameter.

    (#) Clear Exti configuration of a dedicated line using HAL_EXTI_GetConfigLine().
        (++) Provide exiting handle as parameter.

    (#) Register callback to treat Exti interrupts using HAL_EXTI_RegisterCallback().
        (++) Provide exiting handle as first parameter.
        (++) Provide which callback will be registered using one value from
             EXTI_CallbackIDTypeDef.
        (++) Provide callback function pointer.

    (#) Get interrupt pending bit using HAL_EXTI_GetPending().

    (#) Clear interrupt pending bit using HAL_EXTI_GetPending().

    (#) Generate software interrupt using HAL_EXTI_GenerateSWI().

  @endverbatim
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

#include "stm32l4xx_hal_exti.h"

HAL_StatusTypeDef HAL_EXTI_RegisterCallback(EXTI_HandleTypeDef *hexti, EXTI_CallbackIDTypeDef CallbackID, void (*pPendingCbfn)(void))
{
  HAL_StatusTypeDef status = HAL_OK;

  switch (CallbackID)
  {
#ifdef EXTI_IMR1_IM0
    case  HAL_EXTI_COMMON_CB_ID:
      hexti->RisingCallback = pPendingCbfn;
      hexti->FallingCallback = pPendingCbfn;
      break;

    case  HAL_EXTI_RISING_CB_ID:
      hexti->RisingCallback = pPendingCbfn;
      break;

    case  HAL_EXTI_FALLING_CB_ID:
      hexti->FallingCallback = pPendingCbfn;
      break;
#else
    case  HAL_EXTI_COMMON_CB_ID:
      hexti->Callback = pPendingCbfn;
      break;
#endif
    default:
      status = HAL_ERROR;
      break;
  }

  return status;
}

HAL_StatusTypeDef HAL_EXTI_GetHandle(EXTI_HandleTypeDef *hexti, uint32_t ExtiLine)
{

  assert_param(IS_EXTI_LINE(ExtiLine));

  if(hexti == NULL)
  {
    return HAL_ERROR;
  }
  else
  {

    hexti->Line = ExtiLine;
    return HAL_OK;
  }
}

void HAL_EXTI_IRQHandler(EXTI_HandleTypeDef *hexti)
{
  uint32_t maskline;
  uint32_t linepos;
#if defined(EXTI_D1)
  EXTI_Core_TypeDef *EXTI_CurrentCPU;
#if defined(DUAL_CORE) && defined(CORE_CM4)
  EXTI_CurrentCPU = EXTI_D2;
#else
  EXTI_CurrentCPU = EXTI_D1;
#endif
#endif

  linepos = (hexti->Line & EXTI_PIN_MASK);
  maskline = (1 << linepos);

#if defined(EXTI_D1)

  if((EXTI_CurrentCPU->PR1 & maskline) != 0x00u)
  {

    EXTI_CurrentCPU->PR1 = maskline;

    if(hexti->RisingCallback != NULL)
    {
      hexti->RisingCallback();
    }

    if(hexti->FallingCallback != NULL)
    {
      hexti->FallingCallback();
    }
  }
#elif defined(EXTI_IMR1_IM0)
  if((EXTI->PR1 & maskline) != 0x00u)
  {

    EXTI->PR1 = maskline;

    if(hexti->RisingCallback != NULL)
    {
      hexti->RisingCallback();
    }

    if(hexti->FallingCallback != NULL)
    {
      hexti->FallingCallback();
    }
  }
#else

  if((EXTI->PR & maskline) != 0x00u)
  {

    EXTI->PR = maskline;

    if(hexti->Callback != NULL)
    {
      hexti->Callback();
    }
  }
#endif
}
