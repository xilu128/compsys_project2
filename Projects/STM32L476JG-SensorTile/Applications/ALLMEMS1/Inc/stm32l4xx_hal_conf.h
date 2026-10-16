/**
  ******************************************************************************
  * @file    stm32l4xx_hal_conf.h
  * @author  System Research & Applications Team - Catania Lab.
  * @version 4.2.0
  * @date    07-Feb-2022
  * @brief   HAL configuration file.
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

#ifndef __STM32L4xx_HAL_CONF_H
#define __STM32L4xx_HAL_CONF_H

#ifdef __cplusplus
 extern "C" {
#endif

#define HAL_MODULE_ENABLED
#define HAL_ADC_MODULE_ENABLED

#define HAL_CORTEX_MODULE_ENABLED
#define HAL_CRC_MODULE_ENABLED

#define HAL_DFSDM_MODULE_ENABLED
#define HAL_DMA_MODULE_ENABLED

#define HAL_FLASH_MODULE_ENABLED

#define HAL_GPIO_MODULE_ENABLED
#define HAL_I2C_MODULE_ENABLED

#define HAL_PCD_MODULE_ENABLED
#define HAL_PWR_MODULE_ENABLED

#define HAL_RCC_MODULE_ENABLED

#define HAL_RTC_MODULE_ENABLED
#define HAL_SAI_MODULE_ENABLED

#define HAL_SPI_MODULE_ENABLED

#define HAL_TIM_MODULE_ENABLED

#define HAL_UART_MODULE_ENABLED

#if !defined  (HSE_VALUE)
  #define HSE_VALUE    ((uint32_t)8000000U)
#endif

#if !defined  (HSE_STARTUP_TIMEOUT)
  #define HSE_STARTUP_TIMEOUT    ((uint32_t)100U)
#endif

#if !defined  (MSI_VALUE)
  #define MSI_VALUE    ((uint32_t)4000000U)
#endif

#if !defined  (HSI_VALUE)
  #define HSI_VALUE    ((uint32_t)16000000U)
#endif

#if !defined  (LSI_VALUE)
 #define LSI_VALUE  ((uint32_t)32000U)
#endif

#if !defined  (LSE_VALUE)
  #define LSE_VALUE    ((uint32_t)32768U)
#endif

#if !defined  (LSE_STARTUP_TIMEOUT)
  #define LSE_STARTUP_TIMEOUT    ((uint32_t)5000U)
#endif

#if !defined  (EXTERNAL_SAI1_CLOCK_VALUE)
  #define EXTERNAL_SAI1_CLOCK_VALUE    ((uint32_t)48000U)
#endif

#if !defined  (EXTERNAL_SAI2_CLOCK_VALUE)
  #define EXTERNAL_SAI2_CLOCK_VALUE    ((uint32_t)48000U)
#endif

#define  VDD_VALUE                    ((uint32_t)3300U)
#define  TICK_INT_PRIORITY            ((uint32_t)0x0FU)
#define  USE_RTOS                     0U
#define  PREFETCH_ENABLE              0U
#define  INSTRUCTION_CACHE_ENABLE     1U
#define  DATA_CACHE_ENABLE            1U

#define USE_SPI_CRC                   1U

#ifdef HAL_RCC_MODULE_ENABLED
  #include "stm32l4xx_hal_rcc.h"
#endif

#ifdef HAL_EXTI_MODULE_ENABLED
  #include "stm32l4xx_hal_exti.h"
#endif

#ifdef HAL_GPIO_MODULE_ENABLED
  #include "stm32l4xx_hal_gpio.h"
#endif

#ifdef HAL_DMA_MODULE_ENABLED
  #include "stm32l4xx_hal_dma.h"
#endif

#ifdef HAL_DFSDM_MODULE_ENABLED
  #include "stm32l4xx_hal_dfsdm.h"
#endif

#ifdef HAL_CORTEX_MODULE_ENABLED
  #include "stm32l4xx_hal_cortex.h"
#endif

#ifdef HAL_ADC_MODULE_ENABLED
  #include "stm32l4xx_hal_adc.h"
#endif

#ifdef HAL_CAN_MODULE_ENABLED
  #include "stm32l4xx_hal_can.h"
#endif

#ifdef HAL_COMP_MODULE_ENABLED
  #include "stm32l4xx_hal_comp.h"
#endif

#ifdef HAL_CRC_MODULE_ENABLED
  #include "stm32l4xx_hal_crc.h"
#endif

#ifdef HAL_CRYP_MODULE_ENABLED
  #include "stm32l4xx_hal_cryp.h"
#endif

#ifdef HAL_DAC_MODULE_ENABLED
  #include "stm32l4xx_hal_dac.h"
#endif

#ifdef HAL_FIREWALL_MODULE_ENABLED
  #include "stm32l4xx_hal_firewall.h"
#endif

#ifdef HAL_FLASH_MODULE_ENABLED
  #include "stm32l4xx_hal_flash.h"
#endif

#ifdef HAL_SRAM_MODULE_ENABLED
  #include "stm32l4xx_hal_sram.h"
#endif

#ifdef HAL_NOR_MODULE_ENABLED
  #include "stm32l4xx_hal_nor.h"
#endif

#ifdef HAL_NAND_MODULE_ENABLED
  #include "stm32l4xx_hal_nand.h"
#endif

#ifdef HAL_I2C_MODULE_ENABLED
 #include "stm32l4xx_hal_i2c.h"
#endif

#ifdef HAL_IWDG_MODULE_ENABLED
 #include "stm32l4xx_hal_iwdg.h"
#endif

#ifdef HAL_LCD_MODULE_ENABLED
 #include "stm32l4xx_hal_lcd.h"
#endif

#ifdef HAL_LPTIM_MODULE_ENABLED
#include "stm32l4xx_hal_lptim.h"
#endif

#ifdef HAL_OPAMP_MODULE_ENABLED
#include "stm32l4xx_hal_opamp.h"
#endif

#ifdef HAL_PWR_MODULE_ENABLED
 #include "stm32l4xx_hal_pwr.h"
#endif

#ifdef HAL_QSPI_MODULE_ENABLED
 #include "stm32l4xx_hal_qspi.h"
#endif

#ifdef HAL_RNG_MODULE_ENABLED
 #include "stm32l4xx_hal_rng.h"
#endif

#ifdef HAL_RTC_MODULE_ENABLED
 #include "stm32l4xx_hal_rtc.h"
#endif

#ifdef HAL_SAI_MODULE_ENABLED
 #include "stm32l4xx_hal_sai.h"
#endif

#ifdef HAL_SD_MODULE_ENABLED
 #include "stm32l4xx_hal_sd.h"
#endif

#ifdef HAL_SMBUS_MODULE_ENABLED
 #include "stm32l4xx_hal_smbus.h"
#endif

#ifdef HAL_SPI_MODULE_ENABLED
 #include "stm32l4xx_hal_spi.h"
#endif

#ifdef HAL_SWPMI_MODULE_ENABLED
 #include "stm32l4xx_hal_swpmi.h"
#endif

#ifdef HAL_TIM_MODULE_ENABLED
 #include "stm32l4xx_hal_tim.h"
#endif

#ifdef HAL_TSC_MODULE_ENABLED
 #include "stm32l4xx_hal_tsc.h"
#endif

#ifdef HAL_UART_MODULE_ENABLED
 #include "stm32l4xx_hal_uart.h"
#endif

#ifdef HAL_USART_MODULE_ENABLED
 #include "stm32l4xx_hal_usart.h"
#endif

#ifdef HAL_IRDA_MODULE_ENABLED
 #include "stm32l4xx_hal_irda.h"
#endif

#ifdef HAL_SMARTCARD_MODULE_ENABLED
 #include "stm32l4xx_hal_smartcard.h"
#endif

#ifdef HAL_WWDG_MODULE_ENABLED
 #include "stm32l4xx_hal_wwdg.h"
#endif

#ifdef HAL_PCD_MODULE_ENABLED
 #include "stm32l4xx_hal_pcd.h"
#endif

#ifdef HAL_HCD_MODULE_ENABLED
 #include "stm32l4xx_hal_hcd.h"
#endif

#ifdef  USE_FULL_ASSERT

  #define assert_param(expr) ((expr) ? (void)0U : assert_failed((uint8_t *)__FILE__, __LINE__))

  void assert_failed(uint8_t* file, uint32_t line);
#else
  #define assert_param(expr) ((void)0U)
#endif

#ifdef __cplusplus
}
#endif

#endif
