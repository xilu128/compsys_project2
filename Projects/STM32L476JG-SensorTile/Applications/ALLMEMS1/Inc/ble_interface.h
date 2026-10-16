/**
  ******************************************************************************
  * @file    sensor_service.h
  * @author  System Research & Applications Team - Catania Lab.
  * @version 4.2.0
  * @date    07-Feb-2022
  * @brief   Sensors services APIs
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

#ifndef _BLE_INTERFACE_H_
#define _BLE_INTERFACE_H_

#ifdef __cplusplus
 extern "C" {
#endif

#include "bluenrg_types.h"
#include "bluenrg_gatt_server.h"
#include "bluenrg_gap.h"
#include "string.h"
#include "bluenrg_gap_aci.h"
#include "bluenrg_gatt_aci.h"
#include "hci_const.h"
#include "bluenrg_hal_aci.h"
#include "bluenrg_aci_const.h"
#include "hci.h"
#include "hci_tl.h"
#include "hci_le.h"
#include "sm.h"

#include <stdlib.h>
#include "main.h"

extern tBleStatus Add_HW_SW_ServW2ST_Service(void);
extern tBleStatus AccGyroMag_Update(BSP_MOTION_SENSOR_Axes_t *Acc,BSP_MOTION_SENSOR_Axes_t *Gyro,BSP_MOTION_SENSOR_Axes_t *Mag);

extern tBleStatus BatteryReport_Update(uint32_t soc, uint32_t voltage, int32_t current);

extern void     setConnectable(void);

void    HCI_Event_CB(void *pckt);

#define BLE_MANAGER_SDKV2

#define ACC_BLUENRG_CONGESTION

#ifdef ACC_BLUENRG_CONGESTION

#define ACC_BLUENRG_CONGESTION_SKIP 30
#endif

#define W2ST_CONSOLE_MAX_CHAR_LEN 20

#define W2ST_CONSOLE_END_STRING "\0"

#define FROM_MG_TO_G    0.001f

#define FEATURE_MASK_SENSORFUSION_SHORT 0x00000100

#define FEATURE_MASK_ECOMPASS 0x00000040

#define FEATURE_MASK_ACC_EVENTS 0x00000400

#define FEATURE_MASK_TEMP1 0x00040000

#define FEATURE_MASK_TEMP2 0x00010000

#define FEATURE_MASK_PRESS 0x00100000

#define FEATURE_MASK_HUM   0x00080000

#define FEATURE_MASK_ACC   0x00800000

#define FEATURE_MASK_GRYO  0x00400000

#define FEATURE_MASK_MAG   0x00200000

#define FEATURE_MASK_MIC   0x04000000

#define W2ST_COMMAND_CAL_STATUS 0xFF

#define W2ST_COMMAND_CAL_RESET  0x00

#define W2ST_COMMAND_CAL_STOP   0x01

#define W2ST_CONNECT_ENV           (1   )

#define W2ST_CONNECT_ACC_GYRO_MAG  (1<<2)

#define W2ST_CONNECT_AUDIO_LEVEL   (1<<3)

#define W2ST_CONNECT_AR            (1<<4 )

#ifdef ALLMEMS1_MOTIONCP
#define W2ST_CONNECT_CP            (1<<5 )
#endif

#ifdef ALLMEMS1_MOTIONFA
#define W2ST_CONNECT_FA            (1<<6 )
#endif

#define W2ST_CONNECT_QUAT          (1<<7)

#define W2ST_CONNECT_EC            (1<<8)

#define W2ST_CONNECT_GR            (1<<9 )

#define W2ST_CONNECT_ID            (1<<10)

#define W2ST_CONNECT_PE            (1<<11)

#define W2ST_CONNECT_SD            (1<<12)

#define W2ST_CONNECT_TL            (1<<13)

#define W2ST_CONNECT_VC            (1<<14)

#define W2ST_MOTION_ALGORITHM      (1<<15)

#define W2ST_CONNECT_STD_TERM      (1<<16)

#define W2ST_CONNECT_STD_ERR       (1<<17)

#define W2ST_CONNECT_ACC_EVENT     (1<<18)

#define W2ST_CONNECT_BV_AUDIO      (1<<19)
#define W2ST_CONNECT_BV_SYNC       (1<<20)

#define W2ST_CONNECT_GG_EVENT      (1<<21)

#ifdef ALLMEMS1_ENABLE_SD_CARD_LOGGING
#define W2ST_CONNECT_SD_CARD_LOGGING   (1<<22)
#endif

#define W2ST_CHECK_CONNECTION(BleChar) ((ConnectionBleStatus&(BleChar)) ? 1 : 0)
#define W2ST_ON_CONNECTION(BleChar)    (ConnectionBleStatus|=(BleChar))
#define W2ST_OFF_CONNECTION(BleChar)   (ConnectionBleStatus&=(~BleChar))

#ifdef __cplusplus
}
#endif

#endif
