#include "p2_packet.h"

#include <stdio.h>
#include "main.h"
#include "SensorTile.h"
#include "ble_interface.h"
#include "bluenrg_utils.h"
#include "bluenrg_l2cap_aci.h"
#include "uuid_ble_service.h"

int connected = FALSE;
uint8_t set_connectable = TRUE;

volatile uint32_t FeatureMask;

extern uint32_t ConnectionBleStatus;

extern uint32_t FirstConnectionConfig;

extern TIM_HandleTypeDef    TimCCHandle;

extern uint8_t bdaddr[6];
extern uint8_t NodeName[8];

extern uint32_t uhCCR4_Val;

static uint16_t HWServW2STHandle;

static uint16_t AccGyroMagCharHandle;

static uint16_t connection_handle = 0;

static void GAP_ConnectionComplete_CB(uint8_t addr[6], uint16_t handle);
static void GAP_DisconnectionComplete_CB(void);

static void AccGyroMag_AttributeModified_CB(uint8_t *att_data);

#ifdef ACC_BLUENRG_CONGESTION
#define ACI_GATT_UPDATE_CHAR_VALUE safe_aci_gatt_update_char_value
static int32_t breath;

tBleStatus safe_aci_gatt_update_char_value(uint16_t servHandle,
				      uint16_t charHandle,
				      uint8_t charValOffset,
				      uint8_t charValueLen,
				      const uint8_t *charValue)
{
  tBleStatus ret = BLE_STATUS_INSUFFICIENT_RESOURCES;

  if (breath > 0) {
    breath--;
  } else {
    ret = aci_gatt_update_char_value(servHandle,charHandle,charValOffset,charValueLen,charValue);

    if (ret != BLE_STATUS_SUCCESS){
      breath = ACC_BLUENRG_CONGESTION_SKIP;
    }
  }

  return (ret);
}

#else
#define ACI_GATT_UPDATE_CHAR_VALUE aci_gatt_update_char_value
#endif

tBleStatus Add_HW_SW_ServW2ST_Service(void)
{
  tBleStatus ret;

  uint8_t uuid[16];

  uint8_t max_attr_records= 4;

  COPY_HW_SENS_W2ST_SERVICE_UUID(uuid);
  ret = aci_gatt_add_serv(UUID_TYPE_128,  uuid, PRIMARY_SERVICE,
                          1+3*max_attr_records,
                          &HWServW2STHandle);

  if (ret != BLE_STATUS_SUCCESS) {
	  XPRINTF("P2ERROR,ble_service_create_failed\r\n");
    goto fail;
  }

  COPY_ACC_GYRO_MAG_W2ST_CHAR_UUID(uuid);
  ret =  aci_gatt_add_char(HWServW2STHandle, UUID_TYPE_128, uuid, 2+3*3*2,
                           CHAR_PROP_NOTIFY,
                           ATTR_PERMISSION_NONE,
                           GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,
                           16, 0, &AccGyroMagCharHandle);

  if (ret != BLE_STATUS_SUCCESS) {
	  XPRINTF("P2ERROR,ble_characteristic_create_failed\r\n");
    goto fail;
  }

  return BLE_STATUS_SUCCESS;

fail:

  return BLE_STATUS_ERROR;
}

tBleStatus AccGyroMag_Update(BSP_MOTION_SENSOR_Axes_t *Acc,BSP_MOTION_SENSOR_Axes_t *Gyro,BSP_MOTION_SENSOR_Axes_t *Mag)
{
  tBleStatus ret;

  uint8_t buff[2+3*3*2];

  STORE_LE_16(buff   ,(HAL_GetTick()>>3));

  P2_PutI16(buff+2,Acc->x);
  P2_PutI16(buff+4,Acc->y);
  P2_PutI16(buff+6,Acc->z);

  P2_PutI16(buff+8,Gyro->x*10);
  P2_PutI16(buff+10,Gyro->y);
  P2_PutI16(buff+12,Gyro->z);

  P2_PutI16(buff+14,Mag->x);
  P2_PutI16(buff+16,Mag->y);
  P2_PutI16(buff+18,Mag->z);

  ret = ACI_GATT_UPDATE_CHAR_VALUE(HWServW2STHandle, AccGyroMagCharHandle, 0, 2+3*3*2, buff);

  if (ret != BLE_STATUS_SUCCESS){

      static uint32_t last_error_log;
      if (HAL_GetTick() - last_error_log >= P2_ERROR_LOG_PERIOD_MS) {
          last_error_log = HAL_GetTick();
          XPRINTF("P2ERROR,ble_notify_failed\r\n");
      }

    return BLE_STATUS_ERROR;
  }
  return BLE_STATUS_SUCCESS;
}

void setConnectable(void)
{
  char local_name[8] = {AD_TYPE_COMPLETE_LOCAL_NAME,NodeName[1],NodeName[2],NodeName[3],NodeName[4],NodeName[5],NodeName[6],NodeName[7]};
  uint8_t manuf_data[25];

  manuf_data[0 ] = 8U;
  manuf_data[1 ] = 0x09U;
  manuf_data[2 ] = NodeName[1];
  manuf_data[3 ] = NodeName[2];
  manuf_data[4 ] = NodeName[3];
  manuf_data[5 ] = NodeName[4];
  manuf_data[6 ] = NodeName[5];
  manuf_data[7 ] = NodeName[6];
  manuf_data[8 ] = NodeName[7];
  manuf_data[9 ] = 15U;
  manuf_data[10] = 0xFFU;
  manuf_data[11] = 0x30U;
  manuf_data[12] = 0x00U;
#ifdef BLE_MANAGER_SDKV2
  manuf_data[13] = 0x02U;
#else
  manuf_data[13] = 0x01U;
#endif
  manuf_data[14] = 0x02U;
  manuf_data[15] = 0x08U;
  manuf_data[16] = 0x00U;
  manuf_data[17] = 0x00U;
  manuf_data[18] = 0x00U;

  manuf_data[19] = bdaddr[5];
  manuf_data[20] = bdaddr[4];
  manuf_data[21] = bdaddr[3];
  manuf_data[22] = bdaddr[2];
  manuf_data[23] = bdaddr[1];
  manuf_data[24] = bdaddr[0];

  hci_le_set_scan_resp_data(0,NULL);
  aci_gap_set_discoverable(ADV_IND, 0, 0,
                           RANDOM_ADDR,
                           NO_WHITE_LIST_USE,
                           sizeof(local_name), local_name, 0, NULL, 0, 0);

  aci_gap_update_adv_data(25, manuf_data);
}

static void GAP_ConnectionComplete_CB(uint8_t addr[6], uint16_t handle)
{
  connected = TRUE;
  connection_handle = handle;
  BSP_LED_On(LED1);
  XPRINTF("P2INFO,ble_connected\r\n");

#ifdef ALLMEMS1_DEBUG_CONNECTION
  ALLMEMS1_PRINTF("\r\n>>>>>>CONNECTED %x:%x:%x:%x:%x:%x\r\n\r\n",addr[5],addr[4],addr[3],addr[2],addr[1],addr[0]);
#endif

  ConnectionBleStatus=0;

}

static void GAP_DisconnectionComplete_CB(void)
{
  connected = FALSE;
  connection_handle = 0;

  XPRINTF("P2INFO,ble_disconnected\r\n");

  set_connectable = TRUE;

  ConnectionBleStatus=0;

  if(HAL_TIM_OC_Stop_IT(&TimCCHandle, TIM_CHANNEL_4) != HAL_OK){

    Error_Handler();
  }

}

void Read_Request_CB(uint16_t handle)
{

  if(connected)
    aci_gatt_allow_read(connection_handle);
}

void Attribute_Modified_CB(uint16_t attr_handle, uint8_t * att_data, uint8_t data_length)
{

    	if(connected && data_length == 2 && att_data != NULL &&
           attr_handle == AccGyroMagCharHandle + 2) {
      AccGyroMag_AttributeModified_CB(att_data);
    	}

}

static void AccGyroMag_AttributeModified_CB(uint8_t *att_data)
{
  if (att_data[0] == 01) {
    if (W2ST_CHECK_CONNECTION(W2ST_CONNECT_ACC_GYRO_MAG)) return;
    W2ST_ON_CONNECTION(W2ST_CONNECT_ACC_GYRO_MAG);

    if(HAL_TIM_OC_Start_IT(&TimCCHandle, TIM_CHANNEL_4) != HAL_OK){

      Error_Handler();
    }

    {
      uint32_t uhCapture = __HAL_TIM_GET_COUNTER(&TimCCHandle);

      __HAL_TIM_SET_COMPARE(&TimCCHandle, TIM_CHANNEL_4, (uhCapture + uhCCR4_Val));
    }
  } else if (att_data[0] == 0) {
    if (!W2ST_CHECK_CONNECTION(W2ST_CONNECT_ACC_GYRO_MAG)) return;
    W2ST_OFF_CONNECTION(W2ST_CONNECT_ACC_GYRO_MAG);

    if(HAL_TIM_OC_Stop_IT(&TimCCHandle, TIM_CHANNEL_4) != HAL_OK){

      Error_Handler();
    }
  }

    XPRINTF("P2INFO,ble_subscription=%s", W2ST_CHECK_CONNECTION(W2ST_CONNECT_ACC_GYRO_MAG) ? " ON\r\n" : " OFF\r\n");

}

void HCI_Event_CB(void *pckt)
{
  hci_uart_pckt *hci_pckt = pckt;
  hci_event_pckt *event_pckt = (hci_event_pckt*)hci_pckt->data;

  if(hci_pckt->type != HCI_EVENT_PKT) {
    return;
  }

  switch(event_pckt->evt){

  case EVT_DISCONN_COMPLETE:
    {
      GAP_DisconnectionComplete_CB();
    }
    break;
  case EVT_LE_META_EVENT:
    {
      evt_le_meta_event *evt = (void *)event_pckt->data;

      switch(evt->subevent){
      case EVT_LE_CONN_COMPLETE:
        {
          evt_le_connection_complete *cc = (void *)evt->data;
          GAP_ConnectionComplete_CB(cc->peer_bdaddr, cc->handle);
        }
        break;
      }
    }
    break;
  case EVT_VENDOR:
    {
      evt_blue_aci *blue_evt = (void*)event_pckt->data;
      switch(blue_evt->ecode){
      case EVT_BLUE_GATT_READ_PERMIT_REQ:
        {
          evt_gatt_read_permit_req *pr = (void*)blue_evt->data;
          Read_Request_CB(pr->attr_handle);
        }
        break;
      case EVT_BLUE_GATT_ATTRIBUTE_MODIFIED:
        {
          evt_gatt_attr_modified_IDB05A1 *evt = (evt_gatt_attr_modified_IDB05A1*)blue_evt->data;
          Attribute_Modified_CB(evt->attr_handle, evt->att_data,evt->data_length);
        }
        break;
      }
    }
    break;
  }
}
