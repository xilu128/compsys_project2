

#include <stdio.h>
#include <math.h>

#include <limits.h>
#include "main.h"
#include "p2_sensor.h"
#include "p2_motion.h"
#include "p2_packet.h"
#include "p2_config.h"

#include "ble_interface.h"
#include "bluenrg_utils.h"
#include "SensorTile.h"

#include "SensorTile_bus.h"

#define ENABLE_SHUT_DOWN_MODE 0

#define BSP_LSM6DSM_INT2_GPIO_PORT           GPIOA
#define BSP_LSM6DSM_INT2_GPIO_CLK_ENABLE()   __GPIOA_CLK_ENABLE()
#define BSP_LSM6DSM_INT2_GPIO_CLK_DISABLE()  __GPIOA_CLK_DISABLE()
#define BSP_LSM6DSM_INT2                 GPIO_PIN_2
#define BSP_LSM6DSM_INT2_EXTI_IRQn           EXTI2_IRQn

#define BSP_LSM6DSM_CS_PORT GPIOB
#define BSP_LSM6DSM_CS_PIN GPIO_PIN_12
#define BSP_LSM6DSM_CS_GPIO_CLK_ENABLE()  __GPIOB_CLK_ENABLE()

#define BSP_LSM303AGR_M_CS_PORT GPIOB
#define BSP_LSM303AGR_M_CS_PIN GPIO_PIN_1
#define BSP_LSM303AGR_M_CS_GPIO_CLK_ENABLE()  __GPIOB_CLK_ENABLE()

#define BSP_LSM303AGR_X_CS_PORT GPIOC
#define BSP_LSM303AGR_X_CS_PIN GPIO_PIN_4
#define BSP_LSM303AGR_X_CS_GPIO_CLK_ENABLE()  __GPIOC_CLK_ENABLE()

#define BSP_LPS22HB_CS_PORT GPIOA
#define BSP_LPS22HB_CS_PIN GPIO_PIN_3
#define BSP_LPS22HB_CS_GPIO_CLK_ENABLE()  __GPIOA_CLK_ENABLE()

#define LSM_MAG_CS_LOW() HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1,GPIO_PIN_RESET);
#define LSM_MAG_CS_HIGH() HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1,GPIO_PIN_SET);

#define LSM_ACC_CS_LOW() HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4,GPIO_PIN_RESET);
#define LSM_ACC_CS_HIGH() HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4,GPIO_PIN_SET);

extern uint8_t set_connectable;
extern int connected;

extern SPI_HandleTypeDef SPI_SD_Handle;

extern volatile float RMS_Ch[];
extern float DBNOISE_Value_Old_Ch[];
extern uint16_t PCM_Buffer[];
extern uint32_t NumSample;

extern SPI_HandleTypeDef hbusspi2;

extern uint8_t NodeName[8];

uint32_t ConnectionBleStatus  =0;

uint32_t FirstConnectionConfig =0;
uint8_t BufferToWrite[256];
int32_t BytesToWrite;
TIM_HandleTypeDef    TimCCHandle;
TIM_HandleTypeDef    TimEnvHandle;

uint8_t bdaddr[6];
uint32_t uhCCR4_Val = DEFAULT_uhCCR4_Val;
uint32_t MagCalibrationData[5];
uint32_t AccCalibrationData[7];
uint8_t  NodeName[8];

UART_HandleTypeDef UartHandle;

static volatile int MEMSInterrupt        =0;
static volatile uint32_t ReadSensor        =0;
static volatile uint32_t SendAccGyroMag  =0;
static volatile uint32_t TimeStamp = 0;
volatile uint32_t HCI_ProcessEvent=0;
typedef struct  {
	 uint32_t Steps;
	 int32_t Heading;
	 uint32_t Distance;
}COMP_Data;
BSP_MOTION_SENSOR_Axes_t ACC_Value;
COMP_Data COMP_Value;
BSP_MOTION_SENSOR_Axes_t MAG_Value;

static void SystemClock_Config(void);

static void Init_BlueNRG_Custom_Services(void);
static void Init_BlueNRG_Stack(void);

static void InitTimers(void);
static void SendMotionData(void);
static void InitTargetPlatform(void);
static void InitLSM(void);

void MX_UART5_UART_Init();

uint8_t Sensor_IO_SPI_CS_Init_All(void);
static int32_t BSP_LSM303AGR_WriteReg_Mag(uint16_t Reg, uint8_t *pdata, uint16_t len);
static int32_t BSP_LSM303AGR_ReadReg_Mag(uint16_t Reg, uint8_t *pdata, uint16_t len);
static int32_t BSP_LSM303AGR_WriteReg_Acc(uint16_t Reg, uint8_t *pdata, uint16_t len);
static int32_t BSP_LSM303AGR_ReadReg_Acc(uint16_t Reg, uint8_t *pdata, uint16_t len);
static int32_t P2_GyroWriteReg(uint16_t reg, uint8_t *data, uint16_t len);
static int32_t P2_GyroReadReg(uint16_t reg, uint8_t *data, uint16_t len);
void LSM303AGR_SPI_Read_nBytes(SPI_HandleTypeDef* xSpiHandle, uint8_t *val, uint16_t nBytesToRead);
void LSM303AGR_SPI_Read(SPI_HandleTypeDef* xSpiHandle, uint8_t *val);
void LSM303AGR_SPI_Write(SPI_HandleTypeDef* xSpiHandle, uint8_t val);

static const P2_SensorBus SensorBus = {
    BSP_LSM303AGR_ReadReg_Acc, BSP_LSM303AGR_WriteReg_Acc,
    BSP_LSM303AGR_ReadReg_Mag, BSP_LSM303AGR_WriteReg_Mag
};
static const P2_GyroBus GyroBus = {P2_GyroReadReg, P2_GyroWriteReg};
static P2_Motion Motion;
static uint32_t SensorErrors;
static uint32_t AccSamples, MagSamples, BleErrors;
static uint32_t LogDropped;
static void UART5_LogPoll(void);
static uint32_t LastAccTick;
static uint8_t HaveAcc;
static uint32_t LastMagTick;
static uint8_t HaveMag;
static uint8_t SpiFault;
static uint8_t SensorReady;
static uint32_t LastSensorAttempt;
static uint32_t LastGyroAttempt, LastGyroTick, GyroSamples;
static uint8_t GyroReady, HaveGyro;
static P2_Vector GyroValue;

static void InitGyro(void) {
    if (Motion.heading_origin_set) Motion.attitude.mag_guard.reference_lost=true;
    LastGyroAttempt=HAL_GetTick();
    GyroReady=P2_GyroInit(&GyroBus);
    LastGyroAttempt=HAL_GetTick();
    HaveGyro=0;
    LastGyroTick=LastGyroAttempt;
    Motion.attitude.seen=Motion.attitude.ready=Motion.attitude.calibrated=false;
    Motion.attitude.count=0U;
    memset(&Motion.attitude.startup,0,sizeof(Motion.attitude.startup));
    memset(&Motion.attitude.recovery,0,sizeof(Motion.attitude.recovery));
    Motion.heading_valid=false;
    Motion.heading_status=Motion.heading_origin_set ? P2_HEADING_RECOVERING : P2_HEADING_CALIBRATING;
    if (GyroReady) {
        XPRINTF("P2INFO,sensor=LSM6DSM,status=ready,gyro_id=0x6A,odr_hz=104,fullscale_dps=2000,gyro_cal_ms=%lu,heading_stable_ms=%lu,keep_still_until=heading_valid\r\n",
            (unsigned long)P2_GYRO_CAL_MS,
            (unsigned long)(P2_STARTUP_BLOCK_MS*P2_STARTUP_BLOCKS));
    } else {
        if (SensorErrors!=UINT32_MAX) ++SensorErrors;
        XPRINTF("P2ERROR,gyro_init_failed,retry_ms=%lu\r\n",(unsigned long)P2_SENSOR_RETRY_MS);
    }
}

static void InitLSM(void) {
    LastSensorAttempt = HAL_GetTick();
    SensorReady = Sensor_IO_SPI_CS_Init_All() == HAL_OK && P2_SensorInit(&SensorBus);
    HaveAcc = HaveMag = 0;
    LastAccTick = LastMagTick = HAL_GetTick();
    Motion.initialized = Motion.armed = Motion.heading_valid = false;
    if (SensorReady) {
        XPRINTF("P2INFO,sensor=LSM303AGR,status=ready,acc_id=0x33,mag_id=0x40,odr_hz=50\r\n");
        InitGyro();
    } else {
        if (SensorErrors != UINT32_MAX) ++SensorErrors;
        XPRINTF("P2ERROR,sensor_init_failed,retry_ms=%lu\r\n", (unsigned long)P2_SENSOR_RETRY_MS);
    }
}

static void ProcessSensors(void) {
    P2_Vector value;
    uint32_t now = HAL_GetTick();
    if (!SensorReady) {
        if (now - LastSensorAttempt >= P2_SENSOR_RETRY_MS) InitLSM();

        goto telemetry;
    }
    int result;

    if (!GyroReady && now-LastGyroAttempt>=P2_SENSOR_RETRY_MS) {
        InitGyro();
        now=HAL_GetTick();
    }
    if (GyroReady) {
        result=P2_GyroRead(&GyroBus,&value);
        if (result>0) {
            GyroValue=value;
            ++GyroSamples;
            LastGyroTick=now;
            HaveGyro=1;
            if (now-LastGyroAttempt>=100U) P2_MotionGyro(&Motion,P2_GyroToBody(value),now);
        } else if (result<0) {
            if (SensorErrors!=UINT32_MAX) ++SensorErrors;
            HaveGyro=GyroReady=0;
            P2_MotionGyro(&Motion,(P2_Vector){NAN,NAN,NAN},now);
        }
    }
    if (!HaveGyro || now-LastGyroTick>P2_GYRO_FRESH_MS) Motion.heading_valid=false;
    if (GyroReady && now-LastGyroTick>P2_SENSOR_STALL_MS) {
        GyroReady=HaveGyro=0;
        P2_MotionGyro(&Motion,(P2_Vector){NAN,NAN,NAN},now);
        if (SensorErrors!=UINT32_MAX) ++SensorErrors;
    }
    result = P2_SensorReadAcc(&SensorBus, &value);
    if (result > 0) {
        ++AccSamples;
        ACC_Value.x = (int32_t)lroundf(value.x);
        ACC_Value.y = (int32_t)lroundf(value.y);
        ACC_Value.z = (int32_t)lroundf(value.z);
        P2_MotionAcceleration(&Motion, value, now);
        LastAccTick = now;
        HaveAcc = 1;
    } else if (result < 0) {
        if (SensorErrors != UINT32_MAX) ++SensorErrors;
        HaveAcc = 0;
        P2_MotionAcceleration(&Motion,(P2_Vector){NAN,NAN,NAN},now);
    }
    result = P2_SensorReadMag(&SensorBus, &value);
    if (result > 0) {
        ++MagSamples;
        LastMagTick = now;
        HaveMag = 1;
        MAG_Value.x = (int32_t)lroundf(value.x);
        MAG_Value.y = (int32_t)lroundf(value.y);
        MAG_Value.z = (int32_t)lroundf(value.z);
        if (HaveAcc && HaveGyro && now-LastGyroTick<=P2_GYRO_FRESH_MS &&
            now - LastAccTick <= P2_ACC_FRESH_MS) P2_MotionHeading(&Motion, value);
        else Motion.heading_valid = false;
    } else if (result < 0) {
        if (SensorErrors != UINT32_MAX) ++SensorErrors;
        HaveMag = 0;
        Motion.heading_valid = false;
    }
    if (!HaveAcc || !HaveMag || !HaveGyro || now-LastGyroTick>P2_GYRO_FRESH_MS ||
        now - LastAccTick > P2_ACC_FRESH_MS || now - LastMagTick > P2_MAG_FRESH_MS)
        Motion.heading_valid = false;
    if (now - LastAccTick > P2_SENSOR_STALL_MS || now - LastMagTick > P2_SENSOR_STALL_MS) {
        SensorReady = HaveAcc = HaveMag = 0;
        Motion.initialized = Motion.armed = Motion.heading_valid = false;
        if (SensorErrors != UINT32_MAX) ++SensorErrors;
    }
telemetry:
    COMP_Value.Steps = Motion.steps;

    COMP_Value.Heading = P2_HeadingDisplay(Motion.heading_deg, Motion.heading_valid, Motion.heading_status);

    COMP_Value.Distance = Motion.distance_m < 3276.7f ?
        (uint32_t)lroundf(Motion.distance_m * 10.0f) : 32767U;
    static uint32_t last_log;
    if (now - last_log >= P2_LOG_PERIOD_MS) {
        last_log = now;

        bool heading_display_valid = Motion.heading_valid && COMP_Value.Heading >= 0;
        uint32_t heading10 = heading_display_valid ? (uint32_t)COMP_Value.Heading * 10U : 0U;
        double distance = Motion.distance_m;

        uint32_t metres = distance >= UINT32_MAX ? UINT32_MAX : (uint32_t)distance;
        uint32_t tenths = distance >= UINT32_MAX ? 0U :
            (uint32_t)((distance - metres) * 10.0 + 0.5);
        if (tenths == 10U) {
            if (metres < UINT32_MAX) ++metres;
            tenths = 0U;
        }
        XPRINTF("P2DATA,ms=%lu,acc_x=%ld,acc_y=%ld,acc_z=%ld,mag_x=%ld,mag_y=%ld,mag_z=%ld,steps=%lu,heading=%s%ld.%lu,heading_valid=%u,distance=%lu.%lu,sensor_errors=%lu,ble_connected=%d,ble_subscribed=%u,ble_errors=%lu,sample_count=%lu,mag_sample_count=%lu,acc_fresh=%u,mag_fresh=%u,log_dropped=%lu\r\n",
            (unsigned long)now,
            (long)ACC_Value.x, (long)ACC_Value.y, (long)ACC_Value.z,
            (long)MAG_Value.x, (long)MAG_Value.y, (long)MAG_Value.z,
            (unsigned long)Motion.steps,
            "",
            heading_display_valid ? (long)(heading10/10U) : -1L,
            heading_display_valid ? (unsigned long)(heading10%10U) : 0UL,
            (unsigned)heading_display_valid, (unsigned long)metres, (unsigned long)tenths,
            (unsigned long)SensorErrors,
            connected, (unsigned)(W2ST_CHECK_CONNECTION(W2ST_CONNECT_ACC_GYRO_MAG) != 0),
            (unsigned long)BleErrors, (unsigned long)AccSamples, (unsigned long)MagSamples,
            (unsigned)(HaveAcc && now-LastAccTick <= P2_ACC_FRESH_MS),
            (unsigned)(HaveMag && now-LastMagTick <= P2_MAG_FRESH_MS), (unsigned long)LogDropped);

        XPRINTF("P2IMU,ms=%lu,gx_mdps=%ld,gy_mdps=%ld,gz_mdps=%ld,gyro_fresh=%u,gyro_calibrated=%u,gyro_samples=%lu,mag_aiding=%u,coast_ms=%lu,handling=%u,up_x=%ld,up_y=%ld,up_z=%ld\r\n",
            (unsigned long)now,(long)lroundf(GyroValue.x*1000.0f),
            (long)lroundf(GyroValue.y*1000.0f),(long)lroundf(GyroValue.z*1000.0f),
            (unsigned)(HaveGyro && now-LastGyroTick<=P2_GYRO_FRESH_MS),
            (unsigned)Motion.attitude.calibrated,(unsigned long)GyroSamples,
            (unsigned)(Motion.heading_valid && HaveAcc && HaveMag && HaveGyro &&
                now-LastAccTick<=P2_ACC_FRESH_MS && now-LastMagTick<=P2_MAG_FRESH_MS &&
                now-LastGyroTick<=P2_GYRO_FRESH_MS && Motion.attitude.magnetic_accepted),
            (unsigned long)(Motion.attitude.ready ? now-Motion.attitude.trusted_ms : 0U),
            (unsigned)Motion.step_handling,
            (long)lroundf(Motion.attitude.up.x*1000.0f),
            (long)lroundf(Motion.attitude.up.y*1000.0f),
            (long)lroundf(Motion.attitude.up.z*1000.0f));

        const P2_Attitude *f=&Motion.attitude;
        XPRINTF("P2HEAD,ms=%lu,bx=%ld,by=%ld,bz=%ld,rx=%ld,ry=%ld,rz=%ld,ns=%lu,nb=%lu,stable_ms=%lu,span_cd=%ld,ready=%u,zero=%u,status=%u,mag_cd=%ld,innovation_cd=%ld,correction_cdps=%ld,origin_cd=%ld,mag_budget_cd=%ld,mag_delta_cd=%ld,mag_blocked=%u,reference_lost=%u,recovery_ms=%lu,recovery_samples=%lu\r\n",
            (unsigned long)now,(long)lroundf(f->bias.x*1000.0f),
            (long)lroundf(f->bias.y*1000.0f),(long)lroundf(f->bias.z*1000.0f),
            (long)lroundf(f->residual_rate.x*1000.0f),
            (long)lroundf(f->residual_rate.y*1000.0f),(long)lroundf(f->residual_rate.z*1000.0f),
            (unsigned long)f->startup.samples,(unsigned long)f->startup.blocks,
            (unsigned long)f->startup.elapsed_ms,(long)lroundf(f->startup.mean_span*100.0f),
            (unsigned)f->ready,(unsigned)Motion.heading_origin_set,(unsigned)Motion.heading_status,
            (long)lroundf(f->magnetic_heading_deg*100.0f),
            (long)lroundf(f->innovation_deg*100.0f),(long)lroundf(f->correction_dps*100.0f),
            (long)lroundf(Motion.heading_origin_deg*100.0f),
            (long)lroundf(f->mag_guard.correction_deg*100.0f),
            (long)lroundf(f->mag_guard.mean_deg*100.0f),
            (unsigned)f->mag_guard.blocked,(unsigned)f->mag_guard.reference_lost,
            (unsigned long)f->recovery.elapsed_ms,(unsigned long)f->recovery.samples);
        static uint32_t reported_errors;
        static uint8_t reported_reference_loss;
        if (Motion.attitude.mag_guard.reference_lost && !reported_reference_loss) {
            reported_reference_loss=1U;
            XPRINTF("P2WARN,inertial_reference_lost,reset_required=0,recovery=keep_still,steps_preserved=1\r\n");
        } else if (!Motion.attitude.mag_guard.reference_lost && reported_reference_loss) {
            reported_reference_loss=0U;
            XPRINTF("P2INFO,heading_recovered,origin_preserved=1,steps_preserved=1\r\n");
        }
        if (SensorErrors != reported_errors) {
            reported_errors = SensorErrors;
            XPRINTF("P2ERROR,sensor_io_or_stale,count=%lu\r\n", (unsigned long)SensorErrors);
        }
    }
}

int main(void)
{

  HAL_Init();

  SystemClock_Config();

  InitTargetPlatform();

  Init_BlueNRG_Stack();

  Init_BlueNRG_Custom_Services();

  InitTimers();

  if(HAL_TIM_Base_Start_IT(&TimEnvHandle) != HAL_OK){

    Error_Handler();
  }
  connected = FALSE;

  while (1)
  {
      UART5_LogPoll();
      if(!connected) {
          static uint32_t last_blink;
          if (HAL_GetTick() - last_blink >= 500U) {
              last_blink = HAL_GetTick();
        	  BSP_LED_Toggle(LED1);

          }
      }
    if(set_connectable){

      setConnectable();
      set_connectable = FALSE;
    }

    if(HCI_ProcessEvent) {
      HCI_ProcessEvent=0;
      hci_user_evt_proc();
    }

    /* Interrupts schedule work; sensor I/O and motion processing run in the main loop. */
    if (ReadSensor) {
        ReadSensor = 0;
        ProcessSensors();
    }

    if(SendAccGyroMag) {
        SendAccGyroMag=0;
        SendMotionData();
    }

    /* Sleep until an interrupt; WFI does not generate timer interrupts. */
    __WFI();
  }
}

void InitTargetPlatform(void)
{

  BSP_LED_Init( LED1 );

  MX_UART5_UART_Init();

  P2_MotionInit(&Motion);
  P2_MotionRequireGyro(&Motion);
  XPRINTF("P2INFO,boot,firmware=" P2_FIRMWARE_VERSION ",magnetic_calibration=%s,uart=115200_8N1\r\n",
          P2_MAG_CALIBRATED ? "MEASURED_3D" : P2_MAG_CALIBRATION_LABEL);
  InitLSM();

}

void MX_UART5_UART_Init(void)
{
  UartHandle.Instance = UART5;
  UartHandle.Init.BaudRate = 115200;
  UartHandle.Init.WordLength = UART_WORDLENGTH_8B;
  UartHandle.Init.StopBits = UART_STOPBITS_1;
  UartHandle.Init.Parity = UART_PARITY_NONE;
  UartHandle.Init.Mode = UART_MODE_TX_RX;
  UartHandle.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  UartHandle.Init.OverSampling = UART_OVERSAMPLING_16;

  if(HAL_UART_Init(&UartHandle) != HAL_OK)
  {
    Error_Handler();
  }

}

void MX_USART1_UART_Init(void)
{
  UartHandle.Instance = USART1;
  UartHandle.Init.BaudRate = 9600;
  UartHandle.Init.WordLength = UART_WORDLENGTH_8B;
  UartHandle.Init.StopBits = UART_STOPBITS_1;
  UartHandle.Init.Parity = UART_PARITY_NONE;
  UartHandle.Init.Mode = UART_MODE_TX_RX;
  UartHandle.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  UartHandle.Init.OverSampling = UART_OVERSAMPLING_16;

  if(HAL_UART_DeInit(&UartHandle) != HAL_OK)
  {
    Error_Handler();
  }
  if(HAL_UART_Init(&UartHandle) != HAL_OK)
  {
    Error_Handler();

  }

}

static uint8_t LogQueue[P2_LOG_QUEUE_DEPTH][P2_LOG_LINE_BYTES];
static uint16_t LogLengths[P2_LOG_QUEUE_DEPTH];
static unsigned LogHead, LogTail, LogCount;
static uint8_t LogActive;
static void UART5_LogPoll(void) {
    if (LogActive) {
        if (UartHandle.gState != HAL_UART_STATE_READY) return;
        LogActive = 0;
        LogTail = (LogTail + 1U) % P2_LOG_QUEUE_DEPTH;
        --LogCount;
    }
    if (LogCount && UartHandle.gState == HAL_UART_STATE_READY) {
        if (HAL_UART_Transmit_IT(&UartHandle, LogQueue[LogTail], LogLengths[LogTail]) == HAL_OK)
            LogActive = 1;
    }
}
void UART5_Transmit(uint8_t* BufferToWrite, uint16_t BytesToWrite) {
    UART5_LogPoll();
    if (!BytesToWrite || BytesToWrite >= P2_LOG_LINE_BYTES || LogCount == P2_LOG_QUEUE_DEPTH) {
        if (LogDropped != UINT32_MAX) ++LogDropped;
        return;
    }
    memcpy(LogQueue[LogHead], BufferToWrite, BytesToWrite);
    LogLengths[LogHead] = BytesToWrite;
    LogHead = (LogHead + 1U) % P2_LOG_QUEUE_DEPTH;
    ++LogCount;
    UART5_LogPoll();
}

void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
  uint32_t uhCapture=0;

  if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4)
  {
     uhCapture = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_4);

    __HAL_TIM_SET_COMPARE(&TimCCHandle, TIM_CHANNEL_4, (uhCapture + uhCCR4_Val));
    SendAccGyroMag=1;
  }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

  if(htim == (&TimEnvHandle)) {
	  ReadSensor=1;

  }
}

static void SendMotionData(void)
{
  /* A BLE connection alone is insufficient: the client must enable notifications. */
  if (!connected || !W2ST_CHECK_CONNECTION(W2ST_CONNECT_ACC_GYRO_MAG)) return;

  BSP_MOTION_SENSOR_Axes_t summary;
  BSP_MOTION_SENSOR_Axes_t acc = ACC_Value, mag = MAG_Value;
  uint32_t now = HAL_GetTick();

  /* Mark stale samples invalid instead of presenting old values as fresh readings. */
  if (!HaveAcc || now-LastAccTick > P2_ACC_FRESH_MS)
      acc.x = acc.y = acc.z = INT16_MIN;
  if (!HaveMag || now-LastMagTick > P2_MAG_FRESH_MS)
      mag.x = mag.y = mag.z = INT16_MIN;

  /* Gyro slots carry project outputs; cap steps before the packet's x10 scaling. */
  summary.x = COMP_Value.Steps > 3276U ? 3276 : (int32_t)COMP_Value.Steps;
  bool heading_fresh=HaveAcc && HaveMag && HaveGyro &&
      now-LastGyroTick<=P2_GYRO_FRESH_MS && now-LastAccTick<=P2_ACC_FRESH_MS &&
      now-LastMagTick<=P2_MAG_FRESH_MS;

  summary.y = heading_fresh && COMP_Value.Heading < 0 ? COMP_Value.Heading :
      P2_HeadingWire((float)COMP_Value.Heading, Motion.heading_valid && heading_fresh,
      heading_fresh ? Motion.heading_status : P2_HEADING_UNAVAILABLE);
  summary.z = (int32_t)COMP_Value.Distance;
  if (AccGyroMag_Update(&acc, &summary, &mag) != BLE_STATUS_SUCCESS &&
      BleErrors != UINT32_MAX) ++BleErrors;
}

static void InitTimers(void)
{

  uint32_t uwPrescalerValue;

  TIM_OC_InitTypeDef sConfig;

  uwPrescalerValue = (uint32_t) ((SystemCoreClock / 10000) - 1);

  TimEnvHandle.Instance = TIM4;

  TimEnvHandle.Init.Period = 99;
  TimEnvHandle.Init.Prescaler = uwPrescalerValue;
  TimEnvHandle.Init.ClockDivision = 0;
  TimEnvHandle.Init.CounterMode = TIM_COUNTERMODE_UP;
  if(HAL_TIM_Base_Init(&TimEnvHandle) != HAL_OK) {

    Error_Handler();
  }

  uwPrescalerValue = (uint32_t) ((SystemCoreClock / 10000) - 1);

  TimCCHandle.Instance = TIM1;
  TimCCHandle.Init.Period        = 65535;
  TimCCHandle.Init.Prescaler     = uwPrescalerValue;
  TimCCHandle.Init.ClockDivision = 0;
  TimCCHandle.Init.CounterMode   = TIM_COUNTERMODE_UP;
  if(HAL_TIM_OC_Init(&TimCCHandle) != HAL_OK)
  {

    Error_Handler();
  }

  sConfig.OCMode     = TIM_OCMODE_TOGGLE;
  sConfig.OCPolarity = TIM_OCPOLARITY_LOW;

  sConfig.Pulse = DEFAULT_uhCCR1_Val;
  if(HAL_TIM_OC_ConfigChannel(&TimCCHandle, &sConfig, TIM_CHANNEL_1) != HAL_OK)
  {

    Error_Handler();
  }

  sConfig.Pulse = DEFAULT_uhCCR2_Val;
  if(HAL_TIM_OC_ConfigChannel(&TimCCHandle, &sConfig, TIM_CHANNEL_2) != HAL_OK)
  {

    Error_Handler();
  }

  sConfig.Pulse = DEFAULT_uhCCR3_Val;
  if(HAL_TIM_OC_ConfigChannel(&TimCCHandle, &sConfig, TIM_CHANNEL_3) != HAL_OK)
  {

    Error_Handler();
  }

  sConfig.Pulse = DEFAULT_uhCCR4_Val;
  if(HAL_TIM_OC_ConfigChannel(&TimCCHandle, &sConfig, TIM_CHANNEL_4) != HAL_OK)
  {

    Error_Handler();
  }
}

uint8_t Sensor_IO_SPI_CS_Init_All(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;
  uint8_t inData[2];

  BSP_LSM6DSM_CS_GPIO_CLK_ENABLE();
  BSP_LSM303AGR_M_CS_GPIO_CLK_ENABLE();
  BSP_LSM303AGR_X_CS_GPIO_CLK_ENABLE();
  BSP_LPS22HB_CS_GPIO_CLK_ENABLE();

  HAL_GPIO_WritePin(BSP_LSM6DSM_CS_PORT, BSP_LSM6DSM_CS_PIN, GPIO_PIN_SET);
  HAL_GPIO_WritePin(BSP_LSM303AGR_X_CS_PORT, BSP_LSM303AGR_X_CS_PIN,GPIO_PIN_SET);
  HAL_GPIO_WritePin(BSP_LSM303AGR_M_CS_PORT, BSP_LSM303AGR_M_CS_PIN,GPIO_PIN_SET);
  HAL_GPIO_WritePin(BSP_LPS22HB_CS_PORT, BSP_LPS22HB_CS_PIN, GPIO_PIN_SET);

  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;

  GPIO_InitStruct.Pin = BSP_LSM6DSM_CS_PIN;
  HAL_GPIO_Init(BSP_LSM6DSM_CS_PORT, &GPIO_InitStruct);
  HAL_GPIO_WritePin(BSP_LSM6DSM_CS_PORT, BSP_LSM6DSM_CS_PIN, GPIO_PIN_SET);

  GPIO_InitStruct.Pin = BSP_LSM303AGR_X_CS_PIN;
  HAL_GPIO_Init(BSP_LSM303AGR_X_CS_PORT, &GPIO_InitStruct);
  HAL_GPIO_WritePin(BSP_LSM303AGR_X_CS_PORT, BSP_LSM303AGR_X_CS_PIN,GPIO_PIN_SET);

  GPIO_InitStruct.Pin = BSP_LSM303AGR_M_CS_PIN;
  HAL_GPIO_Init(BSP_LSM303AGR_M_CS_PORT, &GPIO_InitStruct);
  HAL_GPIO_WritePin(BSP_LSM303AGR_M_CS_PORT, BSP_LSM303AGR_M_CS_PIN,GPIO_PIN_SET);

  GPIO_InitStruct.Pin = BSP_LPS22HB_CS_PIN;
  HAL_GPIO_Init(BSP_LPS22HB_CS_PORT, &GPIO_InitStruct);
  HAL_GPIO_WritePin(BSP_LPS22HB_CS_PORT, BSP_LPS22HB_CS_PIN, GPIO_PIN_SET);

    if (BSP_SPI2_Init() != BSP_ERROR_NONE) return HAL_ERROR;

  HAL_GPIO_WritePin(BSP_LSM303AGR_M_CS_PORT, BSP_LSM303AGR_M_CS_PIN, GPIO_PIN_RESET);
  inData[0] = (0x62U);
  BSP_SPI2_Send(inData,1);
  inData[0] = 0x20;
  BSP_SPI2_Send(inData,1);
  HAL_GPIO_WritePin(BSP_LSM303AGR_M_CS_PORT, BSP_LSM303AGR_M_CS_PIN, GPIO_PIN_SET);

  HAL_GPIO_WritePin(BSP_LSM303AGR_X_CS_PORT, BSP_LSM303AGR_X_CS_PIN, GPIO_PIN_RESET);
  inData[0] = (0x23U);
  BSP_SPI2_Send(inData,1);
  inData[0] = 0x01;
  BSP_SPI2_Send(inData,1);
  HAL_GPIO_WritePin(BSP_LSM303AGR_X_CS_PORT, BSP_LSM303AGR_X_CS_PIN, GPIO_PIN_SET);

  HAL_GPIO_WritePin(BSP_LPS22HB_CS_PORT, BSP_LPS22HB_CS_PIN, GPIO_PIN_RESET);
  inData[0] = (0x10U);
  BSP_SPI2_Send(inData,1);
  inData[0] = 0x01;
  BSP_SPI2_Send(inData,1);
  HAL_GPIO_WritePin(BSP_LPS22HB_CS_PORT, BSP_LPS22HB_CS_PIN, GPIO_PIN_SET);

  HAL_GPIO_WritePin(BSP_LSM6DSM_CS_PORT, BSP_LSM6DSM_CS_PIN, GPIO_PIN_RESET);
  inData[0] = (0x12U);
  BSP_SPI2_Send(inData,1);
  inData[0] = 0x0C;
  BSP_SPI2_Send(inData,1);
  HAL_GPIO_WritePin(BSP_LSM6DSM_CS_PORT, BSP_LSM6DSM_CS_PIN, GPIO_PIN_SET);

  return HAL_OK;
}

static int32_t BSP_LSM303AGR_WriteReg_Mag(uint16_t Reg, uint8_t *pdata, uint16_t len)
{
  int32_t ret = BSP_ERROR_NONE;
  uint8_t dataReg = (uint8_t)Reg;
  SpiFault = 0;

  LSM_MAG_CS_LOW();

  if (BSP_SPI2_Send(&dataReg, 1) != 1)
  {
    ret = BSP_ERROR_UNKNOWN_FAILURE;
  }

  if (BSP_SPI2_Send(pdata, len) != len)
  {
    ret = BSP_ERROR_UNKNOWN_FAILURE;
  }

  LSM_MAG_CS_HIGH();

  return SpiFault ? BSP_ERROR_UNKNOWN_FAILURE : ret;
}

static int32_t BSP_LSM303AGR_ReadReg_Mag(uint16_t Reg, uint8_t *pdata, uint16_t len)
{
  int32_t ret = BSP_ERROR_NONE;
  uint8_t dataReg = (uint8_t)Reg;
  SpiFault = 0;

  HAL_GPIO_WritePin(BSP_LSM303AGR_M_CS_PORT, BSP_LSM303AGR_M_CS_PIN, GPIO_PIN_RESET);

  LSM303AGR_SPI_Write(&hbusspi2, (dataReg) | 0x80);
  __HAL_SPI_DISABLE(&hbusspi2);
  SPI_1LINE_RX(&hbusspi2);

  if (len > 1)
  {
    LSM303AGR_SPI_Read_nBytes(&hbusspi2, (pdata), len);
  }
  else
  {
    LSM303AGR_SPI_Read(&hbusspi2, (pdata));
  }

  HAL_GPIO_WritePin(BSP_LSM303AGR_M_CS_PORT, BSP_LSM303AGR_M_CS_PIN, GPIO_PIN_SET);
  SPI_1LINE_TX(&hbusspi2);
  __HAL_SPI_ENABLE(&hbusspi2);
  return SpiFault ? BSP_ERROR_UNKNOWN_FAILURE : ret;
}

static int32_t BSP_LSM303AGR_WriteReg_Acc(uint16_t Reg, uint8_t *pdata, uint16_t len)
{
  int32_t ret = BSP_ERROR_NONE;
  uint8_t dataReg = (uint8_t)Reg;
  SpiFault = 0;

  LSM_ACC_CS_LOW();

  if (BSP_SPI2_Send(&dataReg, 1) != 1)
  {
    ret = BSP_ERROR_UNKNOWN_FAILURE;
  }

  if (BSP_SPI2_Send(pdata, len) != len)
  {
    ret = BSP_ERROR_UNKNOWN_FAILURE;
  }

  LSM_ACC_CS_HIGH();

  return SpiFault ? BSP_ERROR_UNKNOWN_FAILURE : ret;
}

static int32_t BSP_LSM303AGR_ReadReg_Acc(uint16_t Reg, uint8_t *pdata, uint16_t len)
{
  int32_t ret = BSP_ERROR_NONE;
  uint8_t dataReg = (uint8_t)Reg;
  SpiFault = 0;

  LSM_ACC_CS_LOW();
  if (len > 1) {
	  LSM303AGR_SPI_Write(&hbusspi2, (dataReg) | 0x80 | 0x40);
  }
  else {
	  LSM303AGR_SPI_Write(&hbusspi2, (dataReg) | 0x80);
  }
  __HAL_SPI_DISABLE(&hbusspi2);
  SPI_1LINE_RX(&hbusspi2);

  if (len > 1)
  {
    LSM303AGR_SPI_Read_nBytes(&hbusspi2, (pdata), len);
  }
  else
  {
    LSM303AGR_SPI_Read(&hbusspi2, (pdata));
  }

  LSM_ACC_CS_HIGH();
  SPI_1LINE_TX(&hbusspi2);
  __HAL_SPI_ENABLE(&hbusspi2);
  return SpiFault ? BSP_ERROR_UNKNOWN_FAILURE : ret;
}

static int32_t P2_GyroWriteReg(uint16_t reg, uint8_t *data, uint16_t len) {
    if (!data || !len) return BSP_ERROR_UNKNOWN_FAILURE;
    SpiFault=0;
    HAL_GPIO_WritePin(BSP_LSM6DSM_CS_PORT,BSP_LSM6DSM_CS_PIN,GPIO_PIN_RESET);

    LSM303AGR_SPI_Write(&hbusspi2,(uint8_t)reg);
    for (uint16_t i=0U;i<len && !SpiFault;++i) LSM303AGR_SPI_Write(&hbusspi2,data[i]);
    HAL_GPIO_WritePin(BSP_LSM6DSM_CS_PORT,BSP_LSM6DSM_CS_PIN,GPIO_PIN_SET);
    return SpiFault ? BSP_ERROR_UNKNOWN_FAILURE : BSP_ERROR_NONE;
}
static int32_t P2_GyroReadReg(uint16_t reg, uint8_t *data, uint16_t len) {
    if (!data || !len) return BSP_ERROR_UNKNOWN_FAILURE;
    SpiFault=0;
    HAL_GPIO_WritePin(BSP_LSM6DSM_CS_PORT,BSP_LSM6DSM_CS_PIN,GPIO_PIN_RESET);
    LSM303AGR_SPI_Write(&hbusspi2,(uint8_t)reg|0x80U);
    __HAL_SPI_DISABLE(&hbusspi2);
    SPI_1LINE_RX(&hbusspi2);
    if (!SpiFault) {
        if (len>1U) LSM303AGR_SPI_Read_nBytes(&hbusspi2,data,len);
        else LSM303AGR_SPI_Read(&hbusspi2,data);
    }
    HAL_GPIO_WritePin(BSP_LSM6DSM_CS_PORT,BSP_LSM6DSM_CS_PIN,GPIO_PIN_SET);
    SPI_1LINE_TX(&hbusspi2);
    __HAL_SPI_ENABLE(&hbusspi2);
    return SpiFault ? BSP_ERROR_UNKNOWN_FAILURE : BSP_ERROR_NONE;
}

void LSM303AGR_SPI_Read_nBytes(SPI_HandleTypeDef* xSpiHandle, uint8_t *val, uint16_t nBytesToRead)
{

  uint32_t saved_mask = __get_PRIMASK();
  uint32_t budget = 100000U;
  __disable_irq();
  __HAL_SPI_ENABLE(xSpiHandle);

  while (nBytesToRead > 1U)
  {
    if (--budget == 0U) {
      SpiFault = 1;
      __HAL_SPI_DISABLE(xSpiHandle);
      __set_PRIMASK(saved_mask);
      return;
    }

    if (xSpiHandle->Instance->SR & SPI_FLAG_RXNE)
    {

      *val = *(__IO uint8_t *) &xSpiHandle->Instance->DR;
      val += sizeof(uint8_t);
      nBytesToRead--;
    }
  }

  __DSB();
  __DSB();
  __HAL_SPI_DISABLE(xSpiHandle);

  __set_PRIMASK(saved_mask);

  budget = 100000U;
  while ((xSpiHandle->Instance->SR & SPI_FLAG_RXNE) != SPI_FLAG_RXNE) { if (--budget == 0U) { SpiFault = 1; return; } }

  *val = *(__IO uint8_t *) &xSpiHandle->Instance->DR;
  budget = 100000U;
  while ((xSpiHandle->Instance->SR & SPI_FLAG_BSY) == SPI_FLAG_BSY) { if (--budget == 0U) { SpiFault = 1; return; } }
}

void LSM303AGR_SPI_Read(SPI_HandleTypeDef* xSpiHandle, uint8_t *val)
{

  uint32_t saved_mask = __get_PRIMASK();
  uint32_t budget = 100000U;
  __disable_irq();
  __HAL_SPI_ENABLE(xSpiHandle);
  __asm("dsb\n");
  __asm("dsb\n");
  __HAL_SPI_DISABLE(xSpiHandle);
  __set_PRIMASK(saved_mask);

  budget = 100000U;
  while ((xSpiHandle->Instance->SR & SPI_FLAG_RXNE) != SPI_FLAG_RXNE) { if (--budget == 0U) { SpiFault = 1; return; } }

  *val = *(__IO uint8_t *) &xSpiHandle->Instance->DR;

  budget = 100000U;
  while ((xSpiHandle->Instance->SR & SPI_FLAG_BSY) == SPI_FLAG_BSY) { if (--budget == 0U) { SpiFault = 1; return; } }
}

void LSM303AGR_SPI_Write(SPI_HandleTypeDef* xSpiHandle, uint8_t val)
{
  uint32_t budget;

  budget = 100000U;
  while ((xSpiHandle->Instance->SR & SPI_FLAG_TXE) != SPI_FLAG_TXE) { if (--budget == 0U) { SpiFault = 1; return; } }

  *((__IO uint8_t*) &xSpiHandle->Instance->DR) = val;

  budget = 100000U;
  while ((xSpiHandle->Instance->SR & SPI_FLAG_FTLVL) != SPI_FTLVL_EMPTY) { if (--budget == 0U) { SpiFault = 1; return; } }
  budget = 100000U;
  while ((xSpiHandle->Instance->SR & SPI_FLAG_BSY) == SPI_FLAG_BSY) { if (--budget == 0U) { SpiFault = 1; return; } }
}

static void Init_BlueNRG_Stack(void)
{
  char BoardName[8] = P2_BLE_NAME;
  uint16_t service_handle, dev_name_char_handle, appearance_char_handle;
  int ret;
  uint8_t  data_len_out;
  uint8_t  hwVersion;
  uint16_t fwVersion;

  for (unsigned i = 0U; i < sizeof(BoardName) - 1U; ++i)
    NodeName[i + 1U] = (uint8_t)BoardName[i];

  hci_init(HCI_Event_CB, NULL);

  getBlueNRGVersion(&hwVersion, &fwVersion);

  aci_hal_read_config_data(CONFIG_DATA_RANDOM_ADDRESS, 6, &data_len_out, bdaddr);

  if ((bdaddr[5] & 0xC0) != 0xC0) {
    XPRINTF("P2ERROR,invalid_ble_address\r\n");
    while(1);
  }

  ret = aci_hal_write_config_data(CONFIG_DATA_PUBADDR_OFFSET, data_len_out,
                                  bdaddr);

  hci_reset();

  ret = aci_gatt_init();
  if(ret){
     XPRINTF("P2ERROR,gatt_init_failed\r\n");
     goto fail;
  }

  ret = aci_gap_init_IDB05A1(GAP_PERIPHERAL_ROLE_IDB05A1, 0, 0x07, &service_handle, &dev_name_char_handle, &appearance_char_handle);

  if(ret != BLE_STATUS_SUCCESS){
     XPRINTF("P2ERROR,gap_init_failed\r\n");
     goto fail;
  }

  ret = aci_gatt_update_char_value(service_handle, dev_name_char_handle, 0,
                                   7 , (uint8_t *)BoardName);

  if(ret){
     XPRINTF("P2ERROR,ble_name_update_failed\r\n");
    while(1);
  }

  ret = aci_gap_set_auth_requirement(MITM_PROTECTION_REQUIRED,
                                     OOB_AUTH_DATA_ABSENT,
                                     NULL, 7, 16,
                                     USE_FIXED_PIN_FOR_PAIRING, 123456,
                                     BONDING);
  if (ret != BLE_STATUS_SUCCESS) {
     XPRINTF("P2ERROR,ble_auth_init_failed\r\n");
     goto fail;
  }

  XPRINTF("P2INFO,ble_initialized\r\n"
         "P2INFO,ble_hw=%d.%d\r\n"
         "P2INFO,ble_fw=%d.%d.%c\r\n"
         "P2INFO,board_name=%s\r\n"
         "P2INFO,ble_address=%x:%x:%x:%x:%x:%x\r\n",
         ((hwVersion>>4)&0x0F),(hwVersion&0x0F),
         fwVersion>>8,
         (fwVersion>>4)&0xF,
         ('a'+(fwVersion&0xF)),
         BoardName,
         bdaddr[5],bdaddr[4],bdaddr[3],bdaddr[2],bdaddr[1],bdaddr[0]);

  aci_hal_set_tx_power_level(1,4);

  return;

fail:
  return;
}

static void Init_BlueNRG_Custom_Services(void)
{
  int ret;

  ret = Add_HW_SW_ServW2ST_Service();
  if(ret == BLE_STATUS_SUCCESS)
  {
     XPRINTF("P2INFO,ble_services_ready\r\n");
  }
  else
  {
     XPRINTF("P2ERROR,ble_service_init_failed\r\n");
  }

}

static void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  HAL_PWR_EnableBkUpAccess();

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK){
    while(1);
  }

  HAL_RCCEx_DisableLSECSS();

  RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState            = RCC_MSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIClockRange       = RCC_MSIRANGE_11;
  RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource       = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM            = 6;
  RCC_OscInitStruct.PLL.PLLN            = 40;
  RCC_OscInitStruct.PLL.PLLP            = 7;
  RCC_OscInitStruct.PLL.PLLQ            = 4;
  RCC_OscInitStruct.PLL.PLLR            = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK){
    while(1);
  }

  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  if(HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    while(1);
  }

  HAL_RCCEx_EnableMSIPLLMode();

  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK){
    while(1);
  }
}

void HAL_Delay(__IO uint32_t Delay)
{
  uint32_t tickstart = 0;
  tickstart = HAL_GetTick();
  while((HAL_GetTick() - tickstart) < Delay){
    __WFI();
  }
}

void Error_Handler(void)
{

  while (1) { BSP_LED_Toggle(LED1); HAL_Delay(150); }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  switch(GPIO_Pin){
  case HCI_TL_SPI_EXTI_PIN:
      hci_tl_lowlevel_isr();
      HCI_ProcessEvent=1;
    break;

  }
}

#ifdef  USE_FULL_ASSERT

void assert_failed(uint8_t* file, uint32_t line)
{

  while (1){
  }
}
#endif
