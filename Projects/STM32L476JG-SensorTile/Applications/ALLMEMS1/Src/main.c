/** CS704 Project 2: real sensor acquisition and motion tracking.
 * See the top-level README.md for units, calibration, build and hardware tests.
 * Original ST platform/BLE infrastructure retained from the course starter.
 */

/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include <math.h>

#include <limits.h>
#include "main.h"
#include "p2_sensor.h"
#include "p2_motion.h"
#include "p2_config.h"

#include "ble_interface.h"
#include "bluenrg_utils.h"
#include "SensorTile.h"


#include "SensorTile_bus.h"

   
/* Private typedef -----------------------------------------------------------*/

  
/* Private define ------------------------------------------------------------*/


/* Shutdown mode enabled as default for SensorTile */
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

/* Imported Variables -------------------------------------------------------------*/
extern uint8_t set_connectable;
extern int connected;

    
/* BlueNRG SPI */
extern SPI_HandleTypeDef SPI_SD_Handle;
    
extern volatile float RMS_Ch[];
extern float DBNOISE_Value_Old_Ch[];
extern uint16_t PCM_Buffer[];
extern uint32_t NumSample;

/* Exported Variables -------------------------------------------------------------*/
extern SPI_HandleTypeDef hbusspi2;

extern uint8_t NodeName[8];

//float sensitivity;

/* Acc sensitivity multiply by FROM_MG_TO_G constant */
//float sensitivity_Mul;


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


/* Private variables ---------------------------------------------------------*/
static volatile int MEMSInterrupt        =0;
static volatile uint32_t ReadSensor        =0;
static volatile uint32_t SendAccGyroMag  =0;
static volatile uint32_t TimeStamp = 0;
volatile uint32_t HCI_ProcessEvent=0;
typedef struct  {
	 uint32_t Steps;
	 uint32_t Heading;
	 uint32_t Distance;
}COMP_Data;
BSP_MOTION_SENSOR_Axes_t ACC_Value;
COMP_Data COMP_Value;
BSP_MOTION_SENSOR_Axes_t MAG_Value;
/* Private function prototypes -----------------------------------------------*/
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
void LSM303AGR_SPI_Read_nBytes(SPI_HandleTypeDef* xSpiHandle, uint8_t *val, uint16_t nBytesToRead);
void LSM303AGR_SPI_Read(SPI_HandleTypeDef* xSpiHandle, uint8_t *val);
void LSM303AGR_SPI_Write(SPI_HandleTypeDef* xSpiHandle, uint8_t val);


static const P2_SensorBus SensorBus = {
    BSP_LSM303AGR_ReadReg_Acc, BSP_LSM303AGR_WriteReg_Acc,
    BSP_LSM303AGR_ReadReg_Mag, BSP_LSM303AGR_WriteReg_Mag
};
static P2_Motion Motion;
static uint32_t SensorErrors;
static uint32_t LastAccTick;
static uint8_t HaveAcc;
static uint32_t LastMagTick;
static uint8_t HaveMag;
static uint8_t SpiFault;

static void InitLSM(void) {
    if (Sensor_IO_SPI_CS_Init_All() != HAL_OK) Error_Handler();
    if (!P2_SensorInit(&SensorBus)) {
        XPRINTF("LSM303AGR identity/configuration failed. Check hardware.\r\n");
        Error_Handler();
    }
    P2_MotionInit(&Motion);
    XPRINTF("LSM303AGR ready: 50 Hz, acc mg, mag mGauss.\r\n");
}

static void ProcessSensors(void) {
    P2_Vector value;
    uint32_t now = HAL_GetTick();
    int result = P2_SensorReadAcc(&SensorBus, &value);
    if (result > 0) {
        ACC_Value.x = (int32_t)lroundf(value.x);
        ACC_Value.y = (int32_t)lroundf(value.y);
        ACC_Value.z = (int32_t)lroundf(value.z);
        P2_MotionAcceleration(&Motion, value, now);
        LastAccTick = now;
        HaveAcc = 1;
    } else if (result < 0) ++SensorErrors;
    result = P2_SensorReadMag(&SensorBus, &value);
    if (result > 0) {
        LastMagTick = now;
        HaveMag = 1;
        MAG_Value.x = (int32_t)lroundf(value.x);
        MAG_Value.y = (int32_t)lroundf(value.y);
        MAG_Value.z = (int32_t)lroundf(value.z);
        if (HaveAcc && now - LastAccTick <= 100U) P2_MotionHeading(&Motion, value);
        else Motion.heading_valid = false;
    } else if (result < 0) ++SensorErrors;
    if (!HaveAcc || !HaveMag || now - LastAccTick > 100U || now - LastMagTick > 200U)
        Motion.heading_valid = false;
    COMP_Value.Steps = Motion.steps;
    COMP_Value.Heading = (uint32_t)lroundf(Motion.heading_deg) % 360U;
    /* BLE distance is tenths of a metre, matching the app's /10 gyroscope scale. */
    COMP_Value.Distance = Motion.distance_m < 3276.7f ?
        (uint32_t)lroundf(Motion.distance_m * 10.0f) : 32767U;
    static uint32_t last_log;
    if (now - last_log >= P2_LOG_PERIOD_MS) {
        last_log = now;
        XPRINTF("steps=%lu heading=%lu valid=%u errors=%lu acc=%ld,%ld,%ld mag=%ld,%ld,%ld\r\n",
            (unsigned long)Motion.steps, (unsigned long)COMP_Value.Heading,
            (unsigned)Motion.heading_valid, (unsigned long)SensorErrors,
            (long)ACC_Value.x, (long)ACC_Value.y, (long)ACC_Value.z,
            (long)MAG_Value.x, (long)MAG_Value.y, (long)MAG_Value.z);
    }
}

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
  
  HAL_Init();

  // Configure the System clock
  SystemClock_Config();

  InitTargetPlatform();

//  Initialize the BlueNRG
  Init_BlueNRG_Stack();

// Initialize the BlueNRG Custom services
  Init_BlueNRG_Custom_Services();

//  initialize timers
  InitTimers();
//
  if(HAL_TIM_Base_Start_IT(&TimEnvHandle) != HAL_OK){
    // Starting Error
    Error_Handler();
  }
  connected = FALSE;


  //***************************************************
  //***************** #CS704 **************************
  //************ Initialise here **********************
  //***************************************************
  //***************************************************
  //***************************************************

  //#CS704 - use this to set BLE Device Name
  NodeName[1] = 'A';
  NodeName[2] = 'B';
  NodeName[3] = 'C';
  NodeName[4] = 'D';
  NodeName[5] = 'E';
  NodeName[6] = 'F';
  NodeName[7] = 'G';

  /* Sensor configuration and motion state initialized in InitTargetPlatform. */
  //***************************************************
  //***************************************************
  //************ Initialise ends **********************
  //***************************************************


  /* Infinite loop */
  while (1)
  {
      if(!connected) {
          static uint32_t last_blink;
          if (HAL_GetTick() - last_blink >= 500U) {
              last_blink = HAL_GetTick();
        	  BSP_LED_Toggle(LED1);
//        	  HAL_UART_Transmit_IT(&UartHandle, (uint8_t*)BufferToWrite, 5);
          }
      }
    if(set_connectable){

      /* Now update the BLE advertize data and make the Board connectable */
      setConnectable();
      set_connectable = FALSE;
    }

    /* handle BLE event */
    if(HCI_ProcessEvent) {
      HCI_ProcessEvent=0;
      hci_user_evt_proc();
    }

    //***************************************************
    //***************************************************
    //***************** #CS704 **************************
    //*********** READ SENSORS & PROCESS ****************
    //***************************************************
    //***************************************************
    //***************************************************

    /* TIM4 polls at 100 Hz; data-ready gates processing to sensor ODR 50 Hz. */
    if (ReadSensor) {
        ReadSensor = 0;
        ProcessSensors();
    }

    //***************************************************
    //***************************************************


    /* Motion Data */
    if(SendAccGyroMag) {
        SendAccGyroMag=0;
        SendMotionData();
    }

    /* Wait for Event */
    __WFI();
  }
}

/**
  * @brief  Initialize all the Target platform's Features
  * @param  None
  * @retval None
  */
void InitTargetPlatform(void)
{


  /* Initialize LED */
  BSP_LED_Init( LED1 );
  /* Initialize UART */
  MX_UART5_UART_Init();

  InitLSM(); //N4S


}

/**
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */
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

//  if(HAL_UART_DeInit(&UartHandle) != HAL_OK)
//  {
//    Error_Handler();
//  }
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

void UART5_Transmit(uint8_t* BufferToWrite, uint16_t BytesToWrite) {

	HAL_UART_Transmit(&UartHandle, (uint8_t*)BufferToWrite, BytesToWrite,1000);
}


/**
  * @brief  Output Compare callback in non blocking mode 
  * @param  htim : TIM OC handle
  * @retval None
  */
void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
  uint32_t uhCapture=0;
  


  /* TIM1_CH4 toggling with frequency = 20 Hz */
  if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4)
  {
     uhCapture = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_4);
    /* Set the Capture Compare Register value */
    __HAL_TIM_SET_COMPARE(&TimCCHandle, TIM_CHANNEL_4, (uhCapture + uhCCR4_Val));
    SendAccGyroMag=1;
  }
}


/**
  * @brief  Period elapsed callback in non blocking mode for Environmental timer
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{


  if(htim == (&TimEnvHandle)) {
	  ReadSensor=1;


  }
}

/**
  * @brief  Send Motion Data Acc/Mag/Gyro to BLE
  * @param  None
  * @retval None
  */
static void SendMotionData(void)
{

  BSP_MOTION_SENSOR_Axes_t summary;
  /* Avoid aliasing unrelated structs and signed 16-bit BLE wraparound. */
  summary.x = COMP_Value.Steps > 3276U ? 3276 : (int32_t)COMP_Value.Steps;
  summary.y = Motion.heading_valid ? (int32_t)COMP_Value.Heading : -1;
  summary.z = (int32_t)COMP_Value.Distance;
  AccGyroMag_Update(&ACC_Value, &summary, &MAG_Value);
}



/**
* @brief  Function for initializing timers for sending the information to BLE:
 *  - 1 for sending MotionFX/AR/CP and Acc/Gyro/Mag
 *  - 1 for sending the Environmental info
 * @param  None
 * @retval None
 */
static void InitTimers(void)
{
  
  uint32_t uwPrescalerValue;
  
  /* Timer Output Compare Configuration Structure declaration */
  TIM_OC_InitTypeDef sConfig;
  
  /* Compute the prescaler value to have TIM4 counter clock equal to 10 KHz Hz */

 // #CS704 -  change TIM4 configuration here to change frequency of execution of *** READ Sensor and Process Block **
  uwPrescalerValue = (uint32_t) ((SystemCoreClock / 10000) - 1);

  /* Set TIM4 instance ( Environmental ) */
  TimEnvHandle.Instance = TIM4;
  /* Initialize TIM4 peripheral */
  TimEnvHandle.Init.Period = 99; /* 10 kHz / 100 = 100 Hz polling */
  TimEnvHandle.Init.Prescaler = uwPrescalerValue;
  TimEnvHandle.Init.ClockDivision = 0;
  TimEnvHandle.Init.CounterMode = TIM_COUNTERMODE_UP;
  if(HAL_TIM_Base_Init(&TimEnvHandle) != HAL_OK) {
    /* Initialization Error */
    Error_Handler();
  }


  /* Compute the prescaler value to have TIM1 counter clock equal to 10 KHz */
  uwPrescalerValue = (uint32_t) ((SystemCoreClock / 10000) - 1); 
  
  /* Set TIM1 instance ( Motion ) */
  TimCCHandle.Instance = TIM1;  
  TimCCHandle.Init.Period        = 65535;
  TimCCHandle.Init.Prescaler     = uwPrescalerValue;
  TimCCHandle.Init.ClockDivision = 0;
  TimCCHandle.Init.CounterMode   = TIM_COUNTERMODE_UP;
  if(HAL_TIM_OC_Init(&TimCCHandle) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }
  
 /* Configure the Output Compare channels */
 /* Common configuration for all channels */
  sConfig.OCMode     = TIM_OCMODE_TOGGLE;
  sConfig.OCPolarity = TIM_OCPOLARITY_LOW;

  /* Code for MotionFX integration - Start Section */
  /* Output Compare Toggle Mode configuration: Channel1 */
  sConfig.Pulse = DEFAULT_uhCCR1_Val;
  if(HAL_TIM_OC_ConfigChannel(&TimCCHandle, &sConfig, TIM_CHANNEL_1) != HAL_OK)
  {
    /* Configuration Error */
    Error_Handler();
  }
  /* Code for MotionFX integration - End Section */

  /* Code for MotionCP & MotionGR integration - Start Section */
  /* Output Compare Toggle Mode configuration: Channel2 */
  sConfig.Pulse = DEFAULT_uhCCR2_Val;
  if(HAL_TIM_OC_ConfigChannel(&TimCCHandle, &sConfig, TIM_CHANNEL_2) != HAL_OK)
  {
    /* Configuration Error */
    Error_Handler();
  }
  /* Code for MotionCP & MotionGR integration - End Section */

  /* Code for MotionAR & MotionID & MotionPE integration - Start Section */
  /* Output Compare Toggle Mode configuration: Channel3 */
  sConfig.Pulse = DEFAULT_uhCCR3_Val;
  if(HAL_TIM_OC_ConfigChannel(&TimCCHandle, &sConfig, TIM_CHANNEL_3) != HAL_OK)
  {
    /* Configuration Error */
    Error_Handler();
  }
  /* Code for MotionAR & MotionID & MotionPE integration - End Section */

  /* Output Compare Toggle Mode configuration: Channel4 */
  sConfig.Pulse = DEFAULT_uhCCR4_Val;
  if(HAL_TIM_OC_ConfigChannel(&TimCCHandle, &sConfig, TIM_CHANNEL_4) != HAL_OK)
  {
    /* Configuration Error */
    Error_Handler();
  }
}


/**
  * @brief  Set all sensor Chip Select high. To be called before any SPI read/write
  * @param  None
  * @retval HAL_StatusTypeDef HAL Status
  */
uint8_t Sensor_IO_SPI_CS_Init_All(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;
  uint8_t inData[2];
  /* Set all the pins before init to avoid glitch */
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


  //setup SPI interface
    if (BSP_SPI2_Init() != BSP_ERROR_NONE) return HAL_ERROR;

  //disable I2C modes on all SPI devices
  //LSM303 Mag
  HAL_GPIO_WritePin(BSP_LSM303AGR_M_CS_PORT, BSP_LSM303AGR_M_CS_PIN, GPIO_PIN_RESET);
  inData[0] = (0x62U);
  BSP_SPI2_Send(inData,1);
  inData[0] = 0x20;
  BSP_SPI2_Send(inData,1);
  HAL_GPIO_WritePin(BSP_LSM303AGR_M_CS_PORT, BSP_LSM303AGR_M_CS_PIN, GPIO_PIN_SET);

  //LSM303 Acc
  HAL_GPIO_WritePin(BSP_LSM303AGR_X_CS_PORT, BSP_LSM303AGR_X_CS_PIN, GPIO_PIN_RESET);
  inData[0] = (0x23U);
  BSP_SPI2_Send(inData,1);
  inData[0] = 0x01;
  BSP_SPI2_Send(inData,1);
  HAL_GPIO_WritePin(BSP_LSM303AGR_X_CS_PORT, BSP_LSM303AGR_X_CS_PIN, GPIO_PIN_SET);

  //LPS22HB
  HAL_GPIO_WritePin(BSP_LPS22HB_CS_PORT, BSP_LPS22HB_CS_PIN, GPIO_PIN_RESET);
  inData[0] = (0x10U);
  BSP_SPI2_Send(inData,1);
  inData[0] = 0x01;
  BSP_SPI2_Send(inData,1);
  HAL_GPIO_WritePin(BSP_LPS22HB_CS_PORT, BSP_LPS22HB_CS_PIN, GPIO_PIN_SET);

  //LSM6DSM
  HAL_GPIO_WritePin(BSP_LSM6DSM_CS_PORT, BSP_LSM6DSM_CS_PIN, GPIO_PIN_RESET);
  inData[0] = (0x12U);
  BSP_SPI2_Send(inData,1);
  inData[0] = 0x0C;
  BSP_SPI2_Send(inData,1);
  HAL_GPIO_WritePin(BSP_LSM6DSM_CS_PORT, BSP_LSM6DSM_CS_PIN, GPIO_PIN_SET);


  return HAL_OK;
}

/**
 * @brief  Write register by SPI bus for LSM303AGR
 * @param  Reg the starting register address to be written
 * @param  pdata the pointer to the data to be written
 * @param  len the length of the data to be written
 * @retval BSP status
 */
static int32_t BSP_LSM303AGR_WriteReg_Mag(uint16_t Reg, uint8_t *pdata, uint16_t len)
{
  int32_t ret = BSP_ERROR_NONE;
  uint8_t dataReg = (uint8_t)Reg;
  SpiFault = 0;

  /* CS Enable */
  LSM_MAG_CS_LOW();

  if (BSP_SPI2_Send(&dataReg, 1) != 1)
  {
    ret = BSP_ERROR_UNKNOWN_FAILURE;
  }

  if (BSP_SPI2_Send(pdata, len) != len)
  {
    ret = BSP_ERROR_UNKNOWN_FAILURE;
  }

  /* CS Disable */
  LSM_MAG_CS_HIGH();

  return SpiFault ? BSP_ERROR_UNKNOWN_FAILURE : ret;
}

/**
* @brief  Read register by SPI bus for LSM303AGR
* @param  Reg the starting register address to be read
* @param  pdata the pointer to the data to be read
* @param  len the length of the data to be read
* @retval BSP status
*/
static int32_t BSP_LSM303AGR_ReadReg_Mag(uint16_t Reg, uint8_t *pdata, uint16_t len)
{
  int32_t ret = BSP_ERROR_NONE;
  uint8_t dataReg = (uint8_t)Reg;
  SpiFault = 0;

  /* CS Enable */
  HAL_GPIO_WritePin(BSP_LSM303AGR_M_CS_PORT, BSP_LSM303AGR_M_CS_PIN, GPIO_PIN_RESET);
//  XPRINTF("Data Read = %d,%d\r\n",(dataReg) | 0x80,pdata[0]);
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

  /* CS Disable */
  HAL_GPIO_WritePin(BSP_LSM303AGR_M_CS_PORT, BSP_LSM303AGR_M_CS_PIN, GPIO_PIN_SET);
  SPI_1LINE_TX(&hbusspi2);
  __HAL_SPI_ENABLE(&hbusspi2);
  return SpiFault ? BSP_ERROR_UNKNOWN_FAILURE : ret;
}

/**
 * @brief  Write register by SPI bus for LSM303AGR
 * @param  Reg the starting register address to be written
 * @param  pdata the pointer to the data to be written
 * @param  len the length of the data to be written
 * @retval BSP status
 */
static int32_t BSP_LSM303AGR_WriteReg_Acc(uint16_t Reg, uint8_t *pdata, uint16_t len)
{
  int32_t ret = BSP_ERROR_NONE;
  uint8_t dataReg = (uint8_t)Reg;
  SpiFault = 0;

  /* CS Enable */
  LSM_ACC_CS_LOW();

  if (BSP_SPI2_Send(&dataReg, 1) != 1)
  {
    ret = BSP_ERROR_UNKNOWN_FAILURE;
  }

  if (BSP_SPI2_Send(pdata, len) != len)
  {
    ret = BSP_ERROR_UNKNOWN_FAILURE;
  }

  /* CS Disable */
  LSM_ACC_CS_HIGH();

  return SpiFault ? BSP_ERROR_UNKNOWN_FAILURE : ret;
}

/**
* @brief  Read register by SPI bus for LSM303AGR
* @param  Reg the starting register address to be read
* @param  pdata the pointer to the data to be read
* @param  len the length of the data to be read
* @retval BSP status
*/
static int32_t BSP_LSM303AGR_ReadReg_Acc(uint16_t Reg, uint8_t *pdata, uint16_t len)
{
  int32_t ret = BSP_ERROR_NONE;
  uint8_t dataReg = (uint8_t)Reg;
  SpiFault = 0;

  /* CS Enable */
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

  /* CS Disable */
  LSM_ACC_CS_HIGH();
  SPI_1LINE_TX(&hbusspi2);
  __HAL_SPI_ENABLE(&hbusspi2);
  return SpiFault ? BSP_ERROR_UNKNOWN_FAILURE : ret;
}


/**
* @brief  This function reads multiple bytes on SPI 3-wire.
* @param  xSpiHandle: SPI Handler.
* @param  val: value.
* @param  nBytesToRead: number of bytes to read.
* @retval None
*/
void LSM303AGR_SPI_Read_nBytes(SPI_HandleTypeDef* xSpiHandle, uint8_t *val, uint16_t nBytesToRead)
{
  /* Interrupts should be disabled during this operation */
  uint32_t saved_mask = __get_PRIMASK();
  uint32_t budget = 100000U;
  __disable_irq();
  __HAL_SPI_ENABLE(xSpiHandle);

  /* Transfer loop */
  while (nBytesToRead > 1U)
  {
    if (--budget == 0U) {
      SpiFault = 1;
      __HAL_SPI_DISABLE(xSpiHandle);
      __set_PRIMASK(saved_mask);
      return;
    }
    /* Check the RXNE flag */
    if (xSpiHandle->Instance->SR & SPI_FLAG_RXNE)
    {
      /* read the received data */
      *val = *(__IO uint8_t *) &xSpiHandle->Instance->DR;
      val += sizeof(uint8_t);
      nBytesToRead--;
    }
  }
  /* In master RX mode the clock is automaticaly generated on the SPI enable.
  So to guarantee the clock generation for only one data, the clock must be
  disabled after the first bit and before the latest bit of the last Byte received */
  /* __DSB instruction are inserted to garantee that clock is Disabled in the right timeframe */

  __DSB();
  __DSB();
  __HAL_SPI_DISABLE(xSpiHandle);

  __set_PRIMASK(saved_mask);

  budget = 100000U;
  while ((xSpiHandle->Instance->SR & SPI_FLAG_RXNE) != SPI_FLAG_RXNE) { if (--budget == 0U) { SpiFault = 1; return; } }
  /* read the received data */
  *val = *(__IO uint8_t *) &xSpiHandle->Instance->DR;
  budget = 100000U;
  while ((xSpiHandle->Instance->SR & SPI_FLAG_BSY) == SPI_FLAG_BSY) { if (--budget == 0U) { SpiFault = 1; return; } }
}

/**
* @brief  This function send a command through SPI bus.
* @param  command: command id.
* @param  uint8_t val: value.
* @retval None
*/
void LSM303AGR_SPI_Read(SPI_HandleTypeDef* xSpiHandle, uint8_t *val)
{
  /* In master RX mode the clock is automaticaly generated on the SPI enable.
  So to guarantee the clock generation for only one data, the clock must be
  disabled after the first bit and before the latest bit */
  /* Interrupts should be disabled during this operation */

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
  /* read the received data */
//  XPRINTF("Before READ, %d,%d\r\n",val[0],xSpiHandle->Instance);
  *val = *(__IO uint8_t *) &xSpiHandle->Instance->DR;
//  XPRINTF("READ, %d\r\n",val[0]);
  budget = 100000U;
  while ((xSpiHandle->Instance->SR & SPI_FLAG_BSY) == SPI_FLAG_BSY) { if (--budget == 0U) { SpiFault = 1; return; } }
}

/**
* @brief  This function send a command through SPI bus.
* @param  command : command id.
* @param  val : value.
* @retval None
*/
void LSM303AGR_SPI_Write(SPI_HandleTypeDef* xSpiHandle, uint8_t val)
{
  uint32_t budget;
  /* check TXE flag */
  budget = 100000U;
  while ((xSpiHandle->Instance->SR & SPI_FLAG_TXE) != SPI_FLAG_TXE) { if (--budget == 0U) { SpiFault = 1; return; } }

  /* Write the data */
  *((__IO uint8_t*) &xSpiHandle->Instance->DR) = val;

  /* Wait BSY flag */
  budget = 100000U;
  while ((xSpiHandle->Instance->SR & SPI_FLAG_FTLVL) != SPI_FTLVL_EMPTY) { if (--budget == 0U) { SpiFault = 1; return; } }
  budget = 100000U;
  while ((xSpiHandle->Instance->SR & SPI_FLAG_BSY) == SPI_FLAG_BSY) { if (--budget == 0U) { SpiFault = 1; return; } }
}




/**
* @brief  Function for De-initializing timers:
 *  - 1 for sending MotionFX/AR/CP and Acc/Gyro/Mag
 *  - 1 for sending the Environmental info
 * @param  None
 * @retval None
 */
/** @brief Initialize the BlueNRG Stack
 * @param None
 * @retval None
 */
static void Init_BlueNRG_Stack(void)
{
  char BoardName[8];
  char customName[8] = "CSys704";
  uint16_t service_handle, dev_name_char_handle, appearance_char_handle;
  int ret;
  uint8_t  data_len_out;
  uint8_t  hwVersion;
  uint16_t fwVersion;
  


//  for(int i=0; i<7; i++)
//    BoardName[i]= NodeName[i+1];

  for(int i=0; i<7; i++)
    BoardName[i]= customName[i];
  
  BoardName[7]= 0;
  
  /* Initialize the BlueNRG SPI driver */
  hci_init(HCI_Event_CB, NULL);

  /* get the BlueNRG HW and FW versions */
  getBlueNRGVersion(&hwVersion, &fwVersion);
  
  aci_hal_read_config_data(CONFIG_DATA_RANDOM_ADDRESS, 6, &data_len_out, bdaddr);

  if ((bdaddr[5] & 0xC0) != 0xC0) {
    XPRINTF("\r\nStatic Random address not well formed.\r\n");
    while(1);
  }
  
  ret = aci_hal_write_config_data(CONFIG_DATA_PUBADDR_OFFSET, data_len_out,
                                  bdaddr);
  
/* Sw reset of the device */
  hci_reset();

  ret = aci_gatt_init();    
  if(ret){
     XPRINTF("\r\nGATT_Init failed\r\n");
     goto fail;
  }

  ret = aci_gap_init_IDB05A1(GAP_PERIPHERAL_ROLE_IDB05A1, 0, 0x07, &service_handle, &dev_name_char_handle, &appearance_char_handle);

  if(ret != BLE_STATUS_SUCCESS){
     XPRINTF("\r\nGAP_Init failed\r\n");
     goto fail;
  }

  ret = aci_gatt_update_char_value(service_handle, dev_name_char_handle, 0,
                                   7/*strlen(BoardName)*/, (uint8_t *)BoardName);

  if(ret){
     XPRINTF("\r\naci_gatt_update_char_value failed\r\n");
    while(1);
  }

  ret = aci_gap_set_auth_requirement(MITM_PROTECTION_REQUIRED,
                                     OOB_AUTH_DATA_ABSENT,
                                     NULL, 7, 16,
                                     USE_FIXED_PIN_FOR_PAIRING, 123456,
                                     BONDING);
  if (ret != BLE_STATUS_SUCCESS) {
     XPRINTF("\r\nGAP setting Authentication failed\r\n");
     goto fail;
  }

  XPRINTF("SERVER: BLE Stack Initialized \r\n"
         "\tHWver= %d.%d\r\n"
         "\tFWver= %d.%d.%c\r\n"
         "\tBoardName= %s\r\n"
         "\tBoardMAC = %x:%x:%x:%x:%x:%x\r\n\n",
         ((hwVersion>>4)&0x0F),(hwVersion&0x0F),
         fwVersion>>8,
         (fwVersion>>4)&0xF,
         ('a'+(fwVersion&0xF)),
         BoardName,
         bdaddr[5],bdaddr[4],bdaddr[3],bdaddr[2],bdaddr[1],bdaddr[0]);

  /* Set output power level */
  aci_hal_set_tx_power_level(1,4);

  return;

fail:
  return;
}

/** @brief Initialize all the Custom BlueNRG services
 * @param None
 * @retval None
 */
static void Init_BlueNRG_Custom_Services(void)
{
  int ret;
  
  ret = Add_HW_SW_ServW2ST_Service();
  if(ret == BLE_STATUS_SUCCESS)
  {
     XPRINTF("HW & SW Service W2ST added successfully\r\n");
  }
  else
  {
     XPRINTF("\r\nError while adding HW & SW Service W2ST\r\n");
  }

}

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow : 
  *            System Clock source            = PLL (MSI)
  *            SYSCLK(Hz)                     = 80000000
  *            HCLK(Hz)                       = 80000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 1
  *            APB2 Prescaler                 = 1
  *            MSI Frequency(Hz)              = 4000000
  *            PLL_M                          = 6
  *            PLL_N                          = 40
  *            PLL_R                          = 4
  *            PLL_P                          = 7
  *            PLL_Q                          = 4
  *            Flash Latency(WS)              = 4
  * @param  None
  * @retval None
  */
static void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
  
  __HAL_RCC_PWR_CLK_ENABLE();
  HAL_PWR_EnableBkUpAccess();
  
  /* Enable the LSE Oscilator */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK){
    while(1);
  }
  
  /* Enable the CSS interrupt in case LSE signal is corrupted or not present */
  HAL_RCCEx_DisableLSECSS();
  
  /* Enable MSI Oscillator and activate PLL with MSI as source */
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
  
  /* Enable MSI Auto-calibration through LSE */
  HAL_RCCEx_EnableMSIPLLMode();
  
  /* Select MSI output as USB clock source */
//  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USB;
//  PeriphClkInitStruct.UsbClockSelection = RCC_USBCLKSOURCE_MSI;
//  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct);
  
  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 
  clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK){
    while(1);
  }
}






/**
  * @brief This function provides accurate delay (in milliseconds) based 
  *        on variable incremented.
  * @note This is a user implementation using WFI state
  * @param Delay: specifies the delay time length, in milliseconds.
  * @retval None
  */
void HAL_Delay(__IO uint32_t Delay)
{
  uint32_t tickstart = 0;
  tickstart = HAL_GetTick();
  while((HAL_GetTick() - tickstart) < Delay){
    __WFI();
  }
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* Distinguishable failure indication; no fake motion on initialization failure. */
  while (1) { BSP_LED_Toggle(LED1); HAL_Delay(150); }
}

/**
 * @brief  EXTI line detection callback.
 * @param  uint16_t GPIO_Pin Specifies the pins connected EXTI line
 * @retval None
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{  
  switch(GPIO_Pin){
  case HCI_TL_SPI_EXTI_PIN: 
      hci_tl_lowlevel_isr();
      HCI_ProcessEvent=1;
    break;

//  case BSP_LSM6DSM_INT2:
//    MEMSInterrupt=1;
//    break;
  }
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: ALLMEMS1_PRINTF("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1){
  }
}
#endif




