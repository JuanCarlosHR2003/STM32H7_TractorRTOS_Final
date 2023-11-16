/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "myprintf.h"
//#include "MPU9250.h"
#include "mpu9250.h"
#include "doublyLinkedList.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#ifndef HSEM_ID_0
#define HSEM_ID_0 (0U) /* HW semaphore 0*/
#endif
#define MPU_SPI hspi3
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

SPI_HandleTypeDef hspi3;

TIM_HandleTypeDef htim13;
TIM_HandleTypeDef htim14;

UART_HandleTypeDef huart3;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */

//float AccData[3], GyroData[3], MagData[3];
// Calibration offsets
float AccOffset[3] = {0.0f, 0.0f, 0.0f};
float GyroOffset[3] = {0.0f, 0.0f, 0.0f};
float MagOffset[3] = {0.0f, 0.0f, 0.0f};

/*typedef struct mpu9250_t{
  int16_t accel_x;
  int16_t accel_y;
  int16_t accel_z;
  int16_t gyro_x;
  int16_t gyro_y;
  int16_t gyro_z;
  int16_t mag_x;
  int16_t mag_y;
  int16_t mag_z;
  float temp;
  float accel_resolution;
  float gyro_resolution;
  float mag_resolution;
  GPIO_TypeDef* cs_gpio_port;
  uint16_t cs_gpio_pin;
  SPI_HandleTypeDef* hspi;
} mpu9250_t;

mpu9250_t mpu;*/ //Desertkun

// mpu9250 mpu; // Danny

struct doubleLinkedList gyro_list[3];
struct doubleLinkedList acce_list[3];
struct doubleLinkedList mag_list[3];
int n_window = 10;

uint8_t ak8963_WhoAmI = 0;
uint8_t mpu9250_WhoAmI = 0;
MPU9250 mpu; //sin1111

osThreadId_t Handle_Action_TractionSetpoint;
osThreadId_t Handle_Task_Steering;
osThreadId_t Handle_Task_StateMachine;
osThreadId_t Handle_Task_UART;
osThreadId_t Handle_Task_MPU9250;

const osThreadAttr_t Attributes_Action_TractionSetpoint = {
  .name = "Action_TractionSetpoint",
  .stack_size = 128 * 8,
  .priority = (osPriority_t) osPriorityNormal,
};

const osThreadAttr_t Attributes_Task_Steering = {
  .name = "Task_Steering",
  .stack_size = 128 * 8,
  .priority = (osPriority_t) osPriorityAboveNormal,
};

const osThreadAttr_t Attributes_Task_StateMachine = {
  .name = "Task_StateMachine",
  .stack_size = 128 * 8,
  .priority = (osPriority_t) osPriorityAboveNormal,
};

const osThreadAttr_t Attributes_Task_UART = {
  .name = "Task_UART",
  .stack_size = 128 * 8,
  .priority = (osPriority_t) osPriorityNormal,
};

// MPU9250 Thread
const osThreadAttr_t Attributes_Task_MPU9250 = {
  .name = "Task_MPU9250",
  .stack_size = 128 * 8,
  .priority = (osPriority_t) osPriorityHigh,
};


static float* traction_setpoint;
static float delta_steering;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM14_Init(void);
static void MX_TIM13_Init(void);
static void MX_SPI3_Init(void);
void StartDefaultTask(void *argument);

/* USER CODE BEGIN PFP */
void Function_Action_TractionSetpoint(void *argument);
void Function_Task_Steering(void *argument);
void Function_Task_StateMachine(void *argument);
void Function_Task_UART(void *argument);
void Function_Task_MPU9250(void *argument);
void calibrate_MPU9250(SPI_HandleTypeDef *spi);
void initDoubleLinkedList(struct doubleLinkedList* list[], int n);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */
/* USER CODE BEGIN Boot_Mode_Sequence_0 */
	int32_t timeout;


/* USER CODE END Boot_Mode_Sequence_0 */

/* USER CODE BEGIN Boot_Mode_Sequence_1 */
  /* Wait until CPU2 boots and enters in stop mode or timeout*/
  timeout = 0xFFFF;
  while((__HAL_RCC_GET_FLAG(RCC_FLAG_D2CKRDY) != RESET) && (timeout-- > 0));
  if ( timeout < 0 )
  {
  Error_Handler();
  }
/* USER CODE END Boot_Mode_Sequence_1 */
  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

/* Configure the peripherals common clocks */
  PeriphCommonClock_Config();
/* USER CODE BEGIN Boot_Mode_Sequence_2 */
/* When system initialization is finished, Cortex-M7 will release Cortex-M4 by means of
HSEM notification */
/*HW semaphore Clock enable*/
__HAL_RCC_HSEM_CLK_ENABLE();
/*Take HSEM */
HAL_HSEM_FastTake(HSEM_ID_0);
/*Release HSEM in order to notify the CPU2(CM4)*/
HAL_HSEM_Release(HSEM_ID_0,0);
/* wait until CPU2 wakes up from stop mode */
timeout = 0xFFFF;
while((__HAL_RCC_GET_FLAG(RCC_FLAG_D2CKRDY) == RESET) && (timeout-- > 0));
if ( timeout < 0 )
{
Error_Handler();
}
/* USER CODE END Boot_Mode_Sequence_2 */

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART3_UART_Init();
  MX_TIM14_Init();
  MX_TIM13_Init();
  MX_SPI3_Init();
  /* USER CODE BEGIN 2 */
  printf("Initializing MPU...\r\n");



    /*//Danny

    mpu.hspi = &hspi3;
    mpu.cs_gpio_port = SPI3_CS_GPIO_Port;
    mpu.cs_gpio_pin  = SPI3_CS_Pin;

    mpu9250_set_accel_resolution(&mpu, ACCEL_RESOLUTION_8G);
    mpu9250_set_gyro_resolution( &mpu, GYRO_RESOLUTION_2000DPS);

    mpu.mag_resolution =  (10.0 * 4912.0 / 8190.0);*/

  for (int i = 0; i < sizeof(gyro_list); i++) {
    DBLL_init(&gyro_list[i], n_window);
    DBLL_init(&acce_list[i], n_window);
    DBLL_init(&mag_list[i], n_window);
  }

  MPU9250_Init(&mpu);
  printf("Calibrating MPU...\r\n");
  calibrate_MPU9250(&MPU_SPI);
  printf("Calibration complete!\r\n");
  printf("Calibration offsets:\r\n");
  printf("Accel: %.2f %.2f %.2f\r\n", AccOffset[0], AccOffset[1], AccOffset[2]);
  printf("Gyro: %.2f %.2f %.2f\r\n", GyroOffset[0], GyroOffset[1], GyroOffset[2]);
  printf("Mag: %.2f %.2f %.2f\r\n", MagOffset[0], MagOffset[1], MagOffset[2]);
  HAL_Delay(2000);


  //MPU9250_Init();


  printf("PreeRTOS\r\n");

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  traction_setpoint = malloc(sizeof(float));
  //float *arg = malloc(sizeof(float));
  /*arg = 0.75f;
  //float starting_traction = 0.75;
  //Handle_Action_TractionSetpoint = osThreadNew(Function_Action_TractionSetpoint, (void *)arg, &Attributes_Action_TractionSetpoint);
   */
  Handle_Task_Steering     = osThreadNew(Function_Task_Steering, NULL, &Attributes_Task_Steering);
  Handle_Task_StateMachine = osThreadNew(Function_Task_StateMachine, NULL, &Attributes_Task_StateMachine);
  Handle_Task_UART         = osThreadNew(Function_Task_UART, NULL, &Attributes_Task_UART);
  Handle_Task_MPU9250      = osThreadNew(Function_Task_MPU9250, NULL, &Attributes_Task_MPU9250);
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_DIRECT_SMPS_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 2;
  RCC_OscInitStruct.PLL.PLLN = 240;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_CKPER;
  PeriphClkInitStruct.CkperClockSelection = RCC_CLKPSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi3.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 0x0;
  hspi3.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi3.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi3.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi3.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi3.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi3.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi3.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi3.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi3.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi3.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

}

/**
  * @brief TIM13 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM13_Init(void)
{

  /* USER CODE BEGIN TIM13_Init 0 */

  /* USER CODE END TIM13_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM13_Init 1 */

  /* USER CODE END TIM13_Init 1 */
  htim13.Instance = TIM13;
  htim13.Init.Prescaler = 0;
  htim13.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim13.Init.Period = 65535;
  htim13.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim13.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim13) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim13) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim13, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM13_Init 2 */


  TIM13->ARR = 63999;
  TIM13->PSC = 74;
  TIM13->CCR1 = (uint32_t)(63999*0.15);
  HAL_TIM_PWM_Start(&htim13, TIM_CHANNEL_1);

  /* USER CODE END TIM13_Init 2 */
  HAL_TIM_MspPostInit(&htim13);

}

/**
  * @brief TIM14 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM14_Init(void)
{

  /* USER CODE BEGIN TIM14_Init 0 */

  /* USER CODE END TIM14_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM14_Init 1 */

  /* USER CODE END TIM14_Init 1 */
  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 0;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 65535;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim14, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM14_Init 2 */

  TIM14->ARR = 63999;
  TIM14->PSC = 74;
  TIM14->CCR1 = (uint32_t)(63999*0.15);
  HAL_TIM_PWM_Start(&htim14, TIM_CHANNEL_1);

  /* USER CODE END TIM14_Init 2 */
  HAL_TIM_MspPostInit(&htim14);

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI3_CS_GPIO_Port, SPI3_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD3_Pin|H_IN_1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B2_Pin */
  GPIO_InitStruct.Pin = B2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI3_CS_Pin */
  GPIO_InitStruct.Pin = SPI3_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SPI3_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD1_Pin LD3_Pin H_IN_1_Pin */
  GPIO_InitStruct.Pin = LD1_Pin|LD3_Pin|H_IN_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void Function_Action_TractionSetpoint(void *argument){
    float setpoint = *((float *)argument);

    //printf("Setpoint: %f\r\n", setpoint);
    TIM14->CCR1 = (uint32_t)(63999*setpoint);


    HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET);
    osDelay(100);
    HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);
    osDelay(100);
    HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET);
	  osDelay(100);
	  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);
	  osDelay(100);

    osThreadTerminate(Handle_Action_TractionSetpoint);
}

void Function_Task_Steering(void *argument){
    for(;;){
      TIM13->CCR1 = (uint32_t)((63999*0.05)+(63999*0.05*delta_steering));
      //printf("Steering: %f\r\n", delta_steering);
      osDelay(100);
    }
}

void Function_Task_StateMachine(void *argument){
  uint8_t state = 0;
  for(;;){
    if(state == 0){
        delta_steering = 0.5f;
        *traction_setpoint = 0.5f;
	    Handle_Action_TractionSetpoint = osThreadNew(Function_Action_TractionSetpoint, (void *)traction_setpoint, &Attributes_Action_TractionSetpoint);
        HAL_GPIO_WritePin(H_IN_1_GPIO_Port, H_IN_1_Pin, GPIO_PIN_RESET);
        osDelay(5000);
        state = 1;
      }else if(state == 1){
        delta_steering = 0.75f;
        HAL_GPIO_WritePin(H_IN_1_GPIO_Port, H_IN_1_Pin, GPIO_PIN_SET);
        osDelay(3000);
        delta_steering = 0.25f;
        osDelay(3000);
        state = 2;
      }else if(state == 2){
        delta_steering = 0.5f;
        HAL_GPIO_WritePin(H_IN_1_GPIO_Port, H_IN_1_Pin, GPIO_PIN_RESET);
        osDelay(5000);
        HAL_GPIO_WritePin(H_IN_1_GPIO_Port, H_IN_1_Pin, GPIO_PIN_SET);
        osDelay(3000);
        state = 0;
      }else{
        state = 0;
      }
  }
}

void Function_Task_UART(void *argument){
  for(;;){
    HAL_GPIO_TogglePin(LD1_GPIO_Port, LD1_Pin);
    printf("Hello World!\r\n");
    /*printf("Accel: %.2f %.2f %.2f\r\n", mpu.accel_x, mpu.accel_y, mpu.accel_z);
    printf("Gyro: %.2f %.2f %.2f\r\n", mpu.gyro_x, mpu.gyro_y, mpu.gyro_z);
    printf("Mag: %.2f %.2f %.2f\r\n", mpu.mag_x, mpu.mag_y, mpu.mag_z);*/
    /*printf("Accel: %d %d %d\r\n", AccData[0], AccData[1], AccData[2]);
	  printf("Gyro: %d %d %d\r\n", GyroData[0], GyroData[1], GyroData[2]);
	  printf("Mag: %d %d %df\r\n", MagData[0], MagData[1],MagData[2]);*/
    /*printf("Accel: %.3f %.3f %.3f\r\n", AccData[0], AccData[1], AccData[2]);
    printf("Gyro: %.3f %.3f %.3f\r\n", GyroData[0], GyroData[1], GyroData[2]);
    printf("Mag: %.3f %.3f %.3ff\r\n", MagData[0], MagData[1],MagData[2]);*/
    printf("Accel: %.3f %.3f %.3f\r\n", acce_list[0]->mean, acce_list[1]->mean, acce_list[2]->mean);
    printf("Gyro: %.3f %.3f %.3f\r\n", gyro_list[0]->mean, gyro_list[1]->mean, gyro_list[2]->mean);
    printf("Mag: %.3f %.3f %.3ff\r\n", mag_list[0]->mean, mag_list[1]->mean,mag_list[2]->mean);
    osDelay(100);
  }
}

void Function_Task_MPU9250(void *argument){
  for(;;){


	//mpu9250_update_accel_gyro(&mpu);


	/*MPU9250_GetData(AccData, GyroData, MagData);

	mpu.accel_x = AccData[0];
	mpu.accel_y = AccData[1];
	mpu.accel_z = AccData[2];
	mpu.gyro_x = GyroData[0];
	mpu.gyro_y = GyroData[1];
	mpu.gyro_z = GyroData[2];
	mpu.mag_x = MagData[0];
	mpu.mag_y = MagData[1];
	mpu.mag_z = MagData[2];*/

	ak8963_WhoAmI = mpu_r_ak8963_WhoAmI(&mpu);
	mpu9250_WhoAmI = mpu_r_WhoAmI(&mpu);
	MPU9250_ReadAccel(&mpu);
	MPU9250_ReadGyro(&mpu);
	MPU9250_ReadMag(&mpu);

  push_back(&acc_list[0], mpu.mpu_data.Accel[0] - AccOffset[0]);
  push_back(&acc_list[1], mpu.mpu_data.Accel[1] - AccOffset[1]);
  push_back(&acc_list[2], mpu.mpu_data.Accel[2] - AccOffset[2]);
  push_back(&gyro_list[0], mpu.mpu_data.Gyro[0] - GyroOffset[0]);
  push_back(&gyro_list[1], mpu.mpu_data.Gyro[1] - GyroOffset[1]);
  push_back(&gyro_list[2], mpu.mpu_data.Gyro[2] - GyroOffset[2]);
  push_back(&mag_list[0], mpu.mpu_data.Magn[0] - MagOffset[0]);
  push_back(&mag_list[1], mpu.mpu_data.Magn[1] - MagOffset[1]);
  push_back(&mag_list[2], mpu.mpu_data.Magn[2] - MagOffset[2]);


  /*
	AccData[0] = mpu.mpu_data.Accel[0] - AccOffset[0];
  AccData[1] = mpu.mpu_data.Accel[1] - AccOffset[1];
  AccData[2] = mpu.mpu_data.Accel[2] - AccOffset[2];
  GyroData[0] = mpu.mpu_data.Gyro[0] - GyroOffset[0];
  GyroData[1] = mpu.mpu_data.Gyro[1] - GyroOffset[1];
  GyroData[2] = mpu.mpu_data.Gyro[2] - GyroOffset[2];
  MagData[0] = mpu.mpu_data.Magn[0] - MagOffset[0];
  MagData[1] = mpu.mpu_data.Magn[1] - MagOffset[1];
  MagData[2] = mpu.mpu_data.Magn[2] - MagOffset[2];
  */

  osDelay(50);
  }
}

void calibrate_MPU9250(SPI_HandleTypeDef *spi){
	float AccAccum[3] = {0.0f, 0.0f, 0.0f};
  float GyroAccum[3] = {0.0f, 0.0f, 0.0f};
  float MagAccum[3] = {0.0f, 0.0f, 0.0f};

  uint8_t num_samples = 50;
  for (uint8_t i = 0; i < num_samples; i++){
    MPU9250_ReadAccel(&mpu);
    MPU9250_ReadGyro(&mpu);
    MPU9250_ReadMag(&mpu);

    AccAccum[0] += mpu.mpu_data.Accel[0];
    AccAccum[1] += mpu.mpu_data.Accel[1];
    AccAccum[2] += mpu.mpu_data.Accel[2];
    GyroAccum[0] += mpu.mpu_data.Gyro[0];
    GyroAccum[1] += mpu.mpu_data.Gyro[1];
    GyroAccum[2] += mpu.mpu_data.Gyro[2];
    MagAccum[0] += mpu.mpu_data.Magn[0];
    MagAccum[1] += mpu.mpu_data.Magn[1];
    MagAccum[2] += mpu.mpu_data.Magn[2];

    HAL_Delay(50);
  }

  AccOffset[0] = AccAccum[0] / num_samples;
  AccOffset[1] = AccAccum[1] / num_samples;
  AccOffset[2] = AccAccum[2] / num_samples;
  GyroOffset[0] = GyroAccum[0] / num_samples;
  GyroOffset[1] = GyroAccum[1] / num_samples;
  GyroOffset[2] = GyroAccum[2] / num_samples;
  MagOffset[0] = MagAccum[0] / num_samples;
  MagOffset[1] = MagAccum[1] / num_samples;
  MagOffset[2] = MagAccum[2] / num_samples;

}

void initDoubleLinkedList(struct doubleLinkedList* list[], int n){
  for(int i = 0; i < n; i++){
    //list[i] = malloc(sizeof(struct doubleLinkedList));
    DBLL_init(&list[i], n_window);
  }
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  /*float setpoint = 0.8f;
  float delta = 0.8f;
  for(;;){
	  osDelay(3000);
	  *traction_setpoint = setpoint;
    delta_steering = delta;
	  setpoint -= 0.1f;
	  delta -= 0.1f;
  
	  Handle_Action_TractionSetpoint = osThreadNew(Function_Action_TractionSetpoint, (void *)traction_setpoint, &Attributes_Action_TractionSetpoint);
  }*/
  for(;;){
    osDelay(10000);
  }
  /* USER CODE END 5 */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM7 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM7) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
