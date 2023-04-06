/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "nmea.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
nmea_t gps;
uint8_t minute = 0;
uint8_t hour = 0;
uint8_t sec = 0;
uint8_t month = 0;
uint8_t day = 0;
uint8_t year = 0;
uint8_t temp = 0;
uint32_t hexPage;
float latitude = 0;
float longitude = 0;
float altitude = 0;
char out[100];

/*TIMETABLE FOR ALL BANDS*/
int time80[] = {2,22,42};
int time40[] = {6,26,46};
int time30[] = {8,28,48};
int time20[] = {10,30,50};
int time17[] = {12,32,52};
int time15[] = {14,34,54};
int time12[] = {16,36,56};
int time10[] = {18,38,58};
int timeOff1[] = {0,20,40};
int timeOff2[] = {4,24,44};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM17_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
void USART1_Send (char chr){
	while (!(USART1->ISR & USART_ISR_TC));
	USART1->TDR = chr;
}

void USART1_Send_String (char* str){
	uint8_t i = 0;
	while(str[i])
	USART1_Send (str[i++]);
}

void USART1_Send_IntToString (uint16_t data){
	char buffer[sizeof(uint16_t) * 8 + 1];
	utoa(data, buffer, 10);
	USART1_Send_String(buffer);
	USART1_Send_String(" \r\n");
}

void SendGpsDataToUsart1() {
	if (nmea_available(&gps)) {
		nmea_gnss_time_h(&gps, &hour);
		nmea_gnss_time_m(&gps, &minute);
		nmea_gnss_time_s(&gps, &sec);
		nmea_gnss_date_d(&gps, &day);
		nmea_gnss_date_m(&gps, &month);
		nmea_gnss_date_y(&gps, &year);
		nmea_gnss_latitude_deg(&gps, &latitude);
		nmea_gnss_longitude_deg(&gps, &longitude);
		nmea_gnss_altitude_m(&gps, &altitude);
		sprintf(out, "Time: %d:%d:%d\r\n Position: %3.2f %3.2f\r\n Date: %d.%d.%d\r\n", hour,
				minute, sec, latitude, longitude, day, month, year);
		USART1_Send_String(out);
		nmea_available_reset(&gps);
	}
}

void SetBootEnable() {
	FLASH->KEYR = (uint32_t) 0x45670123;
	FLASH->KEYR = (uint32_t) 0xCDEF89AB;
	LL_mDelay(1);
	FLASH->OPTKEYR = (uint32_t) 0x08192A3B;
	FLASH->OPTKEYR = (uint32_t) 0x4C5D6E7F;
	LL_mDelay(1);
	FLASH->CR &= ~FLASH_CR_OPTLOCK;
	FLASH->OPTR |= FLASH_OPTR_nBOOT1;
	FLASH->OPTR &= ~FLASH_OPTR_nBOOT_SEL;
	LL_mDelay(1);
	FLASH->CR |= FLASH_CR_OPTSTRT;
	LL_mDelay(1);
	FLASH->CR |= FLASH_CR_OBL_LAUNCH;
	LL_mDelay(1);
	FLASH->CR |= FLASH_CR_LOCK;
}

uint32_t getHexAddressPage(int dataPage){
	uint32_t bits       = PAGE_SECTOR * dataPage;
	uint32_t hexAddress = FLASH_INIT + bits;
	return hexAddress;
}


void memoryPageErase(uint32_t memoryPage){
	HAL_StatusTypeDef eraseHandler = HAL_FLASH_Unlock();

	eraseHandler = FLASH_WaitForLastOperation(500);
	__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR);
	FLASH_PageErase(FLASH_BANK_1, memoryPage);

	eraseHandler = FLASH_WaitForLastOperation(500);
	CLEAR_BIT(FLASH->CR, FLASH_CR_PER);
	HAL_FLASH_Lock();
}

void WriteToEEPROM (uint32_t hexPage, int dataA){
	memoryPageErase(DATA_PAGE);
	HAL_FLASH_Unlock();
	HAL_FLASH_Program(TYPEPROGRAM_DOUBLEWORD, hexPage, dataA);
	HAL_FLASH_Lock();
}

uint32_t ReadFromEEPROM (uint32_t hexAddress){
	return *(uint32_t*)hexAddress;
}

void ResetAllOutputs() {
	LL_GPIO_ResetOutputPin(K1_GPIO_Port, K1_Pin);
	LL_GPIO_ResetOutputPin(K2_GPIO_Port, K2_Pin);
	LL_GPIO_ResetOutputPin(K3_GPIO_Port, K3_Pin);
	LL_GPIO_ResetOutputPin(K4_GPIO_Port, K4_Pin);
	LL_GPIO_ResetOutputPin(K5_GPIO_Port, K5_Pin);
	LL_GPIO_ResetOutputPin(K6_GPIO_Port, K6_Pin);
	LL_GPIO_ResetOutputPin(K7_GPIO_Port, K7_Pin);
	LL_GPIO_ResetOutputPin(K8_GPIO_Port, K8_Pin);
	LL_GPIO_ResetOutputPin(A1_GPIO_Port, A1_Pin);
	LL_GPIO_ResetOutputPin(A2_GPIO_Port, A2_Pin);
	LL_GPIO_ResetOutputPin(A3_GPIO_Port, A3_Pin);
	LL_GPIO_ResetOutputPin(A4_GPIO_Port, A4_Pin);
	LL_GPIO_ResetOutputPin(A5_GPIO_Port, A5_Pin);
	LL_GPIO_ResetOutputPin(A6_GPIO_Port, A6_Pin);
	LL_GPIO_ResetOutputPin(A7_GPIO_Port, A7_Pin);
	LL_GPIO_ResetOutputPin(A8_GPIO_Port, A8_Pin);
}

int IsTime80(){
	if ((minute == (time80[0]-1) || minute == (time80[1]-1) || minute == (time80[2]-1)) && (sec == 59)) return 1;
	if (minute>=time80[0] && minute<timeOff2[0]) return 1;
	if (minute>=time80[1] && minute<timeOff2[1]) return 1;
	if (minute>=time80[2] && minute<timeOff2[2]) return 1;
	return 0;
}

int IsTime40(){
	if ((minute == (time40[0]-1) || minute == (time40[1]-1) || minute == (time40[2]-1)) && (sec == 59)) return 1;
	if (minute>=time40[0] && minute<time30[0]) return 1;
	if (minute>=time40[1] && minute<time30[1]) return 1;
	if (minute>=time40[2] && minute<time30[2]) return 1;
	return 0;
}

int IsTime30(){
	if ((minute == (time30[0]-1) || minute == (time30[1]-1) || minute == (time30[2]-1)) && (sec == 59)) return 1;
	if (minute>=time30[0] && minute<time20[0]) return 1;
	if (minute>=time30[1] && minute<time20[1]) return 1;
	if (minute>=time30[2] && minute<time20[2]) return 1;
	return 0;
}

int IsTime20(){
	if ((minute == (time20[0]-1) || minute == (time20[1]-1)  || minute == (time20[2]-1) ) && (sec == 59)) return 1;
	if (minute>=time20[0] && minute<time17[0]) return 1;
	if (minute>=time20[1] && minute<time17[1]) return 1;
	if (minute>=time20[2] && minute<time17[2]) return 1;
	return 0;
}

int IsTime17(){
	if ((minute == (time17[0]-1) || minute == (time17[1]-1) || minute == (time17[2]-1)) && (sec == 59)) return 1;
	if (minute>=time17[0] && minute<time15[0]) return 1;
	if (minute>=time17[1] && minute<time15[1]) return 1;
	if (minute>=time17[2] && minute<time15[2]) return 1;
	return 0;
}

int IsTime15(){
	if ((minute == (time15[0]-1) || minute == (time15[1]-1) || minute == (time15[2]-1)) && (sec == 59)) return 1;
	if (minute>=time15[0] && minute<time12[0]) return 1;
	if (minute>=time15[1] && minute<time12[1]) return 1;
	if (minute>=time15[2] && minute<time12[2]) return 1;
	return 0;
}

int IsTime12(){
	if ((minute == (time12[0]-1) || minute == (time12[1]-1) || minute == (time12[2]-1)) && (sec == 59)) return 1;
	if (minute>=time12[0] && minute<time10[0]) return 1;
	if (minute>=time12[1] && minute<time10[1]) return 1;
	if (minute>=time12[2] && minute<time10[2]) return 1;
	return 0;
}

int IsTime10(){
	if ((minute == (time10[0]-1) || minute == (time10[1]-1) || minute == (time10[2]-1) ) && (sec == 59)) return 1;
	if (minute>=time10[0] && minute<timeOff1[0]) return 1;
	if (minute>=time10[1] && minute<timeOff1[1]) return 1;
	if (minute>=time10[2] && minute<timeOff1[2]) return 1;
	if (minute<2) return 1;
	return 0;
}

int IsTimeOff() {
	if ((minute == (timeOff1[0] - 1) || minute == (timeOff1[1] - 1)
			|| minute == (timeOff1[2] - 1)) && (sec == 59))
		return 1;
	if (minute >= timeOff1[0] && minute < time80[0])
		return 1;
	if (minute >= timeOff1[1] && minute < time80[1])
		return 1;
	if (minute >= timeOff1[2] && minute < time80[2])
		return 1;
	if (minute < 2)
		return 1;

	if ((minute == (timeOff2[0] - 1) || minute == (timeOff2[1] - 1)
			|| minute == (timeOff2[2] - 1)) && (sec == 59))
		return 1;
	if (minute >= timeOff2[0] && minute < time40[0])
		return 1;
	if (minute >= timeOff2[1] && minute < time40[1])
		return 1;
	if (minute >= timeOff2[2] && minute < time40[2])
		return 1;
	if (minute < 2)
		return 1;
	return 0;
}


void WSPR_Schedule() {
	if (IsTime80()) {
		ResetAllOutputs();
		LL_GPIO_SetOutputPin(K1_GPIO_Port, K1_Pin);
		LL_GPIO_SetOutputPin(A1_GPIO_Port, A1_Pin);
	}
	if (IsTime40()) {
		ResetAllOutputs();
		LL_GPIO_SetOutputPin(K2_GPIO_Port, K2_Pin);
		LL_GPIO_SetOutputPin(A2_GPIO_Port, A2_Pin);
	}
	if (IsTime30()) {
		ResetAllOutputs();
		LL_GPIO_SetOutputPin(K3_GPIO_Port, K3_Pin);
		LL_GPIO_SetOutputPin(A3_GPIO_Port, A3_Pin);
	}
	if (IsTime20()) {
		ResetAllOutputs();
		LL_GPIO_SetOutputPin(K4_GPIO_Port, K4_Pin);
		LL_GPIO_SetOutputPin(A4_GPIO_Port, A4_Pin);
	}
	if (IsTime17()) {
		ResetAllOutputs();
		LL_GPIO_SetOutputPin(K5_GPIO_Port, K5_Pin);
		LL_GPIO_SetOutputPin(A5_GPIO_Port, A5_Pin);
	}
	if (IsTime15()) {
		ResetAllOutputs();
		LL_GPIO_SetOutputPin(K6_GPIO_Port, K6_Pin);
		LL_GPIO_SetOutputPin(A6_GPIO_Port, A6_Pin);
	}
	if (IsTime12()) {
		ResetAllOutputs();
		LL_GPIO_SetOutputPin(K7_GPIO_Port, K7_Pin);
		LL_GPIO_SetOutputPin(A7_GPIO_Port, A7_Pin);
	}
	if (IsTime10()) {
		ResetAllOutputs();
		LL_GPIO_SetOutputPin(K8_GPIO_Port, K8_Pin);
		LL_GPIO_SetOutputPin(A8_GPIO_Port, A8_Pin);
	}
	if (IsTimeOff()) {
		ResetAllOutputs();
	}

}

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

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM17_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  nmea_init(&gps, USART2, 1024);
  LL_TIM_EnableCounter(TIM17);
  LL_TIM_EnableIT_UPDATE(TIM17);
  hexPage = getHexAddressPage(DATA_PAGE);
  WriteToEEPROM(hexPage, 123);
  temp = ReadFromEEPROM(hexPage);
  //SetBootEnable();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
		nmea_loop(&gps);
		SendGpsDataToUsart1();
		WSPR_Schedule();
		LL_mDelay(300);

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

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 8;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM17 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM17_Init(void)
{

  /* USER CODE BEGIN TIM17_Init 0 */

  /* USER CODE END TIM17_Init 0 */

  LL_TIM_InitTypeDef TIM_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM17);

  /* TIM17 interrupt Init */
  NVIC_SetPriority(TIM17_IRQn, 0);
  NVIC_EnableIRQ(TIM17_IRQn);

  /* USER CODE BEGIN TIM17_Init 1 */

  /* USER CODE END TIM17_Init 1 */
  TIM_InitStruct.Prescaler = 31999;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = 499;
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  TIM_InitStruct.RepetitionCounter = 0;
  LL_TIM_Init(TIM17, &TIM_InitStruct);
  LL_TIM_EnableARRPreload(TIM17);
  /* USER CODE BEGIN TIM17_Init 2 */

  /* USER CODE END TIM17_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  LL_USART_InitTypeDef USART_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the peripherals clocks
  */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }

  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_USART1);

  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);
  /**USART1 GPIO Configuration
  PA9   ------> USART1_TX
  PA10   ------> USART1_RX
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_9;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_1;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_10;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_1;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  USART_InitStruct.PrescalerValue = LL_USART_PRESCALER_DIV1;
  USART_InitStruct.BaudRate = 115200;
  USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
  USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
  USART_InitStruct.Parity = LL_USART_PARITY_NONE;
  USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
  USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
  USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
  LL_USART_Init(USART1, &USART_InitStruct);
  LL_USART_SetTXFIFOThreshold(USART1, LL_USART_FIFOTHRESHOLD_1_8);
  LL_USART_SetRXFIFOThreshold(USART1, LL_USART_FIFOTHRESHOLD_1_8);
  LL_USART_DisableFIFO(USART1);
  LL_USART_ConfigAsyncMode(USART1);

  /* USER CODE BEGIN WKUPType USART1 */

  /* USER CODE END WKUPType USART1 */

  LL_USART_Enable(USART1);

  /* Polling USART1 initialisation */
  while((!(LL_USART_IsActiveFlag_TEACK(USART1))) || (!(LL_USART_IsActiveFlag_REACK(USART1))))
  {
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  LL_USART_InitTypeDef USART_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART2);

  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);
  /**USART2 GPIO Configuration
  PA2   ------> USART2_TX
  PA3   ------> USART2_RX
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_2;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_1;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_3;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_1;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USART2 interrupt Init */
  NVIC_SetPriority(USART2_IRQn, 0);
  NVIC_EnableIRQ(USART2_IRQn);

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  USART_InitStruct.PrescalerValue = LL_USART_PRESCALER_DIV1;
  USART_InitStruct.BaudRate = 9600;
  USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
  USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
  USART_InitStruct.Parity = LL_USART_PARITY_NONE;
  USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
  USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
  USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
  LL_USART_Init(USART2, &USART_InitStruct);
  LL_USART_ConfigAsyncMode(USART2);

  /* USER CODE BEGIN WKUPType USART2 */

  /* USER CODE END WKUPType USART2 */

  LL_USART_Enable(USART2);

  /* Polling USART2 initialisation */
  while((!(LL_USART_IsActiveFlag_TEACK(USART2))) || (!(LL_USART_IsActiveFlag_REACK(USART2))))
  {
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOC);
  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);
  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOB);

  /**/
  LL_GPIO_ResetOutputPin(A1_GPIO_Port, A1_Pin);

  /**/
  LL_GPIO_ResetOutputPin(A2_GPIO_Port, A2_Pin);

  /**/
  LL_GPIO_ResetOutputPin(A3_GPIO_Port, A3_Pin);

  /**/
  LL_GPIO_ResetOutputPin(A4_GPIO_Port, A4_Pin);

  /**/
  LL_GPIO_ResetOutputPin(A5_GPIO_Port, A5_Pin);

  /**/
  LL_GPIO_ResetOutputPin(A6_GPIO_Port, A6_Pin);

  /**/
  LL_GPIO_ResetOutputPin(K1_GPIO_Port, K1_Pin);

  /**/
  LL_GPIO_ResetOutputPin(K2_GPIO_Port, K2_Pin);

  /**/
  LL_GPIO_ResetOutputPin(K3_GPIO_Port, K3_Pin);

  /**/
  LL_GPIO_ResetOutputPin(A7_GPIO_Port, A7_Pin);

  /**/
  LL_GPIO_ResetOutputPin(LED_GPIO_Port, LED_Pin);

  /**/
  LL_GPIO_ResetOutputPin(A8_GPIO_Port, A8_Pin);

  /**/
  LL_GPIO_ResetOutputPin(K4_GPIO_Port, K4_Pin);

  /**/
  LL_GPIO_ResetOutputPin(K5_GPIO_Port, K5_Pin);

  /**/
  LL_GPIO_ResetOutputPin(K6_GPIO_Port, K6_Pin);

  /**/
  LL_GPIO_ResetOutputPin(K7_GPIO_Port, K7_Pin);

  /**/
  LL_GPIO_ResetOutputPin(K8_GPIO_Port, K8_Pin);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_15;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_1;
  LL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = A1_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(A1_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = A2_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(A2_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = A3_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(A3_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = A4_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(A4_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = A5_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(A5_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = A6_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(A6_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = K1_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(K1_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = K2_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(K2_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = K3_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(K3_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = A7_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(A7_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = A8_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(A8_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = K4_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(K4_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = K5_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(K5_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = K6_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(K6_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = K7_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(K7_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = K8_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(K8_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

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
