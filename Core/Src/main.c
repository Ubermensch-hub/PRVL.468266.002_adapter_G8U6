/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum AdapterState {PWR_OFF = 0b00, PWR_ON = 0b01, HARD_RESET = 0b11} AdapterState;
typedef enum PWROKState {LOW = 0x00, HIGH = 0x01} PWROKState;

#define MAX31760_CR1_REG 0x00 // Адрес Control Register 1
#define MAX31760_PWM_DUTY_REG 0x50 // Регистр коэффициента заполнения Ш�?М
#define _FAN2_FAIL_DUTY_REGISTER 0x03;     /**< Fail Duty Cycle Reg Address */
#define MAX31760_CR2_REG 0x01// Адрес Control Register 2

#define MAX31760_TACH1_MSB 0x52 //tach_ registers
#define MAX31760_TACH1_LSB 0x53
#define MAX31760_TACH2_MSB 0x54
#define MAX31760_TACH2_LSB 0x55

#define TACH_PULSES_PER_REV 2 // Количество импульсов тахометра за один оборот

uint16_t result = 0;
uint8_t flag_receive = 0;

uint8_t MCU_attach=0;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */
void OpenKey()
{
	HAL_GPIO_WritePin(MCU_HOS_ON_GPIO_Port, MCU_HOS_ON_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MB_BITCH_GPIO_Port, MB_BITCH_Pin, GPIO_PIN_RESET);

}

void CloseKey()
{
	HAL_GPIO_WritePin(MCU_HOS_ON_GPIO_Port, MCU_HOS_ON_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(MB_BITCH_GPIO_Port, MB_BITCH_Pin, GPIO_PIN_SET);

}

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */

uint16_t tachValue2 = 0; //ReadFanTach(0x51); // Чтение тахометрических данных со первого контроллера
uint16_t tachValue3 = 0; //ReadFanTach(0x54); // Чтение тахометрических данных с второго контроллера
uint16_t tachValue4 = 0; //ReadFanTach(0x54); // Чтение тахометрических данных со второго контроллера


void Send_Fan_Tachometer_To_Motherboard();


AdapterState Command = PWR_OFF;
AdapterState Adapter_State = PWR_OFF;
GPIO_PinState MB_State = GPIO_PIN_RESET;
PWROKState PWR_OK_State = GPIO_PIN_RESET;
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint16_t overflow_counter =0;

uint8_t command_i2c = 0;
int8_t temperature = 0;
uint8_t TempMode = 1;

uint16_t minRPM = 0;
uint8_t statusFAN1 = 0;
uint8_t statusFAN2 = 0;

void ResetFan()
{
	while (HAL_I2C_IsDeviceReady(&hi2c1, (0x51 << 1), 10, HAL_MAX_DELAY) != HAL_OK)
	{}
	uint8_t pData[1] = {0x40};
	uint8_t pData_2[1] = {0x02};
	uint8_t pData_3[1] = {0xFF};
	HAL_I2C_Mem_Write(&hi2c1, (0x51 << 1), MAX31760_CR1_REG, 1, pData, 1, 100);
	HAL_Delay(200);
	HAL_I2C_Mem_Write(&hi2c1, (0x51 << 1), 0x02, 1, pData_2, 1, 100);
	HAL_Delay(50);
	HAL_I2C_Mem_Write(&hi2c1, (0x51 << 1), 0x04, 1, pData_3, 1, 100);
	HAL_Delay(50);
	while (HAL_I2C_IsDeviceReady(&hi2c1, (0x54 << 1), 10, HAL_MAX_DELAY) != HAL_OK)
	{}
	HAL_I2C_Mem_Write(&hi2c1, (0x54<<1), MAX31760_CR1_REG, 1, pData, 1, 100);
	HAL_Delay(200);
	HAL_I2C_Mem_Write(&hi2c1, (0x54 << 1), 0x04, 1, pData_3, 1, 100);
	HAL_Delay(50);

}
uint16_t ReadTachRegister(uint8_t address, uint8_t regMSB, uint8_t regLSB)
{
	uint8_t msb, lsb;  // @suppress("Multiple variable declaration")

	HAL_I2C_Mem_Read(&hi2c1, address << 1, regMSB, I2C_MEMADD_SIZE_8BIT, &msb, 1, 100);
	HAL_I2C_Mem_Read(&hi2c1, address << 1, regLSB, I2C_MEMADD_SIZE_8BIT, &lsb, 1, 100);
	return (msb << 8) | lsb; // Объединяем MSB и LSB в 16-битное значение
}


void Read_Fan_Tachometer()
{

	tachValue2 = ReadTachRegister(0x51, MAX31760_TACH1_MSB, MAX31760_TACH1_LSB); // Первый контроллер, второй тахометр
	tachValue3 = ReadTachRegister(0x54, MAX31760_TACH1_MSB, MAX31760_TACH1_LSB); // Второй контроллер, первый тахометр
	tachValue4 = ReadTachRegister(0x54, MAX31760_TACH2_MSB, MAX31760_TACH2_LSB); // Второй контроллер, второй тахометр
}

void ReadStatus()
{
	HAL_I2C_Mem_Read(&hi2c1,( 0x51 << 1), 0x5A, I2C_MEMADD_SIZE_8BIT, &statusFAN1, 1, 100);
	HAL_I2C_Mem_Read(&hi2c1, (0x54 << 1), 0x5A, I2C_MEMADD_SIZE_8BIT, &statusFAN2, 1, 100);
}

uint16_t CalculateRPM(uint16_t tachValue)
{
	if (tachValue == 0) return 0; // �?збегаем деления на ноль
	return (60 * 100000) / (tachValue*TACH_PULSES_PER_REV);
}



uint16_t GetMinRPM(uint16_t rpm2, uint16_t rpm3, uint16_t rpm4)
{
	uint16_t minRPM = rpm3; // Предполагаем, что первое значение минимальное

	if (rpm3 < minRPM) {
		minRPM = rpm3; // Обновляем минимум, если второе значение меньше
	}
	if (rpm4 < minRPM) {
		minRPM = rpm4; // Обновляем минимум, если третье значение меньше
	}

	return minRPM; // Возвращаем минимальное значение
}

void Calculate_RPM()
{

	uint16_t rpm2 = CalculateRPM(tachValue2);
	uint16_t rpm3 = CalculateRPM(tachValue3);
	uint16_t rpm4 = CalculateRPM(tachValue4);

	// Здесь можно сохранить значения RPM в глобальные переменные или передать их дальше
	minRPM = GetMinRPM(rpm2, rpm3, rpm4);
}

void FanContrlSetDuty(I2C_HandleTypeDef *hi2c, uint16_t slaveAddress, uint8_t duty)
{
	uint8_t comm;
	uint8_t aTxBuffer[1] = {0};


	comm = MAX31760_PWM_DUTY_REG;
	aTxBuffer[0] = duty;
	HAL_I2C_Mem_Write(hi2c, slaveAddress, comm, 1, aTxBuffer, 1, 100);



	comm = _FAN2_FAIL_DUTY_REGISTER;
	aTxBuffer[0] = duty;
	HAL_I2C_Mem_Write(hi2c, slaveAddress, comm, 1, aTxBuffer, 1, 100);
}

void SetPWMFrequency(uint8_t address, uint8_t frequencySetting) {
	uint8_t data[2] = {MAX31760_CR1_REG, frequencySetting};

	uint8_t data_2[2] = {MAX31760_CR2_REG, 0x11};

	HAL_I2C_Master_Transmit(&hi2c1, (address << 1), data, 2, 100);


	HAL_I2C_Master_Transmit(&hi2c1, (address << 1), data_2, 2, 100);
}

void WriteMAX31760Register(uint8_t address, uint8_t reg, uint8_t value) {

	uint8_t data[2] = {reg, value};
	HAL_I2C_Master_Transmit(&hi2c1, (address << 1), data, 2, 100);
}

void InitMAX31760(uint8_t address) {
	// Установка частоты Ш�?М 24 кГц
	// Значение для регистра 0x32 зависит от формулы в документации
	// Например, если для 24 кГц нужно значение 0x4B:

	SetPWMFrequency(address, 0x19);
	// Установка коэффициента заполнения Ш�?М 50%
	FanContrlSetDuty(&hi2c1, (address << 1), 0xFF);

}

void SetButton(GPIO_PinState state)
{
	HAL_GPIO_WritePin(PWR_SW_GPIO_Port, PWR_SW_Pin, state);
}

void PressButton(uint16_t time)
{
	SetButton(GPIO_PIN_RESET);
	HAL_Delay(time);
	SetButton(GPIO_PIN_SET);
}

void SetPWROK(PWROKState state)
{
	HAL_GPIO_WritePin(MB_PWROK_GPIO_Port, MB_PWROK_Pin, (state == HIGH ? GPIO_PIN_SET : GPIO_PIN_RESET));
	PWR_OK_State = state;
}
void SetTempMode()
{
	MCU_attach = HAL_GPIO_ReadPin(MCU_ATTACH_IN_GPIO_Port, MCU_ATTACH_IN_Pin);
	if(MCU_attach == 1) {
		TempMode = 0;
	} else if (MCU_attach == 0 && temperature > 50 )
	{
		TempMode = 0;
	}else TempMode = 1;
}

void Send_Fan_Tachometer_To_Motherboard() {
	// Пример: генерация Ш�?М сигнала с частотой 1 кГц и duty cycle, соответствующим значению RPM
	uint32_t period1 = 0;

	if (minRPM > 0) {
		// Вычисляем период таймера для заданного RPM

		period1 = (1000000 / (minRPM * 2));
	} else {
		// Если RPM == 0, устанавливаем максимальный период (или другой порог)
		period1 = 1000 - 1; // Например, 1 мс (частота 1 кГц)
	}
	__HAL_TIM_SET_AUTORELOAD(&htim2, period1 - 1); // Установка периода 1 мс (частота 1 кГц)
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, period1 / 2); // Установка duty cycle пропорционально значению RPM

}
uint8_t i2c_rx_buffer[1]; // Буфер для приёма данных
uint8_t Command_temp = 0;
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
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */
	HAL_Delay(200);
	ResetFan();
	HAL_Delay(200);
	SetButton(GPIO_PIN_SET);
	SetPWROK(HIGH);
	InitMAX31760(0x54); // �?нициализация первого контроллера
	InitMAX31760(0x51); // �?нициализация второго контроллера

	HAL_TIM_Base_Start_IT(&htim3);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2); //запуск Ш�?М
	HAL_GPIO_WritePin(MB_BITCH_GPIO_Port, MB_BITCH_Pin, GPIO_PIN_SET);
	Command_temp = HAL_GPIO_ReadPin(MB_STATUS_LED_GPIO_Port, MB_STATUS_LED_Pin);


	//ReadStatus();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

		Command_temp = HAL_GPIO_ReadPin(MB_STATUS_LED_GPIO_Port, MB_STATUS_LED_Pin);
		HAL_I2C_Slave_Receive_IT(&hi2c2, i2c_rx_buffer, 1);

		// Управление вентиляторами
		if (TempMode == 0) {
			FanContrlSetDuty(&hi2c1, (0x54 << 1), 0xF0); // Установка Ш�?М на 50% для первого контроллера
			FanContrlSetDuty(&hi2c1, (0x51 << 1), 0xF0); // Установка Ш�?М на 50% для второго контроллера
		} else if (TempMode == 1) {
			FanContrlSetDuty(&hi2c1, (0x54 << 1), 0x80); // Установка Ш�?М на 50% для первого контроллера
			FanContrlSetDuty(&hi2c1, (0x51 << 1), 0x80); // Установка Ш�?М на 50% для второго контроллера
		} else {
			FanContrlSetDuty(&hi2c1, (0x54 << 1), 0xFF); // Установка Ш�?М на 50% для первого контроллера
			FanContrlSetDuty(&hi2c1, (0x51 << 1), 0xFF); // Установка Ш�?М на 50% для второго контроллера
		}

		if(Adapter_State == 1)
		{
			HAL_GPIO_WritePin(MB_BITCH_GPIO_Port, MB_BITCH_Pin, GPIO_PIN_RESET);
		}


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
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV8;
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
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* I2C1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(I2C1_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(I2C1_IRQn);
  /* I2C2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(I2C2_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(I2C2_IRQn);
  /* TIM2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM2_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(TIM2_IRQn);
  /* TIM3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM3_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(TIM3_IRQn);
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x10E47DAF;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_ENABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x10801031;
  hi2c2.Init.OwnAddress1 = 74;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 63;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 63999;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, USB_I2C_RES_Pin|PWR_SW_Pin|RST_SW_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(MB_PWROK_GPIO_Port, MB_PWROK_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, MB_BITCH_Pin|MCU_HOS_ON_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : USB_I2C_RES_Pin MB_PWROK_Pin PWR_SW_Pin RST_SW_Pin */
  GPIO_InitStruct.Pin = USB_I2C_RES_Pin|MB_PWROK_Pin|PWR_SW_Pin|RST_SW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : MB_PSON_Pin MCU_ATTACH_IN_Pin */
  GPIO_InitStruct.Pin = MB_PSON_Pin|MCU_ATTACH_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PWR_LED_Pin MB_STATUS_LED_Pin */
  GPIO_InitStruct.Pin = PWR_LED_Pin|MB_STATUS_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : MB_BITCH_Pin MCU_HOS_ON_Pin */
  GPIO_InitStruct.Pin = MB_BITCH_Pin|MCU_HOS_ON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void ProcessComand(uint8_t command)
{
	if(command == 0b00 || command == 0b01 ||command == 0b10 ||command == 0b11)
	{
		Command = command;
		MB_State = HAL_GPIO_ReadPin(MB_PSON_GPIO_Port, MB_PSON_Pin);
		if (Command == PWR_OFF && MB_State == 1) {

			SetButton(0);
			while(MB_State != 0)
			{
				MB_State = HAL_GPIO_ReadPin(MB_PSON_GPIO_Port, MB_PSON_Pin);
			}
			SetButton(1);// Короткое нажатие
			CloseKey();
			SetPWROK(HIGH);
			HAL_Delay(300);
			flag_receive = 0;
			Adapter_State = 0;

		} else if (Command == PWR_ON)
		{

			HAL_Delay(500);
			PressButton(200);
			while(MB_State != 1)
			{
				MB_State = HAL_GPIO_ReadPin(MB_PSON_GPIO_Port, MB_PSON_Pin);
			}
			HAL_Delay(100);
			OpenKey();
			SetPWROK(LOW);
			HAL_Delay(500);
			Adapter_State = 1;
			HAL_Delay(7000);
			flag_receive = 0;
		} else if (Command == HARD_RESET && MB_State == 1) {
			SetButton(0);
			while(MB_State != 0)
			{
				MB_State = HAL_GPIO_ReadPin(MB_PSON_GPIO_Port, MB_PSON_Pin);
			}
			SetButton(1);// Короткое нажатие
			SetPWROK(HIGH);
			CloseKey();
			HAL_Delay(5000);
			OpenKey();
			HAL_Delay(500);
			PressButton(200);
			while(MB_State != 1)
			{
				MB_State = HAL_GPIO_ReadPin(MB_PSON_GPIO_Port, MB_PSON_Pin);
			}
			SetPWROK(LOW);
			HAL_Delay(500);
			Adapter_State = 1;
			HAL_Delay(7000);
			flag_receive = 0;
		}else {

			flag_receive = 0;
			return;
		}
	}
}



void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c) {

	flag_receive = 1;
	Command_temp = HAL_GPIO_ReadPin(MB_STATUS_LED_GPIO_Port, MB_STATUS_LED_Pin);
	if(Command_temp == 1)
	{
		command_i2c = i2c_rx_buffer[0] & 0b11;
		ProcessComand(command_i2c);
	}else
	{
		temperature = i2c_rx_buffer[0];
		flag_receive = 0;
	}
	HAL_I2C_Slave_Receive_IT(&hi2c2, i2c_rx_buffer, 1);
}





void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

	if (htim->Instance == TIM3) // 1 раз в секунду
	{
		Read_Fan_Tachometer();
		Calculate_RPM();
		Send_Fan_Tachometer_To_Motherboard();
		SetTempMode();
		MB_State = HAL_GPIO_ReadPin(MB_PSON_GPIO_Port, MB_PSON_Pin);
		if (MB_State == 1 && Adapter_State == 0)//включение BMC
		{
			OpenKey();
			HAL_Delay(100);
			SetPWROK(LOW);
			Adapter_State = 1;

		} else if (MB_State == 0 && Adapter_State == 1)// выключение BMC
		{
			CloseKey();
			HAL_Delay(100);
			SetPWROK(HIGH);
			Adapter_State = 0;
		}

	}
}

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
