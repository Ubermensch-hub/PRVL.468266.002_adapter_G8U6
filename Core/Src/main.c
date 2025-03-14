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
typedef enum AdapterState {PWR_OFF = 0b00, PWR_ON = 0b01, REBOOT = 0b10, HARD_RESET = 0b11} AdapterState;
typedef enum PWROKState {LOW = 0x00, HIGH = 0x01} PWROKState;

#define NUMBER_OF_FAN_CONTRL 2
#define SIZE 256
#define MAX_TEMP 40

uint16_t Temperature = 0;
uint16_t result = 0;
uint8_t flag_receive = 0;
uint8_t aRxBuffer[SIZE] = {}; //Буфер приема данных по I2C
uint8_t aTxBuffer[SIZE] = {}; //Буфер передачи данных по I2C

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
///@}
/*----------------------------------------------------------------- VARIABLES */

/** FLIR I2C slave interface address */
const uint8_t _FAN2_I2C_ADDRESS         = 0x50;


/**
 * @name    Fan_2 Click Return Values
 *//*-------------------------------------------------------------------------*/
///@{

uint8_t _FAN2_OK     = 0;               /**< NO ERROR */
uint8_t _FAN2_ERR    = 1;               /**< ERROR OCCURED */

///@}
/**
 * @name    Fan_2 Click Read/Write Registers
 *//*-------------------------------------------------------------------------*/
///@{

const uint8_t _FAN2_CONTROL_REGISTER_1 = 0x00;     /**< Control Register 1 Address */
const uint8_t _FAN2_CONTROL_REGISTER_2 = 0x01;     /**< Control Register 2 Address */
const uint8_t _FAN2_CONTROL_REGISTER_3 = 0x02;     /**< Control Register 3 Address */

const uint8_t _FAN2_FAIL_DUTY_REGISTER = 0x03;     /**< Fail Duty Cycle Reg Address */
const uint8_t _FAN2_ALERT_MASK_REGISTER = 0x04;    /**< Alert Mask Register Address */
const uint8_t _FAN2_IDEALITY_FACTOR_REGISTER = 0x05;  /**< Ideality Factor Register Address */

const uint8_t _FAN2_RHSH_REGISTER = 0x06;   /**< Remote High Set-point MSB */
const uint8_t _FAN2_RHSL_REGISTER = 0x07;   /**< Remote High Set-point LSB */
const uint8_t _FAN2_LOTSH_REGISTER = 0x08;  /**< Local Overtemperature Set-point MSB */
const uint8_t _FAN2_LOTSL_REGISTER = 0x09;  /**< Local Overtemperature Set-point LSB */
const uint8_t _FAN2_ROTSH_REGISTER = 0x0A;  /**< Remote Overtemperature Set-point MSB */
const uint8_t _FAN2_ROTSL_REGISTER = 0x0B;  /**< Remote Overtemperature Set-point LSB */
const uint8_t _FAN2_LHSH_REGISTER = 0x0C;  /**< Local High Set-point MSB */
const uint8_t _FAN2_LHSL_REGISTER = 0x0D;  /**< Local High Set-point LSB */

const uint8_t _FAN2_TCTH_REGISTER = 0x0E;  /**< TACH Count Threshold Register MSB */
const uint8_t _FAN2_TCTL_REGISTER = 0x0F;  /**< TACH Count Threshold Register LSB */

const uint8_t _FAN2_DIRECT_CONTROL_REGISTER = 0x50;  /**< Direct Duty-Cycle Control Register */

///@}
/**
 * @name    Fan_2 Click Read Only Registers
 *//*-------------------------------------------------------------------------*/
///@{

const uint8_t _FAN2_PWMV_REGISTER = 0x51;   /**< Current PWM Duty-Cycle Register */
const uint8_t _FAN2_TC1H_REGISTER = 0x52;   /**< TACH1 Count Register, MSB */
const uint8_t _FAN2_TC1L_REGISTER = 0x53;  /**< TACH1 Count Register, LSB */
const uint8_t _FAN2_TC2H_REGISTER = 0x54;  /**< TACH2 Count Register, MSB */
const uint8_t _FAN2_TC2L_REGISTER = 0x55;  /**< TACH2 Count Register, LSB */
const uint8_t _FAN2_RTH_REGISTER = 0x56;  /**< Remote Temperature Reading Register, MSB */
const uint8_t _FAN2_RTL_REGISTER = 0x57;  /**< Remote Temperature Reading Register, LSB */
const uint8_t _FAN2_LTH_REGISTER = 0x58;  /**< Local Temperature Reading Register, MSB */
const uint8_t _FAN2_LTL_REGISTER = 0x59;  /**< Local Temperature Reading Register, LSB */
const uint8_t _FAN2_SR_REGISTER =  0x5A;  /**< Status Register */


///@}
/**
 * @name    Fan_2 Rotation Speed Values
 *//*-------------------------------------------------------------------------*/
///@{
const uint8_t _FAN2_DUTYCYCLE_100 = 0xFF;     /**< 100 percent duty cycle */
const uint8_t _FAN2_DUTYCYCLE_75  = 0xC0;     /**< 75 percent duty cycle */
const uint8_t _FAN2_DUTYCYCLE_50  = 0x80;     /**< 50 percent duty cycle */
const uint8_t _FAN2_DUTYCYCLE_25  = 0x40;     /**< 25 percent duty cycle */
const uint8_t _FAN2_DUTYCYCLE_12  = 0x20;     /**< 12 percent duty cycle */
const uint8_t _FAN2_DUTYCYCLE_0   = 0x00;     /**< duty cycle off */

///@}
/**
 * @name    Fan_2 Command Register Values
 *//*-------------------------------------------------------------------------*/
///@{

const uint8_t _FAN2_CMD1_DEFAULT = 0x01;     /**< Control Register 1 Default Values */
const uint8_t _FAN2_CMD2_DEFAULT = 0x10;     /**< Control Register 2 Default Values */
const uint8_t _FAN2_CMD3_DEFAULT = 0x01;     /**< Control Register 3 Default Values */


const uint8_t _FAN2_CMD1_REMOTE_TEMPERATURE = 0x01;     /**< Use remote temperature */
const uint8_t _FAN2_CMD1_LOCAL_TEMPERATURE = 0x00;      /**< Use local temperature */

const uint8_t _FAN2_CMD2_DIRECT_CONTROL = 0x01;        /**< Direct fan control */
const uint8_t _FAN2_CMD2_AUTOMATIC_CONTROL = 0x00;     /**< Automatic fan control */

const uint8_t _FAN2_CMD3_SLOW_RAMP = 0x00;        /**< Fan speed changes slowly */
const uint8_t _FAN2_CMD3_MEDIUM_RAMP = 0x10;      /**< Fan speed changes at moderate rate */
const uint8_t _FAN2_CMD3_FAST_RAMP = 0x20;        /**< Fan speed changes at high rate */
const uint8_t _FAN2_CMD3_INSTANT_RAMP = 0x30;     /**< Fan speed changes instantly */


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim1;

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
static void MX_TIM1_Init(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */

void Fan_Init(I2C_HandleTypeDef *hi2c, uint16_t slaveAddress);
void Set_Fan_DutyCycle(uint8_t duty_cycle);
void Read_Fan_Tachometer();
void Send_Fan_Tachometer_To_Motherboard();
void Read_ADC_Voltage();
void Set_PWROK_High();
void Set_PWROK_Low();
void Set_PSON_High();
void Set_PSON_Low();
void Set_RST_SW_High();
void Set_RST_SW_Low();
void Close_Power_Switches();
void Open_Power_Switches();

void Emulate_RST_SW_Press();
void Send_Signal_To_Unification_Board();

AdapterState Command = PWR_OFF;
AdapterState Adapter_State = PWR_OFF;
GPIO_PinState MB_State = GPIO_PIN_RESET;
PWROKState PWR_OK_State = GPIO_PIN_RESET;
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint16_t SlaveFanAddresses[NUMBER_OF_FAN_CONTRL] = {0xA2, 0xA8};
uint8_t command_i2c = 0;
uint8_t TempMode = 1;


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
uint8_t i2c_rx_buffer[1]; // Буфер для приёма данных
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
  MX_TIM1_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */
	SetButton(GPIO_PIN_SET);
	SetPWROK(HIGH);
	Fan_Init(&hi2c1, 0xA2);
	Fan_Init(&hi2c1, 0xA8);




  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		//MB_State = HAL_GPIO_ReadPin(MB_PSON_GPIO_Port, MB_PSON_Pin);
/*
		if (MB_State == 1 && Adapter_State == 0)
		{
			HAL_GPIO_WritePin(MCU_HOS_ON_GPIO_Port, MCU_HOS_ON_Pin, RESET);
			HAL_GPIO_WritePin(MB_BITCH_GPIO_Port, MB_BITCH_Pin, RESET);
			HAL_Delay(5);
			HAL_GPIO_WritePin(MCU_HOS_ON_GPIO_Port, MCU_HOS_ON_Pin, SET);
			PressButton(5);
			//HAL_GPIO_WritePin(MB_PSON_SENSE_GPIO_Port, MB_PSON_SENSE_Pin, RESET);
			HAL_Delay(300);
			SetPWROK(LOW);
			Adapter_State = 0;

		} else if (MB_State == 0 && Adapter_State == 1)
		{
			HAL_GPIO_WritePin(MCU_HOS_ON_GPIO_Port, MCU_HOS_ON_Pin, RESET);
			HAL_GPIO_WritePin(MB_BITCH_GPIO_Port, MB_BITCH_Pin, SET);
			SetPWROK(HIGH);
		}
*/		HAL_I2C_Slave_Receive_IT(&hi2c2, i2c_rx_buffer, 1);



		// Управление вентиляторами
		if (TempMode == 1) {
			Set_Fan_DutyCycle(_FAN2_DUTYCYCLE_50);
		} else if (TempMode == 2) {
			Set_Fan_DutyCycle(_FAN2_DUTYCYCLE_75);
		} else {
			Set_Fan_DutyCycle(_FAN2_DUTYCYCLE_100);
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
  HAL_NVIC_SetPriority(I2C1_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(I2C1_IRQn);
  /* I2C2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(I2C2_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(I2C2_IRQn);
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
  hi2c1.Init.Timing = 0x00C12469;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_DISABLE) != HAL_OK)
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
  hi2c2.Init.Timing = 0x00C12469;
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
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_DISABLE) != HAL_OK)
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
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 31;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 49;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_RESET;
  sSlaveConfig.InputTrigger = TIM_TS_TI1FP1;
  sSlaveConfig.TriggerPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sSlaveConfig.TriggerPrescaler = TIM_ICPSC_DIV1;
  sSlaveConfig.TriggerFilter = 0;
  if (HAL_TIM_SlaveConfigSynchro(&htim1, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
  sConfigIC.ICSelection = TIM_ICSELECTION_INDIRECTTI;
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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
  HAL_GPIO_WritePin(GPIOA, MB_FAN_TACH_Pin|MB_PWROK_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, MB_BITCH_Pin|MCU_HOS_ON_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : USB_I2C_RES_Pin MB_FAN_TACH_Pin MB_PWROK_Pin PWR_SW_Pin
                           RST_SW_Pin */
  GPIO_InitStruct.Pin = USB_I2C_RES_Pin|MB_FAN_TACH_Pin|MB_PWROK_Pin|PWR_SW_Pin
                          |RST_SW_Pin;
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
void Fan_Init(I2C_HandleTypeDef *hi2c, uint16_t slaveAddress)
{
	uint8_t aTxBuffer[1];

	// Функция для записи в регистр
	void WriteRegister(uint8_t reg, uint8_t value) {

		aTxBuffer[0] = value;
		HAL_I2C_Mem_Write_IT(hi2c, slaveAddress, reg, 1, aTxBuffer, 100);
	}

	// Запись значений в регистры
	WriteRegister(_FAN2_CONTROL_REGISTER_1, 0x19);
	WriteRegister(_FAN2_CONTROL_REGISTER_2, 0x11);
	WriteRegister(_FAN2_CONTROL_REGISTER_3, 0xA3);
	WriteRegister(_FAN2_FAIL_DUTY_REGISTER, 0xFF);
	WriteRegister(_FAN2_ALERT_MASK_REGISTER, 0xC0);
	WriteRegister(_FAN2_IDEALITY_FACTOR_REGISTER, 0x18);
	WriteRegister(0x06, 0x55);
	WriteRegister(0x07, 0x00);
	WriteRegister(0x08, 0x55);
	WriteRegister(0x09, 0x00);
	WriteRegister(0x0A, 0x6E);
	WriteRegister(0x0B, 0x00);
	WriteRegister(0x0C, 0x46);
	WriteRegister(0x0D, 0x00);
	WriteRegister(0x0E, 0xFF);
	WriteRegister(0x0F, 0xFE);
	WriteRegister(_FAN2_DIRECT_CONTROL_REGISTER, _FAN2_DUTYCYCLE_50);
	WriteRegister(_FAN2_FAIL_DUTY_REGISTER, _FAN2_DUTYCYCLE_50);



}
void Set_Fan_DutyCycle(uint8_t duty_cycle)
{
	uint8_t aTxBuffer[1];

	// Функция для записи в регистр
	HAL_StatusTypeDef WriteRegister_1(uint8_t reg, uint8_t value) {
		// Проверка доступности устройства

		aTxBuffer[0] = value;
		return HAL_I2C_Mem_Write(&hi2c1, 0xA2, reg, 1, aTxBuffer, 1, 100); // Синхронная запись
	}
	HAL_StatusTypeDef WriteRegister_2(uint8_t reg, uint8_t value) {
		// Проверка доступности устройства

		// Запись значения в регистр
		aTxBuffer[0] = value;
		return HAL_I2C_Mem_Write(&hi2c1, 0xA8, reg, 1, aTxBuffer, 1, 100); // Синхронная запись
	}

	// Установка скважности в регистр прямого управления
	WriteRegister_1(_FAN2_DIRECT_CONTROL_REGISTER, duty_cycle);
	// Установка скважности в регистр прямого управления
	WriteRegister_2(_FAN2_DIRECT_CONTROL_REGISTER, duty_cycle) ;

}
void ProcessComand(uint8_t command)
{
	if(command == 0b00 || command == 0b01 ||command == 0b10 ||command == 0b11)
	{
	Command = command;
	MB_State = HAL_GPIO_ReadPin(MB_PSON_GPIO_Port, MB_PSON_Pin);
	if (Command == PWR_OFF && MB_State == 1) {
		//SetPWROK(HIGH);
		//HAL_Delay(400);
		SetButton(0);
		while(MB_State != 0)
		{
			MB_State = HAL_GPIO_ReadPin(MB_PSON_GPIO_Port, MB_PSON_Pin);
		}
		SetButton(1);// Короткое нажатие
		CloseKey();
		HAL_Delay(100);
		// HAL_GPIO_WritePin(MCU_HOS_ON_GPIO_Port, MCU_HOS_ON_Pin, SET);
		//HAL_GPIO_WritePin(MB_BITCH_GPIO_Port, MB_BITCH_Pin, GPIO_PIN_RESET);
		flag_receive = 0;
		Adapter_State = 0;
		SetPWROK(HIGH);
	} else if (Command == PWR_ON && MB_State == 0)
	{
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
	} else if (Command == REBOOT && MB_State == 1) {
		SetButton(0); // Короткое нажатие
		HAL_Delay(500);
		SetPWROK(HIGH);
		HAL_Delay(500);
		SetButton(GPIO_PIN_SET);
		HAL_Delay(150);
		SetPWROK(LOW);
	} else if (Command == HARD_RESET && MB_State == 1) {
		CloseKey();
		SetPWROK(HIGH);
	}else {
		HAL_Delay(300);
		flag_receive = 0;
		return;
		}
	} else
		Temperature = command;
}


void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c) {

	flag_receive = 1;
	if (i2c_rx_buffer)
	command_i2c = i2c_rx_buffer[0] & 0b11;
	ProcessComand(command_i2c);
	HAL_I2C_Slave_Receive_IT(&hi2c2, i2c_rx_buffer, 1);
}

void Read_Fan_Tachometer()
{

}

void Send_Fan_Tachometer_To_Motherboard()
{

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
