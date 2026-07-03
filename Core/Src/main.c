/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 STMicroelectronics.
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
#include "SH1106.h"
#include "fonts.h"
#include "stm32g431xx.h"
#include "stm32g4xx_hal_adc.h"
#include "stm32g4xx_hal_gpio.h"
#include <stdio.h>
#include <string.h>
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
ADC_HandleTypeDef hadc2;
DMA_HandleTypeDef hdma_adc2;

CORDIC_HandleTypeDef hcordic;
DMA_HandleTypeDef hdma_cordic_read;
DMA_HandleTypeDef hdma_cordic_write;

I2C_HandleTypeDef hi2c1;
DMA_HandleTypeDef hdma_i2c1_tx;

I2S_HandleTypeDef hi2s2;
DMA_HandleTypeDef hdma_spi2_tx;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim7;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
volatile uint8_t flagSet = 0;
volatile uint8_t mainTimFlag = 0;
volatile uint8_t scanFlag = 0;
uint8_t prevState[8][6] = {0};

volatile uint32_t phaseAcc = 0;
volatile uint32_t phaseAcc2 = 0;
volatile uint32_t phaseStep2 = 0;
volatile uint32_t phaseStep = 0;
volatile int32_t vibrato = 0;
volatile int16_t bigassBuffer[512] = {0};
const int16_t aaaaSample[326] = {
    510,    510,    690,    690,    888,    888,    1212,   1212,   1541,
    1541,   1849,   1849,   2017,   2017,   2086,   2086,   1918,   1918,
    1719,   1719,   1436,   1436,   1027,   1027,   490,    490,    -71,
    -71,    -535,   -535,   -930,   -930,   -1392,  -1392,  -2016,  -2016,
    -2694,  -2694,  -3331,  -3331,  -3920,  -3920,  -4525,  -4525,  -5009,
    -5009,  -5231,  -5231,  -5121,  -5121,  -4758,  -4758,  -4292,  -4292,
    -3787,  -3787,  -3231,  -3231,  -2583,  -2583,  -1833,  -1833,  -1011,
    -1011,  -145,   -145,   868,    868,    2037,   2037,   3196,   3196,
    4223,   4223,   5073,   5073,   5822,   5822,   6493,   6493,   7026,
    7026,   7395,   7395,   7569,   7569,   7620,   7620,   7489,   7489,
    7150,   7150,   6602,   6602,   5938,   5938,   5237,   5237,   4411,
    4411,   3524,   3524,   2585,   2585,   1642,   1642,   702,    702,
    -201,   -201,   -998,   -998,   -1679,  -1679,  -2261,  -2261,  -2723,
    -2723,  -3076,  -3076,  -3341,  -3341,  -3423,  -3423,  -3311,  -3311,
    -3010,  -3010,  -2554,  -2554,  -1992,  -1992,  -1425,  -1425,  -886,
    -886,   -424,   -424,   -31,    -31,    349,    349,    698,    698,
    997,    997,    1144,   1144,   1100,   1100,   848,    848,    472,
    472,    34,     34,     -453,   -453,   -1057,  -1057,  -1822,  -1822,
    -2706,  -2706,  -3736,  -3736,  -4916,  -4916,  -6140,  -6140,  -7263,
    -7263,  -8228,  -8228,  -8971,  -8971,  -9619,  -9619,  -10240, -10240,
    -10765, -10765, -11140, -11140, -11075, -11075, -10460, -10460, -9281,
    -9281,  -7366,  -7366,  -4893,  -4893,  -2270,  -2270,  -133,   -133,
    1072,   1072,   1772,   1772,   2713,   2713,   4424,   4424,   6866,
    6866,   9551,   9551,   11982,  11982,  13921,  13921,  15185,  15185,
    15317,  15317,  14234,  14234,  12693,  12693,  11558,  11558,  11130,
    11130,  10743,  10743,  9699,   9699,   7890,   7890,   5328,   5328,
    2591,   2591,   -89,    -89,    -2595,  -2595,  -4592,  -4592,  -6022,
    -6022,  -7088,  -7088,  -8471,  -8471,  -10385, -10385, -12386, -12386,
    -13848, -13848, -14484, -14484, -14268, -14268, -13383, -13383, -12084,
    -12084, -10473, -10473, -8892,  -8892,  -7686,  -7686,  -6822,  -6822,
    -5814,  -5814,  -4320,  -4320,  -2374,  -2374,  -392,   -392,   1295,
    1295,   2839,   2839,   4320,   4320,   5678,   5678,   6622,   6622,
    7000,   7000,   7171,   7171,   7306,   7306,   7278,   7278,   6921,
    6921,   6246,   6246,   5524,   5524,   4918,   4918,   4457,   4457,
    3964,   3964,   3315,   3315,   2574,   2574,   1870,   1870,   1360,
    1360,   994,    994,    642,    642,    325,    325,    166,    166,
    182,    182};
uint32_t phaseTable[8][6] = {
    {15624207, 24801882, 39370534, 62496826, 99207528, 157482134},
    {16553270, 26276679, 41711627, 66213081, 105106715, 166846509},
    {17537579, 27839171, 44191930, 70150316, 111356685, 176767719},
    {18580418, 29494575, 46819719, 74321671, 117978298, 187278874},
    {19685267, 31248413, 49603764, 78741067, 124993653, 198415056},
    {20855814, 33106541, 52553357, 83423255, 132426162, 210213429},
    {22095965, 35075158, 55678342, 88383859, 140300631, 222713370},
    {23409859, 37160835, 58989149, 93639437, 148643341, 235956596},
};
uint32_t vibratoLUT[10] = {0,      89478,  178956, 268435, 357913,
                           447392, 536870, 626349, 715827, 805306};
volatile uint32_t counter;

volatile float attack;

volatile uint8_t countFlag;
volatile uint8_t tapFlag;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_CORDIC_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM4_Init(void);
static void MX_I2S2_Init(void);
static void MX_TIM2_Init(void);
static void MX_I2C1_Init(void);
static void MX_ADC2_Init(void);
static void MX_TIM7_Init(void);
/* USER CODE BEGIN PFP */

void countUp(void) { counter++; }

void fillFirstHalf() {
  int32_t smallassBufferIN[128] = {0};
  int32_t smallassBufferOUT[128] = {0};
  for (int i = 0; i < 128; i++) {
    smallassBufferIN[i] = phaseAcc;
    HAL_CORDIC_Calculate(&hcordic, &phaseAcc2, &vibrato, 1, HAL_MAX_DELAY);
    phaseAcc2 += phaseStep2;
    phaseAcc += phaseStep + (int32_t)((int64_t)vibrato * phaseStep >> 31) / 100;
  }
  HAL_CORDIC_Calculate(&hcordic, smallassBufferIN, smallassBufferOUT, 128,
                       HAL_MAX_DELAY);

  for (int i = 0; i < 128; i++) {
    bigassBuffer[2 * i] = (smallassBufferOUT[i] >> 16) * attack;
    bigassBuffer[(2 * i) + 1] = (smallassBufferOUT[i] >> 16) * attack;
  }

  /* Kasane teto just in case you need her

  for (int i = 0; i < 128; i++) {
    uint8_t i2 = ((uint64_t)phaseAcc * 163) >> 32;
    bigassBuffer[2 * i] = aaaaSample[2 * i2] >> 2;
    bigassBuffer[(2 * i) + 1] = aaaaSample[(2 * i2) + 1] >> 2;

    HAL_CORDIC_Calculate(&hcordic, &phaseAcc2, &vibrato, 1, HAL_MAX_DELAY);
    phaseAcc2 += phaseStep2;
    phaseAcc += phaseStep + (int32_t)((int64_t)vibrato * phaseStep >> 31) / 100;
  }
  */
}

void fillSecondHalf() {
  int32_t smallassBufferIN[128] = {0};
  int32_t smallassBufferOUT[128] = {0};
  for (int i = 0; i < 128; i++) {
    smallassBufferIN[i] = phaseAcc;
    phaseAcc += phaseStep;
  }
  HAL_CORDIC_Calculate(&hcordic, smallassBufferIN, smallassBufferOUT, 128,
                       HAL_MAX_DELAY);
  HAL_CORDIC_Calculate(&hcordic, &phaseAcc2, &vibrato, 1, HAL_MAX_DELAY);
  phaseAcc2 += phaseStep2;
  phaseAcc += phaseStep + (int32_t)((int64_t)vibrato * phaseStep >> 31) / 100;

  for (int i = 0; i < 128; i++) {

    bigassBuffer[(2 * i) + 256] = (smallassBufferOUT[i] >> 16) * attack;
    bigassBuffer[(2 * i) + 257] = (smallassBufferOUT[i] >> 16) * attack;
  }

  /* kasane teto, just in case
  for (int i = 0; i < 128; i++) {
    uint8_t i2 = ((uint64_t)phaseAcc * 163) >> 32;
    bigassBuffer[2 * i + 256] = aaaaSample[2 * i2] >> 2;
    bigassBuffer[(2 * i) + 257] = aaaaSample[2 * i2 + 1] >> 2;
    HAL_CORDIC_Calculate(&hcordic, &phaseAcc2, &vibrato, 1, HAL_MAX_DELAY);
    phaseAcc2 += phaseStep2;
    phaseAcc += phaseStep + (int32_t)((int64_t)vibrato * phaseStep >> 31) / 100;
  } */
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

  /* USER CODE BEGIN 1 */
  SystemCoreClockUpdate();

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick.
   */
  HAL_Init();

  /* USER CODE BEGIN Init */
  SystemCoreClockUpdate();
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_CORDIC_Init();
  MX_USART1_UART_Init();
  MX_TIM4_Init();
  MX_I2S2_Init();
  MX_TIM2_Init();
  MX_I2C1_Init();
  MX_ADC2_Init();
  MX_TIM7_Init();
  /* USER CODE BEGIN 2 */
  CORDIC_ConfigTypeDef cordic;
  cordic.Function = CORDIC_FUNCTION_SINE;
  cordic.Scale = CORDIC_SCALE_0;
  cordic.InSize = CORDIC_INSIZE_32BITS;
  cordic.OutSize = CORDIC_OUTSIZE_32BITS;
  cordic.Precision = CORDIC_PRECISION_6CYCLES;
  cordic.NbRead = CORDIC_NBREAD_1;
  cordic.NbWrite = CORDIC_NBWRITE_1;
  if (HAL_CORDIC_Configure(&hcordic, &cordic) != HAL_OK) {
    Error_Handler();
  }
  HAL_TIM_Base_Start_IT(&htim4);
  HAL_TIM_Base_Start_IT(&htim7);
  HAL_TIM_Base_Start_IT(&htim2);
  HAL_StatusTypeDef i2sStatus;
  i2sStatus = HAL_I2S_Transmit_DMA(&hi2s2, bigassBuffer, 512);
  SH1106_Init();
  char textBuffer[50] = {0};
  sprintf(textBuffer, "Hi lol");
  SH1106_Puts(textBuffer, &Font_11x18, SH1106_COLOR_WHITE);
  SH1106_UpdateScreen();

  uint32_t textCounter = 0;
  uint8_t vibratoPrev = 0;
  uint8_t vibratoCurr = 0;
  uint8_t vibratoState = 0;
  uint8_t vibratoCounter = 0;
  uint16_t attackCoeff = 1;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {

    if (flagSet == 1) {
      flagSet = 0;
      HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_6);
      textCounter += 1;
    }
    if (textCounter == 5) {
      sprintf(textBuffer, "hope ya fine");
      SH1106_GotoXY(16, 19);
      SH1106_Puts(textBuffer, &Font_7x10, SH1106_COLOR_WHITE);
      SH1106_UpdateScreen();
    }

    if (textCounter == 10) {
      sprintf(textBuffer, "Welcome to synth");
      SH1106_GotoXY(6, 31);
      SH1106_Puts(textBuffer, &Font_7x10, SH1106_COLOR_WHITE);
      SH1106_UpdateScreen();
    }

    if (textCounter == 15) {
      sprintf(textBuffer, "Press button");
      SH1106_Clear();
      SH1106_GotoXY(0, 0);
      SH1106_Puts(textBuffer, &Font_7x10, SH1106_COLOR_WHITE);
      sprintf(textBuffer, "for vibrato ctrl");
      SH1106_GotoXY(0, 11);
      SH1106_Puts(textBuffer, &Font_7x10, SH1106_COLOR_WHITE);
      SH1106_UpdateScreen();
    }

    vibratoCurr = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14);

    if (vibratoCurr == 0 && vibratoPrev == 1) {
      vibratoState = !vibratoState;
      vibratoCounter += 1;
    }

    vibratoPrev = vibratoCurr; // update state AFTER the edge check
    if (vibratoState == 1 && vibratoCounter % 2 == 1 &&
        vibratoCounter % 3 != 1) {

      HAL_ADC_Start(&hadc2);
      HAL_ADC_PollForConversion(&hadc2, HAL_MAX_DELAY);
      uint32_t ADCData = HAL_ADC_GetValue(&hadc2);
      HAL_ADC_Stop(&hadc2);
      sprintf(textBuffer, "VIBRATO: ");
      SH1106_Clear();
      SH1106_GotoXY(0, 0);
      SH1106_Puts(textBuffer, &Font_11x18, SH1106_COLOR_WHITE);
      uint8_t vibratoInput = 9 - ((ADCData * 10) >> 12);
      sprintf(textBuffer, "%d", vibratoInput);
      SH1106_GotoXY(0, 19);
      SH1106_Puts(textBuffer, &Font_7x10, SH1106_COLOR_WHITE);
      SH1106_UpdateScreen();
      phaseStep2 = vibratoLUT[vibratoInput];
    }

    if (vibratoState == 1 && vibratoCounter % 3 == 1) {

      HAL_ADC_Start(&hadc2);
      HAL_ADC_PollForConversion(&hadc2, HAL_MAX_DELAY);
      uint32_t ADCData = HAL_ADC_GetValue(&hadc2);
      HAL_ADC_Stop(&hadc2);
      sprintf(textBuffer, "ATTACK: ");
      SH1106_Clear();
      SH1106_GotoXY(0, 0);
      SH1106_Puts(textBuffer, &Font_11x18, SH1106_COLOR_WHITE);
      uint8_t attackInput = 9 - ((ADCData * 10) >> 12);
      sprintf(textBuffer, "%d00 ms", attackInput);
      SH1106_GotoXY(0, 19);
      SH1106_Puts(textBuffer, &Font_7x10, SH1106_COLOR_WHITE);
      SH1106_UpdateScreen();
      attackCoeff = 100 * attackInput + 1;
    }

    /* matrix scan + buffer filling */
    if (scanFlag == 1) {
      scanFlag = 0;
      for (uint8_t i = 0; i < 8; i++) {
        HAL_GPIO_WritePin(GPIOA, 0x00FF, 0);
        HAL_GPIO_WritePin(GPIOA, 1 << i, 1);
        /* Give time, if you skip this loop you will get ghost notes */
        for (uint8_t d = 0; d < 100; d++) {
          __NOP();
        }
        for (uint8_t j = 0; j < 6; j++) {
          uint8_t pressed = HAL_GPIO_ReadPin(GPIOB, (1 << j));

          if (pressed != prevState[i][j]) {
            prevState[i][j] = pressed;

            if (pressed) {
              phaseStep = phaseTable[i][j];
              tapFlag = 1;
              attack = 0.0f;
            } else {
              phaseStep = 0;
              tapFlag = 0;
            }
          }
        }
      }
    }
    // tapFlag = 0;

    if (countFlag == 1) {
      countFlag = 0;
      if (tapFlag && attack < 1.0f) {
        attack += 1.0f / (48 * attackCoeff);
        if (attack >= 1.0f)
          attack = 1.0f;
      }
    }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
   */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 21;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) {
    Error_Handler();
  }
}

/**
 * @brief ADC2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC2_Init(void) {

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Common config
   */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.GainCompensation = 0;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.ContinuousConvMode = ENABLE;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc2.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc2) != HAL_OK) {
    Error_Handler();
  }

  /** Configure Regular Channel
   */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_12CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */
}

/**
 * @brief CORDIC Initialization Function
 * @param None
 * @retval None
 */
static void MX_CORDIC_Init(void) {

  /* USER CODE BEGIN CORDIC_Init 0 */

  /* USER CODE END CORDIC_Init 0 */

  /* USER CODE BEGIN CORDIC_Init 1 */

  /* USER CODE END CORDIC_Init 1 */
  hcordic.Instance = CORDIC;
  if (HAL_CORDIC_Init(&hcordic) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN CORDIC_Init 2 */

  /* USER CODE END CORDIC_Init 2 */
}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void) {

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x20B21E5A;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
    Error_Handler();
  }

  /** Configure Analogue filter
   */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK) {
    Error_Handler();
  }

  /** Configure Digital filter
   */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */
  /* USER CODE END I2C1_Init 2 */
}

/**
 * @brief I2S2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2S2_Init(void) {

  /* USER CODE BEGIN I2S2_Init 0 */

  /* USER CODE END I2S2_Init 0 */

  /* USER CODE BEGIN I2S2_Init 1 */

  /* USER CODE END I2S2_Init 1 */
  hi2s2.Instance = SPI2;
  hi2s2.Init.Mode = I2S_MODE_MASTER_TX;
  hi2s2.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s2.Init.DataFormat = I2S_DATAFORMAT_16B;
  hi2s2.Init.MCLKOutput = I2S_MCLKOUTPUT_DISABLE;
  hi2s2.Init.AudioFreq = I2S_AUDIOFREQ_48K;
  hi2s2.Init.CPOL = I2S_CPOL_LOW;
  if (HAL_I2S_Init(&hi2s2) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S2_Init 2 */

  /* USER CODE END I2S2_Init 2 */
}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void) {

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 167;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 2000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK) {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
}

/**
 * @brief TIM4 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM4_Init(void) {

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 1680 - 1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 29999;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK) {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK) {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
}

/**
 * @brief TIM7 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM7_Init(void) {

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 0;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 3499;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK) {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */
}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void) {

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) !=
      HAL_OK) {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) !=
      HAL_OK) {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */
}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void) {

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA,
                    GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 |
                        GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7,
                    GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA0 PA1 PA2 PA3
                           PA4 PA5 PA6 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 |
                        GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB3 PB4
                           PB5 */
  GPIO_InitStruct.Pin =
      GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB14 */
  GPIO_InitStruct.Pin = GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PC6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  if (htim->Instance == TIM7) {
    countUp();
    countFlag = 1;
  }
  if (htim->Instance == TIM4) {
    flagSet = 1;
  }
  if (htim->Instance == TIM2) {
    scanFlag = 1;
  }
}

void HAL_I2S_TxHalfCpltCallback(I2S_HandleTypeDef *hi2s) { fillFirstHalf(); }
void HAL_I2S_TxCpltCallback(I2S_HandleTypeDef *hi2s) { fillSecondHalf(); }

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1) {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line) {
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line
     number, ex: printf("Wrong parameters value: file %s on line %d\r\n", file,
     line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
