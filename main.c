/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : VitalsPatch STM32 -- PHASE 1 (test-data sender)
  ******************************************************************************
  * WHAT CHANGED vs. your version
  *  - The old loop only RECEIVED a byte and toggled the LED. It never SENT
  *    anything, so the HM-19 (and therefore the dashboard) got nothing.
  *  - This version TRANSMITS a vitals line out USART2 -> HM-19 -> BLE every
  *    500 ms, using synthetic test data so you can see the whole pipe work
  *    end to end before real sensors are wired in.
  *  - The AT+NAME line is fixed (your old one sent only 2 bytes = "AT").
  *
  * LINE FORMAT (one record, newline-terminated):
  *     t,hr,spo2,temp,acc,flags\n
  *  e.g.  5234,73,98.2,36.6,1.01,0
  *  The receiver ESP32 prepends "A," or "B," before forwarding to the PC.
  *
  * EVERYTHING below lives inside the CubeMX USER CODE regions, so regenerating
  * from the .ioc will NOT wipe it. Flash the SAME binary to both patient boards;
  * the A/B label is decided by the receiver ESP32, not here.
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdio.h>
#include <string.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdint.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* Alert bitmask -- mirrors your stm32_analysis.c so the dashboard decodes
   the exact same bits. */
#define ALERT_HR_LOW    (1u << 0)
#define ALERT_HR_HIGH   (1u << 1)
#define ALERT_TEMP_HIGH (1u << 2)
#define ALERT_TEMP_LOW  (1u << 3)
#define ALERT_SPO2_LOW  (1u << 4)
#define ALERT_FALL      (1u << 5)

#define SEND_PERIOD_MS  500u   /* how often we push a record */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* Threshold logic -- same numbers as your stm32_analysis.c */
static uint8_t analyze_hr(uint16_t bpm);
static uint8_t analyze_temp(float temp);
static uint8_t analyze_spo2(float spo2);

/* Sends one finished record out USART2 to the HM-19.
   In Phase 2 this is the only function your processing code needs to call --
   it replaces the printf() inside send_to_pi(). */
static void send_vitals(uint32_t t, uint16_t hr, float spo2,
                        float temp, float acc, uint8_t flags);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static uint8_t analyze_hr(uint16_t bpm) {
    uint8_t f = 0;
    if (bpm < 60)  f |= ALERT_HR_LOW;
    if (bpm > 100) f |= ALERT_HR_HIGH;
    return f;
}

static uint8_t analyze_temp(float temp) {
    uint8_t f = 0;
    if (temp > 38.0f) f |= ALERT_TEMP_HIGH;
    if (temp < 35.0f) f |= ALERT_TEMP_LOW;
    return f;
}

static uint8_t analyze_spo2(float spo2) {
    return (spo2 < 90.0f) ? ALERT_SPO2_LOW : 0;
}

static void send_vitals(uint32_t t, uint16_t hr, float spo2,
                        float temp, float acc, uint8_t flags)
{
    /* Format the decimals by hand (all values are positive) so we DON'T depend
       on newlib's %f -- STM32CubeIDE disables float printf by default, which
       would otherwise make spo2/temp/acc come out blank. This way the output
       is correct out of the box with no linker flags needed. */
    int s10  = (int)(spo2 * 10.0f  + 0.5f);   /* SpO2  x10  (e.g. 982) */
    int t10  = (int)(temp * 10.0f  + 0.5f);   /* temp  x10             */
    int a100 = (int)(acc  * 100.0f + 0.5f);   /* accel x100            */

    char line[64];
    int n = snprintf(line, sizeof(line),
                     "%lu,%u,%d.%d,%d.%d,%d.%02d,%u\n",
                     (unsigned long)t, (unsigned)hr,
                     s10 / 10,  s10 % 10,
                     t10 / 10,  t10 % 10,
                     a100 / 100, a100 % 100,
                     (unsigned)flags);
    if (n > 0) {
        HAL_UART_Transmit(&huart2, (uint8_t *)line, (uint16_t)n, HAL_MAX_DELAY);
    }
}

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
  HAL_Init();

  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  MX_GPIO_Init();
  MX_USART2_UART_Init();

  /* USER CODE BEGIN 2 */

  /* Optional: name the HM-19 once at boot. AT commands ONLY work while the
     module is NOT yet connected to a central, so this must happen before the
     receiver ESP32 connects. FIX: send the full string length, not 2 bytes. */
  uint8_t at_cmd[] = "AT+NAMESTM32BLE\r\n";
  uint8_t rx_data[2] = {0};

  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);
  HAL_UART_Transmit(&huart2, at_cmd, (uint16_t)strlen((char *)at_cmd), HAL_MAX_DELAY);

  if (HAL_UART_Receive(&huart2, rx_data, 2, 3000) == HAL_OK) {
      if (rx_data[0] == 'O' && rx_data[1] == 'K') {
          HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);  /* OK seen */
      }
  }

  /* ---- test-data state (Phase 1 only) ---- */
  uint32_t sample = 0;   /* increments once per send */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      /* ----------------------------------------------------------------
       * Synthesize plausible vitals that occasionally trip each alert,
       * so you can confirm the dashboard reacts. Replace this whole block
       * in Phase 2 with real values feeding your stm32_analysis pipeline.
       * ---------------------------------------------------------------- */
      uint16_t hr   = 72 + (uint16_t)(sample % 8);          /* 72..79 normal */
      float    spo2 = 98.0f;
      float    temp = 36.6f;
      float    acc  = 1.00f;

      /* every ~20 s: tachycardia spike (HR high)  */
      if ((sample % 40) >= 36) hr = 112;
      /* every ~30 s: desat dip (SpO2 low)         */
      if ((sample % 60) >= 56) spo2 = 88.0f;
      /* every ~40 s: fever (temp high)            */
      if ((sample % 80) >= 76) temp = 38.6f;

      uint8_t flags = 0;
      flags |= analyze_hr(hr);
      flags |= analyze_temp(temp);
      flags |= analyze_spo2(spo2);

      /* every ~25 s: simulated fall -> impact spike + FALL flag */
      if ((sample % 50) == 49) {
          acc   = 3.20f;
          flags |= ALERT_FALL;
      }

      send_vitals(HAL_GetTick(), hr, spo2, temp, acc, flags);

      HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);  /* LED heartbeat = sending */
      sample++;
      HAL_Delay(SEND_PERIOD_MS);
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

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART2 Initialization Function
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{
  /* USER CODE BEGIN USART2_Init 0 */
  /* USER CODE END USART2_Init 0 */
  /* USER CODE BEGIN USART2_Init 1 */
  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */
  /* USER CODE END USART2_Init 2 */
}

/**
  * @brief GPIO Initialization Function
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */
  /* USER CODE END MX_GPIO_Init_1 */

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
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
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
