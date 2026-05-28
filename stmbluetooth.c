/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : STM32F407 HM-19 Packet Test Receiver (TWO HM-19s)
  ******************************************************************************
  *
  * Test setup:
  *
  * ESP32-C3 #1 -> BLE -> HM-19 #1 -> STM32 USART2  (Patient 1)
  * ESP32-C3 #2 -> BLE -> HM-19 #2 -> STM32 USART?  (Patient 2)  <- NEW, see TODO
  * STM32 prints received packets to laptop / Pi using USB CDC
  *
  * USART2 (HM-19 #1):
  *   PA2 = USART2_TX
  *   PA3 = USART2_RX
  *
  * USART3 (HM-19 #2)  <-- PLACEHOLDER PINS, CONFIRM/CHANGE (see MX_USART3_UART_Init):
  *   PD8 = USART3_TX
  *   PD9 = USART3_RX
  *
  * USB CDC:
  *   Used as STM32 serial display on laptop / link to Pi
  *
  ******************************************************************************
  */
/* USER CODE END Header */

#include "main.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"

#include <stdio.h>
#include <string.h>
#include <stdint.h>

/* Private variables ---------------------------------------------------------*/

I2C_HandleTypeDef hi2c1;
I2S_HandleTypeDef hi2s3;
SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;   /* NEW: HM-19 #2 */

/* Function prototypes -------------------------------------------------------*/

void SystemClock_Config(void);

static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2S3_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);   /* NEW: HM-19 #2 */

void Error_Handler(void);

/* USER CODE BEGIN 0 */

#define RX_LINE_SIZE 512

/* --- HM-19 #1 (Patient 1) on USART2 --- */
uint8_t rx2_byte;
char rx2_line[RX_LINE_SIZE];
uint16_t rx2_index = 0;

/* --- HM-19 #2 (Patient 2) on USART3 --- NEW --- */
uint8_t rx3_byte;
char rx3_line[RX_LINE_SIZE];
uint16_t rx3_index = 0;

/* Send text to laptop over USB CDC */
void send_usb(const char *msg)
{
    uint32_t start = HAL_GetTick();

    while (CDC_Transmit_FS((uint8_t *)msg, strlen(msg)) == USBD_BUSY)
    {
        if ((HAL_GetTick() - start) > 100)
        {
            return;
        }
    }
}

/* Clear receive buffers */
void clear_rx2_buffer(void)
{
    memset(rx2_line, 0, RX_LINE_SIZE);
    rx2_index = 0;
}

void clear_rx3_buffer(void)   /* NEW */
{
    memset(rx3_line, 0, RX_LINE_SIZE);
    rx3_index = 0;
}

/* Basic packet check */
int is_valid_packet_start(const char *line)
{
    if (strncmp(line, "VP", 2) == 0)
    {
        return 1;
    }

    if (strncmp(line, "<VP", 3) == 0)
    {
        return 1;
    }

    if (strncmp(line, "Patient:", 8) == 0)
    {
        return 1;
    }

    if (strncmp(line, "<Patient:", 9) == 0)
    {
        return 1;
    }

    return 0;
}

/* Optional: reject weird non-printable characters */
int is_safe_char(uint8_t c)
{
    if (c >= 32 && c <= 126)
    {
        return 1;
    }

    return 0;
}

/* USER CODE END 0 */

int main(void)
{
    HAL_Init();

    SystemClock_Config();

    MX_GPIO_Init();
    MX_I2C1_Init();
    MX_I2S3_Init();
    MX_SPI1_Init();
    MX_USART2_UART_Init();
    MX_USART3_UART_Init();   /* NEW: HM-19 #2 */
    MX_USB_DEVICE_Init();

    HAL_Delay(2000);

    clear_rx2_buffer();
    clear_rx3_buffer();   /* NEW */

    send_usb("\r\n==============================\r\n");
    send_usb("[STM32] HM-19 Packet Test Started\r\n");
    send_usb("[STM32] Listening on USART2 (Patient 1) + USART3 (Patient 2)\r\n");
    send_usb("[STM32] Waiting for newline-ended packets...\r\n");
    send_usb("==============================\r\n\r\n");

    while (1)
    {
        /* =====================================================================
         *  USART2 : HM-19 #1  (Patient 1)
         *  Timeout is 0 (non-blocking) so this read never stalls the USART3
         *  read below. With two streams a blocking read would let the other
         *  UART overrun and corrupt its packet, so both polls are non-blocking.
         * ===================================================================== */
        if (HAL_UART_Receive(&huart2, &rx2_byte, 1, 0) == HAL_OK)
        {
            if (rx2_byte == '\n')
            {
                rx2_line[rx2_index] = '\0';

                send_usb("[STM32 RAW] ");
                send_usb(rx2_line);
                send_usb("\r\n");

                if (is_valid_packet_start(rx2_line))
                {
                	send_usb("[STM32 CLEAN PACKET] ");
                	send_usb(rx2_line);
                	send_usb("\r\n");

                    HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);
                }
                else
                {
                    send_usb("[STM32 BAD PACKET - ignored]\r\n\r\n");
                }

                clear_rx2_buffer();
            }
            else if (rx2_byte == '\r')
            {
                /* Ignore carriage return. We only use '\n'. */
            }
            else
            {
                if (is_safe_char(rx2_byte))
                {
                    if (rx2_index < RX_LINE_SIZE - 1)
                    {
                        rx2_line[rx2_index++] = (char)rx2_byte;
                    }
                    else
                    {
                        send_usb("[STM32 ERROR] RX buffer overflow. Dropping packet.\r\n\r\n");
                        clear_rx2_buffer();
                    }
                }
                else
                {
                    send_usb("[STM32 WARNING] Non-printable byte ignored\r\n");
                }
            }
        }

        /* =====================================================================
         *  USART3 : HM-19 #2  (Patient 2)   --- NEW ---
         *  Same logic as USART2, separate buffer, "P2" tags on the USB output
         *  so you can tell the two streams apart on the console. The packet
         *  itself still carries Patient:2, so the Pi distinguishes by that.
         * ===================================================================== */
        if (HAL_UART_Receive(&huart3, &rx3_byte, 1, 0) == HAL_OK)
        {
            if (rx3_byte == '\n')
            {
                rx3_line[rx3_index] = '\0';

                send_usb("[STM32 RAW P2] ");
                send_usb(rx3_line);
                send_usb("\r\n");

                if (is_valid_packet_start(rx3_line))
                {
                    send_usb("[STM32 CLEAN PACKET P2] ");
                    send_usb(rx3_line);
                    send_usb("\r\n");

                    HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);
                }
                else
                {
                    send_usb("[STM32 BAD PACKET P2 - ignored]\r\n\r\n");
                }

                clear_rx3_buffer();
            }
            else if (rx3_byte == '\r')
            {
                /* Ignore carriage return. We only use '\n'. */
            }
            else
            {
                if (is_safe_char(rx3_byte))
                {
                    if (rx3_index < RX_LINE_SIZE - 1)
                    {
                        rx3_line[rx3_index++] = (char)rx3_byte;
                    }
                    else
                    {
                        send_usb("[STM32 ERROR P2] RX buffer overflow. Dropping packet.\r\n\r\n");
                        clear_rx3_buffer();
                    }
                }
                else
                {
                    send_usb("[STM32 WARNING P2] Non-printable byte ignored\r\n");
                }
            }
        }
    }
}

/* Clock config from working USB project ------------------------------------*/

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    __HAL_RCC_PWR_CLK_ENABLE();

    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 8;
    RCC_OscInitStruct.PLL.PLLN = 336;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 7;

    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    RCC_ClkInitStruct.ClockType =
        RCC_CLOCKTYPE_HCLK |
        RCC_CLOCKTYPE_SYSCLK |
        RCC_CLOCKTYPE_PCLK1 |
        RCC_CLOCKTYPE_PCLK2;

    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
    {
        Error_Handler();
    }
}

/* USART2 = HM-19 #1 UART -----------------------------------------------------*/

static void MX_USART2_UART_Init(void)
{
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
}

/* USART3 = HM-19 #2 UART ----------------------------------------------------*/
/* ==========================================================================
 *  >>>>>>>>>>  TODO TEAM: SECOND HM-19 UART — CONFIRM / CHANGE PINS  <<<<<<<<<<
 *  --------------------------------------------------------------------------
 *  PLACEHOLDER: USART3 on PD8 (TX) / PD9 (RX), AF7, 9600 8N1.
 *
 *  Change the instance / port / pins / AF below to match how you actually
 *  wire HM-19 #2. If you wire HM-19 #2 to PD8/PD9 you can leave this as-is.
 *
 *  Wiring for the placeholder:
 *      PD8 (STM32 TX) ----> HM-19 #2 RXD
 *      PD9 (STM32 RX) <---- HM-19 #2 TXD
 *      GND            ----- HM-19 #2 GND   (shared ground required)
 *
 *  This function is self-contained: it enables its own peripheral + GPIO
 *  clocks and configures the pins, so you do NOT need to edit CubeMX or
 *  stm32f4xx_hal_msp.c. Baud kept at 9600 to match HM-19 #1.
 * ==========================================================================*/
static void MX_USART3_UART_Init(void)
{
    /* ---- clocks (CHANGE if you move off USART3 / GPIOD) ---- */
    __HAL_RCC_USART3_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();

    /* ---- GPIO: PD8 = TX, PD9 = RX, AF7 (CHANGE pins/port/AF here) ---- */
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    /* ---- UART (CHANGE .Instance if you move off USART3) ---- */
    huart3.Instance = USART3;

    huart3.Init.BaudRate = 9600;
    huart3.Init.WordLength = UART_WORDLENGTH_8B;
    huart3.Init.StopBits = UART_STOPBITS_1;
    huart3.Init.Parity = UART_PARITY_NONE;
    huart3.Init.Mode = UART_MODE_TX_RX;
    huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart3.Init.OverSampling = UART_OVERSAMPLING_16;

    if (HAL_UART_Init(&huart3) != HAL_OK)
    {
        Error_Handler();
    }
}

/* I2C1 init -----------------------------------------------------------------*/

static void MX_I2C1_Init(void)
{
    hi2c1.Instance = I2C1;

    hi2c1.Init.ClockSpeed = 100000;
    hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
    hi2c1.Init.OwnAddress1 = 0;
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2 = 0;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;

    if (HAL_I2C_Init(&hi2c1) != HAL_OK)
    {
        Error_Handler();
    }
}

/* I2S3 init -----------------------------------------------------------------*/

static void MX_I2S3_Init(void)
{
    hi2s3.Instance = SPI3;

    hi2s3.Init.Mode = I2S_MODE_MASTER_TX;
    hi2s3.Init.Standard = I2S_STANDARD_PHILIPS;
    hi2s3.Init.DataFormat = I2S_DATAFORMAT_16B;
    hi2s3.Init.MCLKOutput = I2S_MCLKOUTPUT_ENABLE;
    hi2s3.Init.AudioFreq = I2S_AUDIOFREQ_96K;
    hi2s3.Init.CPOL = I2S_CPOL_LOW;
    hi2s3.Init.ClockSource = I2S_CLOCK_PLL;
    hi2s3.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;

    if (HAL_I2S_Init(&hi2s3) != HAL_OK)
    {
        Error_Handler();
    }
}

/* SPI1 init -----------------------------------------------------------------*/

static void MX_SPI1_Init(void)
{
    hspi1.Instance = SPI1;

    hspi1.Init.Mode = SPI_MODE_MASTER;
    hspi1.Init.Direction = SPI_DIRECTION_2LINES;
    hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
    hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
    hspi1.Init.NSS = SPI_NSS_SOFT;
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
    hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi1.Init.CRCPolynomial = 10;

    if (HAL_SPI_Init(&hspi1) != HAL_OK)
    {
        Error_Handler();
    }
}

/* GPIO init -----------------------------------------------------------------*/

static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOD_CLK_ENABLE();

    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);

    GPIO_InitStruct.Pin = GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
}

/* Error handler -------------------------------------------------------------*/

void Error_Handler(void)
{
    __disable_irq();

    while (1)
    {
    }
}
