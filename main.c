/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : STM32F407 Dual HM-19 USB CDC Patient Monitor
  ******************************************************************************
  *
  * USART2 -> HM19 Patient #1
  * USART3 -> HM19 Patient #2
  *
  * USB CDC -> Raspberry Pi
  *
  ******************************************************************************
  */
/* USER CODE END Header */

#include "main.h"
#include "usb_device.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

#include "usbd_cdc_if.h"

/* Private typedef -----------------------------------------------------------*/

typedef struct
{
    uint32_t timestamp;

    uint16_t hr;
    float spo2;
    float temp_c;

    float ax;
    float ay;
    float az;

    float gx;
    float gy;
    float gz;

} SensorData;

typedef enum
{
    NORMAL,
    FREEFALL,
    IMPACT,
    FLAT

} FallState;

/* Private define ------------------------------------------------------------*/

#define RX_LINE_SIZE 180

#define ALERT_HR_LOW      (1 << 0)
#define ALERT_HR_HIGH     (1 << 1)
#define ALERT_TEMP_HIGH   (1 << 2)
#define ALERT_TEMP_LOW    (1 << 3)
#define ALERT_SPO2_LOW    (1 << 4)
#define ALERT_FALL        (1 << 5)

/* Private variables ---------------------------------------------------------*/

I2C_HandleTypeDef hi2c1;
I2S_HandleTypeDef hi2s3;
SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* UART RX */

uint8_t rx2_byte;
char rx2_line[RX_LINE_SIZE];
uint16_t rx2_index = 0;

uint8_t rx3_byte;
char rx3_line[RX_LINE_SIZE];
uint16_t rx3_index = 0;

/* Patient states */

FallState fall_state_1 = NORMAL;
uint32_t fall_timer_1 = 0;
uint8_t fall_detected_1 = 0;

FallState fall_state_2 = NORMAL;
uint32_t fall_timer_2 = 0;
uint8_t fall_detected_2 = 0;

/* Function prototypes */

void SystemClock_Config(void);

static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2S3_Init(void);
static void MX_SPI1_Init(void);

static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);

void send_packet(char *packet);

uint8_t analyze_hr(uint16_t bpm);
uint8_t analyze_temp(float temp);
uint8_t analyze_spo2(float spo2);

float accel_mag(float ax, float ay, float az);
float gyro_mag(float gx, float gy, float gz);

int is_flat(float az);
int is_still(float gx, float gy, float gz);

void analyze_motion(
    FallState *state,
    uint32_t *timer,
    uint8_t *fall_detected,
    uint32_t t,
    float ax,
    float ay,
    float az,
    float gx,
    float gy,
    float gz
);

void send_patient_packet(
    int patient,
    uint16_t hr,
    float spo2,
    float temp_c,
    uint8_t flags
);

int parse_vp1_packet(char *line, SensorData *d);

void process_sensor_packet(
    int patient,
    SensorData *d,
    FallState *fall_state,
    uint32_t *fall_timer,
    uint8_t *fall_detected
);

/* USER CODE BEGIN 0 */

void send_packet(char *packet)
{
    uint32_t timeout = HAL_GetTick();

    while (CDC_Transmit_FS((uint8_t*)packet, strlen(packet)) == USBD_BUSY)
    {
        if((HAL_GetTick() - timeout) > 100)
            return;
    }
}

uint8_t analyze_hr(uint16_t bpm)
{
    uint8_t f = 0;

    if (bpm < 60)
        f |= ALERT_HR_LOW;

    if (bpm > 100)
        f |= ALERT_HR_HIGH;

    return f;
}

uint8_t analyze_temp(float temp)
{
    uint8_t f = 0;

    if (temp > 38.0f)
        f |= ALERT_TEMP_HIGH;

    if (temp < 35.0f)
        f |= ALERT_TEMP_LOW;

    return f;
}

uint8_t analyze_spo2(float spo2)
{
    if (spo2 < 90.0f)
        return ALERT_SPO2_LOW;

    return 0;
}

float accel_mag(float ax, float ay, float az)
{
    return sqrtf(ax*ax + ay*ay + az*az);
}

float gyro_mag(float gx, float gy, float gz)
{
    return fabsf(gx) + fabsf(gy) + fabsf(gz);
}

int is_flat(float az)
{
    return fabsf(az) < 0.5f;
}

int is_still(float gx, float gy, float gz)
{
    return (fabsf(gx) + fabsf(gy) + fabsf(gz)) < 10.0f;
}

void analyze_motion(
    FallState *state,
    uint32_t *timer,
    uint8_t *fall_detected,
    uint32_t t,
    float ax,
    float ay,
    float az,
    float gx,
    float gy,
    float gz
)
{
    float acc = accel_mag(ax, ay, az);
    float gyro = gyro_mag(gx, gy, gz);

    switch(*state)
    {
        case NORMAL:

            if(acc < 0.5f)
            {
                *state = FREEFALL;
                *timer = t;
            }

            break;

        case FREEFALL:

            if(acc > 2.5f)
            {
                *state = IMPACT;
            }
            else if((t - *timer) > 1000)
            {
                *state = NORMAL;
            }

            break;

        case IMPACT:

            if(is_flat(az) && gyro > 150.0f)
            {
                *state = FLAT;
                *timer = t;
            }
            else
            {
                *state = NORMAL;
            }

            break;

        case FLAT:

            if(is_still(gx, gy, gz) &&
               ((t - *timer) > 1500))
            {
                *fall_detected = 1;
                *state = NORMAL;
            }
            else if(!is_still(gx, gy, gz))
            {
                *state = NORMAL;
            }

            break;
    }
}

void send_patient_packet(
    int patient,
    uint16_t hr,
    float spo2,
    float temp_c,
    uint8_t flags
)
{
    char packet[256];

    snprintf(packet,
             sizeof(packet),
             "Patient:%d HR:%d SPO2:%.1f TEMP:%.1f FLAGS:%d\r\n",
             patient,
             hr,
             spo2,
             temp_c,
             flags);

    send_packet(packet);
}



int parse_vp1_packet(char *line, SensorData *d)
{
    char *token;

    token = strtok(line, ",");

    if(token == NULL) return 0;

    if(strcmp(token, "VP1") != 0)
        return 0;

    token = strtok(NULL, ",");
    if(token == NULL) return 0;
    d->timestamp = strtoul(token, NULL, 10);

    token = strtok(NULL, ",");
    if(token == NULL) return 0;
    d->hr = (uint16_t)atoi(token);

    token = strtok(NULL, ",");
    if(token == NULL) return 0;
    d->spo2 = (float)atof(token);

    token = strtok(NULL, ",");
    if(token == NULL) return 0;
    d->temp_c = (float)atof(token);

    token = strtok(NULL, ",");
    if(token == NULL) return 0;
    d->ax = (float)atof(token);

    token = strtok(NULL, ",");
    if(token == NULL) return 0;
    d->ay = (float)atof(token);

    token = strtok(NULL, ",");
    if(token == NULL) return 0;
    d->az = (float)atof(token);

    token = strtok(NULL, ",");
    if(token == NULL) return 0;
    d->gx = (float)atof(token);

    token = strtok(NULL, ",");
    if(token == NULL) return 0;
    d->gy = (float)atof(token);

    token = strtok(NULL, ",");
    if(token == NULL) return 0;
    d->gz = (float)atof(token);

    return 1;
}

void process_sensor_packet(
    int patient,
    SensorData *d,
    FallState *fall_state,
    uint32_t *fall_timer,
    uint8_t *fall_detected
)
{
    uint8_t flags = 0;

    flags |= analyze_hr(d->hr);
    flags |= analyze_spo2(d->spo2);
    flags |= analyze_temp(d->temp_c);

    analyze_motion(
        fall_state,
        fall_timer,
        fall_detected,
        HAL_GetTick(),
        d->ax,
        d->ay,
        d->az,
        d->gx,
        d->gy,
        d->gz
    );

    if(*fall_detected)
    {
        flags |= ALERT_FALL;
        *fall_detected = 0;
    }

    send_patient_packet(
        patient,
        d->hr,
        d->spo2,
        d->temp_c,
        flags
    );
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
    MX_USART3_UART_Init();

    MX_USB_DEVICE_Init();

    HAL_Delay(2000);

    while (1)
    {
        /* USART2 */
//
//        if(HAL_UART_Receive(&huart2, &rx2_byte, 1, 10) == HAL_OK)
//        {
//            if(rx2_byte == '\n')
//            {
//                rx2_line[rx2_index] = '\0';
//                char debug[256];
//                snprintf(debug, sizeof(debug), "HM19-1 RAW: %s\r\n", rx2_line);
//                send_packet(debug);
//
//                SensorData d;
//
//                if(parse_vp1_packet(rx2_line, &d))
//                {
//                    process_sensor_packet(
//                        1,
//                        &d,
//                        &fall_state_1,
//                        &fall_timer_1,
//                        &fall_detected_1
//                    );
//
//                    HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);
//                }
//
//                rx2_index = 0;
//            }
//            else if(rx2_byte != '\r')
//            {
//                if(rx2_index < RX_LINE_SIZE - 1)
//                {
//                    rx2_line[rx2_index++] = rx2_byte;
//                }
//            }
//        }
    	if (HAL_UART_Receive(&huart2, &rx2_byte, 1, 100) == HAL_OK)
    	{
    	    if (rx2_byte == '\n')
    	    {
    	        rx2_line[rx2_index] = '\0';

    	        // DEBUG SAFE PRINT
    	        char debug[300];
    	        snprintf(debug, sizeof(debug), "HM19-1 RAW FINAL: %s\r\n", rx2_line);
    	        send_packet(debug);

    	        SensorData d;
    	        if (parse_vp1_packet(rx2_line, &d))
    	        {
    	            process_sensor_packet(1, &d,
    	                                  &fall_state_1,
    	                                  &fall_timer_1,
    	                                  &fall_detected_1);
    	        }

    	        rx2_index = 0;
    	    }
    	    else if (rx2_byte != '\r')
    	    {
    	        if (rx2_index < RX_LINE_SIZE - 1)
    	        {
    	            rx2_line[rx2_index++] = rx2_byte;
    	        }
    	    }
    	}



        /* USART3 */

        if(HAL_UART_Receive(&huart3, &rx3_byte, 1, 10) == HAL_OK)
        {
            if(rx3_byte == '\n')
            {
                rx3_line[rx3_index] = '\0';

                SensorData d;

                if(parse_vp1_packet(rx3_line, &d))
                {
                    process_sensor_packet(
                        2,
                        &d,
                        &fall_state_2,
                        &fall_timer_2,
                        &fall_detected_2
                    );

                    HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_13);
                }

                rx3_index = 0;
            }
            else if(rx3_byte != '\r')
            {
                if(rx3_index < RX_LINE_SIZE - 1)
                {
                    rx3_line[rx3_index++] = rx3_byte;
                }
            }
        }
    }
}

/* Clock config from WORKING USB project */

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

    RCC_ClkInitStruct.SYSCLKSource =
        RCC_SYSCLKSOURCE_PLLCLK;

    RCC_ClkInitStruct.AHBCLKDivider =
        RCC_SYSCLK_DIV1;

    RCC_ClkInitStruct.APB1CLKDivider =
        RCC_HCLK_DIV4;

    RCC_ClkInitStruct.APB2CLKDivider =
        RCC_HCLK_DIV2;

    if (HAL_RCC_ClockConfig(
            &RCC_ClkInitStruct,
            FLASH_LATENCY_5) != HAL_OK)
    {
        Error_Handler();
    }
}

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

static void MX_USART3_UART_Init(void)
{
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

/* KEEP YOUR EXISTING:
   MX_GPIO_Init()
   MX_I2C1_Init()
   MX_I2S3_Init()
   MX_SPI1_Init()
   Error_Handler()
*/



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

void Error_Handler(void)
{
    __disable_irq();

    while (1)
    {
    }
}
