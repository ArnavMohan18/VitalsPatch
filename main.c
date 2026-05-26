/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : STM32F407G-DISC1 USB CDC Patient Monitor
  ******************************************************************************
  *
  * Sends analyzed patient telemetry to Raspberry Pi over USB CDC.
  *
  * CN1 = ST-LINK power/programming
  * CN5 = USB CDC data to Raspberry Pi
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/

#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/

#include <stdio.h>
#include <string.h>
#include <math.h>

#include "usbd_cdc_if.h"

/* Private typedef -----------------------------------------------------------*/

typedef struct
{
    uint16_t hr;
    float spo2;
    float temp_c;

    float ax;
    float ay;
    float az;

    float gx;
    float gy;
    float gz;

} FakeData;

typedef enum
{
    NORMAL,
    FREEFALL,
    IMPACT,
    FLAT

} FallState;

/* Private define ------------------------------------------------------------*/

#define ALERT_HR_LOW      (1 << 0)
#define ALERT_HR_HIGH     (1 << 1)
#define ALERT_TEMP_HIGH   (1 << 2)
#define ALERT_TEMP_LOW    (1 << 3)
#define ALERT_SPO2_LOW    (1 << 4)
#define ALERT_FALL        (1 << 5)

#define FAKE_DATA_COUNT 12

const char *state_str[] =
{
    "NORMAL",
    "FREEFALL",
    "IMPACT",
    "FLAT"
};

/* Private variables ---------------------------------------------------------*/

I2C_HandleTypeDef hi2c1;
I2S_HandleTypeDef hi2s3;
SPI_HandleTypeDef hspi1;

FakeData fake_data[FAKE_DATA_COUNT] =
{
    // Normal
    {75,98.0,36.5, 0.0,0.0,1.0, 2,1,1},

    // Normal
    {78,97.0,36.6, 0.1,0.1,1.0, 3,2,1},

    // High HR
    {120,98.0,36.7, 0.0,0.0,1.0, 2,2,1},

    // Low HR
    {45,99.0,36.5, 0.0,0.0,1.0, 1,1,1},

    // Low SPO2
    {85,86.0,36.6, 0.0,0.0,1.0, 2,1,1},

    // Temp spike
    {88,97.0,39.5, 0.0,0.0,1.0, 3,2,1},

    // Temp low
    {76,98.0,33.0, 0.0,0.0,1.0, 2,1,1},

    // Walking
    {80,98.0,36.5, 0.3,0.2,1.0, 10,8,6},

    // FREEFALL
    {92,96.0,36.7, 0.1,0.1,0.1, 20,15,10},

    // IMPACT
    {94,95.0,36.8, 3.5,2.0,0.2, 180,160,140},

    // FLAT/STILL
    {93,95.0,36.8, 0.0,0.0,0.1, 1,1,1},

    // Recovery
    {82,98.0,36.5, 0.0,0.0,1.0, 3,2,1}
};

int data_index = 0;

FallState fall_state = NORMAL;

uint32_t fall_timer = 0;

uint8_t fall_detected = 0;

/* Private function prototypes -----------------------------------------------*/

void SystemClock_Config(void);

static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2S3_Init(void);
static void MX_SPI1_Init(void);

void send_packet(char *packet);

uint8_t analyze_hr(uint16_t bpm);
uint8_t analyze_temp(float temp);
uint8_t analyze_spo2(float spo2);

float accel_mag(float ax, float ay, float az);
float gyro_mag(float gx, float gy, float gz);

int is_flat(float az);
int is_still(float gx, float gy, float gz);

void analyze_motion(
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

/* Private user code ---------------------------------------------------------*/

void send_packet(char *packet)
{
    while (CDC_Transmit_FS((uint8_t*)packet, strlen(packet)) == USBD_BUSY)
    {
        HAL_Delay(1);
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

    switch(fall_state)
    {
        case NORMAL:

            if(acc < 0.5f)
            {
                fall_state = FREEFALL;
                fall_timer = t;
            }

            break;

        case FREEFALL:

            if(acc > 2.5f)
            {
                fall_state = IMPACT;
            }
            else if((t - fall_timer) > 1000)
            {
                fall_state = NORMAL;
            }

            break;

        case IMPACT:

            if(is_flat(az) && gyro > 150.0f)
            {
                fall_state = FLAT;
                fall_timer = t;
            }
            else
            {
                fall_state = NORMAL;
            }

            break;

        case FLAT:

            if(is_still(gx, gy, gz) &&
               ((t - fall_timer) > 1500))
            {
                fall_detected = 1;
                fall_state = NORMAL;
            }
            else if(!is_still(gx, gy, gz))
            {
                fall_state = NORMAL;
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

    int err[6] = {0};

    if(flags & ALERT_HR_HIGH)   err[0] = 1;
    if(flags & ALERT_HR_LOW)    err[1] = 2;
    if(flags & ALERT_TEMP_HIGH) err[2] = 3;
    if(flags & ALERT_TEMP_LOW)  err[3] = 4;
    if(flags & ALERT_SPO2_LOW)  err[4] = 5;
    if(flags & ALERT_FALL)      err[5] = 6;

    float temp_f = (temp_c * 9.0f / 5.0f) + 32.0f;
    int t1 = (int)(temp_f * 10);
    int t2 = (int)((temp_f + 0.1f) * 10);
    int t3 = (int)((temp_f - 0.1f) * 10);

    snprintf(packet,
             sizeof(packet),

             "Patient:%d;"
             "HR:[%d,%d,%d];"
             "SPO2:[%d,%d,%d];"
             "TEMP:[%d.%d,%d.%d,%d.%d];"
             "ERR:[%d,%d,%d,%d,%d,%d]\r\n",

             patient,

             hr,
             hr + 1,
             hr - 1,

             (int)spo2,
             (int)(spo2 + 1),
             (int)(spo2 - 1),

             t1/10, abs(t1%10),
             t2/10, abs(t2%10),
             t3/10, abs(t3%10),

             err[0],
             err[1],
             err[2],
             err[3],
             err[4],
             err[5]
    );
    send_packet(packet);
}

/**
  * @brief  The application entry point.
  */

int main(void)
{
    HAL_Init();

    SystemClock_Config();

    MX_GPIO_Init();
    MX_I2C1_Init();
    MX_I2S3_Init();
    MX_SPI1_Init();
    MX_USB_DEVICE_Init();

    HAL_Delay(2000);

    while (1)
    {
        FakeData d = fake_data[data_index];

        uint8_t flags = 0;

        uint32_t current_time = HAL_GetTick();

        flags |= analyze_hr(d.hr);
        flags |= analyze_spo2(d.spo2);
        flags |= analyze_temp(d.temp_c);

        analyze_motion(
            current_time,
            d.ax,
            d.ay,
            d.az,
            d.gx,
            d.gy,
            d.gz
        );

        if(fall_detected)
        {
            flags |= ALERT_FALL;
            fall_detected = 0;
        }

        send_patient_packet(
            1,
            d.hr,
            d.spo2,
            d.temp_c,
            flags
        );

        HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);

        data_index++;

        if(data_index >= FAKE_DATA_COUNT)
        {
            data_index = 0;
        }

        HAL_Delay(1000);
    }
}

/**
  * @brief System Clock Configuration
  */

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

