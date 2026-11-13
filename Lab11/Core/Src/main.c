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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <math.h>
#include <stdint.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
    float ax, ay, az;
    float offset_x, offset_y, offset_z;
    float roll, pitch;
} LSM_Data;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// Accelerometer definitions, retained from Lab 9
#define LSM303_ADDR (0x19U << 1)
#define WHO_AM_I_REG 0x0FU
#define CTRL_REG1_A 0x20U
#define CTRL_REG4_A 0x23U
#define OUT_X_L_A 0x28U
#define ACC_G_PER_LSB 0.004f
#define RAD_TO_DEG 57.2957795f

// Gyroscope definitions, retained from Lab 8
#define GYRO_CTRL_REG1 0x20U
#define GYRO_OUT_X_L 0x28U
#define GYRO_READ_BIT 0x80U
#define GYRO_SENSITIVITY 0.00875f
#define LOOP_PERIOD_MS 10U
#define FILTER_ALPHA 0.98f

// ===== PD CONTROLLER =====
#define KP 30.0f
#define KD 0.5f

#define MAX_PWM 350
#define MIN_PWM 0

#define ANGLE_DEADBAND 0.5f
#define SAFETY_ANGLE 25.0f
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
static LSM_Data sensor = {0};
static float x_dps = 0.0f, y_dps = 0.0f, z_dps = 0.0f;
static float gyro_x_bias = 0.0f;
static float gyro_angle = 0.0f;
static float filtered_angle = 0.0f;
static float acc_angle = 0.0f;
static char uart_buf[128];
static float gyro_y_bias = 0.0f;
static float target_angle = 0.0f;
static int16_t motor_pwm = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

  uint8_t gyro_read_register(uint8_t reg)
{
    uint8_t tx[2] = {
        (uint8_t)(reg | 0x80),
        0x00
    };

    uint8_t rx[2] = {0};

    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);

    HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(
        &hspi1,
        tx,
        rx,
        2,
        100
    );

    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);

    if (status != HAL_OK)
    {
        return 0x00;
    }

    return rx[1];
}
  static HAL_StatusTypeDef Init_LSM(void)
  {
      uint8_t config = 0x67;  // 200 Hz, all axes enabled, normal mode
      if (HAL_I2C_Mem_Write(&hi2c1, LSM303_ADDR, CTRL_REG1_A,
                            I2C_MEMADD_SIZE_8BIT, &config, 1, 100) != HAL_OK)
          return HAL_ERROR;

      config = 0x00;         // +/-2g, normal resolution
      if (HAL_I2C_Mem_Write(&hi2c1, LSM303_ADDR, CTRL_REG4_A,
                            I2C_MEMADD_SIZE_8BIT, &config, 1, 100) != HAL_OK)
          return HAL_ERROR;

      HAL_Delay(50);
      return HAL_OK;
  }
          

  static HAL_StatusTypeDef Read_LSM(LSM_Data *s)
  {
      uint8_t data[6];
      // Bit 7 requests sequential register-address increment.
      if (HAL_I2C_Mem_Read(&hi2c1, LSM303_ADDR, OUT_X_L_A | 0x80U,
                          I2C_MEMADD_SIZE_8BIT, data, 6, 100) != HAL_OK)
          return HAL_ERROR;

      int16_t raw_x = (int16_t)(((uint16_t)data[1] << 8) | data[0]);
      int16_t raw_y = (int16_t)(((uint16_t)data[3] << 8) | data[2]);
      int16_t raw_z = (int16_t)(((uint16_t)data[5] << 8) | data[4]);

      // LSM303DLHC normal mode: signed 10-bit values left-aligned in 16 bits.
      // Integer division by 64 is used to avoid implementation-specific right
      // shifting of negative signed integers.
      s->ax = (raw_x / 64) * ACC_G_PER_LSB - s->offset_x;
      s->ay = (raw_y / 64) * ACC_G_PER_LSB - s->offset_y;
      s->az = (raw_z / 64) * ACC_G_PER_LSB - s->offset_z;

      s->roll = atan2f(s->ay, sqrtf(s->ax*s->ax + s->az*s->az)) * RAD_TO_DEG;
      s->pitch = atan2f(-s->ax, sqrtf(s->ay*s->ay + s->az*s->az)) * RAD_TO_DEG;
      return HAL_OK;
  }

  static HAL_StatusTypeDef Offset_LSM(LSM_Data *s)
  {
      float sx = 0, sy = 0, sz = 0;
      const int samples = 20;
      s->offset_x = s->offset_y = s->offset_z = 0.0f;

      for (int i = 0; i < samples; ++i)
      {
          if (Read_LSM(s) != HAL_OK)
              return HAL_ERROR;
          sx += s->ax;
          sy += s->ay;
          sz += s->az;
          HAL_Delay(20);
      }
      // Calibrate only while the board is flat and motionless.
      s->offset_x = sx / samples;
      s->offset_y = sy / samples;
      float avg_z = sz / samples;
      s->offset_z = avg_z - ((avg_z >= 0.0f) ? 1.0f : -1.0f);
      return HAL_OK;
  }

  

// Lab 8 SPI gyroscope functions, adapted for compact 6-byte reads.
static HAL_StatusTypeDef gyro_write_register(uint8_t reg, uint8_t value)
{
    uint8_t tx[2] = {reg, value};
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);
    HAL_StatusTypeDef rc = HAL_SPI_Transmit(&hspi1, tx, 2, 100);
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);
    return rc;
}

static HAL_StatusTypeDef gyro_read_xyz(void)
{
    // Read X-axis
    uint8_t x_low  = gyro_read_register(0x28);
    uint8_t x_high = gyro_read_register(0x29);

    // Read Y-axis
    uint8_t y_low  = gyro_read_register(0x2A);
    uint8_t y_high = gyro_read_register(0x2B);

    // Read Z-axis
    uint8_t z_low  = gyro_read_register(0x2C);
    uint8_t z_high = gyro_read_register(0x2D);

    // Combine bytes
    int16_t raw_x = (int16_t)(
        ((uint16_t)x_high << 8) | x_low
    );

    int16_t raw_y = (int16_t)(
        ((uint16_t)y_high << 8) | y_low
    );

    int16_t raw_z = (int16_t)(
        ((uint16_t)z_high << 8) | z_low
    );

    // Convert to degrees per second
    x_dps = raw_x * GYRO_SENSITIVITY;
    y_dps = raw_y * GYRO_SENSITIVITY;
    z_dps = raw_z * GYRO_SENSITIVITY;

    return HAL_OK;
}

static HAL_StatusTypeDef calibrate_gyro_x(void)
{
    float sum = 0.0f;
    for (unsigned i = 0; i < 100; ++i) {
        if (gyro_read_xyz() != HAL_OK) return HAL_ERROR;
        sum += x_dps;
        HAL_Delay(10);
    }
    gyro_x_bias = sum / 100.0f;
    return HAL_OK;
}

static HAL_StatusTypeDef calibrate_gyro_y(void)
{
    float sum = 0.0f;

    for (unsigned i = 0; i < 100; i++)
    {
        if (gyro_read_xyz() != HAL_OK)
        {
            return HAL_ERROR;
        }

        sum += y_dps;

        HAL_Delay(10);
    }

    gyro_y_bias = sum / 100.0f;

    return HAL_OK;
}


// static void print_angles(void)
// {
//     int n = snprintf(
//         uart_buf,
//         sizeof(uart_buf),
//         "%.2f,%.2f,%.2f\r\n",
//         (double)acc_angle,
//         (double)gyro_angle,
//         (double)filtered_angle
//     );

//     if (n > 0 && n < (int)sizeof(uart_buf))
//     {
//         HAL_UART_Transmit(
//             &huart1,
//             (uint8_t *)uart_buf,
//             (uint16_t)n,
//             100
//         );
//     }
// }

static void print_angles(void)
{
    float gy = -(y_dps - gyro_y_bias);

    int n = snprintf(
    uart_buf,
    sizeof(uart_buf),
    "Target: %.2f | Angle: %.2f | Gyro: %.2f | PWM: %d\r\n",
    (double)target_angle,
    (double)filtered_angle,
    (double)gy,
    (int)motor_pwm
);

    if (n > 0 && n < (int)sizeof(uart_buf))
    {
        HAL_UART_Transmit(
            &huart1,
            (uint8_t *)uart_buf,
            (uint16_t)n,
            100
        );
    }
}

void Motor_Control(int16_t pwm)
{
    // Limit PWM
    if (pwm > MAX_PWM)
        pwm = MAX_PWM;

    if (pwm < -MAX_PWM)
        pwm = -MAX_PWM;

    // Stop motors
    if (pwm == 0)
    {
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 0);

        HAL_GPIO_WritePin(GPIOB,
                          GPIO_PIN_0 | GPIO_PIN_1 |
                          GPIO_PIN_10 | GPIO_PIN_11,
                          GPIO_PIN_RESET);
        return;
    }

    // Forward
    if (pwm > 0)
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);

        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET);
    }
    // Backward
    else
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);

        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET);

        pwm = -pwm;
    }

    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, pwm);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, pwm);
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
  MX_SPI1_Init();
  MX_TIM2_Init();
  MX_USB_DEVICE_Init();
  MX_USART1_UART_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
HAL_Delay(100);

// Check accelerometer
uint8_t id = 0;

if (HAL_I2C_Mem_Read(
        &hi2c1,
        LSM303_ADDR,
        WHO_AM_I_REG,
        I2C_MEMADD_SIZE_8BIT,
        &id,
        1,
        100
    ) != HAL_OK || id != 0x33)
{
    Error_Handler();
}

// Initialize accelerometer
if (Init_LSM() != HAL_OK)
{
    Error_Handler();
}

// Initialize gyroscope
if (gyro_write_register(GYRO_CTRL_REG1, 0x8F) != HAL_OK)
{
    Error_Handler();
}

HAL_Delay(100);

// Temporary: skip flat-board accelerometer offset calibration
sensor.offset_x = 0.0f;
sensor.offset_y = 0.0f;
sensor.offset_z = 0.0f;

if (calibrate_gyro_y() != HAL_OK)
{
    Error_Handler();
}

// Initial angle
if (Read_LSM(&sensor) != HAL_OK)
{
    Error_Handler();
}

acc_angle = sensor.pitch;

filtered_angle = acc_angle;
gyro_angle = acc_angle;

// ===== AUTOMATIC UPRIGHT REFERENCE =====
// Robot must be upright and stationary during startup.

// ===== AUTOMATIC TARGET ANGLE CALIBRATION =====
float angle_sum = 0.0f;

for (int i = 0; i < 100; i++)
{
    if (Read_LSM(&sensor) != HAL_OK)
    {
        Error_Handler();
    }

    angle_sum += sensor.pitch;

    HAL_Delay(10);
}

target_angle = angle_sum / 100.0f;

// Initialize filter at the reference angle
acc_angle = target_angle;
filtered_angle = target_angle;
gyro_angle = target_angle;

// Sampling variables
uint32_t last_sample_ms = HAL_GetTick();
uint32_t next_sample_ms = last_sample_ms + LOOP_PERIOD_MS;

unsigned print_counter = 0;

HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);

motor_pwm = 0;
Motor_Control(0);

/* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    uint32_t now = HAL_GetTick();

    if ((int32_t)(now - next_sample_ms) >= 0)
    {
        // Calculate elapsed time
        float dt = (now - last_sample_ms) * 0.001f;

        last_sample_ms = now;

        next_sample_ms += LOOP_PERIOD_MS;

        if ((int32_t)(now - next_sample_ms) >= 0)
        {
            next_sample_ms = now + LOOP_PERIOD_MS;
        }

        // Read accelerometer and gyroscope
        if (Read_LSM(&sensor) != HAL_OK ||
            gyro_read_xyz() != HAL_OK)
        {
            Error_Handler();
        }

        // Accelerometer tilt angle
        acc_angle = sensor.pitch;

        // Remove gyroscope bias
        float gy = -(y_dps - gyro_y_bias);
        gyro_angle += gy * dt;

        filtered_angle = FILTER_ALPHA * (filtered_angle + gy * dt)+ (1.0f - FILTER_ALPHA) * acc_angle;

        // Print every 10 samples
        if (++print_counter >= 10U)
        {
            print_counter = 0;
            print_angles();
        }

// ===== PD BALANCING CONTROLLER =====

float error = target_angle - filtered_angle;
float control = KP * error - KD * gy;

if (fabsf(error) > SAFETY_ANGLE ||
    fabsf(error) < ANGLE_DEADBAND)
{
    motor_pwm = 0;
    Motor_Control(0);
}
else
{
    int16_t pwm = (int16_t)control;

    // Limit maximum PWM
    if (pwm > MAX_PWM)  pwm = MAX_PWM;
    if (pwm < -MAX_PWM) pwm = -MAX_PWM;

    // Minimum effective PWM
    if (pwm > 0 && pwm < 300)
        pwm = 300;
    else if (pwm < 0 && pwm > -300)
        pwm = -300;

    // Keep the sign that passed your direction test
    //motor_pwm = -pwm;
Motor_Control(pwm);
}

// ===== MOTOR PWM TEST =====

// Motor_Control(300);
// HAL_Delay(1000);

// Motor_Control(0);
// HAL_Delay(2000);
  }
    // if (gyro_read_xyz() == HAL_OK)
    // {
    //     int len = snprintf(
    //         uart_buf,
    //         sizeof(uart_buf),
    //         "%.2f,%.2f,%.2f\r\n",
    //         (double)x_dps,
    //         (double)y_dps,
    //         (double)z_dps
    //     );

    //     if (len > 0 && len < sizeof(uart_buf))
    //     {
    //         HAL_UART_Transmit(
    //             &huart1,
    //             (uint8_t *)uart_buf,
    //             (uint16_t)len,
    //             100
    //         );
    //     }
    // }
  
    // // Toggle LD3 (PE9)
    // HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_9);

    // // UART test
    // const char msg[] = "HELLO STM32\r\n";

    // HAL_UART_Transmit(
    //     &huart1,
    //     (uint8_t *)msg,
    //     sizeof(msg) - 1,
    //     100
    // );

    // HAL_Delay(500);

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB|RCC_PERIPHCLK_USART1
                              |RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  PeriphClkInit.USBClockSelection = RCC_USBCLKSOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
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
  hi2c1.Init.Timing = 0x00201D2B;
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
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 47;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 9999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
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
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 47;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 999;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

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
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GYRO_CS_GPIO_Port, GYRO_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_10|GPIO_PIN_11, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, LD4_Pin|LD3_Pin|LD5_Pin|LD7_Pin
                          |LD9_Pin|LD10_Pin|LD8_Pin|LD6_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : DRDY_Pin MEMS_INT3_Pin MEMS_INT4_Pin MEMS_INT1_Pin
                           MEMS_INT2_Pin */
  GPIO_InitStruct.Pin = DRDY_Pin|MEMS_INT3_Pin|MEMS_INT4_Pin|MEMS_INT1_Pin
                          |MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : GYRO_CS_Pin LD4_Pin LD3_Pin LD5_Pin
                           LD7_Pin LD9_Pin LD10_Pin LD8_Pin
                           LD6_Pin */
  GPIO_InitStruct.Pin = GYRO_CS_Pin|LD4_Pin|LD3_Pin|LD5_Pin
                          |LD7_Pin|LD9_Pin|LD10_Pin|LD8_Pin
                          |LD6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB10 PB11 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_10|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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

  // Stop both motors
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 0);

  // Set all motor direction pins LOW
  HAL_GPIO_WritePin(GPIOB,
      GPIO_PIN_0 | GPIO_PIN_1 |
      GPIO_PIN_10 | GPIO_PIN_11,
      GPIO_PIN_RESET);

  __disable_irq();

  while (1)
  {
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
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
