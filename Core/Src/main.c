/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "usbd_cdc_if.h"
#include "stm32f3xx_hal.h"

#include <stdio.h>
#include <string.h>
#include <math.h>  // For fabsf and fmaxf
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DEBUG 1

#ifdef DEBUG
#define LED_PIN_UART GPIO_PIN_14
#define LED_PIN_SEND_MODE GPIO_PIN_15
#define LED_PIN_MAGNET GPIO_PIN_13
#define LED_PIN_ACCEL GPIO_PIN_12
#define LED_PIN_GYRO GPIO_PIN_11

#define LED_PORT GPIOC
#endif

#define HEADER 0xAAAB
#define BUFFER_SIZE 64

//#define ENABLE_MAGNETOMETER 1
//#define ENABLE_ACCELEROMETER 1
//#define ENABLE_GYROSCOPE 1

#define MODE_NONE 0
#define MODE_BINARY_UART 1
#define MODE_ASCII_UART 2
#define MODE_BINARY_CDC 3
#define MODE_ASCII_CDC 4

#define RX_BUFFER_SIZE 256

#define APP_RX_DATA_SIZE 2048
#define APP_TX_DATA_SIZE 2048
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
volatile uint8_t data_ready_mag = 0;
volatile uint8_t data_ready_acc = 0;
volatile uint8_t data_ready_gyr = 0;
volatile uint16_t packet_number = 0;

volatile uint8_t transmission_mode = MODE_NONE;

volatile uint8_t rx_buffer[RX_BUFFER_SIZE];
volatile uint8_t shadow_buffer[RX_BUFFER_SIZE];
volatile uint16_t rx_index = 0;
volatile uint8_t rx_data_ready = 0;
volatile uint8_t responseComplete = 0; // 0 = not complete, 1 = complete
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
void Init_All_Sensors(void);
void Pack_Data(uint8_t *binary_buffer, int16_t x, int16_t y, int16_t z);
void Transmit_Data_ASCII(const char *sensor_label, float x, float y, float z);
uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len);
uint8_t i2c1_pisiRegister(uint8_t device, uint8_t reg, uint8_t value);
void i2c1_beriRegistre(uint8_t device, uint8_t reg, uint8_t* data, uint8_t length);
float convertToGauss(int16_t raw_value);
void Verify_Sensors(void);
void ClearInterrupts(void);

#if ENABLE_MAGNETOMETER
void Handle_Magnetometer(void);
#endif
#if ENABLE_ACCELEROMETER
void Handle_Accelerometer(void);
#endif
#if ENABLE_GYROSCOPE
void Handle_Gyroscope(void);
#endif
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == GPIO_PIN_0) { // Button press
        transmission_mode = (transmission_mode + 1) % 5;

#ifdef DEBUG
        HAL_GPIO_TogglePin(GPIOE, LED_PIN_SEND_MODE);
#endif
  	  Send_AT_Command();
    }

    #if ENABLE_MAGNETOMETER
    if (GPIO_Pin == GPIO_PIN_2) { // DRDY for magnetometer
        data_ready_mag = 1;
#ifdef DEBUG
        HAL_GPIO_TogglePin(GPIOE, LED_PIN_MAGNET);
#endif
    }
    #endif
    #if ENABLE_ACCELEROMETER
    if (GPIO_Pin == GPIO_PIN_4) { // INT1 for accelerometer
        data_ready_acc = 1;
#ifdef DEBUG
        HAL_GPIO_TogglePin(GPIOE, LED_PIN_ACCEL);
#endif
    }
    #endif
    #if ENABLE_GYROSCOPE
    if (GPIO_Pin == GPIO_PIN_1) { // INT2 for gyroscope
        data_ready_gyr = 1;
#ifdef DEBUG
        HAL_GPIO_TogglePin(GPIOE, LED_PIN_GYRO);
#endif
    }
    #endif
}

/* Sensor initialization */
void Init_All_Sensors(void) {
    #if ENABLE_MAGNETOMETER
    // Initialize magnetometer
    i2c1_pisiRegister(0x1E, 0x60, 0x8C);  // CFG_REG_A: Enable XYZ, 20 Hz
    i2c1_pisiRegister(0x1E, 0x61, 0x00);  // CFG_REG_B: ±50 gauss
    i2c1_pisiRegister(0x1E, 0x62, 0x01);  // CFG_REG_C: DRDY interrupt
    HAL_Delay(10);
    #endif

    #if ENABLE_ACCELEROMETER
    // Initialize accelerometer
    i2c1_pisiRegister(0x19, 0x20, 0x4F);  // CTRL_REG1: Enable XYZ, 50 Hz
    i2c1_pisiRegister(0x19, 0x23, 0x80);  // CTRL_REG4: ±4g scale
    HAL_Delay(10);
    #endif

    #if ENABLE_GYROSCOPE
    // Initialize gyroscope
    i2c1_pisiRegister(0x6B, 0x20, 0x0F); // CTRL_REG1: Enable XYZ, 200 Hz
    i2c1_pisiRegister(0x6B, 0x23, 0x30); // CTRL_REG4: ±500 dps
    HAL_Delay(10);
    #endif
}

/* Verify Sensor Communication */
void Verify_Sensors(void) {
    uint8_t who_am_i = 0;

    #if ENABLE_MAGNETOMETER
    i2c1_beriRegistre(0x1E, 0x4F, &who_am_i, 1); // Read WHO_AM_I register
    if (who_am_i != 0x6E) {
        CDC_Transmit_FS((uint8_t *)"Magnetometer communication failed\n", 34);
    } else {
        CDC_Transmit_FS((uint8_t *)"Magnetometer initialized\n", 25);
    }
    HAL_Delay(10);
    #endif

    #if ENABLE_ACCELEROMETER
    i2c1_beriRegistre(0x19, 0x0F, &who_am_i, 1); // Read WHO_AM_I register
    if (who_am_i != 0x33) {
        CDC_Transmit_FS((uint8_t *)"Accelerometer communication failed\n", 36);
    } else {
        CDC_Transmit_FS((uint8_t *)"Accelerometer initialized\n", 27);
    }
    HAL_Delay(10);
    #endif

    #if ENABLE_GYROSCOPE
    i2c1_beriRegistre(0x6B, 0x0F, &who_am_i, 1); // Read WHO_AM_I register
    if (who_am_i != 0xD4) {
        CDC_Transmit_FS((uint8_t *)"Gyroscope communication failed\n", 32);
    } else {
        CDC_Transmit_FS((uint8_t *)"Gyroscope initialized\n", 23);
    }
    HAL_Delay(10);
    #endif
}

/* Clear Interrupt Flags */
void ClearInterrupts(void) {
    uint8_t dummy[6];

    #if ENABLE_MAGNETOMETER
    i2c1_beriRegistre(0x1E, 0x68, dummy, 6); // Clear magnetometer DRDY
    #endif

    #if ENABLE_ACCELEROMETER
    i2c1_beriRegistre(0x19, 0x28 | 0x80, dummy, 6); // Clear accelerometer interrupt
    #endif

    #if ENABLE_GYROSCOPE
    i2c1_beriRegistre(0x6B, 0x28 | 0x80, dummy, 6); // Clear gyroscope interrupt
    #endif
}

/* I2C helper functions */
uint8_t i2c1_pisiRegister(uint8_t device, uint8_t reg, uint8_t value) {
    device <<= 1;
    return HAL_I2C_Mem_Write(&hi2c1, device, reg, I2C_MEMADD_SIZE_8BIT, &value, 1, HAL_MAX_DELAY);
}

void i2c1_beriRegistre(uint8_t device, uint8_t reg, uint8_t* data, uint8_t length) {
    if ((length > 1) && (device == 0x1E)) {
        reg |= 0x80; // Auto-increment for magnetometer
    }
    device <<= 1;
    HAL_I2C_Mem_Read(&hi2c1, device, reg, I2C_MEMADD_SIZE_8BIT, data, length, HAL_MAX_DELAY);
}

float convertToGauss(int16_t raw_value) {
    return raw_value * (50.0f / 32768.0f);
}

/* Data packing function */
void Pack_Data(uint8_t *binary_buffer, int16_t x, int16_t y, int16_t z) {
    binary_buffer[0] = HEADER & 0xFF;
    binary_buffer[1] = (HEADER >> 8) & 0xFF;
    binary_buffer[2] = packet_number & 0xFF;
    binary_buffer[3] = (packet_number >> 8) & 0xFF;
    binary_buffer[4] = x & 0xFF;
    binary_buffer[5] = (x >> 8) & 0xFF;
    binary_buffer[6] = y & 0xFF;
    binary_buffer[7] = (y >> 8) & 0xFF;
    binary_buffer[8] = z & 0xFF;
    binary_buffer[9] = (z >> 8) & 0xFF;
}

/* ASCII transmission function */
void Transmit_Data_ASCII(const char *sensor_label, float x, float y, float z) {
    char ascii_buffer[BUFFER_SIZE];
    snprintf(ascii_buffer, BUFFER_SIZE, "{\"%s\":%d,\"X\":%.3f,\"Y\":%.3f,\"Z\":%.3f}\n\r",
             sensor_label, packet_number, x, y, z);
    if (transmission_mode == MODE_ASCII_UART) {
        HAL_UART_Transmit(&huart2, (uint8_t *)ascii_buffer, strlen(ascii_buffer), HAL_MAX_DELAY);
    } else if (transmission_mode == MODE_ASCII_CDC) {
        CDC_Transmit_FS((uint8_t *)ascii_buffer, strlen(ascii_buffer));
    }
}

#if ENABLE_MAGNETOMETER
/* Magnetometer handler */
void Handle_Magnetometer(void) {
    data_ready_mag = 0;

    int16_t raw_data[3];
    i2c1_beriRegistre(0x1E, 0x68, (uint8_t*)raw_data, 6);
    ClearInterrupts(); // Clear interrupt flags

    if (transmission_mode == MODE_BINARY_UART || transmission_mode == MODE_BINARY_CDC) {
        uint8_t binary_buffer[10];
        Pack_Data(binary_buffer, raw_data[0], raw_data[1], raw_data[2]);
        if (transmission_mode == MODE_BINARY_UART) {
            HAL_UART_Transmit(&huart2, binary_buffer, 10, HAL_MAX_DELAY);
        } else {
            CDC_Transmit_FS(binary_buffer, 10);
        }
    } else if (transmission_mode == MODE_ASCII_UART || transmission_mode == MODE_ASCII_CDC) {
        float x = convertToGauss(raw_data[0]);
        float y = convertToGauss(raw_data[1]);
        float z = convertToGauss(raw_data[2]);
        Transmit_Data_ASCII("MAG", x, y, z);
    }
    packet_number++;
}
#endif

#if ENABLE_ACCELEROMETER
/* Accelerometer handler */
void Handle_Accelerometer(void) {
    data_ready_acc = 0;

    int16_t raw_data[3];
    i2c1_beriRegistre(0x19, 0x28 | 0x80, (uint8_t*)raw_data, 6);
    ClearInterrupts(); // Clear interrupt flags

    if (transmission_mode == MODE_BINARY_UART || transmission_mode == MODE_BINARY_CDC) {
        uint8_t binary_buffer[10];
        Pack_Data(binary_buffer, raw_data[0], raw_data[1], raw_data[2]);
        if (transmission_mode == MODE_BINARY_UART) {
            HAL_UART_Transmit(&huart2, binary_buffer, 10, HAL_MAX_DELAY);
        } else {
            CDC_Transmit_FS(binary_buffer, 10);
        }
    } else if (transmission_mode == MODE_ASCII_UART || transmission_mode == MODE_ASCII_CDC) {
        float x = raw_data[0] * (4.0f / 32768.0f);
        float y = raw_data[1] * (4.0f / 32768.0f);
        float z = raw_data[2] * (4.0f / 32768.0f);
        Transmit_Data_ASCII("ACC", x, y, z);
    }
    packet_number++;
}
#endif

#if ENABLE_GYROSCOPE
/* Gyroscope handler */
void Handle_Gyroscope(void) {
    data_ready_gyr = 0;

    int16_t raw_data[3];
    i2c1_beriRegistre(0x6B, 0x28 | 0x80, (uint8_t*)raw_data, 6);
    ClearInterrupts(); // Clear interrupt flags

    if (transmission_mode == MODE_BINARY_UART || transmission_mode == MODE_BINARY_CDC) {
        uint8_t binary_buffer[10];
        Pack_Data(binary_buffer, raw_data[0], raw_data[1], raw_data[2]);
        if (transmission_mode == MODE_BINARY_UART) {
            HAL_UART_Transmit(&huart2, binary_buffer, 10, HAL_MAX_DELAY);
        } else {
            CDC_Transmit_FS(binary_buffer, 10);
        }
    } else if (transmission_mode == MODE_ASCII_UART || transmission_mode == MODE_ASCII_CDC) {
        float x = raw_data[0] * (500.0f / 32768.0f);
        float y = raw_data[1] * (500.0f / 32768.0f);
        float z = raw_data[2] * (500.0f / 32768.0f);
        Transmit_Data_ASCII("GYR", x, y, z);
    }
    packet_number++;
}
#endif

void Clear_RX_Buffer() {
    memset(rx_buffer, 0, RX_BUFFER_SIZE);
    rx_index = 0;
}

void Send_AT_Command() {
    uint8_t tx_data[] = "AT\r\n";

   HAL_UART_Transmit(&huart2, tx_data, sizeof(tx_data) - 1, HAL_MAX_DELAY);
}

void SendCommand(const char* cmd) {
    // Clear the buffer before sending a new command
    Clear_RX_Buffer();
    // Clear the flag
    responseComplete = 0;

    // Transmit the command
    HAL_UART_Transmit(&huart2, (uint8_t*)cmd, strlen(cmd), HAL_MAX_DELAY);
}

void HandleUartReceive() {
/*#ifdef DEBUG

        char c = rx_buffer[rx_index];

        char dbg[32];
        if (c == '\r') {
            strcpy(dbg, "Got \\r\r\n");
        } else if (c == '\n') {
            strcpy(dbg, "Got \\n\r\n");
        } else {
            snprintf(dbg, sizeof(dbg), "Got %c\r\n", c);
        }
        CDC_Transmit_FS((uint8_t*)dbg, strlen(dbg));
#endif*/
#ifdef DEBUG
        char dbg[300];
        snprintf(dbg, sizeof(dbg), "Buffer: %s\r\n", rx_buffer);
        CDC_Transmit_FS((uint8_t*)dbg, strlen(dbg));
#endif


        // Move to the next buffer index
        /*rx_index++;
        if (rx_index >= RX_BUFFER_SIZE)
        {
            rx_index = 0; // Wrap around if buffer is full
        }*/

        // Restart UART reception for the next byte
        //HAL_UART_Receive_IT(&huart2, (uint8_t *)&rx_buffer[rx_index], 1);

        if (strstr((char*)rx_buffer, "OK") != NULL ||
            strstr((char*)rx_buffer, "ERROR") != NULL)
        {
            responseComplete = 1;
        }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2) // Check if it's USART2
    {
    	//rx_data_ready = 1;
  	  /*if (rx_data_ready)
  	  {*/
  	      //rx_data_ready = 0;
    	rx_index++;
        if (rx_index >= RX_BUFFER_SIZE) rx_index = 0;

        if (strstr((char *)rx_buffer, "OK") || strstr((char *)rx_buffer, "ERROR")) {
            responseComplete = 1;
        }

    	HAL_UART_Receive_IT(&huart2, (uint8_t *)&rx_buffer[rx_index], 1);
        // Ensure null-termination
        if (rx_index < RX_BUFFER_SIZE) {
            rx_buffer[rx_index] = '\0';
        }
  	      //HandleUartReceive();
  	      //continue;
  	  //}
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
  MX_USB_DEVICE_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  //HAL_Delay(2000);
  //Init_All_Sensors();
  //Verify_Sensors(); // Verify all sensor communication
  //ClearInterrupts(); // Clear interrupt flags

  HAL_UART_Receive_IT(&huart2, (uint8_t *)&rx_buffer[rx_index], 1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  /*if (rx_data_ready)
	  {
	      rx_data_ready = 0;
	      HandleUartReceive();
	      continue;
	  } else */
	  HAL_Delay(50);

      if (responseComplete == 1) {
		HAL_UART_AbortReceive_IT(&huart2);

#ifdef DEBUG
	      uint8_t result0 = CDC_Transmit_FS((uint8_t *)"Command recognized\r\n", 21);
	      if (result0 != USBD_OK)
	      {
	          // Optionally log or handle transmission failure
	          CDC_Transmit_FS((uint8_t*)"CDC Transmission 0 Failed\r\n", 26);
	      }
	      HAL_Delay(10); // Add a delay between transmissions
#endif


#ifdef DEBUG
	      CDC_Transmit_FS((uint8_t *)"===UART_RECEIVE_START===\r\n", 26);
	      HAL_Delay(10);
	      CDC_Transmit_FS((uint8_t *)rx_buffer, strlen((char *)rx_buffer));
	      HAL_Delay(10);
	      CDC_Transmit_FS((uint8_t *)"===UART_RECEIVE_END===\r\n", 24);

        HAL_Delay(10); // Add a delay between transmissions

	      uint8_t result2 = CDC_Transmit_FS((uint8_t*)"Cleaned Buffer\r\n", 16);
	      if (result2 != USBD_OK)
	      {
	          // Optionally log or handle transmission failure
	          CDC_Transmit_FS((uint8_t*)"CDC Transmission 2 Failed\r\n", 27);
	      }
	      HAL_Delay(10); // Add a delay between transmissions
#endif

	      // Here we can parse rx_buffer if needed
	      // Then clear it after we’re done
        Clear_RX_Buffer();
	      responseComplete = 0;

	      HAL_UART_Receive_IT(&huart2, &rx_buffer[rx_index], 1);
	      continue;
	  }

      if (transmission_mode == MODE_NONE) {
          continue;
      }

      #if ENABLE_MAGNETOMETER
      if (data_ready_mag) {
          Handle_Magnetometer();
      }
      #endif
      #if ENABLE_ACCELEROMETER
      if (data_ready_acc) {
          Handle_Accelerometer();
      }
      #endif
      #if ENABLE_GYROSCOPE
      if (data_ready_gyr) {
          Handle_Gyroscope();
      }
      #endif
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB|RCC_PERIPHCLK_USART2
                              |RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
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
  hi2c1.Init.Timing = 0x0000020B;
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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
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
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
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
  HAL_GPIO_WritePin(GPIOE, CS_I2C_SPI_Pin|LD4_Pin|LD3_Pin|LD5_Pin
                          |LD7_Pin|LD9_Pin|LD10_Pin|LD8_Pin
                          |LD6_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PE2 PE4 PE1 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_4|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : CS_I2C_SPI_Pin */
  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(CS_I2C_SPI_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MEMS_INT4_Pin */
  GPIO_InitStruct.Pin = MEMS_INT4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MEMS_INT4_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin LD3_Pin LD5_Pin LD7_Pin
                           LD9_Pin LD10_Pin LD8_Pin LD6_Pin */
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin|LD5_Pin|LD7_Pin
                          |LD9_Pin|LD10_Pin|LD8_Pin|LD6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI2_TSC_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_TSC_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

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
