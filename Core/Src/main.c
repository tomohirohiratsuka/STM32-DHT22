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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "stdio.h"
#include "stdbool.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
    float temperature;
    float humidity;
    bool allBits[40];
} DHT22_DataTypeDef;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DHT22_GPIO_PORT GPIOA
#define DHT22_GPIO_PIN GPIO_PIN_0
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
void DWT_Init(void) {
    if (!(CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk)) {
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;  // トレース機能を有効にする
        DWT->CYCCNT = 0;                                // サイクルカウンタをリセット
        DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;            // サイクルカウンタを有効にする
    }
}

void debugPrint(UART_HandleTypeDef *huart, char *message) {
    HAL_UART_Transmit(huart, (uint8_t *) message, strlen(message),
                      HAL_MAX_DELAY);
}

void delay_us(uint16_t micros) {
    uint32_t cycles = (SystemCoreClock / 1000000) * micros; // クロックサイクル数を計算
    uint32_t start = DWT->CYCCNT;

    // サイクル数が加算された後の値がオーバーフローするかどうかをチェックし、適切に処理
    while ((DWT->CYCCNT - start) < cycles);
}

void GPIO_SetMode(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, uint32_t Mode,
                  uint32_t Pull, uint32_t Speed) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    GPIO_InitStruct.Pin = GPIO_Pin;
    GPIO_InitStruct.Mode = Mode;
    GPIO_InitStruct.Pull = Pull;
    GPIO_InitStruct.Speed = Speed;

    HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

void send_start_signal(void) {
    // ピンを出力モードに設定
    GPIO_SetMode(DHT22_GPIO_PORT, DHT22_GPIO_PIN, GPIO_MODE_OUTPUT_PP,
                 GPIO_NOPULL, GPIO_SPEED_FREQ_LOW);

    HAL_GPIO_WritePin(DHT22_GPIO_PORT, DHT22_GPIO_PIN, GPIO_PIN_RESET);
    delay_us(1000); // 1 ms

    HAL_GPIO_WritePin(DHT22_GPIO_PORT, DHT22_GPIO_PIN, GPIO_PIN_SET);
    delay_us(30); // 20-40 us

    // ピンを入力モードに設定
    GPIO_SetMode(
            DHT22_GPIO_PORT,
            DHT22_GPIO_PIN,
            GPIO_MODE_INPUT,
            GPIO_NOPULL,
            GPIO_SPEED_FREQ_LOW);

    // wait while pin high
    while (HAL_GPIO_ReadPin(DHT22_GPIO_PORT, DHT22_GPIO_PIN) == GPIO_PIN_SET);
    //wait while pin low
    while (HAL_GPIO_ReadPin(DHT22_GPIO_PORT, DHT22_GPIO_PIN) == GPIO_PIN_RESET);
    //wait while pin high
    while (HAL_GPIO_ReadPin(DHT22_GPIO_PORT, DHT22_GPIO_PIN) == GPIO_PIN_SET);
}

// Utility function to convert bit array to byte
static uint8_t bitsToByte(bool bits[8]) {
    uint8_t byte = 0;
    for (int i = 0; i < 8; i++) {
        if (bits[i]) {
            byte |= (1 << (7 - i));
        }
    }
    return byte;
}

// Calculate the actual values and perform checksum validation
static void decodeData(DHT22_DataTypeDef *result, bool bits[40]) {
    bool rh_integral_bits[8], rh_decimal_bits[8], t_integral_bits[8], t_decimal_bits[8], checksum_bits[8];
    memcpy(rh_integral_bits, &bits[0], 8);
    memcpy(rh_decimal_bits, &bits[8], 8);
    memcpy(t_integral_bits, &bits[16], 8);
    memcpy(t_decimal_bits, &bits[24], 8);
    memcpy(checksum_bits, &bits[32], 8);

    uint8_t rh_integral = bitsToByte(rh_integral_bits);
    uint8_t rh_decimal = bitsToByte(rh_decimal_bits);
    uint8_t t_integral = bitsToByte(t_integral_bits);
    uint8_t t_decimal = bitsToByte(t_decimal_bits);
    uint8_t checksum_received = bitsToByte(checksum_bits);

    uint8_t checksum_calculated = rh_integral + rh_decimal + t_integral + t_decimal;

    // Calculate humidity and temperature correctly
    result->humidity = (rh_integral * 256 + rh_decimal) / 10.0f;
    float temp = (t_integral * 256 + t_decimal) / 10.0f;
    result->temperature = (bits[16] ? -temp : temp); // Adjust sign based on the highest bit of the temperature data

    // Checksum validation
    if (checksum_received != checksum_calculated) {
        debugPrint(&huart2, "Checksum error!\r\n");
    }
}


// Main data reading function
DHT22_DataTypeDef DHT22_ReadData(void) {
    bool allBits[40];
    DHT22_DataTypeDef result = {0.0f, 0.0f, {false}};

    send_start_signal();

    for (int i = 0; i < 40; i++) {
        // Wait for signal LOW period
        while (HAL_GPIO_ReadPin(DHT22_GPIO_PORT, DHT22_GPIO_PIN) == GPIO_PIN_RESET);

        // Measure the length of the HIGH period
        uint32_t start = DWT->CYCCNT;
        while (HAL_GPIO_ReadPin(DHT22_GPIO_PORT, DHT22_GPIO_PIN) == GPIO_PIN_SET);
        uint32_t cycles = DWT->CYCCNT - start;

        float spentUs = cycles * (1000000.0f / SystemCoreClock);
        allBits[i] = (spentUs > 40); // Determine if the bit is '0' or '1'
    }

    decodeData(&result, allBits);
    memcpy(result.allBits, allBits, sizeof(allBits));

    return result;
}

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

static void MX_GPIO_Init(void);

static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

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

    /* USER CODE END 1 */

    /* MCU Configuration--------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* USER CODE BEGIN Init */

    /* USER CODE END Init */

    /* Configure the system clock */
    SystemClock_Config();

    /* USER CODE BEGIN SysInit */
    DWT_Init();

    /* USER CODE END SysInit */

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_USART2_UART_Init();
    /* USER CODE BEGIN 2 */

    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1) {
        // センサーからデータを読み取る
        DHT22_DataTypeDef sensor_data;
        sensor_data = DHT22_ReadData();

        // データを文字列にフォーマット
//		char bitsString[41]; // 40 bits + null terminator
//		for (int i = 0; i < 40; i++) {
//		    bitsString[i] = '0' + sensor_data.allBits[i]; // '0' または '1' の文字に変換
//		}
//		bitsString[40] = '\0'; // 文字列の終端

        // 変換された文字列を出力
//		debugPrint(&huart2, "All Bits: ");
//		debugPrint(&huart2, bitsString);
//		debugPrint(&huart2, "\r\n");

        char buffer[100];
        sprintf(buffer, "Temperature: %.2f C, Humidity: %.2f %%\r\n",
                sensor_data.temperature, sensor_data.humidity);
        debugPrint(&huart2, buffer);

        debugPrint(&huart2, "Continue...\r\n");
        // 次のデータ読み取りまでの間隔
        HAL_Delay(4000);

    }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

    /** Initializes the RCC Oscillators according to the specified parameters
    * in the RCC_OscInitTypeDef structure.
    */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM = 16;
    RCC_OscInitStruct.PLL.PLLN = 336;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
    RCC_OscInitStruct.PLL.PLLQ = 2;
    RCC_OscInitStruct.PLL.PLLR = 2;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
    */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                  | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
        Error_Handler();
    }
}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void) {

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
    if (HAL_UART_Init(&huart2) != HAL_OK) {
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
static void MX_GPIO_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pin : B1_Pin */
    GPIO_InitStruct.Pin = B1_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pin : DHT22_Input_Pin */
    GPIO_InitStruct.Pin = DHT22_Input_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(DHT22_Input_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pin : LD2_Pin */
    GPIO_InitStruct.Pin = LD2_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
