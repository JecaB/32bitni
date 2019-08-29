/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32l0xx_hal.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
#define THUMB_CS	GPIO_PIN_4
#define RF_CS			GPIO_PIN_12

#define MAX_NOOP        0x00    /**< No operation adresa. */
#define MAX_DIGIT0      0x01    /**< Adresa cifre 0 ili reda 0. */
#define MAX_DIGIT1      0x02    /**< Adresa cifre 1 ili reda 1. */
#define MAX_DIGIT2      0x03    /**< Adresa cifre 2 ili reda 2. */
#define MAX_DIGIT3      0x04    /**< Adresa cifre 3 ili reda 3. */
#define MAX_DIGIT4      0x05    /**< Adresa cifre 4 ili reda 4. */
#define MAX_DIGIT5      0x06    /**< Adresa cifre 5 ili reda 5. */
#define MAX_DIGIT6      0x07    /**< Adresa cifre 6 ili reda 6. */
#define MAX_DIGIT7      0x08    /**< Adresa cifre 7 ili reda 7. */
#define MAX_DECODEMODE  0x09    /**< Adresa registra za izbor moda dekodiranja. Mi koristimo NO DECODE MODE(0x00). */
#define MAX_INTENSITY   0x0A    /**< Adresa registra za kontrolu intenziteta osvetljaja dioda(od 0x00 do 0x0F). */
#define MAX_SCANLIMIT   0x0B    /**< Adresa registra za kontrolu broja prikazanih cifara. Mi prikazujemo sve cifre(0x07), odnosno sve clanove niza. */
#define MAX_SHUTDOWN    0x0C    /**< Adresa registra koji ukljucuje/iskljucuje kontroler. */
#define MAX_DISPLAYTEST 0x0F    /**< Adresa registra za izbor normalnog moda rada(0x00) ili display test moda(0x01). */

const uint8_t ucNumber[50][8] = {
	{0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xC0, 0xC0},	// Char 000 (.)
	{0x00, 0x00, 0x00, 0x00, 0x00, 0xC0, 0xC0, 0x00},	// Char 001 (.)
	{0x00, 0x00, 0x00, 0x00, 0xC0, 0xC0, 0x00, 0x00},	// Char 002 (.)
	{0x00, 0x00, 0x00, 0xC0, 0xC0, 0x00, 0x00, 0x00},	// Char 003 (.)
	{0x00, 0x00, 0xC0, 0xC0, 0x00, 0x00, 0x00, 0x00},	// Char 004 (.)
	{0x00, 0xC0, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00},	// Char 005 (.)
	{0xC0, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},	// Char 006 (.)
	{0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x60, 0x60},	// Char 007 (.)
	{0x00, 0x00, 0x00, 0x00, 0x00, 0x60, 0x60, 0x00},	// Char 008 (.)
	{0x00, 0x00, 0x00, 0x00, 0x60, 0x60, 0x00, 0x00},	// Char 009 (.)
	{0x00, 0x00, 0x00, 0x60, 0x60, 0x00, 0x00, 0x00},	// Char 010 (.)
	{0x00, 0x00, 0x60, 0x60, 0x00, 0x00, 0x00, 0x00},	// Char 011 (.)
	{0x00, 0x60, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00},	// Char 012 (.)
	{0x60, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},	// Char 013 (.)
	{0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x30, 0x30},	// Char 014 (.)
	{0x00, 0x00, 0x00, 0x00, 0x00, 0x30, 0x30, 0x00},	// Char 015 (.)
	{0x00, 0x00, 0x00, 0x00, 0x30, 0x30, 0x00, 0x00},	// Char 016 (.)
	{0x00, 0x00, 0x00, 0x30, 0x30, 0x00, 0x00, 0x00},	// Char 017 (.)
	{0x00, 0x00, 0x30, 0x30, 0x00, 0x00, 0x00, 0x00},	// Char 018 (.)
	{0x00, 0x30, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00},	// Char 019 (.)
	{0x30, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},	// Char 020 (.)
	{0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x18, 0x18},	// Char 021 (.)
	{0x00, 0x00, 0x00, 0x00, 0x00, 0x18, 0x18, 0x00},	// Char 022 (.)
	{0x00, 0x00, 0x00, 0x00, 0x18, 0x18, 0x00, 0x00},	// Char 023 (.)
	{0x00, 0x00, 0x00, 0x18, 0x18, 0x00, 0x00, 0x00},	// Char 024 (.)
	{0x00, 0x00, 0x18, 0x18, 0x00, 0x00, 0x00, 0x00},	// Char 025 (.)
	{0x00, 0x18, 0x18, 0x00, 0x00, 0x00, 0x00, 0x00},	// Char 026 (.)
	{0x18, 0x18, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},	// Char 027 (.)
	{0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0C, 0x0C},	// Char 028 (.)
	{0x00, 0x00, 0x00, 0x00, 0x00, 0x0C, 0x0C, 0x00},	// Char 029 (.)
	{0x00, 0x00, 0x00, 0x00, 0x0C, 0x0C, 0x00, 0x00},	// Char 030 (.)
	{0x00, 0x00, 0x00, 0x0C, 0x0C, 0x00, 0x00, 0x00},	// Char 031 (.)
	{0x00, 0x00, 0x0C, 0x0C, 0x00, 0x00, 0x00, 0x00},	// Char 032 ( )
	{0x00, 0x0C, 0x0C, 0x00, 0x00, 0x00, 0x00, 0x00},	// Char 033 (!)
	{0x0C, 0x0C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},	// Char 034 (")
	{0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x06},	// Char 035 (#)
	{0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x06, 0x00},	// Char 036 ($)
	{0x00, 0x00, 0x00, 0x00, 0x06, 0x06, 0x00, 0x00},	// Char 037 (%)
	{0x00, 0x00, 0x00, 0x06, 0x06, 0x00, 0x00, 0x00},	// Char 038 (&)
	{0x00, 0x00, 0x06, 0x06, 0x00, 0x00, 0x00, 0x00},	// Char 039 (')
	{0x00, 0x06, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00},	// Char 040 (()
	{0x06, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},	// Char 041 ())
	{0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x03},	// Char 042 (*)
	{0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x03, 0x00},	// Char 043 (+)
	{0x00, 0x00, 0x00, 0x00, 0x03, 0x03, 0x00, 0x00},	// Char 044 (,)
	{0x00, 0x00, 0x00, 0x03, 0x03, 0x00, 0x00, 0x00},	// Char 045 (-)
	{0x00, 0x00, 0x03, 0x03, 0x00, 0x00, 0x00, 0x00},	// Char 046 (.)
	{0x00, 0x03, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00},	// Char 047 (/)
	{0x03, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},	// Char 048 (0)
	{0xFF, 0xC3, 0xA5, 0x99, 0x99, 0xA5, 0xC3, 0xFF},
};

volatile uint8_t int_flag = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI2_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
uint32_t GetADC(uint8_t channel) {    // Returns 0..4095
  uint8_t tmp[2];
	uint32_t result;
	HAL_GPIO_WritePin(GPIOA, THUMB_CS, GPIO_PIN_RESET);

	uint8_t read_address = 0x06;
	channel = channel << 6;
	HAL_StatusTypeDef write_status = HAL_SPI_Transmit(&hspi1, &read_address, 1, 500);

	HAL_StatusTypeDef read_status1 = HAL_SPI_TransmitReceive(&hspi1, &channel, &tmp[0], 1, 500);
	HAL_StatusTypeDef read_status2 = HAL_SPI_TransmitReceive(&hspi1, 0, &tmp[1], 1, 500);
                                                 // and form 12-bit ADC value

	result = (uint32_t)tmp[0] & 0x0000000f;
	result= (((uint32_t)result)<<8) | (uint32_t)tmp [1]; 
	HAL_GPIO_WritePin(GPIOA, THUMB_CS, GPIO_PIN_SET);                              // Deselect MCP3204
  return result;                                    // Returns 12-bit ADC value
}

void sendPacket(uint8_t reg, uint8_t data)
{
	// CS
	HAL_GPIO_WritePin(GPIOB, RF_CS, GPIO_PIN_RESET);

	//uint16_t packet = (reg << 8) | data;
	uint8_t packet[2];
	packet[0] = reg;
	packet[1] = data;

	HAL_SPI_Transmit(&hspi2, (uint8_t*)&packet, 2, 100);

	// CS
	HAL_GPIO_WritePin(GPIOB, RF_CS, GPIO_PIN_SET);
}

void matrix_init()
{
	
    // Initialise MAX7219 with 8x8 led matrix
    sendPacket(MAX_NOOP, 0x00);           // NO OP (seems needed after power on)
    sendPacket(MAX_SCANLIMIT, 0x07);      // Enable all digits (always needed for current/8 per row)
    sendPacket(MAX_INTENSITY, 0x07);      // Display intensity (0x00 to 0x0F)
    sendPacket(MAX_DECODEMODE, 0);        // No BCD decoding for led matrix

    // Clear all rows/digits
    sendPacket(MAX_DIGIT0, 0);
    sendPacket(MAX_DIGIT1, 0);
    sendPacket(MAX_DIGIT2, 0);
    sendPacket(MAX_DIGIT3, 0);
    sendPacket(MAX_DIGIT4, 0);
    sendPacket(MAX_DIGIT5, 0);
    sendPacket(MAX_DIGIT6, 0);
    sendPacket(MAX_DIGIT7, 0);
    sendPacket(MAX_SHUTDOWN, 1); // Wake oscillators/display up
}

void DisplayDigit(uint8_t ucDigit){

    uint8_t ucRow;
    for(ucRow=0;ucRow<8;ucRow++)
        sendPacket(MAX_DIGIT0+ucRow, ucNumber[ucDigit][ucRow]);

}
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{
uint32_t	for_rev,left_right;
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_SPI1_Init();
  MX_SPI2_Init();

  /* USER CODE BEGIN 2 */
	HAL_GPIO_WritePin(GPIOB, RF_CS, GPIO_PIN_RESET);
	matrix_init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
	for_rev = GetADC(0);
	left_right = GetADC(1);
		
	uint8_t row=(for_rev / 551)*7 + left_right / 551;

	if(int_flag==1) row = 49;
	DisplayDigit(row);

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_5;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* SPI1 init function */
static void MX_SPI1_Init(void)
{

  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* SPI2 init function */
static void MX_SPI2_Init(void)
{

  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
     PA2   ------> USART2_TX
     PA3   ------> USART2_RX
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, SPI1_CS_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Thumb_exti_Pin */
  GPIO_InitStruct.Pin = Thumb_exti_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(Thumb_exti_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : USART_TX_Pin USART_RX_Pin */
  GPIO_InitStruct.Pin = USART_TX_Pin|USART_RX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF4_USART2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : SPI1_CS_Pin LD2_Pin */
  GPIO_InitStruct.Pin = SPI1_CS_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI2_CS_Pin */
  GPIO_InitStruct.Pin = SPI2_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SPI2_CS_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
