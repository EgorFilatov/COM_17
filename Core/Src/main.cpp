#include "main.h"

#include <SpiDevice.h>
#include <Timer.h>

#define READY 0
#define BUSY 1
#define UART_SPEED 				115200	//Скорость UART
#define UART_TX_PERIOD_MS 		5000	//Период выдачи по UART (мс)
#define UART_TX_MIN_PERIOD_MS 	3		//Минимальный период выдачи по UART (мс)
#define UART_TX_AFTER_EVENT_MS	40000	//Время выдачи по UART после события ТС (мс)

SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_rx;
DMA_HandleTypeDef hdma_spi1_tx;

UART_HandleTypeDef huart5;
DMA_HandleTypeDef hdma_uart5_rx;
DMA_HandleTypeDef hdma_uart5_tx;

uint8_t a { 0 };
uint8_t b[6] { 0 };
uint8_t c { 0 };
uint8_t d { 0 };

uint8_t currentBuffIndex { 0 };
uint8_t numOfSpiDevices { 0 };

uint8_t spiTxRxState { READY };
uint8_t uartRxState { READY };
uint8_t uartTxState { READY };

Timer uartTxMinPeriodTim(UART_TX_MIN_PERIOD_MS);
Timer uartTxPeriodTim(UART_TX_PERIOD_MS);
Timer uartTxAfterEventTim(UART_TX_AFTER_EVENT_MS);

/* Буфер uart:
 0-1: 	Стартовые байты(0x55,0xAA)
 2: 	Размер массива(0x05)
 3: 	Адрес платы (0xFF- команда на перезагрузку)
 4-7: 	Данные от платы
 8-9:	Контрольная сумма */

uint8_t uartRxBuff_0[10] { 0 };
uint8_t uartRxBuff_1[10] { 0 };
uint8_t *uartRxBuffPtr[2] { uartRxBuff_0, uartRxBuff_1 };
uint8_t uartRxBuffState[2] { READY };
uint8_t currentUartRxBuffIndex { 0 };

uint8_t uartTxBuff_0[10] { 0x55, 0xAA, 0x05, 0, 0, 0, 0, 0, 0, 0 };
uint8_t uartTxBuff_1[10] { 0x55, 0xAA, 0x05, 0, 0, 0, 0, 0, 0, 0 };
uint8_t *uartTxBuffPtr[2] { uartTxBuff_0, uartTxBuff_1 };
uint8_t currentUartTxBuffIndex { 0 };

SpiDevice spiDevice[17];
uint8_t iwdgFlag { 1 };

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_UART5_Init(void);
static void MX_SPI1_Init(void);

void spiDeviceInit() {
	spiDevice[0].setCS(GPIOA, 10);
	spiDevice[1].setCS(GPIOA, 9);
	spiDevice[2].setCS(GPIOA, 8);
	spiDevice[3].setCS(GPIOC, 9);
	spiDevice[4].setCS(GPIOC, 8);
	spiDevice[5].setCS(GPIOC, 7);
	spiDevice[6].setCS(GPIOC, 6);
	spiDevice[7].setCS(GPIOB, 15);
	spiDevice[8].setCS(GPIOB, 14);
	spiDevice[9].setCS(GPIOB, 13);
	spiDevice[10].setCS(GPIOB, 12);
	spiDevice[11].setCS(GPIOA, 3);
	spiDevice[12].setCS(GPIOA, 2);
	spiDevice[13].setCS(GPIOA, 1);
	spiDevice[14].setCS(GPIOC, 3);
	spiDevice[15].setCS(GPIOC, 2);
	spiDevice[16].setCS(GPIOC, 1);
	numOfSpiDevices = SpiDevice::getNumOfDevices();
}

void spiTxRx() {
	if (spiTxRxState != READY ||
		SpiDevice::getTxBuffState(currentBuffIndex) != READY ||
		SpiDevice::getRxBuffState(currentBuffIndex) != READY) return;

	uint8_t currentDeviceIndex = SpiDevice::getCurrentDeviceIndex();
	uint8_t previousDeviceIndex = currentDeviceIndex - 1;
	if (currentDeviceIndex == numOfSpiDevices || currentDeviceIndex == 0) {
		SpiDevice::setCurrentDeviceIndex(0);
		currentDeviceIndex = 0;
		previousDeviceIndex = numOfSpiDevices - 1;
		for (uint8_t i = 0; i < numOfSpiDevices; ++i) {
			spiDevice[i].setTxBuff(spiDevice[i].getTxBuffPtr(currentBuffIndex), currentBuffIndex ^ 1);
		}
		SpiDevice::setTxBuffState(currentBuffIndex, BUSY);
		SpiDevice::setRxBuffState(currentBuffIndex, BUSY);
		currentBuffIndex ^= 1;
		SpiDevice::setTxBuffState(currentBuffIndex, READY);
		SpiDevice::setRxBuffState(currentBuffIndex, READY);
	}
	spiDevice[previousDeviceIndex].deselect();
	spiDevice[currentDeviceIndex].select();
	SpiDevice::increaseCurrentDeviceIndex();
	spiTxRxState = BUSY;
	HAL_SPI_TransmitReceive_IT(&hspi1, spiDevice[currentDeviceIndex].getTxBuffPtr(currentBuffIndex ^ 1),
									   spiDevice[currentDeviceIndex].getRxBuffPtr(currentBuffIndex ^ 1), 6);
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi) {
	spiTxRxState = READY;
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi) {

}

void processSpiRxData() {
	uint8_t buffIndex = currentBuffIndex;
	uint8_t rxBuffState = SpiDevice::getRxBuffState(buffIndex);
	if (rxBuffState != READY) {
		rxBuffState = SpiDevice::getRxBuffState(currentBuffIndex ^ 1);
		if (rxBuffState != READY) {
			return;
		} else {
			buffIndex ^= 1;
		}
	}
	SpiDevice::setRxBuffState(buffIndex, BUSY);
	for (uint8_t i = 0; i < 6; ++i) {
		b[i] = spiDevice[3].getRxBuffPtr(buffIndex)[i];
	}
	for (uint8_t i = 0; i < numOfSpiDevices; ++i) {
		// Проверка контрольной суммы принятых по SPI данных
		if (spiDevice[i].verifyRxChecksum(buffIndex)) {
			uint8_t *buffPtr = spiDevice[i].getRxBuffPtr(buffIndex);
			// Установка типа платы
			spiDevice[i].setType((DeviceType) buffPtr[1]);
			// Проверка на изменения в данных
			spiDevice[i].isRxBuffChanged(buffIndex);
		}
	}
}

void setUartSendNeededOnTimeEvent() {
	if (!(uartTxPeriodTim.isOn())) {
		uartTxPeriodTim.start();
	}
	if (uartTxPeriodTim.isEvent()) {
		uartTxPeriodTim.reset();
		for (uint8_t i = 0; i < numOfSpiDevices; ++i) {
			if (spiDevice[i].getType() != NOT_DEFINED) {
				spiDevice[i].setUartSendNeeded(1);
			}
		}
	}
}

void HAL_IncTick(void)
{
  uwTick += uwTickFreq;
  ++Timer::timerClassCounter;
}

void uartTx() {
	if (uartTxState != READY) return;

	uint8_t i { 0 };
	while (spiDevice[i].isUartSendNeeded() == 0 && i < numOfSpiDevices) {
		++i;
	}
	if (i != numOfSpiDevices) {
		spiDevice[i].setUartSendNeeded(0);
		// Заполнение данных от платы
		for (uint8_t i = 0; i < 4; ++i) {
			uartTxBuffPtr[currentBuffIndex][i + 4] = spiDevice[i].getRxBuffPtr(currentBuffIndex)[i + 2];
		}
		// Заполнение номера порта
		uartTxBuffPtr[currentBuffIndex][3] = i;
		// Подсчет и заполнение контрольной суммы
		uint16_t checksum { 0 };
		for (uint8_t i = 0; i < 8; ++i) {
			checksum += uartTxBuffPtr[currentBuffIndex][i];
		}
		uartTxBuffPtr[currentBuffIndex][8] = (uint8_t) checksum;
		uartTxBuffPtr[currentBuffIndex][9] = (uint8_t) (checksum >> 8);
		uartTxState = BUSY;
		// Передача данных по uart
		HAL_UART_Transmit_DMA(&huart5, uartTxBuffPtr[currentBuffIndex], 10);
	} else {
		SpiDevice::setRxBuffState(currentBuffIndex, READY);
	}
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
	uartTxState = READY;
}

void uartRx() {
	if (uartRxState != READY) return;

	uartRxBuffState[currentUartRxBuffIndex] = BUSY;
	currentUartRxBuffIndex ^= 1;
	uartRxBuffState[currentUartRxBuffIndex] = READY;
	uartRxState = BUSY;
	HAL_UART_Receive_IT(&huart5, uartRxBuffPtr[currentUartRxBuffIndex ^ 1], 10);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	uartRxState = READY;
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart){
	HAL_UART_AbortReceive_IT(&huart5);
	__HAL_UART_CLEAR_PEFLAG(huart);
	__HAL_UART_CLEAR_FEFLAG(huart);
	__HAL_UART_CLEAR_NEFLAG(huart);
	__HAL_UART_CLEAR_OREFLAG(huart);
	uartRxState = READY;
}

void processUartRxBuff() {
	if (uartRxBuffState[currentUartRxBuffIndex] != READY ) return;

	uartRxBuffState[currentUartRxBuffIndex] = BUSY;
	uint16_t checksum { 0 };
	for (uint8_t i = 0; i < 8; ++i) {
		checksum += uartRxBuffPtr[currentUartRxBuffIndex][i];
	}
	uartTxBuffPtr[currentUartRxBuffIndex][8] = (uint8_t) checksum;
	uartTxBuffPtr[currentUartRxBuffIndex][9] = (uint8_t) (checksum >> 8);
	if ((uint8_t) checksum == uartRxBuffPtr[currentUartRxBuffIndex][8] &&
		(uint8_t) (checksum >> 8) == uartRxBuffPtr[currentUartRxBuffIndex][9]) {
		SpiDevice::setTxBuffState(currentBuffIndex, BUSY);
		uint8_t currentDevice = uartRxBuffPtr[currentUartRxBuffIndex][3];
		spiDevice[currentDevice].setTxBuffData(&(uartRxBuffPtr[currentUartRxBuffIndex][4]), currentBuffIndex);
		spiDevice[currentDevice].calculateTxChecksum(currentBuffIndex);
		SpiDevice::setTxBuffState(currentBuffIndex, READY);
	}
}

int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_UART5_Init();
  MX_SPI1_Init();

  spiDeviceInit();

  while (1)
  {
	  spiTxRx();
	  processSpiRxData();
	  //setUartSendNeededOnTimeEvent();
	  uartTx();
	  uartRx();
	  processUartRxBuff();
  }
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 96;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enables the Clock Security System
  */
  HAL_RCC_EnableCSS();
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
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 115200;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */

  /* USER CODE END UART5_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream7_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream7_IRQn);
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_8
                          |GPIO_PIN_9|GPIO_PIN_10, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC0 PC1 PC2 PC3
                           PC6 PC7 PC8 PC9 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA1 PA2 PA3 PA8
                           PA9 PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_8
                          |GPIO_PIN_9|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB12 PB13 PB14 PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB4 PB5 PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
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
