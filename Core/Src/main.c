/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "wizchip_conf.h"
#include "socket.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TIMEOUT 2000
#define DEVICE_ID 0x200
#define ATTEMPTS 5
#define DEBUG 1
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan;

SPI_HandleTypeDef hspi2;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
DS18B20 temperatureSensor;

CAN_RxHeaderTypeDef rxHeader; //CAN Bus Transmit Header
CAN_TxHeaderTypeDef txHeader; //CAN Bus Receive Header
uint8_t canRX[8];
uint8_t canTX[8];
uint32_t canMailbox;

int16_t failsafe = 0;
uint8_t newData = 0;
uint8_t attempts = 0;
uint32_t currentId = 0x00;
uint32_t ids[8] = {0x210, 0x211, 0x212, 0x213, 0x214, 0x215, 0x216, 0x217};
uint8_t buff[10];
char *arr[2];
int32_t ret = 0;
uint8_t i = 0;
char *p;
uint8_t id = 0;
float temp = 0.0f;
uint8_t r = 0;
char *pEnd = 0;
char tempBuff[8];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI2_Init(void);
static void MX_CAN_Init(void);
/* USER CODE BEGIN PFP */

void trace(const char *buf);

// включить модуль W5500 сигналом SCNn=0
__attribute__((unused)) void cs_sel()
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET); //CS LOW
}

// выключить модуль W5500 сигналом SCNn=1
__attribute__((unused)) void cs_desel()
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET); //CS HIGH
}

// принять байт через SPI
__attribute__((unused)) uint8_t spi_rb(void)
{
    uint8_t rbuf;
    HAL_SPI_Receive(&hspi2, &rbuf, 1, 0xFFFFFFFF);
    return rbuf;
}

// передать байт через SPI
__attribute__((unused)) void spi_wb(uint8_t b)
{
    HAL_SPI_Transmit(&hspi2, &b, 1, 0xFFFFFFFF);
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

float getTemp()
{
    DS18B20_InitializationCommand(&temperatureSensor);
    DS18B20_SkipRom(&temperatureSensor);
    DS18B20_ConvertT(&temperatureSensor, DS18B20_DATA);
    DS18B20_InitializationCommand(&temperatureSensor);
    DS18B20_SkipRom(&temperatureSensor);
    DS18B20_ReadScratchpad(&temperatureSensor);
    return temperatureSensor.temperature;
}

uint8_t sendCommandToCANSlave(uint32_t localDeviceIid, uint8_t *command)
{
    currentId = ids[localDeviceIid - 1];
    txHeader.StdId = currentId;
    attempts = ATTEMPTS + 1;
    strcpy((char *) canTX, command);
    failsafe = 0;

    while(1)
    {
        if (newData)
        {
            break;
        }

        if (attempts == 0)
        {
            trace("Fail to send\r\n");
            return 0;
        }

        if (failsafe <= 0)
        {
            trace("Send command ");
            trace(canTX);
            trace(" to slave\r\n");
            HAL_CAN_AddTxMessage(&hcan, &txHeader, canTX, &canMailbox);
            failsafe = TIMEOUT;
            attempts--;
        }
    }

    trace("OK\r\n");
    memset(canTX, 0, sizeof(canTX));
    strcpy((char *) canTX, "OK");
    trace("Send command ");
    trace(canTX);
    trace(" to slave\r\n");
    HAL_CAN_AddTxMessage(&hcan, &txHeader, canTX, &canMailbox);
    newData = 0;
    return 1;
}

void sendToSocket(uint8_t *data)
{
    if ((send(0, data, strlen(data))) == (int16_t) strlen(data))
    {
        trace("Send to socket: ");
        trace(data);
        trace("\r\n");
    }
    else
    {
        trace("Error while sending to socket!\r\n");
    }
}

void tcp_server()
{
    trace("Try open socket\r\n");

    if(socket(0, Sn_MR_TCP, 5000, 0) != 0)
    {
        trace("Error open socket\r\n");
        return;
    }

    trace("Socket opened, try listen\r\n");

    if(listen(0) != SOCK_OK)
    {
        trace("Error listen socket\r\n");
        return;
    }

    trace("Socked listened, wait for input connection\r\n");

    while(getSn_SR(0) == SOCK_LISTEN)
    {
        HAL_Delay(200);
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
    }

    trace("Input connection\r\n");

    if(getSn_SR(0) != SOCK_ESTABLISHED)
    {
        trace("Error socket status\r\n");
        return;
    }

    while ((ret = recv(0, (uint8_t *) buff, sizeof(buff)-1)) != SOCKERR_SOCKSTATUS)
    {
        if (ret > 0)
        {
            p = strtok(buff, "-");
            while (p != NULL)
            {
                arr[i++] = p;
                p = strtok(NULL, "-");
            }

            id = strtol(arr[0], &pEnd, 10);

            if (id == 0)
            {
                if (strcmp((char *) arr[1], "get_tmp") == 0)
                {
                    trace("Get temperature from master device\r\n");
                    temp = getTemp();
                    r = snprintf(tempBuff, sizeof tempBuff, "%.*f", 2, temp);
                    if (r < 0)
                    {
                        sendToSocket("ERROR");
                    }
                    else
                    {
                        sendToSocket(tempBuff);
                    }
                }
                else if (strcmp((char *) arr[1], "rl1_off") == 0)
                {
                    trace("Relay 1 OFF on master\r\n");
                    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
                    sendToSocket("OK");
                }
                else if (strcmp((char *) arr[1], "rl1_on") == 0)
                {
                    trace("Relay 1 ON on master\r\n");
                    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
                    sendToSocket("OK");
                }
                else if (strcmp((char *) arr[1], "rl2_off") == 0)
                {
                    trace("Relay 2 OFF on master\r\n");
                    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
                    sendToSocket("OK");
                }
                else if (strcmp((char *) arr[1], "rl2_on") == 0)
                {
                    trace("Relay 2 ON on master\r\n");
                    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
                    sendToSocket("OK");
                }
                else if (strcmp((char *) arr[1], "rl3_off") == 0)
                {
                    trace("Relay 3 OFF on master\r\n");
                    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
                    sendToSocket("OK");
                }
                else if (strcmp((char *) arr[1], "rl3_on") == 0)
                {
                    trace("Relay 3 ON on master\r\n");
                    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
                    sendToSocket("OK");
                }
                else if (strcmp((char *) arr[1], "rl4_off") == 0)
                {
                    trace("Relay 4 OFF on master\r\n");
                    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
                    sendToSocket("OK");
                }
                else if (strcmp((char *) arr[1], "rl4_on") == 0)
                {
                    trace("Relay 4 ON on master\r\n");
                    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
                    sendToSocket("OK");
                }
                else
                {
                    sendToSocket("UNKNOWN_COMMAND");
                    trace("Unknown command for master device\r\n");
                }
            }
            else if (id >= 1 && id <= 7)
            {
                if (strcmp((char *) arr[1], "get_tmp") == 0)
                {
                    trace("Get temperature from device with id ");
                    trace(arr[0]);
                    trace("\r\n");
                    if (sendCommandToCANSlave(id, arr[1]))
                    {
                        sendToSocket(canRX);
                    }
                    else
                    {
                        sendToSocket("FAILED");
                    }
                }
                else if (strcmp((char *) arr[1], "led_on") == 0)
                {
                    if (id != 2)
                    {
                        trace("Power on LED on device with id ");
                        trace(arr[0]);
                        trace("\r\n");
                        if (sendCommandToCANSlave(id, arr[1]))
                        {
                            sendToSocket(canRX);
                        }
                        else
                        {
                            sendToSocket("FAILED");
                        }
                    }
                    else
                    {
                        sendToSocket("UNKNOWN_COMMAND");
                        trace("Unknown command for slave device with id ");
                        trace(arr[0]);
                        trace("\r\n");
                    }
                }
                else if (strcmp((char *) arr[1], "led_off") == 0)
                {
                    if (id != 2)
                    {
                        trace("Power off LED on device with id ");
                        trace(arr[0]);
                        trace("\r\n");
                        if (sendCommandToCANSlave(id, arr[1]))
                        {
                            sendToSocket(canRX);
                        }
                        else
                        {
                            sendToSocket("FAILED");
                        }
                    }
                    else
                    {
                        sendToSocket("UNKNOWN_COMMAND");
                        trace("Unknown command for slave device with id ");
                        trace(arr[0]);
                        trace("\r\n");
                    }
                }
                else if (id == 2) {
                    if (strcmp((char *) arr[1], "rl1_on") == 0)
                    {
                        trace("Relay 1 ON on device with id ");
                        trace(arr[0]);
                        trace("\r\n");
                        if (sendCommandToCANSlave(id, arr[1]))
                        {
                            sendToSocket(canRX);
                        }
                        else
                        {
                            sendToSocket("FAILED");
                        }
                    }
                    else if (strcmp((char *) arr[1], "rl1_off") == 0)
                    {
                        trace("Relay 1 OFF on device with id ");
                        trace(arr[0]);
                        trace("\r\n");
                        if (sendCommandToCANSlave(id, arr[1]))
                        {
                            sendToSocket(canRX);
                        }
                        else
                        {
                            sendToSocket("FAILED");
                        }
                    }
                    else if (strcmp((char *) arr[1], "rl2_on") == 0)
                    {
                        trace("Relay 2 ON on device with id ");
                        trace(arr[0]);
                        trace("\r\n");
                        if (sendCommandToCANSlave(id, arr[1]))
                        {
                            sendToSocket(canRX);
                        }
                        else
                        {
                            sendToSocket("FAILED");
                        }
                    }
                    else if (strcmp((char *) arr[1], "rl2_off") == 0)
                    {
                        trace("Relay 2 OFF on device with id ");
                        trace(arr[0]);
                        trace("\r\n");
                        if (sendCommandToCANSlave(id, arr[1]))
                        {
                            sendToSocket(canRX);
                        }
                        else
                        {
                            sendToSocket("FAILED");
                        }
                    }
                    else if (strcmp((char *) arr[1], "rl3_on") == 0)
                    {
                        trace("Relay 3 ON on device with id ");
                        trace(arr[0]);
                        trace("\r\n");
                        if (sendCommandToCANSlave(id, arr[1]))
                        {
                            sendToSocket(canRX);
                        }
                        else
                        {
                            sendToSocket("FAILED");
                        }
                    }
                    else if (strcmp((char *) arr[1], "rl3_off") == 0)
                    {
                        trace("Relay 3 OFF on device with id ");
                        trace(arr[0]);
                        trace("\r\n");
                        if (sendCommandToCANSlave(id, arr[1]))
                        {
                            sendToSocket(canRX);
                        }
                        else
                        {
                            sendToSocket("FAILED");
                        }
                    }
                    else if (strcmp((char *) arr[1], "rl4_on") == 0)
                    {
                        trace("Relay 4 ON on device with id ");
                        trace(arr[0]);
                        trace("\r\n");
                        if (sendCommandToCANSlave(id, arr[1]))
                        {
                            sendToSocket(canRX);
                        }
                        else
                        {
                            sendToSocket("FAILED");
                        }
                    }
                    else if (strcmp((char *) arr[1], "rl4_off") == 0)
                    {
                        trace("Relay 4 OFF on device with id ");
                        trace(arr[0]);
                        trace("\r\n");
                        if (sendCommandToCANSlave(id, arr[1]))
                        {
                            sendToSocket(canRX);
                        }
                        else
                        {
                            sendToSocket("FAILED");
                        }
                    }
                }
                else
                {
                    sendToSocket("UNKNOWN_COMMAND");
                    trace("Unknown command for slave device with id ");
                    trace(arr[0]);
                    trace("\r\n");
                }

                memset(canTX, 0, sizeof(canTX));
                memset(canRX, 0, sizeof(canRX));
            }
            else
            {
                sendToSocket("INVALID_ID");
                trace("Invalid ID\r\n");
            }

            memset(arr, 0, sizeof(arr));
            i = 0;
            p = 0;
            id = 0;
            pEnd = 0;
        }
        memset(buff, 0, sizeof(buff));
        ret = 0;
    }

    trace("Connection closed\r\n");

    disconnect(0);
    close(0);
}

void trace(const char *buf) {
#pragma clang diagnostic push
#pragma ide diagnostic ignored "Simplify"
    if (DEBUG)
    {
        uint8_t size = strlen(buf);
        uint16_t time = 87 * size / 1000 + 5;
        HAL_UART_Transmit(&huart1, (uint8_t *) buf, size, time);
    }
#pragma clang diagnostic pop
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
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_SPI2_Init();
  MX_CAN_Init();
  /* USER CODE BEGIN 2 */
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      tcp_server();
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 9;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_4TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_3TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */
  CAN_FilterTypeDef canFilter;

  canFilter.FilterBank = 0;
  canFilter.FilterMode = CAN_FILTERMODE_IDMASK;
  canFilter.FilterFIFOAssignment = CAN_RX_FIFO0;
  canFilter.FilterIdHigh = 0;
  canFilter.FilterIdLow = 0;
  canFilter.FilterMaskIdHigh = 0;
  canFilter.FilterMaskIdLow = 0;
  canFilter.FilterScale = CAN_FILTERSCALE_32BIT;
  canFilter.FilterActivation = ENABLE;
  canFilter.SlaveStartFilterBank = 14;

  HAL_CAN_ConfigFilter(&hcan,&canFilter);

  if (HAL_CAN_Start(&hcan) != HAL_OK)
  {
      Error_Handler();
  }

  if (HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
  {
      Error_Handler();
  }

  txHeader.DLC = 8;
  txHeader.IDE = CAN_ID_STD;
  txHeader.RTR = CAN_RTR_DATA;
  txHeader.StdId = DEVICE_ID;
  txHeader.ExtId = 0x00;
  txHeader.TransmitGlobalTime = DISABLE;
  /* USER CODE END CAN_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */
  reg_wizchip_cs_cbfunc(cs_sel, cs_desel);
  reg_wizchip_spi_cbfunc(spi_rb, spi_wb);

  uint8_t bufSize[] = {2, 2, 2, 2};
  wizchip_init(bufSize, bufSize);
  wiz_NetInfo netInfo ={
          .mac = {0x00, 0x08, 0xdc, 0xab, 0xcd, 0xef},
          .ip = {192, 168, 10, 75},
          .sn = {255, 255, 255, 0},
          .gw = {192, 168, 10, 1}
  };
  wizchip_setnetinfo(&netInfo);
  wizchip_getnetinfo(&netInfo);
  /* USER CODE END SPI2_Init 2 */

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
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  if (HAL_HalfDuplex_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */
  DS18B20_Init(&temperatureSensor, &huart2);
  DS18B20_InitializationCommand(&temperatureSensor);
  DS18B20_ReadRom(&temperatureSensor);
  DS18B20_ReadScratchpad(&temperatureSensor);
  uint8_t settings[3];
  settings[0] = temperatureSensor.temperatureLimitHigh;
  settings[1] = temperatureSensor.temperatureLimitLow;
  settings[2] = DS18B20_12_BITS_CONFIG;
  DS18B20_InitializationCommand(&temperatureSensor);
  DS18B20_SkipRom(&temperatureSensor);
  DS18B20_WriteScratchpad(&temperatureSensor, settings);
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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_8, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA1 PA4 PA5 PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan1)
{
    if (HAL_CAN_GetRxMessage(hcan1, CAN_RX_FIFO0, &rxHeader, canRX) != HAL_OK)
    {
        Error_Handler();
    }

    if (rxHeader.StdId == currentId)
    {
        newData = 1;
        trace("New Data\n");
    } else {
        trace("Data from unknown device\n");
        newData = 0;
    }
}
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
