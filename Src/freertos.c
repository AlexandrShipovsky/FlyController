/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
//#include "queue. h"
#include "cmsis_os.h"
#include "usb_device.h"
#include "cli.h"
#include "api.h"
#include "lwip.h"
#include "string.h"

#include "confighawk.h"
#include "prothawk.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BUFSIZE 256 // Размер буфера для принимаемых пакетов
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
uint8_t net_state = 0;                            // Состояние подключения
xQueueHandle GroundStationDataQueueHandle = NULL; // Очередь передачи принятых байт от задачи приема к задаче парсинга
struct netconn *nc;
struct netbuf *nb;

PitchRollAccelTypeDef PitchRollAccel; // Структура со значениями положения двигателей

/* USER CODE END Variables */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void PingHandler(uint8_t *pbuf);         // Обработчик команды PING
void PilotCommandHandler(uint8_t *pbuf); // Обработчик команды
/* USER CODE END FunctionPrototypes */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize);

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize)
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const *argument)
{

  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();

  /* init code for LWIP */

  GroundStationDataQueueHandle = xQueueCreate(16, BUFSIZE);

  MX_LWIP_Init();
  /* USER CODE BEGIN 5 */
  extern struct netif gnetif;
  uint8_t PhyStat;

  /* Infinite loop */
  for (;;)
  {
    PhyStat = gnetif.flags; //  Статистика подключения
    if (PhyStat != 15)
    {
      HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET);
    }
    else
    {
      HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);
    }
    HAL_GPIO_TogglePin(LD1_GPIO_Port, LD1_Pin);
    vTaskDelay(500);
  };

  /* USER CODE END 5 */
}

void ethernetif_notify_conn_changed(struct netif *netif)
{
  /* NOTE : This is function could be implemented in user file 
            when the callback is needed,
  */
  BaseType_t xHigherPriorityTaskWoken;
  uint8_t PhyStat;
  extern struct netif gnetif;
  extern osThreadId ConGroundStatioHandle;

  PhyStat = gnetif.flags;
  if (PhyStat == 15)
  {
    xHigherPriorityTaskWoken = xTaskResumeFromISR(ConGroundStatioHandle);
    if (xHigherPriorityTaskWoken == pdTRUE)
    {
      portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
  }
  else
  {
    vTaskSuspend(ConGroundStatioHandle);
    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
    net_state = 0;
  }
}
/* USER CODE BEGIN Header_StartConGroundStation */
/**
* @brief Function implementing the ConGroundStatio thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartConGroundStation */
void StartConGroundStation(void const *argument)
{
  /* USER CODE BEGIN StartConGroundStation */

  int8_t res;

  volatile uint16_t len;
  uint8_t buf[BUFSIZE];

  extern struct netif gnetif;

  ip_addr_t local_ip;
  ip_addr_t remote_ip;
  /* Infinite loop */
  for (;;)
  {
    vTaskDelay(1);

    if (net_state)
    {

      res = netconn_recv(nc, &nb);
      if (res != 0)
      {
        HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
        netconn_delete(nc);
        net_state = 0;
      }
      else
      {
        HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
        len = netbuf_len(nb);
        netbuf_copy(nb, buf, len);
        netbuf_delete(nb);
        if (GroundStationDataQueueHandle != NULL)
        {
          if (xQueueSendToBack(GroundStationDataQueueHandle,
                               (void *)buf,
                               (TickType_t)10) != pdPASS)
          {
            /* Failed to post the message, even after 10 ticks. */
          }
        }

        memset(buf, 0x00, sizeof(buf));
      }
    }
    else
    {
      HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
      netconn_delete(nc);
      while (gnetif.ip_addr.addr == 0)
      {
        vTaskDelay(1);
      }
      local_ip = gnetif.ip_addr;
      ip4addr_aton(IpGroundStation, &remote_ip);
      nc = netconn_new(NETCONN_TCP);
      if (nc != NULL)
      {
        res = netconn_bind(nc, &local_ip, PortGroundStation);
        if (res == ERR_OK)
        {
          res = netconn_connect(nc, &remote_ip, PortGroundStation);
          if (res == ERR_OK)
          {

            net_state = 1;
            continue;
          }
        }
      }
    }
  }
}
/* USER CODE END StartConGroundStation */

/* USER CODE BEGIN Header_StartParserGroundStationTask */
/**
* @brief Function implementing the ParserGroundSta thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartParserGroundStationTask */
void StartParserGroundStationTask(void const *argument)
{
  /* USER CODE BEGIN StartParserGroundStationTask */
  uint8_t pbuf[BUFSIZE];
  memset(pbuf, 0x00, sizeof(pbuf));
  /* Infinite loop */
  for (;;)
  {

    if (GroundStationDataQueueHandle != NULL)
    {
      if (xQueueReceive(GroundStationDataQueueHandle,
                        pbuf,
                        (TickType_t)0) == pdPASS)
      {
        /* *pxRxedPointer now points to xMessage. */
      }
    }
    uint16_t i = 0;
    while (pbuf[i] != 0x00)
    {
      switch (pbuf[i])
      {
      case PreFlightTestRequest:
        i += CommandSize[PreFlightTestRequest];
        break;

      case PreFlightTestResponse:
        i += CommandSize[PreFlightTestResponse];
        break;

      case WingCalibrationRequest:
        i += CommandSize[WingCalibrationRequest];
        break;

      case WingCalibrationResponse:
        i += CommandSize[WingCalibrationResponse];
        break;

      case PilotCommand:
        PilotCommandHandler(&pbuf[i]);
        i += CommandSize[PilotCommand];
        break;

      case PilotCommandResponse:
        i += CommandSize[PilotCommandResponse];
        break;

      case PING:
        PingHandler(&pbuf[i]);
        i += CommandSize[PING];
        break;
      }
    }
    memset(pbuf, 0x00, sizeof(pbuf)); // Очистить буфер
    vTaskDelay(1);
  }
}
/* USER CODE END StartParserGroundStationTask */
/* USER CODE BEGIN Header_StartCANTask */
/**
* @brief Function implementing the CANTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartCANTask */
void StartCANTask(void const *argument)
{
  /* USER CODE BEGIN StartCANTask */
  extern CAN_HandleTypeDef hcan1;
  uint8_t buf[8] = {'5', 0xAA, 0xAA, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
  uint32_t TxMailBox; //= CAN_TX_MAILBOX0;
  /* Infinite loop */
  for (;;)
  {
    CAN_TxHeaderTypeDef TxHeader;
    TxHeader.DLC = 8;
    TxHeader.StdId = 0x0000;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.TransmitGlobalTime = DISABLE;

    // if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) > 0U)
    // {
    taskENTER_CRITICAL();
    if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, buf, &TxMailBox) != HAL_OK)
    {
      //Error_Handler();
    }
    taskEXIT_CRITICAL();
    vTaskDelay(500);
  }
  /* USER CODE END StartCANTask */
}

/* USER CODE BEGIN Header_StartCliTask */
/**
* @brief Function implementing the cliTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartCliTask */
void StartCliTask(void const *argument)
{
  /* USER CODE BEGIN StartCliTask */
  /* Infinite loop */
  for (;;)
  {
    //DBG_CLI_USB_Task();
    vTaskDelay(10);
  }
  /* USER CODE END StartCliTask */
}

/******************************************************************/
/*Обработчики команд*/
void PingHandler(uint8_t *pbuf)
{
  int8_t res;
  res = netconn_write(nc, (char const *)pbuf, CommandSize[PING], NETCONN_COPY);
  if (res != ERR_OK)
  {
  }
}
/*
*
*
*
*/
void PilotCommandHandler(uint8_t *pbuf)
{
  //int8_t res;
  uint8_t i = 1;

  memcpy(&PitchRollAccel.Pitch, &pbuf[i], sizeof(PitchRollAccel.Pitch));
  i += 2;
  memcpy(&PitchRollAccel.Roll, &pbuf[i], sizeof(PitchRollAccel.Roll));
  i += 2;
  memcpy(&PitchRollAccel.Accel, &pbuf[i], sizeof(PitchRollAccel.Accel));
}
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
