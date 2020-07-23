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
#include "tcp.h"
#include "lwip.h"
#include "string.h"

#include "confighawk.h"
#include "prothawk.h"
#include "telemetry.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BUFSIZE 1024 // Размер буфера для принимаемых пакетов
#define BUFSIZEUART 32
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
uint8_t net_state = 0;                             // Состояние подключения
xQueueHandle GroundStationDataQueueHandle = NULL;  // Очередь передачи принятых байт от задачи приема к задаче парсинга
xQueueHandle ElMotorCANQueueHandle = NULL;         // Очередь передачи принятых по CAN1 байт
xQueueHandle AssistEquipmentCANQueueHandle = NULL; // Очередь передачи принятых по CAN2 байт
struct netconn *nc;
struct netbuf *nb;
struct netbuf *nb_send;

uint8_t BufUART[BUFSIZEUART] = {
    0,
};
HAL_StatusTypeDef ErrUART;

ElMotorUnitParametersTypeDef ElMotorUnitParameters; // Структура с параметрами блока управления приводами
PropultionParametersTypeDef PropultionParameters;   // Структура с параметрами блока управления силовой установкой
/* USER CODE END Variables */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void PingHandler(uint8_t *pbuf);            // Обработчик команды PING
void PilotCommandHandler(uint8_t *pbuf);    // Обработчик команды
void TestModeHandler(void);                 // Обработчик команды начала предполетного тестирования всех систем
void CalibrationHandler(uint8_t *pilotbuf); // Обработчик команды окончания калибровки приводов
/* USER CODE END FunctionPrototypes */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

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

  MX_LWIP_Init();
  /* USER CODE BEGIN 5 */
  //extern UART_HandleTypeDef huart8;

  //ErrUART = HAL_UART_Receive_DMA(&huart8, (uint8_t *)BufUART, BUFSIZEUART);

  /* Infinite loop */
  for (;;)
  {
    HAL_GPIO_TogglePin(LD1_GPIO_Port, LD1_Pin);
    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
    //ErrUART = HAL_UART_Transmit_IT(&huart8, (uint8_t *)"Golosuy za popravki\n\r", 20);
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
  extern osThreadId ConGndStatTaskHandle;

  PhyStat = gnetif.flags;
  if (PhyStat == 15)
  {
    xHigherPriorityTaskWoken = xTaskResumeFromISR(ConGndStatTaskHandle);
    if (xHigherPriorityTaskWoken == pdTRUE)
    {
      portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
  }
  else
  {
    vTaskSuspend(ConGndStatTaskHandle);
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

  GroundStationDataQueueHandle = xQueueCreate(16, BUFSIZE);

  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
  while (gnetif.ip_addr.addr == 0)
  {
    vTaskDelay(1);
  }
  local_ip = gnetif.ip_addr;
  ip4addr_aton(IpGroundStation, &remote_ip);
  nc = netconn_new(NETCONN_UDP);
  if (nc != NULL)
  {
    res = netconn_bind(nc, &local_ip, PortGroundStation);
    if (res == ERR_OK)
    {
      res = netconn_connect(nc, &remote_ip, PortGroundStation);
      if (res == ERR_OK)
      {
        net_state = 1;
      }
    }
  }
  /* Infinite loop */
  for (;;)
  {
    vTaskDelay(1);
    res = netconn_recv(nc, &nb);
    if (res != 0)
    {
      HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
      net_state = 0;
    }
    else
    {
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
}
/* USER CODE END StartConGroundStation */

/* USER CODE BEGIN Header_StartParserGroundStationTask */
/**
* @brief Function implementing the ParserGroundSta thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartParserGroundStation */
void StartParserGroundStation(void const *argument)
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
                        (TickType_t)5) == pdPASS)
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
        TestModeHandler();
        i += CommandSize[PreFlightTestRequest];
        break;

      case PreFlightTestResponse:
        i += CommandSize[PreFlightTestResponse];
        break;

      case WingCalibrationRequest:
        CalibrationHandler(&pbuf[i]);
        i += CommandSize[WingCalibrationRequest];
        break;

      case WingCalibrationResponse:
        i += CommandSize[WingCalibrationResponse];
        break;
      case PING:
        PingHandler(&pbuf[i]);
        i += (uint16_t)CommandSize[PING];
        break;

      case PilotCommand:
        PilotCommandHandler(&pbuf[i]);
        i += (uint16_t)CommandSize[PilotCommand];
        break;

      case PilotCommandResponse:
        i += (uint16_t)CommandSize[PilotCommandResponse];
        break;

      default:
        memset(pbuf, 0x00, sizeof(pbuf)); // Очистить буфер
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
  uint8_t canbuf[8];
  const char testbuf[9] = {PreFlightTestResponse};
  const char calibbuf[9] = {WingCalibrationResponse};
  int8_t res;

  ElMotorCANQueueHandle = xQueueCreate(8, 8);
  AssistEquipmentCANQueueHandle = xQueueCreate(8, 8);
  /* Infinite loop */
  for (;;)
  {
    if (ElMotorCANQueueHandle != NULL)
    {
      if (xQueueReceive(ElMotorCANQueueHandle,
                        canbuf,
                        (TickType_t)0) == pdPASS)
      {
        /* *pxRxedPointer now points to xMessage. */
      }
      switch (canbuf[0])
      {
      case PitchRollCommand:
        memcpy(&ElMotorUnitParameters.Pitch, &canbuf[1], sizeof(ElMotorUnitParameters.Pitch));
        memcpy(&ElMotorUnitParameters.Roll, &canbuf[3], sizeof(ElMotorUnitParameters.Roll));
        break;
      case TestMode:
        taskENTER_CRITICAL();
        nb_send = netbuf_new();
        netbuf_alloc(nb_send, CommandSize[PreFlightTestResponse]);
        pbuf_take(nb_send->p, (void *)testbuf, CommandSize[PreFlightTestResponse]);
        res = netconn_send(nc, nb_send);
        netbuf_delete(nb_send);
        taskEXIT_CRITICAL();
        if (res != ERR_OK)
        {
        }
        break;
      case CalibComplied:
        taskENTER_CRITICAL();
        nb_send = netbuf_new();
        netbuf_alloc(nb_send, CommandSize[WingCalibrationResponse]);
        pbuf_take(nb_send->p, (void *)calibbuf, CommandSize[WingCalibrationResponse]);
        res = netconn_send(nc, nb_send);
        netbuf_delete(nb_send);
        taskEXIT_CRITICAL();
        if (res != ERR_OK)
        {
        }
        break;
      case PitchMinMax:
        memcpy(&ElMotorUnitParameters.MinPitch, &canbuf[1], sizeof(ElMotorUnitParameters.MinPitch));
        memcpy(&ElMotorUnitParameters.MaxPitch, &canbuf[3], sizeof(ElMotorUnitParameters.MaxPitch));
        break;
      case RollMinMax:
        memcpy(&ElMotorUnitParameters.MinRoll, &canbuf[1], sizeof(ElMotorUnitParameters.MinRoll));
        memcpy(&ElMotorUnitParameters.MaxRoll, &canbuf[3], sizeof(ElMotorUnitParameters.MaxRoll));
        break;
      case VBATCommand:
        memcpy(&ElMotorUnitParameters.VBAT, &canbuf[1], sizeof(ElMotorUnitParameters.VBAT));
        break;
      case PitchForceCommand:
        memcpy(&ElMotorUnitParameters.PitchForce, &canbuf[1], sizeof(ElMotorUnitParameters.PitchForce));
        break;
      default:
        break;
      }
    }
    memset(canbuf, 0x00, sizeof(canbuf)); // Очистить буфер
    vTaskDelay(1);

    if (AssistEquipmentCANQueueHandle != NULL)
    {
      if (xQueueReceive(AssistEquipmentCANQueueHandle,
                        canbuf,
                        (TickType_t)0) == pdPASS)
      {
        /* *pxRxedPointer now points to xMessage. */
      }
      switch (canbuf[0])
      {
      case HeaderPropultionCommand:
        memcpy(&PropultionParameters.RPM, &canbuf[1], sizeof(PropultionParameters.RPM));
        memcpy(&PropultionParameters.FuelCapacity, &canbuf[3], sizeof(PropultionParameters.FuelCapacity));
        memcpy(&PropultionParameters.ThrottlePosition, &canbuf[4], sizeof(PropultionParameters.ThrottlePosition));
        break;
      default:
        break;
      }
    }
    memset(canbuf, 0x00, sizeof(canbuf)); // Очистить буфер
    vTaskDelay(1);
  }
  /* USER CODE END StartCANTask */
}

/*CAN1 Callback*/
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  uint8_t buf[8];
    CAN_RxHeaderTypeDef RxHeader;
    BaseType_t xHigherPriorityTaskWoken;
    xHigherPriorityTaskWoken = pdFALSE;
    
  if (hcan->Instance == CAN1)
  {
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, buf) != HAL_OK)
    {
      //Ошибка
    }
    else
    {
      if (ElMotorCANQueueHandle != NULL)
      {
        xQueueSendToBackFromISR(ElMotorCANQueueHandle,
                                (void *)buf,
                                &xHigherPriorityTaskWoken);
      }
    }
  }

  if (hcan->Instance == CAN2)
  {
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, buf) != HAL_OK)
    {
      //Ошибка
    }
    else
    {
      if (AssistEquipmentCANQueueHandle != NULL)
      {
        xQueueSendToBackFromISR(AssistEquipmentCANQueueHandle,
                                (void *)buf,
                                &xHigherPriorityTaskWoken);
      }
    }
  }
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
    DBG_CLI_Serial_Task();
  }
  /* USER CODE END StartCliTask */
}

/******************************************************************/
/*Обработчики команд*/
void PingHandler(uint8_t *pingbuf)
{
  int8_t res;

  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
  taskENTER_CRITICAL();
  nb_send = netbuf_new();
  netbuf_alloc(nb_send, CommandSize[PING]);
  pbuf_take(nb_send->p, (void *)pingbuf, CommandSize[PING]);
  res = netconn_send(nc, nb_send);
  netbuf_delete(nb_send);
  taskEXIT_CRITICAL();
  if (res != ERR_OK)
  {
  }
}
/*
*
*
*
*/
void PilotCommandHandler(uint8_t *pilotbuf)
{
  int8_t res;
  uint8_t SendTCPBuf[44];
  extern CAN_HandleTypeDef hcan1;
  extern CAN_HandleTypeDef hcan2;

  uint32_t TxMailBox; //= CAN_TX_MAILBOX0;
  CAN_TxHeaderTypeDef TxHeader;

  extern SemaphoreHandle_t SemaphoreForSendTelemetry; // Семафор, разрешающий отправку телеметрии с �?НС раз в TimeSendTelemetry*vTaskDelay миллисекунд
  extern IMUTelemetryTypeDef IMUTelemetry;

  // Передача на блок управления приводами
  TxHeader.DLC = 5;
  TxHeader.StdId = 0x0000;
  TxHeader.RTR = CAN_RTR_DATA;
  TxHeader.IDE = CAN_ID_STD;
  TxHeader.TransmitGlobalTime = DISABLE;

  if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, pilotbuf, &TxMailBox) != HAL_OK)
  {
    //Error_Handler
  }

  // Передача на блок управления силовой установкой
  TxHeader.DLC = 2;
  TxHeader.StdId = 0x0000;
  TxHeader.RTR = CAN_RTR_DATA;
  TxHeader.IDE = CAN_ID_STD;
  TxHeader.TransmitGlobalTime = DISABLE;

  uint8_t PropultionBuf[2];

  PropultionBuf[0] = (uint8_t)HeaderPropultionCommand;
  PropultionBuf[1] = pilotbuf[5];

  if (HAL_CAN_AddTxMessage(&hcan2, &TxHeader, PropultionBuf, &TxMailBox) != HAL_OK)
  {
    //Error_Handler
  }

  SendTCPBuf[0] = PilotCommandResponse;
  SendTCPBuf[1] = (uint8_t)(ElMotorUnitParameters.Pitch & 0xFF);
  SendTCPBuf[2] = (uint8_t)(ElMotorUnitParameters.Pitch >> 8);
  SendTCPBuf[3] = (uint8_t)(ElMotorUnitParameters.Roll & 0xFF);
  SendTCPBuf[4] = (uint8_t)(ElMotorUnitParameters.Roll >> 8);
  SendTCPBuf[5] = 0x00;
  SendTCPBuf[6] = 0x00;
  SendTCPBuf[7] = (uint8_t)(ElMotorUnitParameters.MinPitch & 0xFF);
  SendTCPBuf[8] = (uint8_t)(ElMotorUnitParameters.MinPitch >> 8);
  SendTCPBuf[9] = (uint8_t)(ElMotorUnitParameters.MaxPitch & 0xFF);
  SendTCPBuf[10] = (uint8_t)(ElMotorUnitParameters.MaxPitch >> 8);
  SendTCPBuf[11] = (uint8_t)(ElMotorUnitParameters.MinRoll & 0xFF);
  SendTCPBuf[12] = (uint8_t)(ElMotorUnitParameters.MinRoll >> 8);
  SendTCPBuf[13] = (uint8_t)(ElMotorUnitParameters.MaxRoll & 0xFF);
  SendTCPBuf[14] = (uint8_t)(ElMotorUnitParameters.MaxRoll >> 8);
  memcpy(&SendTCPBuf[15], &ElMotorUnitParameters.VBAT, sizeof(ElMotorUnitParameters.VBAT));
  memcpy(&SendTCPBuf[19], &ElMotorUnitParameters.PitchForce, sizeof(ElMotorUnitParameters.PitchForce));

  if (SemaphoreForSendTelemetry != NULL)
  {
    if (xSemaphoreTake(SemaphoreForSendTelemetry, (TickType_t)0))
    {
      SendTCPBuf[23] = TelemetryResponse;
      memcpy(&SendTCPBuf[24], &IMUTelemetry.altitude, sizeof(IMUTelemetry.altitude));
      memcpy(&SendTCPBuf[28], &IMUTelemetry.yaw, sizeof(IMUTelemetry.yaw));
      memcpy(&SendTCPBuf[32], &IMUTelemetry.pitch, sizeof(IMUTelemetry.pitch));
      memcpy(&SendTCPBuf[36], &IMUTelemetry.roll, sizeof(IMUTelemetry.roll));
      memcpy(&SendTCPBuf[40], &PropultionParameters.RPM, sizeof(PropultionParameters.RPM));
      memcpy(&SendTCPBuf[42], &PropultionParameters.FuelCapacity, sizeof(PropultionParameters.FuelCapacity));
      memcpy(&SendTCPBuf[43], &PropultionParameters.ThrottlePosition, sizeof(PropultionParameters.ThrottlePosition));
      taskENTER_CRITICAL();
      nb_send = netbuf_new();
      netbuf_alloc(nb_send, CommandSize[PilotCommandResponse] + CommandSize[TelemetryResponse]);
      pbuf_take(nb_send->p, (void *)SendTCPBuf, CommandSize[PilotCommandResponse] + CommandSize[TelemetryResponse]);
      res = netconn_send(nc, nb_send);
      netbuf_delete(nb_send);
      taskEXIT_CRITICAL();
      return;
    }
  }

  taskENTER_CRITICAL();
  nb_send = netbuf_new();
  netbuf_alloc(nb_send, CommandSize[PilotCommandResponse]);
  pbuf_take(nb_send->p, (void *)SendTCPBuf, CommandSize[PilotCommandResponse]);
  res = netconn_send(nc, nb_send);
  netbuf_delete(nb_send);
  taskEXIT_CRITICAL();
  if (res != ERR_OK)
  {
  }
  return;
}
/*
*
*
*
*/
void TestModeHandler(void)
{
  uint8_t ElMotorBuf[8];

  extern CAN_HandleTypeDef hcan1;
  uint32_t TxMailBox; //= CAN_TX_MAILBOX0;
  CAN_TxHeaderTypeDef TxHeader;
  // Передача на блок управления приводами
  TxHeader.DLC = 8;
  TxHeader.StdId = 0x0000;
  TxHeader.RTR = CAN_RTR_DATA;
  TxHeader.IDE = CAN_ID_STD;
  TxHeader.TransmitGlobalTime = DISABLE;

  ElMotorBuf[0] = TestMode;
  if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, ElMotorBuf, &TxMailBox) != HAL_OK)
  {
    //Error_Handler();
  }
}
/*
*
*
*
*
*/
void CalibrationHandler(uint8_t *pilotbuf)
{
  uint8_t ElMotorBuf[8];

  extern CAN_HandleTypeDef hcan1;
  uint32_t TxMailBox; //= CAN_TX_MAILBOX0;
  CAN_TxHeaderTypeDef TxHeader;
  // Передача команды на блок управления приводами
  TxHeader.DLC = 8;
  TxHeader.StdId = 0x0000;
  TxHeader.RTR = CAN_RTR_DATA;
  TxHeader.IDE = CAN_ID_STD;
  TxHeader.TransmitGlobalTime = DISABLE;

  ElMotorBuf[0] = PitchMinMax;
  // Получение данных от наземной станции
  memcpy(&ElMotorBuf[1], &pilotbuf[1], 4);
  if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, ElMotorBuf, &TxMailBox) != HAL_OK)
  {
    //Error_Handler();
  }

  ElMotorBuf[0] = RollMinMax;
  // Получение данных от наземной станции
  memcpy(&ElMotorBuf[1], &pilotbuf[5], 4);
  if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, ElMotorBuf, &TxMailBox) != HAL_OK)
  {
    //Error_Handler();
  }

  ElMotorBuf[0] = CalibComplied;
  if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, ElMotorBuf, &TxMailBox) != HAL_OK)
  {
    //Error_Handler();
  }
}
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
