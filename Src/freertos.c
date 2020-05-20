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
#include "cmsis_os.h"
#include "usb_device.h"
#include "cli.h"
#include "api.h"
#include "lwip.h"

#include "confighawk.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
uint8_t net_state = 0; // Состояние подключения
/* USER CODE END Variables */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

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
  extern struct netif gnetif;
  uint8_t PhyStat;
  
  /* Infinite loop */
  for (;;)
  {
    PhyStat = gnetif.flags; //  Статистика подключения
    if(PhyStat != 15)
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
/* USER CODE BEGIN Header_StartNetTest */
/**
* @brief Function implementing the NetTestTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartNetTest */
void StartNetTest(void const *argument)
{
  /* USER CODE BEGIN StartNetTest */
  
  /* Infinite loop */
  for (;;)
  {
   
    vTaskDelay(200);
  }

  /* USER CODE END StartNetTest */
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
  struct netconn *nc;
  struct netbuf *nb;
  volatile uint16_t len;
  char buf[1024];

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
      ip4addr_aton("192.168.168.106", &remote_ip);
      nc = netconn_new(NETCONN_TCP);
      if (nc != NULL)
      {
        res = netconn_bind(nc, &local_ip, 8889);
        if (res == ERR_OK)
        {
          res = netconn_connect(nc, &remote_ip, 8889);
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
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
