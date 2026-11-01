/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    can.c
  * @brief   This file provides code for the configuration
  *          of the CAN instances.
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
#include "can.h"

/* USER CODE BEGIN 0 */
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "usart.h"
#include "isotp.h"
#include <string.h>
#include <stdio.h>

/*
 * CAN 接口层（F407 作为 UDS 客户端 / OTA 主站）
 *
 * 接收：FIFO0 中断 -> FreeRTOS 队列，任务侧用 can_if_recv() 阻塞取帧。
 * 发送：3 个发送邮箱，满了先等空位，超时返回错误。
 *
 * 过滤器只放行 0x580~0x58F（节点响应帧），请求帧 0x601~0x60F 由本机发出，
 * 不需要接收，因此没必要占用接收带宽。
 */
#define CAN_IF_RX_QUEUE_DEPTH   32U

static QueueHandle_t     s_canRxQueue   = NULL;
static volatile uint32_t s_canRxDrop    = 0U;
static volatile uint32_t s_canErrCnt    = 0U;
static volatile uint32_t s_canBusOffCnt = 0U;

/* USER CODE END 0 */

CAN_HandleTypeDef hcan1;

/* CAN1 init function */
void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 6;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_11TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = ENABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = ENABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = ENABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */
  boot_log("[boot] 4 can msp ok\r\n");
  /* USER CODE END CAN1_Init 2 */

}

void HAL_CAN_MspInit(CAN_HandleTypeDef* canHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspInit 0 */

  /* USER CODE END CAN1_MspInit 0 */
    /* CAN1 clock enable */
    __HAL_RCC_CAN1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**CAN1 GPIO Configuration
    PA11     ------> CAN1_RX
    PA12     ------> CAN1_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* CAN1 interrupt Init */
    HAL_NVIC_SetPriority(CAN1_TX_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(CAN1_TX_IRQn);
    HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
    HAL_NVIC_SetPriority(CAN1_SCE_IRQn, 7, 0);
    HAL_NVIC_EnableIRQ(CAN1_SCE_IRQn);
  /* USER CODE BEGIN CAN1_MspInit 1 */

  /* USER CODE END CAN1_MspInit 1 */
  }
}

void HAL_CAN_MspDeInit(CAN_HandleTypeDef* canHandle)
{

  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspDeInit 0 */

  /* USER CODE END CAN1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_CAN1_CLK_DISABLE();

    /**CAN1 GPIO Configuration
    PA11     ------> CAN1_RX
    PA12     ------> CAN1_TX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_11|GPIO_PIN_12);

    /* CAN1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(CAN1_TX_IRQn);
    HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);
    HAL_NVIC_DisableIRQ(CAN1_SCE_IRQn);
  /* USER CODE BEGIN CAN1_MspDeInit 1 */

  /* USER CODE END CAN1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

/* 自测阶段调度器还没启动，只能用轮询发送；正常运行时用 vTaskDelay 让出 CPU */
static void can_if_delay_ms(uint32_t ms)
{
  if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
  {
    vTaskDelay(pdMS_TO_TICKS(ms) == 0U ? 1U : pdMS_TO_TICKS(ms));
  }
  else
  {
    HAL_Delay(ms);
  }
}

static void can_if_print(const char *s)
{
  uart_send((const uint8_t *)s, (uint16_t)strlen(s));
}

/*
 * CAN 启动任务：在调度器启动后执行初始化与自测。
 * 放在任务里而不是 main 里，是为了让 CAN 的任何异常都不会卡住系统启动。
 */
void can_start_task(void *arg)
{
  char line[64];
  int rc;

  (void)arg;

  /* 调度器已启动、全部任务已创建，这里的余量才是真实可用的堆空间 */
  (void)snprintf(line, sizeof(line), "[can] heap free=%u\r\n",
                 (unsigned)xPortGetFreeHeapSize());
  can_if_print(line);

  rc = can_if_init();
  if (rc != 0)
  {
    (void)snprintf(line, sizeof(line), "[can] init fail rc=%d\r\n", rc);
    can_if_print(line);
    vTaskDelete(NULL);
    return;
  }

#if CAN_IF_SELFTEST_ON_BOOT
  (void)can_if_selftest_loopback();
#endif

#if ISOTP_SELFTEST_ON_BOOT
  {
    /* ISO-TP 自测：拆包 / 序号循环 / 长度字段 / 流控解析，全部在回环里完成 */
    char l[56];
    int  n = isotp_selftest();

    if (n > 0) {
      (void)snprintf(l, sizeof(l), "[iso] selftest ok frames=%d\r\n", n);
    } else {
      (void)snprintf(l, sizeof(l), "[iso] selftest FAIL rc=%d\r\n", n);
    }
    can_if_print(l);
  }
#endif

  vTaskDelete(NULL);
}

/* 32 位掩码过滤器：STID 在 [31:21]，IDE/RTR 为 0，低 4 位忽略 -> 0x580~0x58F */
static int can_if_config_filter(void)
{
  CAN_FilterTypeDef f;

  memset(&f, 0, sizeof(f));
  f.FilterBank           = 0U;
  f.FilterMode           = CAN_FILTERMODE_IDMASK;
  f.FilterScale          = CAN_FILTERSCALE_32BIT;
  f.FilterIdHigh         = (uint16_t)((CAN_IF_RESP_BASE_ID << 5) & 0xFFFFU);
  f.FilterIdLow          = 0x0000U;
  f.FilterMaskIdHigh     = (uint16_t)((0x7F0U << 5) & 0xFFFFU);
  f.FilterMaskIdLow      = 0x0000U;
  f.FilterFIFOAssignment = CAN_RX_FIFO0;
  f.FilterActivation     = ENABLE;

  return (HAL_CAN_ConfigFilter(&hcan1, &f) == HAL_OK) ? 0 : -1;
}

/* 以指定模式重新初始化 CAN：先停再配，避免 HAL 状态机拒绝 */
static int can_if_start(uint32_t mode, uint8_t enable_irq)
{
  (void)HAL_CAN_DeactivateNotification(&hcan1,
                                       CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_ERROR);
  (void)HAL_CAN_Stop(&hcan1);

  hcan1.Init.Mode = mode;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)          return -1;
  if (can_if_config_filter() != 0)             return -2;
  if (HAL_CAN_Start(&hcan1) != HAL_OK)         return -3;

  if (enable_irq)
  {
    uint32_t its = CAN_IT_RX_FIFO0_MSG_PENDING;

#if CAN_IF_ENABLE_ERROR_IRQ
    its |= CAN_IT_ERROR;
#endif

    if (HAL_CAN_ActivateNotification(&hcan1, its) != HAL_OK)
      return -4;
  }
  return 0;
}

int can_if_init(void)
{
  if (s_canRxQueue == NULL)
  {
    /* 调度器启动前创建，安全；之后不再重建 */
    s_canRxQueue = xQueueCreate(CAN_IF_RX_QUEUE_DEPTH, sizeof(can_frame_t));
    if (s_canRxQueue == NULL) return -1;
  }
  return can_if_start(CAN_MODE_NORMAL, 1U);
}

int can_if_reinit(uint32_t mode, uint8_t enable_irq)
{
  return can_if_start(mode, enable_irq);
}

int can_if_send(uint32_t std_id, const uint8_t *data, uint8_t dlc, uint32_t timeout_ms)
{
  CAN_TxHeaderTypeDef hdr;
  uint32_t mailbox = 0U;
  uint32_t start   = HAL_GetTick();

  if ((data == NULL) || (dlc > 8U)) return -1;
  if (std_id > 0x7FFU)              return -1;

  memset(&hdr, 0, sizeof(hdr));
  hdr.StdId              = std_id;
  hdr.IDE                = CAN_ID_STD;
  hdr.RTR                = CAN_RTR_DATA;
  hdr.DLC                = dlc;
  hdr.TransmitGlobalTime = DISABLE;

  while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0U)
  {
    if ((HAL_GetTick() - start) >= timeout_ms) return -2;
    can_if_delay_ms(1U);
  }

  if (HAL_CAN_AddTxMessage(&hcan1, &hdr, (uint8_t *)data, &mailbox) != HAL_OK) return -3;

  /*
   * AutoRetransmission is enabled in CAN1_Init, so a frame that nobody
   * acknowledges is retried by the hardware indefinitely and keeps its
   * mailbox occupied. The bus then stays jammed and every later send fails
   * with "no free mailbox", which looks exactly like "the other node is
   * dead". Abort the request once the caller's timeout expires.
   */
  start = HAL_GetTick();
  while (HAL_CAN_IsTxMessagePending(&hcan1, mailbox) != 0U)
  {
    if ((HAL_GetTick() - start) >= timeout_ms)
    {
      (void)HAL_CAN_AbortTxRequest(&hcan1, mailbox);
      return -4;
    }
    can_if_delay_ms(1U);
  }

  return 0;
}

int can_if_recv(can_frame_t *out, uint32_t timeout_ms)
{
  if ((s_canRxQueue == NULL) || (out == NULL)) return -1;
  if (xTaskGetSchedulerState() == taskSCHEDULER_NOT_STARTED) return -2;

  return (xQueueReceive(s_canRxQueue, out, pdMS_TO_TICKS(timeout_ms)) == pdTRUE)
         ? 0 : -3;
}

/*
 * 回环自测：不接收发器也能验证「位时序 + 过滤器 + 邮箱 + FIFO」这条链路。
 * 流程：切 Loopback -> 发 3 帧并逐帧比对 -> 切回 Normal 并恢复中断。
 * 返回 0 表示 3 帧全部正确回环。
 */
int can_if_selftest_loopback(void)
{
  uint8_t tx[8];
  uint8_t rx[8];
  CAN_RxHeaderTypeDef hdr;
  uint32_t sent = 0U;
  uint32_t got  = 0U;
  uint32_t i;
  char line[48];
  int rc;

  rc = can_if_start(CAN_MODE_LOOPBACK, 0U);
  if (rc == 0)
  {
    for (i = 0U; i < 3U; i++)
    {
      uint32_t t0;
      uint32_t j;

      for (j = 0U; j < 8U; j++) tx[j] = (uint8_t)(0xA0U + i + j);

      if (can_if_send((uint32_t)(CAN_IF_RESP_BASE_ID + 1U), tx, 8U, 50U) != 0)
        continue;
      sent++;

      t0 = HAL_GetTick();
      while ((HAL_GetTick() - t0) < 50U)
      {
        if (HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0) > 0U)
        {
          if (HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &hdr, rx) == HAL_OK)
          {
            if ((hdr.StdId == (CAN_IF_RESP_BASE_ID + 1U)) &&
                (hdr.DLC == 8U) && (memcmp(rx, tx, 8U) == 0))
            {
              got++;
            }
          }
          break;
        }
        can_if_delay_ms(1U);
      }
    }
  }

  /* 无论成功失败都要回到 Normal，否则后面收不到任何响应帧 */
  rc = can_if_start(CAN_MODE_NORMAL, 1U);

  (void)snprintf(line, sizeof(line), "[can] loopback %lu/%lu%s\r\n",
                 (unsigned long)got, (unsigned long)sent,
                 (rc == 0) ? "" : " mode-restore-fail");
  can_if_print(line);

  return ((sent == 3U) && (got == 3U) && (rc == 0)) ? 0 : -1;
}

uint32_t can_if_rx_drop_count(void)   { return s_canRxDrop; }
uint32_t can_if_error_count(void)     { return s_canErrCnt; }
uint32_t can_if_busoff_count(void)    { return s_canBusOffCnt; }

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  CAN_RxHeaderTypeDef hdr;
  can_frame_t f;
  BaseType_t woken = pdFALSE;

  if ((hcan->Instance != CAN1) || (s_canRxQueue == NULL)) return;

  /* 一次把 FIFO0 里的帧全部搬走，避免中断反复进出 */
  while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0U)
  {
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &hdr, f.data) != HAL_OK) break;

    f.id  = (hdr.IDE == CAN_ID_STD) ? hdr.StdId : hdr.ExtId;
    f.dlc = (uint8_t)hdr.DLC;
    f.ide = (uint8_t)hdr.IDE;

    if (xQueueSendFromISR(s_canRxQueue, &f, &woken) != pdTRUE)
    {
      s_canRxDrop++;   /* 队列满：说明任务侧没跟上，丢弃并计数 */
    }
  }

  portYIELD_FROM_ISR(woken);
}

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
{
  if (hcan->Instance != CAN1) return;

  s_canErrCnt++;

  if ((hcan->Instance->ESR & CAN_ESR_BOFF) != 0U)
  {
    s_canBusOffCnt++;
  }

  /*
   * 关键防御：ESR 的 EPVF / BOFF 是只读标志，节点处于错误被动或 Bus-Off 时
   * 一直为 1，HAL 无法清除。若错误中断保持使能，SCE 中断会持续触发形成中断风暴，
   * 把任务和喂狗全部饿死（现象是串口输出突然停止、约 8 秒后被 IWDG 复位）。
   * 这里直接关掉错误中断，只保留计数，等总线故障排除后再手动重新使能。
   */
  __HAL_CAN_DISABLE_IT(hcan, CAN_IT_ERROR);
}

/* USER CODE END 1 */

