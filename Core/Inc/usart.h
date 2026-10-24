/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.h
  * @brief   This file contains all the function prototypes for
  *          the usart.c file
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USART_H__
#define __USART_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
#include "stm32f4xx_hal.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "stream_buffer.h"
/* USER CODE END Includes */

extern UART_HandleTypeDef huart1;

extern UART_HandleTypeDef huart2;

/* USER CODE BEGIN Private defines */

//#define EN_USART1_RX  1   // 1 = 启用串口接收中断；0 = 不启用
#define CercleBufferSize  512         // DMA 环形接收缓冲区大小（必须远大于单帧 268 字节）
#define DATA_FRAME_LEN      256          // 每包数据固定 256 字节
#define FRAME_POOL_SIZE     4            // 帧池里 4 个 slot
#define FRAME_QUEUE_DEPTH   4            // 帧队列深度
#define AT_LINE_MAX         128          // AT 行最大长度

/* 帧格式常量 */
#define FRAME_HEADER_1      0x55
#define FRAME_HEADER_2      0x44
#define FRAME_TAIL_1        0x33
#define FRAME_TAIL_2        0x22

/* 帧总长（固定）：2(头) + 1(总) + 1(当前) + 2(实长) + 4(CRC) + 256(数据) + 2(尾) */
#define FRAME_TOTAL_LEN     (2 + 1 + 1 + 2 + 4 + DATA_FRAME_LEN + 2)   // = 268

/* 结束帧的特殊包号 */
#define PACKET_END          0xFF
/* USER CODE END Private defines */

void MX_USART1_UART_Init(void);
void MX_USART2_UART_Init(void);

/* USER CODE BEGIN Prototypes */
extern uint8_t           CercleBuffer[CercleBufferSize];   /* DMA 环形接收缓冲区 */
extern volatile uint16_t read_pos;                         /* 已取出数据的读位置 */

void uart_send(const uint8_t *data, uint16_t len);         /* 带互斥保护的串口发送 */

/* USART1 = 调试口（向 PC 输出日志）；USART2 = ESP8266（AT + 透传） */
void uart2_send(const uint8_t *data, uint16_t len);
void uart2_send_bytes(const char *s, uint16_t len);
void uart2_send_at(const char *cmd);

/* 启动打点：调度器启动前可用，直接阻塞发送到 USART1 */
void boot_log(const char *s);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __USART_H__ */

