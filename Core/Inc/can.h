/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    can.h
  * @brief   This file contains all the function prototypes for
  *          the can.c file
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
#ifndef __CAN_H__
#define __CAN_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern CAN_HandleTypeDef hcan1;

/* USER CODE BEGIN Private defines */

/* 11 位诊断风格寻址：请求 0x600+node，响应 0x580+node，node 1~15 */
#define CAN_IF_REQ_BASE_ID      0x600U
#define CAN_IF_RESP_BASE_ID     0x580U
#define CAN_IF_NODE_MIN         1U
#define CAN_IF_NODE_MAX         15U

/* 上电做一次 Loopback 自测（不接收发器也能验证 CAN 外设链路），量产可关掉 */
#ifndef CAN_IF_SELFTEST_ON_BOOT
#define CAN_IF_SELFTEST_ON_BOOT 1
#endif

/*
 * 错误中断开关：默认关闭。
 * bxCAN 的 Error Passive / Bus-Off 标志是只读的，清不掉；节点一旦进入错误被动，
 * SCE 中断就会持续触发形成中断风暴，把任务和喂狗全部饿死。
 * 只有在总线上真的接了节点、并且确认错误回调会及时关中断时才打开。
 */
#ifndef CAN_IF_ENABLE_ERROR_IRQ
#define CAN_IF_ENABLE_ERROR_IRQ 0
#endif

typedef struct {
    uint32_t id;        /* 标准帧为 11 位 StdId，扩展帧为 29 位 ExtId */
    uint8_t  dlc;
    uint8_t  ide;       /* CAN_ID_STD / CAN_ID_EXT */
    uint8_t  data[8];
} can_frame_t;

/* USER CODE END Private defines */

void MX_CAN1_Init(void);

/* USER CODE BEGIN Prototypes */

/* 配置过滤器、启动 CAN、打开 FIFO0 接收通知。 */
int      can_if_init(void);

/* 调度器启动后运行的 CAN 初始化任务：初始化 + 回环自测，完成后自删除 */
void     can_start_task(void *arg);

/* 发送一帧标准帧。timeout_ms 内没有空闲邮箱则返回 -2。 */
int      can_if_send(uint32_t std_id, const uint8_t *data, uint8_t dlc, uint32_t timeout_ms);

/* 从接收队列取一帧，超时返回 -3。必须在调度器启动后调用。 */
int      can_if_recv(can_frame_t *out, uint32_t timeout_ms);

/* 回环自测，返回 0 表示 3 帧全部正确回环 */
int      can_if_selftest_loopback(void);

/*
 * 重新初始化 CAN 到指定模式（CAN_MODE_NORMAL / CAN_MODE_LOOPBACK）。
 * enable_irq=1 时打开 FIFO0 接收中断。
 * 供自测临时切模式用，调用方必须成对恢复成 (CAN_MODE_NORMAL, 1)。
 */
int      can_if_reinit(uint32_t mode, uint8_t enable_irq);

/* 诊断计数：队列溢出 / 错误中断 / Bus-Off */
uint32_t can_if_rx_drop_count(void);
uint32_t can_if_error_count(void);
uint32_t can_if_busoff_count(void);

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __CAN_H__ */

