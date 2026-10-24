/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : freertos.c
 * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32f4xx_hal.h"
#include <stdio.h>
#include "string.h"
#include "usart.h"
#include "iwdg.h"
#include "wifi.h"
#include "ota_receive.h"
#include "stream_buffer.h"
#include "queue.h"
#include "QDTFT_demo.h"
#include "Lcd_Driver.h"
#include "GUI.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/*
 * 心跳位图：每个关键任务每轮循环给自己置一位，
 * feeddog 任务按 1s 窗口检查所有位是否都出现，全齐才喂狗。
 * 这样 IWDG 保护的是“任务级存活”，而不只是“CPU 还在跑”。
 */
#define HB_WIFI         (1U << 0)
#define HB_FLASH        (1U << 1)
#define HB_UPGRADE      (1U << 2)
#define HB_APP          (1U << 3)
#define HB_ALL          (HB_WIFI | HB_FLASH | HB_UPGRADE | HB_APP)

#define HB_CHECK_MS     100U
#define HB_WINDOW_TICKS (1000U / HB_CHECK_MS)   /* 1s 内必须收到所有任务的心跳 */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* 结束帧解析出的固件信息，供 request_upgrade_task 写入 config */
static uint32_t g_firmware_size = 0;
static uint32_t g_firmware_crc  = 0;
static uint32_t g_target_slot   = 0;   /* 本次下载写入的槽：0 = A，1 = B */

/* 各关键任务的心跳位图，由 feeddog 任务按窗口收集 */
static volatile uint32_t g_hb_mask = 0;

/* 固件下载中 / 元数据写入中：两条流程不能同时操作 Flash */
static volatile uint32_t g_ota_busy  = 0;
static volatile uint32_t g_meta_busy = 0;

static void hb_set(uint32_t bit)
{
	taskENTER_CRITICAL();
	g_hb_mask |= bit;
	taskEXIT_CRITICAL();
}

/*
 * 元数据临界区：ota_receive.c 里改用静态缓冲后，同一时刻只允许一处
 * 做"读-改-写"。这里用原子 test-and-set，避免"先判断再置位"的竞态。
 */
static int meta_lock(void)
{
	int ok = 0;

	taskENTER_CRITICAL();
	if (g_meta_busy == 0U)
	{
		g_meta_busy = 1U;
		ok = 1;
	}
	taskEXIT_CRITICAL();
	return ok;
}

static void meta_unlock(void)
{
	g_meta_busy = 0U;
}

/* USER CODE END Variables */
osThreadId ledHandle;
osThreadId oledmenuHandle;
osThreadId wifi_getHandle;
osThreadId ota_flashHandle;
osThreadId request_upgradeHandle;
osThreadId lcdHandle;
osThreadId feeddogTaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void ledTask(void const * argument);
void oledmenuTask(void const * argument);
void wifi_getTask(void const * argument);
void ota_flashTask(void const * argument);
void request_upgradeTask(void const * argument);
void lcd_task(void const * argument);
void feeddog(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

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

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
	/* 创建流缓冲区与队列：必须在任何任务使用它们之前调用 */
	ota_receive_init();
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
	/* xFrameQueue / atQueue 由 ota_receive_init() 创建：元素分别是 fw_frame_t 和 AT_LINE_MAX 字节的行，
	   不能用 CMSIS 消息队列（只支持 1/2/4/8 字节元素） */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of led */
  osThreadDef(led, ledTask, osPriorityNormal, 0, 128);
  ledHandle = osThreadCreate(osThread(led), NULL);

  /* definition and creation of oledmenu */
  osThreadDef(oledmenu, oledmenuTask, osPriorityNormal, 0, 256);
  oledmenuHandle = osThreadCreate(osThread(oledmenu), NULL);

  /* definition and creation of wifi_get */
  osThreadDef(wifi_get, wifi_getTask, osPriorityHigh, 0, 384);
  wifi_getHandle = osThreadCreate(osThread(wifi_get), NULL);

  /* definition and creation of ota_flash */
  osThreadDef(ota_flash, ota_flashTask, osPriorityHigh, 0, 768);
  ota_flashHandle = osThreadCreate(osThread(ota_flash), NULL);

  /* definition and creation of request_upgrade */
  osThreadDef(request_upgrade, request_upgradeTask, osPriorityNormal, 0, 512);
  request_upgradeHandle = osThreadCreate(osThread(request_upgrade), NULL);

  /* definition and creation of lcd */
  osThreadDef(lcd, lcd_task, osPriorityBelowNormal, 0, 256);
  lcdHandle = osThreadCreate(osThread(lcd), NULL);

  /* definition and creation of feeddogTask */
  osThreadDef(feeddogTask, feeddog, osPriorityHigh, 0, 512);
  feeddogTaskHandle = osThreadCreate(osThread(feeddogTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
	/* ota_flashTask 通过这个句柄通知 request_upgradeTask，必须在任务启动前绑定 */
	xrequest_upgrade_taskHandle = request_upgradeHandle;
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_ledTask */
/**
 * @brief  Function implementing the led thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_ledTask */
void ledTask(void const * argument)
{
  /* USER CODE BEGIN ledTask */
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1,GPIO_PIN_RESET);
    osDelay(500);

	/* Infinite loop */
	for (;;)
	{
		hb_set(HB_APP);
		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_1);

		osDelay(500);
	}
  /* USER CODE END ledTask */
}

/* USER CODE BEGIN Header_oledmenuTask */
/**
 * @brief Function implementing the oledmenu thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_oledmenuTask */
void oledmenuTask(void const * argument)
{
  /* USER CODE BEGIN oledmenuTask */
	/* Infinite loop */
	for (;;)
	{
		static const char msg[] = "here is oledmenuTask!\r\n";
		uart_send((const uint8_t *)msg, sizeof(msg) - 1);
		osDelay(10000);
	}
  /* USER CODE END oledmenuTask */
}

/* USER CODE BEGIN Header_wifi_getTask */
/**
 * @brief Function implementing the wifi_get thread.
 * @param argument: Not used
 * @retval None
 */
/* ==================== 逐字节解析帧状态机 + AT 行处理 ==================== */
/* USER CODE END Header_wifi_getTask */
void wifi_getTask(void const * argument)
{
  /* USER CODE BEGIN wifi_getTask */
	/* Infinite loop */

	uint8_t byte;
	uint8_t line[AT_LINE_MAX];
	size_t line_len = 0;
	parser_t parser;

	/* 初始化解析器 */
	memset(&parser, 0, sizeof(parser));
	parser.state = 0; /* ST_HEADER_1 */

	TickType_t last_report = xTaskGetTickCount();

	for (;;)
	{
		hb_set(HB_WIFI);

		/*
		 * 每秒打印一次状态。
		 * 注意：这里不能调用 osDelay()，否则整个循环被拖成 1 字节/秒，
		 * OTA 数据会被 StreamBuffer 丢弃，永远收不到结束帧。
		 */
		if ((xTaskGetTickCount() - last_report) >= pdMS_TO_TICKS(1000))
		{
			char version_str[32];
			last_report = xTaskGetTickCount();

			/* 将 0x00010001 拆解成 V1.1 的字符串 */
			sprintf(version_str, "Version: V%d.%d\r\n",
			        (CURRENT_VERSION >> 16) & 0xFF, CURRENT_VERSION & 0xFF);
			uart_send((uint8_t *)version_str, strlen(version_str));

			static const char msg[] = "here is wifi_getTask!\r\n";
			uart_send((const uint8_t *)msg, sizeof(msg) - 1);
		}

		/* 一次读一个字节；20ms 超时只是为了回到循环顶部打印状态 */
		if (xStreamBufferReceive(uartStreamBuffer, &byte, 1, pdMS_TO_TICKS(20)) != 1)
			continue;

		/* ---------- 状态机 ---------- */
		switch (parser.state)
		{

		/* ===== 状态 0：等 0x55，同时处理 AT 行 ===== */
		case 0: /* ST_HEADER_1 */
			/* AT 行处理：遇到 \n 提交一行 */
			if (byte == '\n' && line_len > 0)
			{
				line[line_len] = '\0';
				xQueueSend(atQueue, line, 0);
				line_len = 0;
			}
			else if (byte != '\r')
			{
				if (line_len < AT_LINE_MAX - 1)
					line[line_len++] = byte;
			}

			/* 检测帧头起点 */
			if (byte == FRAME_HEADER_1)
			{
				parser.state = 1; /* ST_HEADER_2 */
			}
			break;

		/* ===== 状态 1：等 0x44 ===== */
		case 1: /* ST_HEADER_2 */
			if (byte == FRAME_HEADER_2)
			{
				/* 帧头确认，从帧池分配 slot */
				parser.slot = frame_pool_alloc();
				if (parser.slot == NULL)
				{
					parser.state = 0; /* 无空闲 slot，丢弃这一帧 */
					break;
				}
				parser.crc32 = 0;
				parser.crc_byte_idx = 0;
				parser.data_idx = 0;
				parser.state = 2; /* ST_TOTAL */
			}
			else if (byte == FRAME_HEADER_1)
			{
				/* 连续 0x55，保持在 ST_HEADER_2 */
				parser.state = 1;
			}
			else
			{
				/* 不是 0x44，回起点；当前字节按 AT 处理 */
				if (byte == '\n' && line_len > 0)
				{
					line[line_len] = '\0';
					xQueueSend(atQueue, line, 0);
					line_len = 0;
				}
				else if (byte != '\r')
				{
					if (line_len < AT_LINE_MAX - 1)
						line[line_len++] = byte;
				}
				parser.state = 0;
			}
			break;

		/* ===== 状态 2：读总包数 ===== */
		case 2: /* ST_TOTAL */
			parser.total_packets = byte;
			parser.state = 3; /* ST_CURRENT */
			break;

		/* ===== 状态 3：读当前包号 ===== */
		case 3: /* ST_CURRENT */
			parser.current_packet = byte;
			parser.data_len = 0;
			parser.len_byte_idx = 0;
			parser.state = 4; /* ST_LEN */
			break;

		/* ===== 状态 4：读实际长度（2 字节，小端） ===== */
		case 4: /* ST_LEN */
			parser.data_len |= (uint16_t)((uint16_t)byte << (parser.len_byte_idx * 8));
			parser.len_byte_idx++;
			if (parser.len_byte_idx >= 2)
			{
				if (parser.data_len == 0 || parser.data_len > DATA_FRAME_LEN)
				{
					/* 非法长度，回起点，释放 slot */
					frame_pool_free(parser.slot);
					parser.slot = NULL;
					parser.state = 0;
				}
				else
				{
					parser.crc_byte_idx = 0;
					parser.state = 5; /* ST_CRC */
				}
			}
			break;

		/* ===== 状态 5：读 4 字节 CRC32（小端） ===== */
		case 5: /* ST_CRC */
			parser.crc32 |= ((uint32_t)byte) << (parser.crc_byte_idx * 8);
			parser.crc_byte_idx++;
			if (parser.crc_byte_idx >= 4)
			{
				parser.data_idx = 0;
				parser.state = 6; /* ST_DATA */
			}
			break;

		/* ===== 状态 6：读数据（固定 DATA_FRAME_LEN 字节） ===== */
		case 6: /* ST_DATA */
			parser.slot->buf[parser.data_idx++] = byte;
			if (parser.data_idx >= DATA_FRAME_LEN)
			{
				parser.state = 7; /* ST_TAIL_1 */
			}
			break;

		/* ===== 状态 7：等 0x33 ===== */
		case 7: /* ST_TAIL_1 */
			if (byte == FRAME_TAIL_1)
			{
				parser.state = 8; /* ST_TAIL_2 */
			}
			else
			{
				frame_pool_free(parser.slot);
				parser.slot = NULL;
				parser.state = 0;
			}
			break;

		/* ===== 状态 8：等 0x22，帧完成 ===== */
		case 8: /* ST_TAIL_2 */
			if (byte == FRAME_TAIL_2)
			{
				/* 帧完成，校验 CRC */
				parser.crc_calc = calculate_crc32_ex(
					parser.total_packets,
					parser.current_packet,
					parser.data_len,
					parser.slot->buf);

				if (parser.crc_calc == parser.crc32)
				{
					/* CRC 通过，交给 Flash 写入 */
					fw_frame_t frame;
					frame.total_packets = parser.total_packets;
					frame.current_packet = parser.current_packet;
					frame.data_len = parser.data_len;
					frame.crc32 = parser.crc32;
					frame.slot = parser.slot;

					if (xQueueSend(xFrameQueue, &frame, 0) != pdTRUE)
					{
						/* 队列满，归还 slot */
						frame_pool_free(parser.slot);
					}
					/* 注意：slot 在这里不 free，由 Flash 任务 free */
				}
				else
				{
					/* CRC 失败，归还 slot */
					frame_pool_free(parser.slot);
				}
				parser.slot = NULL;
				parser.state = 0;
				line_len = 0;
			}
			else
			{
				frame_pool_free(parser.slot);
				parser.slot = NULL;
				parser.state = 0;
			}
			break;

		default:
			parser.state = 0;
			break;
		}
	}
  /* USER CODE END wifi_getTask */
}

/* USER CODE BEGIN Header_ota_flashTask */
/**
 * @brief Function implementing the ota_flash thread.
 * @param argument: Not used
 * @retval None
 */
/* ==================== Flash 写任务 ==================== */
/* USER CODE END Header_ota_flashTask */
void ota_flashTask(void const * argument)
{
  /* USER CODE BEGIN ota_flashTask */
	/* Infinite loop */

	fw_frame_t frame;
	uint32_t write_addr;
	uint32_t received_count = 0;        /* 已成功写入的数据帧数，用于收齐校验 */
	uint32_t erased_sector_base = 0xFFFFFFFFU;  /* 本次传输已擦除的扇区基址 */
	uint32_t target_base = 0;           /* 本次传输的目标槽基址 */
	uint32_t target_slot = 0;           /* 本次传输的目标槽：0 = A，1 = B */

	(void)argument;

	for (;;)
	{
		hb_set(HB_FLASH);

		/* 200ms 超时只是为了周期性回到循环顶部打卡，收帧仍按队列事件触发 */
		if (xQueueReceive(xFrameQueue, &frame, pdMS_TO_TICKS(200)) != pdTRUE)
			continue;

		/* 判断是否是结束帧 */
		if (frame.current_packet == PACKET_END)
		{
			uint32_t fw_size = 0;
			uint32_t fw_crc = 0;
			int verified = 0;

			/* 结束帧负载：data[0:4]=固件总长(小端), data[4:8]=整包 CRC32(小端) */
			if (frame.data_len >= 8)
			{
				fw_size = (uint32_t)frame.slot->buf[0] | ((uint32_t)frame.slot->buf[1] << 8) | ((uint32_t)frame.slot->buf[2] << 16) | ((uint32_t)frame.slot->buf[3] << 24);
				fw_crc = (uint32_t)frame.slot->buf[4] | ((uint32_t)frame.slot->buf[5] << 8) | ((uint32_t)frame.slot->buf[6] << 16) | ((uint32_t)frame.slot->buf[7] << 24);
			}
			frame_pool_free(frame.slot);

			/*
			 * 全量校验：收齐包数 + 直接读回目标槽算整包 CRC + 向量表检查。
			 * 只有这三项都通过，才允许把 active_slot 切到新槽。
			 */
			if (target_base != 0U && fw_size != 0U && fw_size <= SLOT_CAPACITY &&
			    received_count == frame.total_packets)
			{
				uint32_t calc_crc = ota_crc32_bytes((const void *)target_base, fw_size);

				if (calc_crc == fw_crc && ota_image_vectors_valid(target_base))
				{
					g_firmware_size = fw_size;
					g_firmware_crc  = fw_crc;
					g_target_slot   = target_slot;
					verified = 1;
				}
			}

			if (verified)
			{
				xTaskNotify(xrequest_upgrade_taskHandle, 0x01, eSetValueWithOverwrite);
			}
			else
			{
				char err[72];
				int n = sprintf(err, "ota verify fail! recv=%lu/%u size=%lu\r\n",
				                (unsigned long)received_count,
				                (unsigned int)frame.total_packets,
				                (unsigned long)fw_size);
				uart_send((const uint8_t *)err, (uint32_t)n);
			}

			received_count = 0;
			erased_sector_base = 0xFFFFFFFFU;
			target_base = 0;   /* 等待下一轮的包 0 */
			g_ota_busy = 0;
			continue;          /* 结束帧不写 Flash，直接继续 */
		}

		/* 新一轮传输开始：目标槽 = 当前运行槽之外的另一个槽 */
		if (frame.current_packet == 0)
		{
			received_count = 0;
			erased_sector_base = 0xFFFFFFFFU;
			g_ota_busy = 1;    /* 下载期间禁止 feeddog 写元数据确认 */

			/*
			 * 只用“本镜像的链接地址”判断当前运行槽，不去猜元数据。
			 * 这样无论元数据是否损坏、是否与实际运行槽不一致，
			 * 都绝不会把正在运行的固件擦掉。
			 */
			target_slot = (uint32_t)(APP_IMAGE_SLOT ^ 1U);
			target_base = (target_slot == 0U) ? SLOT_A_BASE : SLOT_B_BASE;
		}

		if (target_base == 0U)
		{
			/* 没收到包 0，无法确定目标槽，丢弃 */
			frame_pool_free(frame.slot);
			continue;
		}

		write_addr = target_base + ((uint32_t)frame.current_packet * DATA_FRAME_LEN);

		/* 边界保护：不能写穿目标槽容量 */
		if (write_addr + DATA_FRAME_LEN > (target_base + SLOT_CAPACITY))
		{
			frame_pool_free(frame.slot);
			continue;
		}

		/* 等 feeddog 的元数据写入结束，避免两条流程同时操作 Flash */
		while (g_meta_busy)
		{
			osDelay(10);
		}

		{
			uint32_t sector_base = get_sector_base(write_addr);

			/* 首次写到一个扇区时才擦除；擦除单位取该扇区的真实大小（16/64/128KB） */
			if (sector_base != erased_sector_base)
			{
				uint32_t sector_size = get_sector_size(get_sector(sector_base));

				if (flash_erase(sector_base, sector_size) != Status_OK)
				{
					frame_pool_free(frame.slot);
					continue;
				}
				erased_sector_base = sector_base;
			}
		}

		/* 写 Flash，只写实际长度，不写 padding */
		if (flash_write(write_addr, frame.slot->buf, frame.data_len) == Status_OK)
			received_count++; /* 写入成功，计入已收帧数 */

		/* 归还 slot */
		frame_pool_free(frame.slot);
	}
  /* USER CODE END ota_flashTask */
}

/* USER CODE BEGIN Header_request_upgradeTask */
/**
 * @brief Function implementing the request_upgrade thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_request_upgradeTask */
void request_upgradeTask(void const * argument)
{
  /* USER CODE BEGIN request_upgradeTask */
	/* Infinite loop */

	(void)argument;

	for (;;)
	{
		uint8_t current_slot;
		uint8_t target_slot;
		int ret;

		hb_set(HB_UPGRADE);

		/* 200ms 超时用于周期打卡；只有收到通知才继续升级流程 */
		if (xTaskNotifyWait(0, 0, NULL, pdMS_TO_TICKS(200)) != pdTRUE)
			continue;

		if (xWifisendTaskHandle != NULL)
			xTaskNotify(xWifisendTaskHandle, 0x02, eSetValueWithOverwrite);

		target_slot = (uint8_t)g_target_slot;
		if (target_slot > 1U || g_firmware_size == 0U)
			continue;

		/* 当前运行槽一定是目标槽的另一半 */
		current_slot = (uint8_t)(target_slot ^ 1U);

		/*
		 * 切换标志：active_slot 指向新槽，Bootloader 下次启动直接跳过去。
		 * 元数据整块读写都在 ota_receive.c 内部用静态缓冲完成，这里不再占栈。
		 */
		while (!meta_lock())
			osDelay(10);
		ret = ota_meta_request_switch(current_slot, target_slot,
		                              g_firmware_size, g_firmware_crc);
		meta_unlock();

		if (ret == 0)
		{
			static const char ok[] = "upgrade ok, reboot to new slot...\r\n";
			uart_send((const uint8_t *)ok, sizeof(ok) - 1);
			osDelay(50);
			NVIC_SystemReset();
		}
		else
		{
			static const char err[] = "meta save fail!\r\n";
			uart_send((const uint8_t *)err, sizeof(err) - 1);
		}
	}
  /* USER CODE END request_upgradeTask */
}

/* USER CODE BEGIN Header_lcd_task */
/**
* @brief Function implementing the lcd thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_lcd_task */
void lcd_task(void const * argument)
{
  /* USER CODE BEGIN lcd_task */
  
  /* Infinite loop */
  for(;;)
  {
	// QDTFT_Test_Demo();
    osDelay(10);
  }
  /* USER CODE END lcd_task */
}

/* USER CODE BEGIN Header_feeddog */
/**
* @brief Function implementing the feeddogTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_feeddog */
void feeddog(void const * argument)
{
  /* USER CODE BEGIN feeddog */
	uint32_t seen = 0;
	uint32_t ticks = 0;
	uint32_t healthy_ms = 0;
	int confirmed = 0;
	uint8_t mem_reported = 0;
	uint16_t up_sec = 0;

	(void)argument;

	/*
	 * 每 100ms 收集一次心跳，按 1s 窗口判定：
	 *   - 窗口内四个关键任务都打过卡 → 喂狗；
	 *   - 有任务缺席 → 不喂，交给 IWDG 复位（新固件会被 Bootloader 回滚）。
	 * 连续 5s 健康后，确认本次试运行成功，清掉 trial_pending。
	 */
	for (;;)
	{
		taskENTER_CRITICAL();
		seen |= g_hb_mask;
		g_hb_mask = 0;
		taskEXIT_CRITICAL();

		if (++ticks >= HB_WINDOW_TICKS)
		{
			/*
			 * 上电 10s 打印一次内存余量：
			 *   heap  = 当前剩余 / 历史最低剩余（字节）
			 *   stack = 各任务栈自启动以来的最小剩余量（word），越小越接近溢出
			 */
			if (!mem_reported && ++up_sec >= 10U)
			{
				char mb[160];

				mem_reported = 1;
				(void)snprintf(mb, sizeof(mb), "[mem] heap free=%u min=%u\r\n",
				               (unsigned)xPortGetFreeHeapSize(),
				               (unsigned)xPortGetMinimumEverFreeHeapSize());
				boot_log(mb);

				(void)snprintf(mb, sizeof(mb),
				               "[mem] stack left(w): led=%u oled=%u wifi=%u flash=%u upg=%u lcd=%u dog=%u\r\n",
				               (unsigned)uxTaskGetStackHighWaterMark(ledHandle),
				               (unsigned)uxTaskGetStackHighWaterMark(oledmenuHandle),
				               (unsigned)uxTaskGetStackHighWaterMark(wifi_getHandle),
				               (unsigned)uxTaskGetStackHighWaterMark(ota_flashHandle),
				               (unsigned)uxTaskGetStackHighWaterMark(request_upgradeHandle),
				               (unsigned)uxTaskGetStackHighWaterMark(lcdHandle),
				               (unsigned)uxTaskGetStackHighWaterMark(feeddogTaskHandle));
				boot_log(mb);
			}

			if (seen == HB_ALL)
			{
				HAL_IWDG_Refresh(&hiwdg);

				if (!confirmed)
				{
					healthy_ms += 1000U;
					/* 下载/写元数据期间不确认，避免和 ota_flashTask 抢 Flash */
					if (healthy_ms >= OTA_CONFIRM_WINDOW_MS && !g_ota_busy && meta_lock())
					{
						int ret;
						static const char c1[] = "[hb] confirm begin\r\n";
						static const char c2[] = "[hb] confirm done\r\n";
						/* 诊断打印一律走轮询，避免依赖串口互斥锁与 DMA */
						boot_log(c1);

						ret = ota_app_confirm_healthy(APP_IMAGE_BASE);
						meta_unlock();

						boot_log(c2);

						if (ret >= 0)
						{
							confirmed = 1;
							if (ret == 1)
							{
								static const char msg[] = "trial confirmed\r\n";
								uart_send((const uint8_t *)msg, sizeof(msg) - 1);
							}
						}
					}
				}
			}
			else
			{
				/* 有任务缺席：打印缺了哪一位（bit0=WIFI bit1=FLASH bit2=UPGRADE bit3=APP） */
				char hb_line[32];
				(void)snprintf(hb_line, sizeof(hb_line), "[hb] miss=%02lX\r\n",
				               (unsigned long)(HB_ALL & ~seen));
				boot_log(hb_line);

				/* 有任务缺席：清零健康计时，且本窗口不喂狗 */
				healthy_ms = 0;
			}

			seen = 0;
			ticks = 0;
		}

		osDelay(HB_CHECK_MS);
	}
  /* USER CODE END feeddog */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/*
 * FreeRTOS 断言钩子。
 * 常见触发原因：在优先级不合法（数值 < configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY）
 * 的中断里调用了 FromISR API、在调度器挂起时调用了阻塞 API、队列/信号量句柄为空。
 * 这里直接轮询 USART1 打印行号，不依赖互斥锁与中断。
 */
void vAssertCalled(uint32_t line)
{
	char buf[40];
	(void)snprintf(buf, sizeof(buf), "\r\n[ASSERT] line=%lu\r\n", (unsigned long)line);
	boot_log(buf);
	for (;;)
	{
	}
}

/*
 * 栈溢出钩子。ota_config_t 的读写已经改成静态缓冲，调用方不再在栈上复制
 * 302 字节结构体；这里保留钩子作为兜底，出现即说明某个任务栈给小了。
 */
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
	char buf[64];
	(void)xTask;
	(void)snprintf(buf, sizeof(buf), "\r\n[STACK] overflow task=%s\r\n", pcTaskName);
	boot_log(buf);
	for (;;)
	{
	}
}

/* USER CODE END Application */
