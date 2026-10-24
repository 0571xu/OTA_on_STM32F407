#ifndef WIFI_H
#define WIFI_H

#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include <string.h>
#include "usart.h"
#include "ota_receive.h"   /* AT_LINE_MAX / atQueue / TickType_t 等 */

/*
 * ESP8266 走 USART2（PD5/PD6），USART1 只做调试输出。
 * 实现在 MDK-ARM/wifi.c（该目录不在 IncludePath 里，所以头文件放在 Core/Inc）。
 */
int  wait_response(const char *expect, uint32_t timeout_ms);
int  wait_connect_response(uint32_t timeout_ms);
void connect_init(int status);

#endif
