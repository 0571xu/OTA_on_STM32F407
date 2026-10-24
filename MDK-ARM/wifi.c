#include "wifi.h"

/*
 * ESP8266 AT 配置流程（从 F103 工程移植）
 *
 * 串口约定：USART1 只负责向 PC 输出调试信息，ESP8266 接在 USART2（PD5/PD6）。
 * 因此本文件所有发送都走 uart2_send_*，不再使用 uart1_send_*。
 *
 * 三个函数：
 *   wait_response()         等待某个关键字（OK / > / 自定义）
 *   wait_connect_response() 解析 AT+CIPSTATUS 的 STATUUS/STATUS 字段
 *   connect_init()          退透传 -> AT 初始化 -> 连 WiFi -> 连 TCP -> 进透传
 */

int wait_response(const char *expect, uint32_t timeout_ms)
{
    char line[AT_LINE_MAX];
    TickType_t start = xTaskGetTickCount();

    while ((xTaskGetTickCount() - start) < pdMS_TO_TICKS(timeout_ms))
    {
        if (xQueueReceive(atQueue, line, pdMS_TO_TICKS(100)) == pdTRUE)
        {
            if (strstr(line, expect) != NULL)   return 1;
            if (strstr(line, "ERROR") != NULL ||
                strstr(line, "FAIL")  != NULL)  return 0;
        }
    }
    return 0;
}

/* 返回值说明：
 *  3  -> STATUS:3（已建立 TCP/UDP 连接）
 *  1  -> STATUS:1 或 5（未连上 WiFi / 未获取 IP）
 *  2  -> STATUS:2 或 4（已获取 IP 但未连 TCP，或连接已断开）
 * -1  -> 模块返回 ERROR / FAIL
 *  0  -> 超时
 */
int wait_connect_response(uint32_t timeout_ms)
{
    char line[AT_LINE_MAX];
    TickType_t start = xTaskGetTickCount();

    while ((xTaskGetTickCount() - start) < pdMS_TO_TICKS(timeout_ms))
    {
        if (xQueueReceive(atQueue, line, pdMS_TO_TICKS(500)) == pdTRUE)
        {
            char *p = strstr(line, "STATUS:");
            if (p != NULL && strlen(p) > 7)
            {
                char st = p[7];
                if (st == '3')              return 3;
                if (st == '1' || st == '5') return 1;
                if (st == '2' || st == '4') return 2;
            }

            if (strstr(line, "ERROR") != NULL || strstr(line, "FAIL") != NULL)
                return -1;
        }
    }
    return 0;
}

void connect_init(int status)
{
    int i = status;
    int retry = 0;

    /* ---------- 1. 退出透传 ---------- */
    uart2_send_bytes("+++", 3);
    if (wait_response("+++", 1500) != 1)
    {
        vTaskDelay(pdMS_TO_TICKS(500));
        uart2_send_bytes("+++", 3);
        (void)wait_response("+++", 1500);
    }
    vTaskDelay(pdMS_TO_TICKS(500));

    /* ---------- 2. 发送 AT 命令配置 ---------- */
    while (i != 3 && retry < 3)
    {
        if (i == 0)
        {
            uart2_send_at("ATE0");
            (void)wait_response("OK", 1500);

            uart2_send_at("AT+UART=115200,8,1,0,1");
            while (!wait_response("OK", 1500)) { }

            uart2_send_at("AT+CWMODE=1");
            while (!wait_response("OK", 1500)) { }
            i++;
        }

        if (i == 1)
        {
            uart2_send_at("AT+CWJAP=\"TP-LINK_3372\",\"12345678\"");
            while (!wait_response("OK", 5000)) { }   /* 连 WiFi 通常要几秒 */
            i++;
        }

        if (i == 2)
        {
            uart2_send_at("AT+CIPSTART=\"TCP\",\"192.168.1.103\",7755");
            while (!wait_response("OK", 3000)) { }
            i++;
        }

        retry++;
    }

    /* ---------- 3. 进入透传 ---------- */
    uart2_send_at("AT+CIPMODE=1");
    while (!wait_response("OK", 1500)) { }

    uart2_send_at("AT+CIPSEND");
    while (!wait_response(">", 1500)) { }
}
