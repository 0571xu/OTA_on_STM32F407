/**************************************
 * 文件名  ：isotp.h
 * 描述    ：ISO-TP (ISO 15765-2) 发送端，F407 主站下发固件用
 *********************************************************/
#ifndef ISOTP_H
#define ISOTP_H

#include "main.h"

/* 12 位长度字段上限。F103 镜像 64KB 由上层分块，每块不超过这个值 */
#define ISOTP_MAX_PDU           4095U

/*
 * 等流控帧（N_Bs）的默认超时。
 * 不能用 ISO 默认的 1 秒：F103 在 Bootloader 里边收边写 Flash，
 * 擦一页要 20~40ms，连续擦除时 CPU 停指，很可能超过 1 秒才回得出流控。
 */
#define ISOTP_N_BS_TIMEOUT      5000U

/* 单帧塞进发送邮箱的超时 */
#define ISOTP_FRAME_TX_TIMEOUT  50U

/* 返回码 */
#define ISOTP_E_PARAM           (-1)    /* 参数非法 */
#define ISOTP_E_TOO_LONG        (-2)    /* 超过 4095 字节，需上层分块 */
#define ISOTP_E_FC_TIMEOUT      (-3)    /* 等流控超时 */
#define ISOTP_E_FC_OVFLW        (-4)    /* 接收方缓存不够，回 0x32 溢出 */
#define ISOTP_E_FC_BAD          (-5)    /* 收到的不是流控帧 / 帧太短 */
#define ISOTP_E_SEND            (-6)    /* 邮箱一直占满，发不出去 */

typedef struct {
    uint32_t pdus;          /* 调用 isotp_send 的次数 */
    uint32_t frames_sent;   /* 实际发出的 CAN 帧数 */
    uint32_t fc_received;   /* 收到的流控帧数 */
    uint32_t fc_timeouts;   /* 等流控超时次数 */
    uint32_t fc_bad;        /* 流控帧格式非法次数 */
    int32_t  last_error;    /* 最近一次错误码，0 表示成功 */
} isotp_stat_t;

/* 上电做一次回环自测：验证拆包、序号、长度字段、流控解析 */
#ifndef ISOTP_SELFTEST_ON_BOOT
#define ISOTP_SELFTEST_ON_BOOT 1
#endif

/*
 * 发送一个 PDU。
 *   tx_id / rx_id ：本机发帧用的 ID、等待流控用的 ID
 *   len > 7       ：自动走 首帧 + 连续帧，并按接收方回的 BS/STmin 控制节奏
 *   n_bs_timeout_ms 传 0 则用 ISOTP_N_BS_TIMEOUT
 * 返回 0 成功，负数为 ISOTP_E_* 错误码。
 */
int  isotp_send(uint32_t tx_id, uint32_t rx_id, const uint8_t *data,
                uint32_t len, uint32_t n_bs_timeout_ms);

const isotp_stat_t *isotp_get_stat(void);
void isotp_reset_stat(void);

/* 回环自测：成功返回总帧数（>0），失败返回负的错误码 */
int  isotp_selftest(void);

#endif /* ISOTP_H */
