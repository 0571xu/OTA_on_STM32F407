/**************************************
 * 文件名  ：uds.h
 * 描述    ：UDS 子集客户端（F407 主站侧）
 * 说明    ：只实现 OTA 下发需要的五个服务：
 *           0x10 会话控制 / 0x3E 保持连接 / 0x34 请求下载 / 0x36 传数据 / 0x37 结束传输
 *********************************************************/
#ifndef UDS_H
#define UDS_H

#include "main.h"

/* 寻址：请求 0x600+node，响应 0x580+node（与 can.h 的分区定义一致） */
#define UDS_REQ_ID(node)        (0x600U + (uint32_t)(node))
#define UDS_RESP_ID(node)       (0x580U + (uint32_t)(node))

/* 会话类型 */
#define UDS_SESSION_DEFAULT     0x01U
#define UDS_SESSION_PROGRAM     0x02U

/* F103 片内 Flash 起始地址：RequestDownload 里的地址参数 */
#define UDS_F103_FLASH_BASE     0x08000000U

/*
 * F103 单槽方案：0x34 收到后就整片擦除（约 1~2 秒），
 * 所以这个服务的响应超时必须放宽，不能按 P2=1 秒算。
 * 如果 F103 先回 0x7F 34 78（pending），则再延长到 P2*。
 */
#define UDS_P2_MS               1000U
#define UDS_P2STAR_MS           5000U
#define UDS_P2_ERASE_MS         6000U

/* 返回码 */
#define UDS_E_PARAM             (-1)
#define UDS_E_TIMEOUT           (-2)    /* 没等到响应 */
#define UDS_E_NRC               (-3)    /* 收到否定响应，具体 NRC 见 uds_stat_t.last_nrc */
#define UDS_E_BAD_RESP          (-4)    /* 响应格式不认识（含多帧响应） */
#define UDS_E_ISOTP             (-5)    /* 底层 ISO-TP 发送失败，具体码见 isotp_get_stat() */
#define UDS_E_TOO_LONG          (-6)    /* 请求超过 ISO-TP 单包上限 */
#define UDS_E_VERIFY            (-7)    /* 镜像长度与暂存区不符 */

typedef struct {
    uint32_t requests;      /* 发出的请求数 */
    uint32_t nrc_counts;    /* 否定响应次数 */
    uint32_t timeouts;      /* 超时次数 */
    uint8_t  last_nrc;      /* 最近一次否定响应的 NRC */
    int32_t  last_error;    /* 最近一次错误码 */
} uds_stat_t;

const uds_stat_t *uds_get_stat(void);
void uds_reset_stat(void);

/* 0x10：切换会话。type = UDS_SESSION_DEFAULT / UDS_SESSION_PROGRAM */
int uds_session(uint8_t node, uint8_t type);

/* 0x3E：保持连接。suppress=1 时请求里带 0x80，从机不回响应 */
int uds_tester_present(uint8_t node, uint8_t suppress);

/*
 * 0x34：请求下载。
 *   addr / size ：目标地址与总长度
 *   max_block   ：输出，从机告知的单块最大字节数（含 0x36 的 SID 与服务号）
 * 返回 0 成功。
 */
int uds_request_download(uint8_t node, uint32_t addr, uint32_t size, uint32_t *max_block);

/* 0x36：传一块数据，内部自动维护块计数器 1~255 循环 */
int uds_transfer_data(uint8_t node, const uint8_t *data, uint32_t len);

/* 0x37：结束传输 */
int uds_transfer_exit(uint8_t node);

/*
 * 全流程：把 S4 暂存区里的前 size 字节下发到指定节点。
 * 顺序：0x10 02 -> 0x34 -> 0x36 xN -> 0x37
 * 注意：擦除由 F103 在 0x34 内部完成，这里不单独发 0x31。
 */
int uds_flash_staging(uint8_t node, uint32_t size);

/* 单个 0x36 块的数据长度上限（受 ISO-TP 4095 限制，这里取 1024 便于对页） */
#define UDS_BLOCK_SIZE          1024U

#endif /* UDS_H */
