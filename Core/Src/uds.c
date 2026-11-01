/**************************************
 * 文件名  ：uds.c
 * 描述    ：UDS 子集客户端（F407 主站侧）
 * 说明    ：请求走 0x600+node，响应走 0x580+node；传输层交给 isotp.c。
 *
 * 只实现 OTA 需要的五个服务：
 *   0x10 会话控制  0x3E 保持连接  0x34 请求下载  0x36 传数据  0x37 结束传输
 *
 * 响应只解析**单帧**（数据场首字节高 4 位为 0）。我们这五个服务的正响应
 * 都只有 2~6 字节，够用；如果将来要读长数据（如 0x22 读版本），
 * 需要在这里补多帧接收状态机，不能沿用现在的逻辑。
 *********************************************************/

#include "can.h"
#include "isotp.h"
#include "uds.h"
#include "ota_receive.h"
#include <string.h>

/* ==================== 缓冲（放 .bss，不占任务栈） ==================== */

static uint8_t  s_req[UDS_BLOCK_SIZE + 2U];   /* [0]=SID [1]=服务号 [2..]=数据 */
static uint8_t  s_resp[64];
static uint8_t  s_block_cnt = 1U;             /* 0x36 块计数器，1~255 循环 */
static uds_stat_t s_stat;

const uds_stat_t *uds_get_stat(void)
{
    return &s_stat;
}

void uds_reset_stat(void)
{
    memset(&s_stat, 0, sizeof(s_stat));
}

/* ==================== 响应等待 ==================== */

/*
 * 等一帧响应并做基本校验。
 * 返回后 *len 为响应数据长度，内容在 s_resp 里。
 */
static int uds_wait_response(uint8_t node, uint8_t sid, uint32_t *len, uint32_t timeout_ms)
{
    uint32_t    rx_id = UDS_RESP_ID(node);
    uint32_t    t0    = HAL_GetTick();
    can_frame_t f;

    for (;;) {
        uint32_t now = HAL_GetTick();
        uint32_t left;
        uint32_t n;

        if ((now - t0) >= timeout_ms) {
            s_stat.timeouts++;
            return UDS_E_TIMEOUT;
        }
        left = timeout_ms - (now - t0);

        if (can_if_recv(&f, left) != 0) {
            s_stat.timeouts++;
            return UDS_E_TIMEOUT;
        }

        if (f.id != rx_id) {
            continue;                       /* 其它节点的帧，忽略 */
        }
        if ((f.dlc < 2U) || ((f.data[0] & 0xF0U) != 0x00U)) {
            return UDS_E_BAD_RESP;          /* 不是单帧：当前不支持多帧响应 */
        }

        n = (uint32_t)(f.data[0] & 0x0FU);
        if ((n == 0U) || (n > (uint32_t)(f.dlc - 1U)) || (n > sizeof(s_resp))) {
            return UDS_E_BAD_RESP;
        }
        memcpy(s_resp, &f.data[1], n);
        *len = n;

        /* 否定响应：7F <SID> <NRC> */
        if (s_resp[0] == 0x7FU) {
            s_stat.nrc_counts++;
            s_stat.last_nrc = (n >= 3U) ? s_resp[2] : 0U;

            if (s_stat.last_nrc == 0x78U) {             /* responsePending：接着等 */
                t0       = HAL_GetTick();
                timeout_ms = UDS_P2STAR_MS;
                continue;
            }
            return UDS_E_NRC;
        }

        if (s_resp[0] != (uint8_t)(sid + 0x40U)) {
            return UDS_E_BAD_RESP;          /* 正响应 SID 不匹配 */
        }

        return 0;
    }
}

/* 发请求 + 等响应 */
static int uds_request(uint8_t node, const uint8_t *req, uint32_t len,
                       uint8_t sid, uint32_t *resp_len, uint32_t timeout_ms)
{
    int rc;

    s_stat.requests++;

    rc = isotp_send(UDS_REQ_ID(node), UDS_RESP_ID(node), req, len, 0U);
    if (rc != 0) {
        s_stat.last_error = UDS_E_ISOTP;
        return UDS_E_ISOTP;
    }

    rc = uds_wait_response(node, sid, resp_len, timeout_ms);
    s_stat.last_error = (rc == 0) ? 0 : rc;
    return rc;
}

/* ==================== 0x10 会话控制 ==================== */

int uds_session(uint8_t node, uint8_t type)
{
    uint32_t rlen = 0U;

    if ((node < 1U) || (node > 15U)) {
        return UDS_E_PARAM;
    }

    s_req[0] = 0x10U;
    s_req[1] = type;

    return uds_request(node, s_req, 2U, 0x10U, &rlen, UDS_P2_MS);
}

/* ==================== 0x3E 保持连接 ==================== */

int uds_tester_present(uint8_t node, uint8_t suppress)
{
    uint32_t rlen = 0U;
    int      rc;

    if ((node < 1U) || (node > 15U)) {
        return UDS_E_PARAM;
    }

    s_req[0] = 0x3EU;
    s_req[1] = suppress ? 0x80U : 0x00U;

    s_stat.requests++;

    /* 抑制正响应：从机不回，等响应会白等一个超时 */
    if (suppress) {
        rc = isotp_send(UDS_REQ_ID(node), UDS_RESP_ID(node), s_req, 2U, 0U);
        s_stat.last_error = (rc == 0) ? 0 : UDS_E_ISOTP;
        return (rc == 0) ? 0 : UDS_E_ISOTP;
    }

    return uds_request(node, s_req, 2U, 0x3EU, &rlen, UDS_P2_MS);
}

/* ==================== 0x34 请求下载 ==================== */

int uds_request_download(uint8_t node, uint32_t addr, uint32_t size, uint32_t *max_block)
{
    uint32_t rlen = 0U;
    uint32_t nb;
    uint32_t i;
    uint32_t v;
    int      rc;

    if ((node < 1U) || (node > 15U) || (size == 0U)) {
        return UDS_E_PARAM;
    }

    /* 请求 11 字节，超过 7 字节，必然走 ISO-TP 的 首帧 + 连续帧 */
    s_req[0]  = 0x34U;
    s_req[1]  = 0x00U;                       /* 不压缩、不加密 */
    s_req[2]  = 0x44U;                       /* 4 字节地址 + 4 字节长度 */
    s_req[3]  = (uint8_t)(addr >> 24);
    s_req[4]  = (uint8_t)(addr >> 16);
    s_req[5]  = (uint8_t)(addr >> 8);
    s_req[6]  = (uint8_t)(addr);
    s_req[7]  = (uint8_t)(size >> 24);
    s_req[8]  = (uint8_t)(size >> 16);
    s_req[9]  = (uint8_t)(size >> 8);
    s_req[10] = (uint8_t)(size);

    s_block_cnt = 1U;

    /* 从机可能在这期间整片擦除（1~2 秒），所以用加长的超时 */
    rc = uds_request(node, s_req, 11U, 0x34U, &rlen, UDS_P2_ERASE_MS);
    if (rc != 0) {
        return rc;
    }

    /* 正响应：74 <LFI> <maxNumberOfBlockLength，大端> */
    if ((rlen < 2U) || (s_resp[0] != 0x74U)) {
        return UDS_E_BAD_RESP;
    }

    nb = (uint32_t)((s_resp[1] >> 4) & 0x0FU);
    if ((nb == 0U) || (nb > 4U) || ((2U + nb) > rlen)) {
        return UDS_E_BAD_RESP;
    }

    v = 0U;
    for (i = 0U; i < nb; i++) {
        v = (v << 8) | s_resp[2U + i];
    }

    if (max_block != NULL) {
        /* 从机没给有效值时按本端缓冲上限处理 */
        *max_block = (v == 0U) ? (UDS_BLOCK_SIZE + 2U) : v;
    }

    return 0;
}

/* ==================== 0x36 传数据 ==================== */

int uds_transfer_data(uint8_t node, const uint8_t *data, uint32_t len)
{
    uint32_t rlen = 0U;
    int      rc;

    if ((node < 1U) || (node > 15U) || (data == NULL) || (len == 0U)) {
        return UDS_E_PARAM;
    }
    if (len > UDS_BLOCK_SIZE) {
        return UDS_E_TOO_LONG;
    }

    s_req[0] = 0x36U;
    s_req[1] = s_block_cnt;
    if (data != &s_req[2]) {
        memcpy(&s_req[2], data, len);        /* 调用方可能已经把数据放在 s_req+2 */
    }

    rc = uds_request(node, s_req, len + 2U, 0x36U, &rlen, UDS_P2_MS);
    if (rc != 0) {
        return rc;
    }

    /* 正响应：76 <块计数器>，计数器必须和发出去的一致 */
    if ((rlen < 2U) || (s_resp[0] != 0x76U) || (s_resp[1] != s_block_cnt)) {
        return UDS_E_BAD_RESP;
    }

    s_block_cnt = (s_block_cnt == 255U) ? 1U : (uint8_t)(s_block_cnt + 1U);

    return 0;
}

/* ==================== 0x37 结束传输 ==================== */

int uds_transfer_exit(uint8_t node)
{
    uint32_t rlen = 0U;

    if ((node < 1U) || (node > 15U)) {
        return UDS_E_PARAM;
    }

    s_req[0] = 0x37U;

    return uds_request(node, s_req, 1U, 0x37U, &rlen, UDS_P2_MS);
}

/* ==================== 全流程 ==================== */

int uds_flash_staging(uint8_t node, uint32_t size)
{
    uint32_t off       = 0U;
    uint32_t max_block = 0U;
    uint32_t n;
    int      rc;

    if ((size == 0U) || (size > STAGING_SIZE)) {
        return UDS_E_PARAM;
    }

    rc = uds_session(node, UDS_SESSION_PROGRAM);
    if (rc != 0) {
        return rc;
    }

    rc = uds_request_download(node, UDS_F103_FLASH_BASE, size, &max_block);
    if (rc != 0) {
        return rc;
    }

    while (off < size) {
        n = size - off;
        if (n > UDS_BLOCK_SIZE) {
            n = UDS_BLOCK_SIZE;
        }
        /* max_block 是「SID + 计数器 + 数据」的总长 */
        if ((max_block > 2U) && ((n + 2U) > max_block)) {
            n = max_block - 2U;
        }

        /* 直接读到请求缓冲的数据区，省 1KB 的额外缓冲 */
        if (ota_staging_read(off, &s_req[2], n) != 0) {
            return UDS_E_VERIFY;
        }

        rc = uds_transfer_data(node, &s_req[2], n);
        if (rc != 0) {
            return rc;
        }

        off += n;
    }

    return uds_transfer_exit(node);
}
