/**************************************
 * 文件名  ：isotp.c
 * 描述    ：ISO-TP (ISO 15765-2) 发送端最小实现 + 回环自测
 * 说明    ：F407 是主站，只负责下发；接收方（F103）回的是流控帧和 UDS 响应单帧。
 *
 * 帧格式（标准帧，8 字节数据场）：
 *   单帧 SF : [0x0L] [L 字节数据 ...]                    L = 1~7
 *   首帧 FF : [0x1H] [LL] [6 字节数据 ...]              长度 = HHLL（12 位）
 *   连续帧 CF: [0x2N] [7 字节数据 ...]                  N = 序号 1~15，循环
 *   流控 FC : [0x3S] [BS] [STmin] ...                   S = 0 继续 / 1 等待 / 2 溢出
 *********************************************************/

#include "can.h"
#include "isotp.h"
#include <string.h>
#include "FreeRTOS.h"
#include "task.h"

/* ==================== 发送统计 ==================== */

static isotp_stat_t s_stat;

const isotp_stat_t *isotp_get_stat(void)
{
    return &s_stat;
}

void isotp_reset_stat(void)
{
    memset(&s_stat, 0, sizeof(s_stat));
}

/* 调度器起来后用 vTaskDelay（让出 CPU），否则退回忙等 */
static void isotp_delay_ms(uint32_t ms)
{
    if (ms == 0U) {
        return;
    }
    if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) {
        vTaskDelay(pdMS_TO_TICKS(ms));
    } else {
        HAL_Delay(ms);
    }
}

/* ==================== 帧构造 ==================== */

/* 返回该帧的实际 DLC（数据字节数） */
static uint32_t isotp_build_sf(const uint8_t *src, uint32_t len, uint8_t *out)
{
    out[0] = (uint8_t)(len & 0x0FU);
    memcpy(&out[1], src, len);
    return len + 1U;
}

static uint32_t isotp_build_ff(const uint8_t *src, uint32_t len, uint8_t *out)
{
    out[0] = (uint8_t)(0x10U | ((len >> 8) & 0x0FU));
    out[1] = (uint8_t)(len & 0xFFU);
    memcpy(&out[2], src, 6U);
    return 8U;
}

static uint32_t isotp_build_cf(uint8_t sn, const uint8_t *src, uint32_t len, uint8_t *out)
{
    out[0] = (uint8_t)(0x20U | (sn & 0x0FU));
    memcpy(&out[1], src, len);
    return len + 1U;
}

/* ==================== 流控帧解析 ==================== */

typedef struct {
    uint8_t status;      /* 0 继续 / 1 等待 / 2 溢出 */
    uint8_t bs;          /* 0 表示之后不再需要流控 */
    uint8_t stmin_ms;    /* 已归一化为毫秒 */
} isotp_fc_t;

static int isotp_parse_fc(const can_frame_t *f, isotp_fc_t *fc)
{
    if ((f->dlc < 3U) || ((f->data[0] & 0xF0U) != 0x30U)) {
        return -1;
    }

    fc->status = (uint8_t)(f->data[0] & 0x0FU);
    fc->bs     = f->data[1];

    if (f->data[2] <= 0x7FU) {
        fc->stmin_ms = f->data[2];              /* 0~127ms */
    } else if (f->data[2] <= 0xF9U) {
        fc->stmin_ms = 1U;                      /* 100~900us，按 1ms 处理 */
    } else {
        fc->stmin_ms = 0x7FU;                   /* 保留值按 127ms 处理 */
    }

    return 0;
}

/* ==================== 等待流控 ==================== */

static int isotp_wait_fc(uint32_t rx_id, uint32_t timeout_ms, isotp_fc_t *fc)
{
    uint32_t    t0 = HAL_GetTick();
    can_frame_t f;
    int         rc;

    for (;;) {
        uint32_t now  = HAL_GetTick();
        uint32_t left;

        if ((now - t0) >= timeout_ms) {
            s_stat.fc_timeouts++;
            return ISOTP_E_FC_TIMEOUT;
        }
        left = timeout_ms - (now - t0);

        rc = can_if_recv(&f, left);
        if (rc != 0) {
            s_stat.fc_timeouts++;
            return ISOTP_E_FC_TIMEOUT;
        }

        if (f.id != rx_id) {
            continue;                           /* 不是给我的流控（比如 UDS 响应），忽略 */
        }
        if (isotp_parse_fc(&f, fc) != 0) {
            s_stat.fc_bad++;
            return ISOTP_E_FC_BAD;
        }

        s_stat.fc_received++;

        if (fc->status == 0U) {
            return 0;                           /* 继续发 */
        }
        if (fc->status == 1U) {
            t0 = HAL_GetTick();                 /* 等待：N_Bs 重新计时 */
            continue;
        }
        return ISOTP_E_FC_OVFLW;                /* 0x2 溢出；0x3 保留，同样按失败处理 */
    }
}

/* ==================== 发送 ==================== */

int isotp_send(uint32_t tx_id, uint32_t rx_id, const uint8_t *data,
               uint32_t len, uint32_t n_bs_timeout_ms)
{
    uint8_t    buf[8];
    uint32_t   off;
    uint8_t    sn;
    uint8_t    bs_left;
    isotp_fc_t fc;
    int        rc;

    if ((data == NULL) || (len == 0U)) {
        return ISOTP_E_PARAM;
    }
    if (len > ISOTP_MAX_PDU) {
        return ISOTP_E_TOO_LONG;
    }
    if (n_bs_timeout_ms == 0U) {
        n_bs_timeout_ms = ISOTP_N_BS_TIMEOUT;
    }

    s_stat.pdus++;

    /* 单帧：7 字节以内不需要流控 */
    if (len <= 7U) {
        uint32_t n = isotp_build_sf(data, len, buf);

        rc = can_if_send(tx_id, buf, (uint8_t)n, ISOTP_FRAME_TX_TIMEOUT);
        if (rc != 0) {
            s_stat.last_error = ISOTP_E_SEND;
            return ISOTP_E_SEND;
        }
        s_stat.frames_sent++;
        s_stat.last_error = 0;
        return 0;
    }

    /* 首帧 */
    (void)isotp_build_ff(data, len, buf);
    rc = can_if_send(tx_id, buf, 8U, ISOTP_FRAME_TX_TIMEOUT);
    if (rc != 0) {
        s_stat.last_error = ISOTP_E_SEND;
        return ISOTP_E_SEND;
    }
    s_stat.frames_sent++;

    /* 等第一帧流控，没拿到就不发后续帧 */
    rc = isotp_wait_fc(rx_id, n_bs_timeout_ms, &fc);
    if (rc != 0) {
        s_stat.last_error = rc;
        return rc;
    }

    off     = 6U;
    sn      = 1U;
    bs_left = fc.bs;

    while (off < len) {
        uint32_t chunk = len - off;

        if (chunk > 7U) {
            chunk = 7U;
        }

        (void)isotp_build_cf(sn, &data[off], chunk, buf);
        rc = can_if_send(tx_id, buf, (uint8_t)(chunk + 1U), ISOTP_FRAME_TX_TIMEOUT);
        if (rc != 0) {
            s_stat.last_error = ISOTP_E_SEND;
            return ISOTP_E_SEND;
        }
        s_stat.frames_sent++;

        off += chunk;
        sn   = (uint8_t)(sn + 1U);              /* 序号 1~15，0 不参与计数 */
        if (sn > 15U) {
            sn = 1U;
        }

        /* BS 个连续帧后要重新等流控；BS = 0 表示整包发完为止都不需要 */
        if ((fc.bs != 0U) && (off < len)) {
            bs_left--;
            if (bs_left == 0U) {
                rc = isotp_wait_fc(rx_id, n_bs_timeout_ms, &fc);
                if (rc != 0) {
                    s_stat.last_error = rc;
                    return rc;
                }
                bs_left = fc.bs;
                /*
                 * 注意：这里**不能**把 sn 归 1。
                 * SN 是「整个 PDU 内」的 1~15 循环序号，流控帧只重新分配发送额度，
                 * 不重启序号；接收端（f103_node/isotp.c）也是连续递增的。
                 * 早期版本在这里写了 sn = 1，BS=0 时永远不会执行到，
                 * 一旦接收方改成 BS=1..N 就会立刻因序号错位丢掉整包。
                 */
            }
        }

        isotp_delay_ms(fc.stmin_ms);
    }

    s_stat.last_error = 0;
    return 0;
}

/* ==================== 回环自测 ==================== */

/*
 * 思路：F407 同时扮演"发送方"和"接收方"。
 *   - 发送用 0x581，接收方用 0x582 回流控，两个 ID 都落在过滤器放行的 0x580~0x58F 内；
 *   - 回环模式下自己发的帧会进自己的 FIFO0，于是可以逐帧读出、重组、比对；
 *   - 全程直接轮询 FIFO0，不碰接收队列，避免和其它任务抢帧。
 *
 * 这里刻意不用生产用的 0x600+node / 0x580+node：自测验证的是帧格式与序号逻辑，
 * 与寻址无关，用哪个 ID 不影响结论。
 */
#define ISOTP_TEST_LEN      300U
#define ISOTP_TEST_TX_ID    0x581U
#define ISOTP_TEST_FC_ID    0x582U

static uint8_t s_test_tx[ISOTP_TEST_LEN];
static uint8_t s_test_rx[ISOTP_TEST_LEN];

/* 直接从 FIFO0 取帧（自测期间接收中断是关的，帧只会留在 FIFO 里） */
static int isotp_test_poll(can_frame_t *out, uint32_t timeout_ms)
{
    uint32_t t0 = HAL_GetTick();

    while ((HAL_GetTick() - t0) < timeout_ms) {
        if (HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0) > 0U) {
            CAN_RxHeaderTypeDef hdr;

            if (HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &hdr, out->data) == HAL_OK) {
                out->id  = hdr.StdId;
                out->dlc = (uint8_t)hdr.DLC;
                out->ide = hdr.IDE;
                return 0;
            }
        }
        isotp_delay_ms(1U);
    }
    return -1;
}

/* 纯逻辑部分：流控帧解析的边界情况，不依赖总线 */
static int isotp_selftest_parse(void)
{
    can_frame_t f;
    isotp_fc_t  fc;

    memset(&f, 0, sizeof(f));

    /* 正常 CTS，BS=0，STmin=0 */
    f.dlc = 8U;
    f.data[0] = 0x30U; f.data[1] = 0x00U; f.data[2] = 0x00U;
    if ((isotp_parse_fc(&f, &fc) != 0) || (fc.status != 0U) || (fc.stmin_ms != 0U)) {
        return -1;
    }

    /* STmin = 0xF5（500us）应归一化为 1ms */
    f.data[2] = 0xF5U;
    if ((isotp_parse_fc(&f, &fc) != 0) || (fc.stmin_ms != 1U)) {
        return -2;
    }

    /* STmin = 0x20（32ms） */
    f.data[2] = 0x20U;
    if ((isotp_parse_fc(&f, &fc) != 0) || (fc.stmin_ms != 32U)) {
        return -3;
    }

    /* 溢出状态 */
    f.data[0] = 0x32U; f.data[1] = 0x00U; f.data[2] = 0x00U;
    if ((isotp_parse_fc(&f, &fc) != 0) || (fc.status != 2U)) {
        return -4;
    }

    /* 帧太短必须拒绝 */
    f.dlc = 2U;
    if (isotp_parse_fc(&f, &fc) == 0) {
        return -5;
    }

    return 0;
}

int isotp_selftest(void)
{
    uint8_t     frame[8];
    can_frame_t f;
    uint32_t    i;
    uint32_t    off;
    uint32_t    frames;
    uint8_t     sn;
    int         rc;

    /* 纯逻辑先测，失败就不用切模式了 */
    if (isotp_selftest_parse() != 0) {
        return -1;
    }

    for (i = 0U; i < ISOTP_TEST_LEN; i++) {
        s_test_tx[i] = (uint8_t)(i * 7U + 3U);
    }
    memset(s_test_rx, 0, sizeof(s_test_rx));

    if (can_if_reinit(CAN_MODE_LOOPBACK, 0U) != 0) {
        return -2;
    }

    /* ① 首帧：长度字段必须能被还原成 300 */
    (void)isotp_build_ff(s_test_tx, ISOTP_TEST_LEN, frame);
    if (can_if_send(ISOTP_TEST_TX_ID, frame, 8U, ISOTP_FRAME_TX_TIMEOUT) != 0) {
        rc = -3;
        goto out;
    }
    if (isotp_test_poll(&f, 50U) != 0) {
        rc = -4;
        goto out;
    }
    if ((f.id != ISOTP_TEST_TX_ID) || (f.dlc != 8U) || ((f.data[0] & 0xF0U) != 0x10U)) {
        rc = -5;
        goto out;
    }
    if (((((uint32_t)(f.data[0] & 0x0FU)) << 8) | f.data[1]) != ISOTP_TEST_LEN) {
        rc = -6;
        goto out;
    }
    memcpy(s_test_rx, &f.data[2], 6U);
    off = 6U;

    /* ② 模拟接收方回流控：BS=0 不限制，STmin=0 */
    frame[0] = 0x30U;
    frame[1] = 0x00U;
    frame[2] = 0x00U;
    frame[3] = 0x00U;
    frame[4] = 0x00U;
    frame[5] = 0x00U;
    frame[6] = 0x00U;
    frame[7] = 0x00U;
    if (can_if_send(ISOTP_TEST_FC_ID, frame, 8U, ISOTP_FRAME_TX_TIMEOUT) != 0) {
        rc = -7;
        goto out;
    }
    if (isotp_test_poll(&f, 50U) != 0) {
        rc = -8;
        goto out;
    }
    if ((f.id != ISOTP_TEST_FC_ID) || ((f.data[0] & 0xF0U) != 0x30U)) {
        rc = -9;
        goto out;
    }

    /* ③ 连续帧：序号 1~15 循环，逐帧读回重组 */
    sn     = 1U;
    frames = 2U;                                 /* 首帧 + 一帧流控 */
    while (off < ISOTP_TEST_LEN) {
        uint32_t n = ISOTP_TEST_LEN - off;

        if (n > 7U) {
            n = 7U;
        }

        (void)isotp_build_cf(sn, &s_test_tx[off], n, frame);
        if (can_if_send(ISOTP_TEST_TX_ID, frame, (uint8_t)(n + 1U), ISOTP_FRAME_TX_TIMEOUT) != 0) {
            rc = -10;
            goto out;
        }
        if (isotp_test_poll(&f, 50U) != 0) {
            rc = -11;
            goto out;
        }
        if (f.id != ISOTP_TEST_TX_ID) {
            rc = -12;
            goto out;
        }
        if ((f.data[0] & 0xF0U) != 0x20U) {
            rc = -13;
            goto out;
        }
        if ((f.data[0] & 0x0FU) != sn) {
            rc = -14;
            goto out;
        }

        memcpy(&s_test_rx[off], &f.data[1], n);
        off  += n;
        frames++;

        sn++;
        if (sn > 15U) {
            sn = 1U;
        }
    }

    /* ④ 内容必须与源数据逐字节一致 */
    if (memcmp(s_test_tx, s_test_rx, ISOTP_TEST_LEN) != 0) {
        rc = -15;
        goto out;
    }

    rc = (int)frames;

out:
    /* 无论成败都要切回正常模式，否则后面收不到任何响应帧 */
    if (can_if_reinit(CAN_MODE_NORMAL, 1U) != 0) {
        rc = -16;
    }
    return rc;
}
