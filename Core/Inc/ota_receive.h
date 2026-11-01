#ifndef OTA_RECEIVE_H
#define OTA_RECEIVE_H

#include "main.h"
#include <stdint.h>
#include <string.h>
#include "FreeRTOS.h"
#include "stream_buffer.h"
#include "queue.h"
#include "task.h"

/* ==================== Flash 分区 ==================== */
/*
 * 0x08000000 - 0x08007FFF  Bootloader    S0 ~ S1  (32KB)
 * 0x08008000 - 0x0800BFFF  元数据副本 0   S2       (16KB)
 * 0x0800C000 - 0x0800FFFF  元数据副本 1   S3       (16KB)
 * 0x08010000 - 0x0801FFFF  F103 镜像暂存   S4       (64KB)
 * 0x08020000 - 0x0805FFFF  APP A         S5 ~ S6  (256KB)
 * 0x08060000 - 0x0809FFFF  APP B         S7 ~ S8  (256KB)
 * 0x080A0000 - 0x080FFFFF  预留           S9 ~ S11 (384KB)
 *
 * 2026-10-25 由「448KB 双槽」调整为「256KB 双槽 + 64KB 暂存区」：
 * 多设备 OTA 需要地方存放待下发给 F103 的镜像，原布局没有空闲扇区。
 *
 * 两个槽的“可用容量”必须一致（都按 256KB 限制），否则某个镜像能放进 B 却放不进 A。
 * 两份元数据必须位于不同扇区：擦除粒度是扇区，同扇区会一擦全没。
 * S4 单独成区是因为它是 64KB 扇区，正好等于 F103 片内 Flash 容量。
 */
#define BOOTLOADER_BASE     0x08000000U
#define BOOTLOADER_SIZE     (32U * 1024U)

#define SLOT_A_BASE         0x08020000U
#define SLOT_B_BASE         0x08060000U
#define SLOT_CAPACITY       (256U * 1024U)
#define SLOT_A_END          (SLOT_A_BASE + SLOT_CAPACITY)   /* 0x08060000 */
#define SLOT_B_END          (SLOT_B_BASE + SLOT_CAPACITY)   /* 0x080A0000，可用上限 */
#define SLOT_B_ERASE_END    0x080A0000U                     /* B 物理末端，S8 末尾 */

/* F103 镜像暂存区：从服务器下载完整镜像后先存这里，校验通过再经 CAN 下发 */
#define STAGING_BASE        0x08010000U
#define STAGING_SIZE        (64U * 1024U)
#define STAGING_END         (STAGING_BASE + STAGING_SIZE)   /* 0x08020000 */

#define META0_BASE          0x08008000U
#define META1_BASE          0x0800C000U
#define META_SECTOR_SIZE    (16U * 1024U)                   /* S2、S3 均为 16KB */

#define FLASH_BASE_ADDR     0x08000000U
#define FLASH_END_ADDR      0x08100000U

/*
 * 当前 APP 构建版本链接到哪个槽。
 *   默认（不定义 APP_SLOT_B）：A 版镜像，链接地址 0x08020000
 *   定义 APP_SLOT_B        ：B 版镜像，链接地址 0x08060000
 *
 * 运行在哪个槽由链接地址唯一确定，不能靠元数据去猜：
 * 一旦元数据与实际运行槽不一致，就可能把正在运行的固件擦掉。
 */
#ifdef APP_SLOT_B
#define APP_IMAGE_BASE      SLOT_B_BASE
#define APP_IMAGE_SLOT      1U
#else
#define APP_IMAGE_BASE      SLOT_A_BASE
#define APP_IMAGE_SLOT      0U
#endif

/* ==================== OTA 参数区结构 ==================== */
#define OTA_META_MAGIC      0x4F544132U      /* "OTA2"，与 Bootloader 保持一致 */
#ifdef APP_SLOT_B
#define CURRENT_VERSION     0x00010007U      /* V1.7 */
#else
#define CURRENT_VERSION     0x00010006U      /* V1.6 */
#endif

#define OTA_MAX_TOTAL_CHUNKS        2048U
#define OTA_BITMAP_BYTES            ((OTA_MAX_TOTAL_CHUNKS + 7U) / 8U)  /* 每 8 包 1 字节 */

#define SLOT_STATE_EMPTY    0U
#define SLOT_STATE_VALID    1U
#define SLOT_STATE_INVALID  2U

/* 新固件连续健康运行多久才认为试运行通过（ms） */
#define OTA_CONFIRM_WINDOW_MS       5000U

#pragma pack(1)
typedef struct {
    uint32_t magic;          /* 固定标识 0x4F544132 */
    uint32_t seq;            /* 写入序号，越大越新（用于判定有效副本） */
    uint8_t  active_slot;    /* 本次启动的槽：0 = A，1 = B */
    uint8_t  prev_slot;      /* 试运行失败时回滚到哪个槽 */
    uint8_t  trial_pending;  /* 1 = 新固件待确认 */
    uint8_t  trial_count;    /* 试运行失败启动次数，达到上限则回滚 */
    uint8_t  slot_state[2];  /* 各槽状态：EMPTY / VALID / INVALID */
    uint8_t  dl_state;       /* 断点续传预留：0 = IDLE，1 = DOWNLOADING */
    uint8_t  dl_slot;        /* 断点续传预留：本次下载目标槽 */
    uint16_t dl_total;       /* 断点续传预留：本次下载总包数 */
    uint32_t slot_size[2];   /* 各槽固件长度（0 表示未知，跳过整包 CRC） */
    uint32_t slot_crc[2];    /* 各槽整包 CRC32 */
    uint32_t version[2];     /* 各槽固件版本号 */
    uint8_t  rx_bitmap[OTA_BITMAP_BYTES];   /* 断点续传：已收包位图 */
    uint32_t meta_crc;       /* 本结构 CRC32，计算时该字段按 0xFFFFFFFF 处理 */
} ota_config_t;
#pragma pack()

/* Bootloader 与 APP 的结构体布局必须一致 */
typedef char ota_config_size_check[(sizeof(ota_config_t) == 302) ? 1 : -1];

/* ==================== 帧格式 ==================== */
#define DATA_FRAME_LEN      256
#define FRAME_POOL_SIZE     4
#define STREAM_BUF_SIZE     1024
#define FRAME_QUEUE_DEPTH   4
#define AT_LINE_MAX         128

#define FRAME_HEADER_1      0x55
#define FRAME_HEADER_2      0x44
#define FRAME_TAIL_1        0x33
#define FRAME_TAIL_2        0x22
#define PACKET_END          0xFF

/* ==================== 状态枚举 ==================== */
typedef enum {
    Status_OK      = 0x00,
    Status_ERROR   = 0x01,
    Status_BUSY    = 0x02,
    Status_TIMEOUT = 0x03
} StatusTypeDef;

/* ==================== 数据结构 ==================== */
typedef struct {
    uint8_t  buf[DATA_FRAME_LEN];
    uint8_t  in_use;
} frame_slot_t;

typedef struct {
    uint8_t       total_packets;
    uint8_t       current_packet;
    uint16_t      data_len;          /* 实际长度 2 字节，最大 256 */
    uint32_t      crc32;
    frame_slot_t *slot;
} fw_frame_t;

typedef struct {
    uint8_t   state;
    uint8_t   total_packets;
    uint8_t   current_packet;
    uint16_t  data_len;
    uint32_t  crc32;
    uint32_t  crc_calc;
    uint16_t  data_idx;
    frame_slot_t *slot;
    uint8_t   crc_byte_idx;
    uint8_t   len_byte_idx;          /* 读取 2 字节实际长度用的计数 */
} parser_t;

/* ==================== 全局句柄 ==================== */
extern StreamBufferHandle_t uartStreamBuffer;
extern QueueHandle_t        xFrameQueue;
extern QueueHandle_t        atQueue;
extern TaskHandle_t xrequest_upgrade_taskHandle;
extern TaskHandle_t xWifisendTaskHandle;

/* ==================== 函数声明 ==================== */
void           ota_receive_init(void);
frame_slot_t*  frame_pool_alloc(void);
void           frame_pool_free(frame_slot_t *slot);
uint32_t       calculate_crc32_ex(uint8_t total, uint8_t current, uint16_t len, uint8_t *data);

/* 整包 / 元数据 CRC（与 Bootloader 使用同一套算法） */
uint32_t       ota_crc32_bytes(const void *data, uint32_t len);

/* 元数据双副本：seq 大且 meta_crc 正确的一份生效 */
int            ota_meta_load(ota_config_t *out);
int            ota_meta_save(ota_config_t *cfg);

/* 切槽：把 active_slot 指向 target_slot 并置 trial_pending，由调用方负责重启。
 * 返回 0 成功，-1 参数非法或写入失败。 */
int            ota_meta_request_switch(uint8_t current_slot, uint8_t target_slot,
                                       uint32_t image_size, uint32_t image_crc);

/* APP 启动：元数据缺失时建立首份配置，并修正与实际运行槽不一致的状态。
 * 注意：这里不清 trial_pending，试运行确认交给心跳任务。 */
void           ota_app_boot_init(uint32_t image_base);

/* 心跳健康运行满 OTA_CONFIRM_WINDOW_MS 后调用：
 * 1 = 确认成功（清掉 trial_pending），0 = 无需确认，-1 = 写入失败 */
int            ota_app_confirm_healthy(uint32_t image_base);

int            ota_image_vectors_valid(uint32_t slot_base);

/* Flash 接口 */
uint32_t       get_sector(uint32_t addr);        /* 获取地址所在扇区号 */
uint32_t       get_sector_base(uint32_t addr);  /* 获取地址所在扇区基址 */
uint32_t       get_sector_size(uint32_t sector);/* 扇区大小：16KB / 64KB / 128KB */
extern StatusTypeDef  flash_erase(uint32_t start_addr, uint32_t size);
extern StatusTypeDef  flash_write(uint32_t dest_addr, uint8_t *src, uint32_t size);

/* ==================== F103 镜像暂存区（S4） ==================== */
/*
 * 用途：F407 从服务器下载完整 F103 镜像 → 存 S4 → 整包 CRC 校验通过 → 再经 CAN 下发。
 * 先落盘再下发，而不是一边收一边转发：CAN 侧失败可原地重试，不必重新下载。
 *
 * 约束：
 *   1. 写之前必须先 erase（擦除粒度是整个 S4 扇区，1~2 秒）；
 *   2. offset 必须 4 字节对齐，因为底层按字编程；
 *   3. 写入长度不必对齐，尾部不足 4 字节由底层补 0xFF。
 */
int  ota_staging_erase(void);
int  ota_staging_write(uint32_t offset, const uint8_t *src, uint32_t len);
int  ota_staging_read(uint32_t offset, uint8_t *dst, uint32_t len);
int  ota_staging_verify(uint32_t size, uint32_t crc);   /* 读回整包算 CRC 并比对 */

/*
 * 暂存区自测：擦 S4、写固定图案、读回比对。
 * 注意它会擦掉已暂存的镜像，正式使用前必须改成 0。
 */
#ifndef OTA_STAGING_SELFTEST_ON_BOOT
#define OTA_STAGING_SELFTEST_ON_BOOT 1
#endif
int  ota_staging_selftest(void);

#endif /* OTA_RECEIVE_H */
