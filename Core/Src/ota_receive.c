/**************************************
 * 文件名  ：ota_receive.c
 * 描述    ：OTA 帧接收、CRC 校验、Flash 写入、元数据双副本
 * 说明    ：配合 usart.c 中串口接收的功能实现
 *********************************************************/
 
#include "main.h"
#include "ota_receive.h"
#include <string.h>

/* ==================== 全局变量 ==================== */

StreamBufferHandle_t uartStreamBuffer = NULL;
QueueHandle_t        xFrameQueue      = NULL;
QueueHandle_t        atQueue          = NULL;
TaskHandle_t xrequest_upgrade_taskHandle;
TaskHandle_t xWifisendTaskHandle      = NULL;

/* 帧池 */
static frame_slot_t frame_pool[FRAME_POOL_SIZE];

/* ==================== 帧池管理 ==================== */

frame_slot_t* frame_pool_alloc(void)
{
    for (int i = 0; i < FRAME_POOL_SIZE; i++) {
        if (!frame_pool[i].in_use) {
            frame_pool[i].in_use = 1;
            return &frame_pool[i];
        }
    }
    return NULL;   /* 无空闲 slot */
}

void frame_pool_free(frame_slot_t *slot)
{
    if (slot) {
        slot->in_use = 0;
    }
}

/* ==================== 初始化 ==================== */

void ota_receive_init(void)
{
    memset(frame_pool, 0, sizeof(frame_pool));

    uartStreamBuffer = xStreamBufferCreate(STREAM_BUF_SIZE, 1);   /* 触发水位=1 */
    xFrameQueue      = xQueueCreate(FRAME_QUEUE_DEPTH, sizeof(fw_frame_t));
    atQueue          = xQueueCreate(4, AT_LINE_MAX);

    configASSERT(uartStreamBuffer != NULL);
    configASSERT(xFrameQueue != NULL);
    configASSERT(atQueue != NULL);
}

/* ==================== CRC32 计算（含帧头字段） ==================== */

uint32_t calculate_crc32_ex(uint8_t total, uint8_t current, uint16_t len, uint8_t *data)
{
    uint32_t i;
    uint32_t words;
    uint32_t rest;
    uint32_t *p;
    uint8_t tmp[4];

    __HAL_RCC_CRC_CLK_ENABLE();   /* 使能 CRC 时钟（AHB1） */
    CRC->CR = CRC_CR_RESET;       /* 复位 CRC 数据寄存器 */

    /* 先算 4 字节头：total(1B) + current(1B) + len(2B,小端)，刚好拼成 1 个 32 位字 */
    uint32_t header = ((uint32_t)total)
                    | ((uint32_t)current << 8)
                    | ((uint32_t)len << 16);
    CRC->DR = header;

    /* 再算实际数据，只算 len 字节 */
    p     = (uint32_t *)data;
    words = len / 4;
    rest  = len % 4;

    for (i = 0; i < words; i++) {
        CRC->DR = p[i];
    }

    /* 剩余 1~3 字节，用 0xFF 补齐到 4 字节 */
    if (rest > 0) {
        tmp[0] = 0xFF;
        tmp[1] = 0xFF;
        tmp[2] = 0xFF;
        tmp[3] = 0xFF;
        for (i = 0; i < rest; i++) {
            tmp[i] = data[words * 4 + i];
        }
        CRC->DR = *(uint32_t *)tmp;
    }

    return CRC->DR;
}

/*
 * 整包 / 元数据 CRC：按小端组字，尾部不足 4 字节用 0xFF 补齐。
 * 与 Bootloader 中的实现必须完全一致。
 */
uint32_t ota_crc32_bytes(const void *data, uint32_t len)
{
    const uint8_t *p = (const uint8_t *)data;
    uint32_t i;
    uint32_t j;
    uint32_t word;

    __HAL_RCC_CRC_CLK_ENABLE();
    CRC->CR = CRC_CR_RESET;

    for (i = 0; i + 4U <= len; i += 4U) {
        word = (uint32_t)p[i]
             | ((uint32_t)p[i + 1U] << 8)
             | ((uint32_t)p[i + 2U] << 16)
             | ((uint32_t)p[i + 3U] << 24);
        CRC->DR = word;
    }

    if (i < len) {
        word = 0xFFFFFFFFU;
        for (j = 0; i + j < len; j++) {
            word &= ~(0xFFU << (8U * j));
            word |= ((uint32_t)p[i + j] << (8U * j));
        }
        CRC->DR = word;
    }

    return CRC->DR;
}

/* ==================== 扇区信息 ==================== */

/* 获取指定地址所在的扇区编号 (F407 专用) */
uint32_t get_sector(uint32_t addr)
{
    if (addr < 0x08004000U) return FLASH_SECTOR_0;
    if (addr < 0x08008000U) return FLASH_SECTOR_1;
    if (addr < 0x0800C000U) return FLASH_SECTOR_2;
    if (addr < 0x08010000U) return FLASH_SECTOR_3;
    if (addr < 0x08020000U) return FLASH_SECTOR_4;  /* 64KB */
    if (addr < 0x08040000U) return FLASH_SECTOR_5;  /* 128KB 开始 */
    if (addr < 0x08060000U) return FLASH_SECTOR_6;
    if (addr < 0x08080000U) return FLASH_SECTOR_7;
    if (addr < 0x080A0000U) return FLASH_SECTOR_8;
    if (addr < 0x080C0000U) return FLASH_SECTOR_9;
    if (addr < 0x080E0000U) return FLASH_SECTOR_10;
    return FLASH_SECTOR_11;
}

uint32_t get_sector_base(uint32_t addr)
{
    if (addr < 0x08004000U) return 0x08000000U;
    if (addr < 0x08008000U) return 0x08004000U;
    if (addr < 0x0800C000U) return 0x08008000U;
    if (addr < 0x08010000U) return 0x0800C000U;
    if (addr < 0x08020000U) return 0x08010000U;
    if (addr < 0x08040000U) return 0x08020000U;
    if (addr < 0x08060000U) return 0x08040000U;
    if (addr < 0x08080000U) return 0x08060000U;
    if (addr < 0x080A0000U) return 0x08080000U;
    if (addr < 0x080C0000U) return 0x080A0000U;
    if (addr < 0x080E0000U) return 0x080C0000U;
    return 0x080E0000U;
}

uint32_t get_sector_size(uint32_t sector)
{
    if (sector <= FLASH_SECTOR_3) {
        return 16U * 1024U;
    }
    if (sector == FLASH_SECTOR_4) {
        return 64U * 1024U;
    }
    return 128U * 1024U;
}

/* ==================== Flash 擦写（带区域保护） ==================== */

typedef struct {
    uint32_t start;
    uint32_t end;      /* 不包含 */
} flash_region_t;

/*
 * 只允许擦写这五个区域。Bootloader、以及任何越界地址都会被直接拒绝，
 * 避免 firmware_size 异常或地址算错时误擦 Bootloader / 元数据。
 * B 槽的物理末端是 0x080A0000（S8 末尾），但镜像可用上限仍由 SLOT_CAPACITY 限制。
 */
static const flash_region_t g_flash_regions[] = {
    { STAGING_BASE, STAGING_END },
    { SLOT_A_BASE,  SLOT_A_END },
    { SLOT_B_BASE,  SLOT_B_ERASE_END },
    { META0_BASE,   META0_BASE + META_SECTOR_SIZE },
    { META1_BASE,   META1_BASE + META_SECTOR_SIZE }
};

static int flash_range_allowed(uint32_t start, uint32_t size)
{
    uint32_t end;
    uint32_t i;

    if (size == 0U) {
        return 0;
    }
    if (start > 0xFFFFFFFFU - size) {
        return 0;
    }
    end = start + size;

    for (i = 0; i < (sizeof(g_flash_regions) / sizeof(g_flash_regions[0])); i++) {
        if (start >= g_flash_regions[i].start && end <= g_flash_regions[i].end) {
            return 1;
        }
    }
    return 0;
}

/* Flash 擦除 (按扇区粒度，支持跨扇区连续擦除) */
StatusTypeDef flash_erase(uint32_t start_addr, uint32_t size)
{
    FLASH_EraseInitTypeDef erase_init;
    uint32_t sector_error = 0;
    uint32_t start_sector;
    uint32_t end_sector;

    if (!flash_range_allowed(start_addr, size)) {
        return Status_ERROR;
    }

    start_sector = get_sector(start_addr);
    end_sector   = get_sector(start_addr + size - 1U);

    if (HAL_FLASH_Unlock() != HAL_OK)
        return Status_ERROR;

    erase_init.TypeErase    = FLASH_TYPEERASE_SECTORS;
    erase_init.VoltageRange = FLASH_VOLTAGE_RANGE_3; // 2.7V~3.6V
    erase_init.Sector       = start_sector;
    erase_init.NbSectors    = end_sector - start_sector + 1U; // 计算连续要擦除的扇区数

    if (HAL_FLASHEx_Erase(&erase_init, &sector_error) != HAL_OK) {
        HAL_FLASH_Lock();
        return Status_ERROR;
    }

    HAL_FLASH_Lock();
    return Status_OK;
}

/* Flash 写入 */
StatusTypeDef flash_write(uint32_t dest_addr, uint8_t *src, uint32_t size)
{
    uint32_t i;
    uint32_t word;

    if (!flash_range_allowed(dest_addr, size)) {
        return Status_ERROR;
    }

    if (HAL_FLASH_Unlock() != HAL_OK)
        return Status_ERROR;

    for (i = 0; i < size; i += 4) {
        // 最后一个不足 4 字节时用 0xFF 补齐（Flash 擦除后默认值）
        uint8_t b0 = (i + 0 < size) ? src[i + 0] : 0xFF;
        uint8_t b1 = (i + 1 < size) ? src[i + 1] : 0xFF;
        uint8_t b2 = (i + 2 < size) ? src[i + 2] : 0xFF;
        uint8_t b3 = (i + 3 < size) ? src[i + 3] : 0xFF;

        word = (uint32_t)b0
             | ((uint32_t)b1 << 8)
             | ((uint32_t)b2 << 16)
             | ((uint32_t)b3 << 24);

        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, dest_addr + i, word) != HAL_OK) {
            HAL_FLASH_Lock();
            return Status_ERROR;
        }
    }

    HAL_FLASH_Lock();
    return Status_OK;
}

/* ==================== F103 镜像暂存区（S4） ==================== */

/*
 * 从 Flash 直接读回并累加到硬件 CRC，尾部不足 4 字节补 0xFF。
 * 算法与 ota_crc32_bytes 完全一致，区别是不把整包读进 RAM：
 * F103 镜像最大 64KB，先 memcpy 到缓冲区会把堆直接吃穿。
 */
static uint32_t ota_staging_crc32(uint32_t addr, uint32_t len)
{
    const volatile uint8_t *p = (const volatile uint8_t *)addr;
    uint32_t i = 0U;
    uint32_t word;
    uint32_t j;

    __HAL_RCC_CRC_CLK_ENABLE();
    CRC->CR = CRC_CR_RESET;

    for (; i + 4U <= len; i += 4U) {
        word = (uint32_t)p[i]
             | ((uint32_t)p[i + 1U] << 8)
             | ((uint32_t)p[i + 2U] << 16)
             | ((uint32_t)p[i + 3U] << 24);
        CRC->DR = word;
    }

    if (i < len) {
        word = 0xFFFFFFFFU;
        for (j = 0U; i + j < len; j++) {
            word &= ~(0xFFU << (8U * j));
            word |= ((uint32_t)p[i + j] << (8U * j));
        }
        CRC->DR = word;
    }

    return CRC->DR;
}

/* 擦除整个 S4（64KB 扇区，约 1~2 秒，期间 CPU 停指） */
int ota_staging_erase(void)
{
    return (flash_erase(STAGING_BASE, STAGING_SIZE) == Status_OK) ? 0 : -1;
}

int ota_staging_write(uint32_t offset, const uint8_t *src, uint32_t len)
{
    if ((src == NULL) || (len == 0U))  return -1;
    if ((offset % 4U) != 0U)           return -2;   /* 底层按字编程，起始地址必须对齐 */
    if (offset > STAGING_SIZE)         return -3;
    if (len > (STAGING_SIZE - offset)) return -3;

    return (flash_write(STAGING_BASE + offset, (uint8_t *)src, len) == Status_OK) ? 0 : -4;
}

int ota_staging_read(uint32_t offset, uint8_t *dst, uint32_t len)
{
    if ((dst == NULL) || (len == 0U))  return -1;
    if (offset > STAGING_SIZE)         return -2;
    if (len > (STAGING_SIZE - offset)) return -2;

    memcpy(dst, (const void *)(STAGING_BASE + offset), len);
    return 0;
}

int ota_staging_verify(uint32_t size, uint32_t crc)
{
    if ((size == 0U) || (size > STAGING_SIZE)) return -1;

    return (ota_staging_crc32(STAGING_BASE, size) == crc) ? 0 : -2;
}

/*
 * 自测：擦除 -> 三处写图案 -> 读回比对 -> 验证尾部补齐与 CRC 通路。
 * 覆盖三个易错点：首地址、中间地址、尾部最后一个字；外加"长度不齐"的写入。
 * 返回 0 成功，负数为失败步骤，便于定位。
 */
int ota_staging_selftest(void)
{
    uint8_t  wr[16];
    uint8_t  rd[16];
    uint32_t crc;
    uint32_t i;

    if (ota_staging_erase() != 0) return -1;

    /* ① 起始地址 */
    for (i = 0U; i < sizeof(wr); i++) wr[i] = (uint8_t)(0x5AU + i);
    if (ota_staging_write(0U, wr, sizeof(wr)) != 0) return -2;
    if (ota_staging_read(0U, rd, sizeof(rd)) != 0)  return -2;
    if (memcmp(wr, rd, sizeof(wr)) != 0)            return -2;

    /* ② 中间地址 */
    for (i = 0U; i < sizeof(wr); i++) wr[i] = (uint8_t)(0xA5U ^ i);
    if (ota_staging_write(STAGING_SIZE / 2U, wr, sizeof(wr)) != 0) return -3;
    if (ota_staging_read(STAGING_SIZE / 2U, rd, sizeof(rd)) != 0)  return -3;
    if (memcmp(wr, rd, sizeof(wr)) != 0)                           return -3;

    /* ③ 尾部最后一个字：边界与对齐最容易在这里出错 */
    for (i = 0U; i < sizeof(wr); i++) wr[i] = (uint8_t)(0x3CU + i);
    if (ota_staging_write(STAGING_SIZE - sizeof(wr), wr, sizeof(wr)) != 0) return -4;
    if (ota_staging_read(STAGING_SIZE - sizeof(wr), rd, sizeof(rd)) != 0)  return -4;
    if (memcmp(wr, rd, sizeof(wr)) != 0)                                   return -4;

    /* ④ 长度不齐（写 3 字节）：验证底层补 0xFF */
    wr[0] = 0x11U;
    wr[1] = 0x22U;
    wr[2] = 0x33U;
    wr[3] = 0xFFU;
    if (ota_staging_write(16U, wr, 3U) != 0) return -5;
    if (ota_staging_read(16U, rd, 4U) != 0)  return -5;
    if (memcmp(wr, rd, 4U) != 0)             return -5;

    /* ⑤ CRC 通路：正确 CRC 必须通过，错误 CRC 必须被拒绝 */
    crc = ota_staging_crc32(STAGING_BASE, 1040U);
    if (ota_staging_verify(1040U, crc) != 0)        return -6;
    if (ota_staging_verify(1040U, crc ^ 1U) == 0)   return -6;

    return 0;
}

/* ==================== 元数据双副本 ==================== */

/*
 * 元数据工作缓冲（放 .bss，不放栈）。
 *
 * ota_config_t 有 302 字节，最早每个调用者都在栈上各留一份，
 * 最深路径 confirm(302) → save(302) → crc(302) 接近 900 字节，
 * 直接把 512 字节的喂狗任务栈踩穿（表现为 HardFault UNDEFINSTR）。
 * 现在统一复用这两块静态缓冲，调用方零栈开销。
 *
 * 并发前提：同一时刻只允许一处进入"读-改-写"流程。
 * 任务上下文由 freertos.c 的 g_meta_busy 串行化；
 * APP 启动阶段调度器尚未启动，天然独占。
 */
static ota_config_t s_meta;
static ota_config_t s_crc_tmp;

static uint32_t ota_meta_crc(const ota_config_t *cfg)
{
    memcpy(&s_crc_tmp, cfg, sizeof(s_crc_tmp));
    memset(&s_crc_tmp.meta_crc, 0xFF, sizeof(s_crc_tmp.meta_crc));  /* 计算时该字段按 0xFFFFFFFF */

    return ota_crc32_bytes(&s_crc_tmp, sizeof(s_crc_tmp));
}

static int ota_meta_is_valid(const ota_config_t *cfg)
{
    if (cfg->magic != OTA_META_MAGIC) {
        return 0;
    }
    if (cfg->active_slot > 1U || cfg->prev_slot > 1U) {
        return 0;
    }
    if (cfg->trial_pending > 1U) {
        return 0;
    }
    if (cfg->slot_state[0] > SLOT_STATE_INVALID ||
        cfg->slot_state[1] > SLOT_STATE_INVALID) {
        return 0;
    }
    if (cfg->slot_size[0] > SLOT_CAPACITY ||
        cfg->slot_size[1] > SLOT_CAPACITY) {
        return 0;
    }
    return (ota_meta_crc(cfg) == cfg->meta_crc) ? 1 : 0;
}

/* 只取最新副本的 seq：避免为了一个字段在栈上复制整份 302 字节结构体 */
static uint32_t ota_meta_latest_seq(void)
{
    const ota_config_t *copy0 = (const ota_config_t *)META0_BASE;
    const ota_config_t *copy1 = (const ota_config_t *)META1_BASE;
    int valid0 = ota_meta_is_valid(copy0);
    int valid1 = ota_meta_is_valid(copy1);

    if (!valid0 && !valid1) {
        return 0U;
    }
    if (valid1 && (!valid0 || copy1->seq > copy0->seq)) {
        return copy1->seq;
    }
    return copy0->seq;
}

/*
 * 读元数据：两份副本分别校验 magic + meta_crc，取 seq 较大的一份。
 * 返回 0 成功，-1 表示两份都无效。
 */
int ota_meta_load(ota_config_t *out)
{
    const ota_config_t *copy0 = (const ota_config_t *)META0_BASE;
    const ota_config_t *copy1 = (const ota_config_t *)META1_BASE;
    int valid0 = ota_meta_is_valid(copy0);
    int valid1 = ota_meta_is_valid(copy1);

    if (!valid0 && !valid1) {
        return -1;
    }

    if (valid1 && (!valid0 || copy1->seq > copy0->seq)) {
        memcpy(out, copy1, sizeof(*out));
    } else {
        memcpy(out, copy0, sizeof(*out));
    }

    return 0;
}

/* 选一个“非活动副本”作为本次写入目标：优先无效副本，其次 seq 较小的旧副本 */
static uint32_t ota_meta_write_addr(void)
{
    const ota_config_t *copy0 = (const ota_config_t *)META0_BASE;
    const ota_config_t *copy1 = (const ota_config_t *)META1_BASE;
    int valid0 = ota_meta_is_valid(copy0);
    int valid1 = ota_meta_is_valid(copy1);

    if (!valid0) {
        return META0_BASE;
    }
    if (!valid1) {
        return META1_BASE;
    }
    return (copy0->seq <= copy1->seq) ? META0_BASE : META1_BASE;
}

/*
 * 写元数据：写非活动副本 → 读回校验 → 下一次读取时 seq 更大的新副本生效。
 * 由于写入目标始终是另一份副本所在扇区，擦除过程中活动副本不会被破坏。
 */
int ota_meta_save(ota_config_t *cfg)
{
    uint32_t addr;

    cfg->magic = OTA_META_MAGIC;
    cfg->seq   = ota_meta_latest_seq() + 1U;
    if (cfg->seq == 0U) {
        cfg->seq = 1U;
    }
    addr       = ota_meta_write_addr();

    cfg->meta_crc = ota_meta_crc(cfg);

    if (flash_erase(addr, META_SECTOR_SIZE) != Status_OK) {
        return -1;
    }
    if (flash_write(addr, (uint8_t *)cfg, sizeof(*cfg)) != Status_OK) {
        return -2;
    }
    if (memcmp((const void *)addr, cfg, sizeof(*cfg)) != 0) {
        return -3;
    }

    return 0;
}

/* ==================== APP 启动确认 ==================== */

/* 检查槽内固件的向量表：SP 落在 SRAM，复位向量落在本槽范围内 */
int ota_image_vectors_valid(uint32_t slot_base)
{
    uint32_t sp = *(__IO uint32_t *)slot_base;
    uint32_t reset_vector = *(__IO uint32_t *)(slot_base + 4U);

    if ((sp & 0x7U) != 0U) {
        return 0;
    }
    if (sp < 0x20000000U || sp > 0x20020000U) {   /* F407VG: 128KB SRAM */
        return 0;
    }

    reset_vector &= ~1U;
    if (reset_vector < slot_base || reset_vector >= (slot_base + SLOT_CAPACITY)) {
        return 0;
    }

    return 1;
}

static int ota_slot_index_from_base(uint32_t image_base)
{
    if (image_base == SLOT_A_BASE) {
        return 0;
    }
    if (image_base == SLOT_B_BASE) {
        return 1;
    }
    return -1;
}

/*
 * APP 每次启动时调用（不负责试运行确认）：
 *   1. 元数据缺失 → 用当前运行的槽建立第一份元数据（工厂镜像，size/crc 未知）；
 *   2. active_slot 与实际运行槽不一致 → 说明 Bootloader 做了回退，修正为当前槽，
 *      并把启动失败的另一个槽标记为 INVALID；
 *   3. version 记录为当前固件版本；
 *   4. trial_pending 原样保留，等心跳任务确认健康后再清。
 */
void ota_app_boot_init(uint32_t image_base)
{
    ota_config_t *meta = &s_meta;
    int slot = ota_slot_index_from_base(image_base);
    int dirty = 0;

    if (slot < 0) {
        return;   /* 镜像链接地址不在 A/B 槽内，不写元数据 */
    }

    if (ota_meta_load(meta) != 0) {
        memset(meta, 0, sizeof(*meta));
        meta->magic = OTA_META_MAGIC;
        meta->active_slot = (uint8_t)slot;
        meta->prev_slot = (uint8_t)(slot ^ 1);
        meta->slot_state[slot] = SLOT_STATE_VALID;
        meta->version[slot] = CURRENT_VERSION;
        meta->slot_size[slot] = 0;   /* 工厂镜像没有 CRC 信息，Bootloader 跳过整包校验 */
        meta->slot_crc[slot] = 0;
        meta->trial_pending = 0;
        ota_meta_save(meta);
        return;
    }

    if (meta->active_slot != (uint8_t)slot) {
        /*
         * active_slot 指向的槽没能启动，Bootloader 回退到了当前槽。
         * 新的 Bootloader 会自己写回元数据，这里只做兜底：
         * 修正 active_slot，并把启动失败的槽标记为 INVALID，避免反复尝试。
         */
        uint8_t failed = meta->active_slot;
        meta->prev_slot = failed;
        meta->active_slot = (uint8_t)slot;
        meta->slot_state[failed] = SLOT_STATE_INVALID;
        meta->trial_pending = 0;
        meta->trial_count = 0;
        dirty = 1;
    }
    if (meta->slot_state[slot] != SLOT_STATE_VALID) {
        meta->slot_state[slot] = SLOT_STATE_VALID;
        dirty = 1;
    }
    if (meta->version[slot] != CURRENT_VERSION) {
        meta->version[slot] = CURRENT_VERSION;
        dirty = 1;
    }

    if (dirty) {
        ota_meta_save(meta);
    }
}

/*
 * 心跳任务在“所有关键任务连续健康运行 OTA_CONFIRM_WINDOW_MS”后调用。
 * 只有 trial_pending == 1 且当前运行的正是 active_slot 时才确认。
 */
int ota_app_confirm_healthy(uint32_t image_base)
{
    ota_config_t *meta = &s_meta;
    int slot = ota_slot_index_from_base(image_base);

    if (slot < 0) {
        return -1;
    }
    if (ota_meta_load(meta) != 0) {
        return -1;
    }
    if (meta->active_slot != (uint8_t)slot || meta->trial_pending == 0U) {
        return 0;   /* 没有待确认的试运行 */
    }

    meta->trial_pending = 0;
    meta->trial_count = 0;
    if (ota_meta_save(meta) != 0) {
        return -1;
    }
    return 1;
}

/*
 * 下载完成后请求切换到目标槽：填好元数据写回，下次复位由 Bootloader 跳过去。
 *
 * current_slot 是当前正在运行的槽，target_slot 是刚下载好的槽。
 * trial_pending 置 1，由新固件的心跳任务在健康运行 OTA_CONFIRM_WINDOW_MS 后清零；
 * 若一直不确认，Bootloader 会在 trial_count 达到上限后回滚到 prev_slot。
 *
 * 调用者必须保证与其它元数据操作互斥（任务上下文用 g_meta_busy 串行化）。
 */
int ota_meta_request_switch(uint8_t current_slot, uint8_t target_slot,
                            uint32_t image_size, uint32_t image_crc)
{
    ota_config_t *meta = &s_meta;

    if (current_slot > 1U || target_slot > 1U || current_slot == target_slot) {
        return -1;
    }

    if (ota_meta_load(meta) != 0) {
        /* 元数据缺失（工厂状态）：把当前实际运行的槽登记为 VALID */
        memset(meta, 0, sizeof(*meta));
        meta->magic = OTA_META_MAGIC;
        meta->active_slot = current_slot;
        meta->prev_slot = target_slot;
        meta->slot_state[current_slot] = SLOT_STATE_VALID;
        meta->version[current_slot] = CURRENT_VERSION;
        meta->slot_size[current_slot] = 0;
        meta->slot_crc[current_slot] = 0;
    }

    meta->magic = OTA_META_MAGIC;
    meta->prev_slot = current_slot;
    meta->active_slot = target_slot;
    meta->trial_pending = 1;
    meta->trial_count = 0;
    meta->slot_state[target_slot] = SLOT_STATE_VALID;
    meta->slot_state[current_slot] = SLOT_STATE_VALID;
    meta->version[target_slot] = CURRENT_VERSION;  /* 新固件启动后会修正为自己的版本 */
    meta->slot_size[target_slot] = image_size;
    meta->slot_crc[target_slot] = image_crc;
    meta->dl_state = 0;
    meta->dl_slot = target_slot;
    meta->dl_total = 0;
    memset(meta->rx_bitmap, 0, sizeof(meta->rx_bitmap));

    return (ota_meta_save(meta) == 0) ? 0 : -1;
}
