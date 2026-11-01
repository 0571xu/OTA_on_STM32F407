# STM32F407 A/B 双槽 OTA 工程

> 硬件：STM32F407VGT6（1MB Flash / 128KB SRAM）
> 工具链：Keil MDK 5.06 + ARMCC V5.06 update 7
> 说明：本文件描述"当前代码已经做到什么"，不含规划中的功能。

---

## 1. 项目构成

| 工程 | 目录 | 作用 |
|---|---|---|
| Bootloader | `bootloaderCode/` | 上电后决定跳哪个槽、维护试运行计数与回滚 |
| APP | `bootloader_test/` | 业务固件，含 A/B 两个构建 Target |
| 上位机脚本 | `ota_send.py` | 通过串口把固件打包下发 |

APP 的两个 Target：

| Target | 预定义宏 | 链接地址 | 说明 |
|---|---|---|---|
| `bootloader_test` | `USE_HAL_DRIVER,STM32F407xx` | `0x08020000` | A 版 |
| `bootloader_test_slotB` | `USE_HAL_DRIVER,STM32F407xx,APP_SLOT_B` | `0x08060000` | B 版 |

**运行在哪个槽由链接地址唯一确定**，不靠元数据猜。`APP_IMAGE_BASE` / `APP_IMAGE_SLOT` 由 `APP_SLOT_B` 宏切换。

---

## 2. 目前已实现的功能

### 2.1 F407 自身 OTA（完整可用）

| 功能 | 状态 | 说明 |
|---|---|---|
| A/B 双槽升级 | 已实现 | 收到镜像写入"非当前运行槽"，绝不擦正在跑的固件 |
| 试运行（trial） | 已实现 | 新固件连续健康运行 5 秒才确认；否则计数，连续失败 3 次自动回滚 |
| 元数据双副本 | 已实现 | S2 / S3 各一份，`seq` 大且 `meta_crc` 正确的一份生效 |
| 掉电保护 | 已实现 | 写新副本前只擦"旧副本"所在扇区，另一份始终可读 |
| 整包校验 | 已实现 | 收完后直接读回 Flash 算 CRC32 + 向量表合法性检查，双重通过才切槽 |
| 逐帧 CRC | 已实现 | 每帧 268 字节，帧头字段 + 数据一起参与硬件 CRC32 |
| 心跳式看门狗 | 已实现 | 4 个关键任务 1 秒内都打卡才喂狗，保护"任务级存活" |
| 启动打点 | 已实现 | `[boot] 1..7` 逐步打印，定位卡在哪一步 |
| 栈/堆水位上报 | 已实现 | 上电 10 秒打印一次 `[mem]` |

### 2.2 CAN（只到自测层）

| 功能 | 状态 | 说明 |
|---|---|---|
| CAN1 500kbps 初始化 | 已实现 | PSC=6 / BS1=11TQ / BS2=2TQ / SJW=1TQ |
| 回环自检 | 已实现 | 上电发 3 帧对比，串口打印 `[can] loopback 3/3` |
| 收发任务骨架 | 已实现 | `can_start_task` / `can_if_send` / `can_if_recv` |
| ISO-TP（ISO 15765-2） | 发送端已实现 | `isotp.c`：SF / FF / CF 拆包、序号 1~15 循环、流控 BS/STmin、N_Bs 5 秒超时；上电回环自测 300 字节 |
| UDS 子集 | 主链路已实现 | `uds.c`：0x10 会话 / 0x3E 保活 / 0x34 请求下载 / 0x36 传数据 / 0x37 结束；含 0x78 pending 与 NRC 处理；尚未接入任务调用 |
| F103 节点 OTA | **部分实现** | S4 暂存区擦写/读回/整包校验已完成（含上电自测）；TCP 下载与 CAN 下发未做 |

### 2.3 WiFi（只到"函数已移植"）

| 功能 | 状态 | 说明 |
|---|---|---|
| AT 配置三函数 | 已移植 | `wifi.c` 里的 `wait_response` / `wait_connect_response` / `connect_init` |
| AT 行解析 | 已实现 | `wifi_getTask` 在等帧头的同时把 AT 行投递到 `atQueue` |
| TCP 下载固件 | **未实现** | `connect_init()` 目前没有被任何任务调用 |
| 服务器取固件 | **未实现** | `otasend` 改 TCP 的部分还没接 |

### 2.4 明确不做的事

- **不再使用 AT24C64 EEPROM**：F103 端改为"单槽 + Bootloader 内接收"，失败次数耗尽后通过 CAN 上报 F407。

---

## 3. 引脚分配

### 3.1 串口

| 功能 | 引脚 | 外设 | 参数 |
|---|---|---|---|
| 调试串口 TX | `PA9` | USART1 | 115200 8N1 |
| 调试串口 RX | `PA10` | USART1 | 接 PC，收 OTA 帧；DMA2_Stream2/7 + IDLE 中断 |
| ESP8266 TX | `PD5` | USART2 | 115200 8N1；DMA1_Stream5/6 |
| ESP8266 RX | `PD6` | USART2 | |

> USART1 是**调试 + OTA 数据**共用口，ESP8266 走 USART2，两者互不干扰。

### 3.2 CAN

| 功能 | 引脚 | 外设 | 参数 |
|---|---|---|---|
| CAN1_RX | `PA11` | CAN1 | 500kbps |
| CAN1_TX | `PA12` | CAN1 | 外接 TJA1050（需 5V 供电） |

### 3.3 LCD（SPI1，只写不读）

| 功能 | 引脚 | 说明 |
|---|---|---|
| SPI1_SCK | `PA5` | 时钟 |
| SPI1_MOSI | `PA7` | 数据 |
| LCD_CS | `PA4` | 片选 |
| LCD_RS | `PA3` | 命令/数据 |
| LCD_RET | `PA2` | 复位 |
| LCD_BL | `PA6` | 背光 |

### 3.4 其它

| 功能 | 引脚 | 说明 |
|---|---|---|
| 运行指示灯 | `PA1` | 500ms 翻转；Bootloader 在"无可用固件"时也用它闪灯报警 |
| IWDG | 内部 | LSI 32kHz / 64 分频，重装 4095 → 约 **8.2 秒**超时 |

---

## 4. Flash 分区

### 4.1 布局

| 区域 | 起始地址 | 结束地址 | 大小 | 扇区 |
|---|---|---|---|---|
| Bootloader | `0x08000000` | `0x08007FFF` | 32KB | S0 ~ S1 |
| 元数据副本 0 | `0x08008000` | `0x0800BFFF` | 16KB | S2 |
| 元数据副本 1 | `0x0800C000` | `0x0800FFFF` | 16KB | S3 |
| **F103 镜像暂存** | `0x08010000` | `0x0801FFFF` | 64KB | S4 |
| **APP 槽 A** | `0x08020000` | `0x0805FFFF` | 256KB | S5 ~ S6 |
| **APP 槽 B** | `0x08060000` | `0x0809FFFF` | 256KB | S7 ~ S8 |
| 预留 | `0x080A0000` | `0x080FFFFF` | 384KB | S9 ~ S11 |

当前 APP 编译产物约 28KB（`Code=27448 RO=1088 RW=256 ZI=31048`），256KB 槽有约 9 倍余量。

### 4.2 设计约束

1. **两份元数据必须在不同扇区**：F407 最小擦除单位是扇区，同扇区放两份等于没有副本。
2. **两个槽容量必须一致**：都受 `SLOT_CAPACITY = 256KB` 限制，否则大镜像能升到 B 却回不到 A。
3. **槽起点必须落在 128KB 扇区边界**：A 从 S5、B 从 S7 起，否则擦 B 槽首个扇区会连带擦掉 A 槽尾部。
4. **暂存区是单扇区**：S4 = 64KB，正好等于 F103 整片 Flash；写坏时只擦这一个扇区。

### 4.3 元数据（`ota_config_t`，302 字节）

| 字段 | 作用 |
|---|---|
| `magic` / `meta_crc` | 合法性判定（`0x4F544132` = "OTA2"） |
| `seq` | 双副本新旧比较，越大越新 |
| `active_slot` / `prev_slot` | 本次启动槽 / 回滚目标槽 |
| `trial_pending` / `trial_count` | 试运行标记与失败计数（上限 3） |
| `slot_state[2]` | 各槽 EMPTY / VALID / INVALID |
| `slot_size[2]` / `slot_crc[2]` | 各槽镜像长度与整包 CRC32 |
| `version[2]` | 各槽版本号 |
| `dl_state` / `dl_slot` / `dl_total` / `rx_bitmap` | **断点续传预留字段，当前未启用** |

Bootloader 与 APP 各有一份结构体定义，**布局必须完全一致**；两边都用编译期断言把大小钉死在 302 字节。

### 4.4 启动决策（Bootloader）

1. 元数据有效 → 检查 `active_slot` 的状态、size、CRC、向量表；
2. 都合法 → 若 `trial_pending`，本次是 IWDG 复位则 `trial_count++`，达到 3 次回滚到 `prev_slot` 并标记失败槽；
3. `active_slot` 本身不合法 → 直接回退另一槽，不消耗试运行次数；
4. 元数据无效 → 按向量表依次尝试 A、B（出厂首次烧录的兜底路径）。

---

## 5. 目录结构

> 本文件位于 `bootloader_test/`，下面这张图描述的是仓库根目录 `F407/` 的整体结构。

```
F407/
├── bootloaderCode/          Bootloader 工程（Keil）
│   └── Core/Src/main.c      分区常量、元数据、启动决策、跳转
├── bootloader_test/         APP 工程（Keil，A/B 两个 Target）
│   ├── README.md            本文件
│   ├── Core/Inc/            ota_receive.h（分区/元数据/帧格式）、can.h、isotp.h、wifi.h
│   ├── Core/Src/            ota_receive.c、can.c、isotp.c、freertos.c（任务与心跳）
│   └── MDK-ARM/wifi.c       ESP8266 AT 配置（不在 IncludePath，故 .c 放这里）
├── ota_send.py              上位机发送脚本
├── OTA问题与踩坑总结.md      问题清单与复盘
└── bootloader_test - 副本 (2)/  旧 448KB 分区布局的工程快照（仅供参考，勿改勿用）
```

---

## 6. 使用方法

### 6.1 编译

Keil 打开 `bootloader_test/MDK-ARM/bootloader_test.uvprojx`，在工具栏 Target 下拉框里选：

- `bootloader_test` → A 版
- `bootloader_test_slotB` → B 版

Bootloader 单独打开 `bootloaderCode/MDK-ARM/bootloaderCode.uvprojx`。

命令行编译（可选）：

```powershell
& "C:\Keil_v5\UV4\UV4.exe" -b "D:\Code\stm学习\F407\bootloader_test\MDK-ARM\bootloader_test.uvprojx" -t bootloader_test -j0 -o build_A.log
```

退出码：`0` = 无错误无警告，`1` = 有警告，`2` = 有错误。
命令行模式下 Log 只会把该 Target 的产物写到工程目录；`build_A.log` 可直接删除。

### 6.2 烧录顺序（顺序不能颠倒）

1. **整片擦除**（Erase Full Chip）；
2. 烧 `bootloaderCode.hex` → `0x08000000`；
3. 烧 A 版 `bootloader_test.hex` → `0x08020000`；
4. 上电，串口 115200 应看到：

```
bootloader begin
boot slot A
[boot] build <日期> <时间>
[boot] 1 usart1 ok
[boot] 4 can msp ok
[boot] 5 usart2 ok
[boot] 6 meta ok
[boot] 7 start scheduler
```

> **改过分区布局后，第 1 步必须做整片擦除。**
> 元数据里不存槽地址，只存 size / crc。旧布局留下的元数据格式仍然合法，新 Bootloader 会拿旧 CRC 去校验新地址上的新固件，必然不匹配 → 两个槽都判无效 → 串口只剩 `no valid app`。

日常只改 APP、不动分区时，用默认的 "Erase Sectors" 即可，避免把 Bootloader 一起擦掉。

### 6.3 执行一次 OTA

设备当前跑 A，就发 B 镜像；跑 B，就发 A 镜像。

```powershell
python ota_send.py "D:\Code\stm学习\F407\bootloader_test\MDK-ARM\bootloader_test_slotB\bootloader_test_slotB.hex" -p COM4 -b 115200
```

只打包不发送（检查镜像地址与 CRC）：

```powershell
python ota_send.py firmware.hex --dry-run
```

脚本会从 `.hex` 的记录地址判断这是 A 版还是 B 版，地址不属于两个槽会直接报错退出。`.bin` 不含地址信息，需要自己确认。

### 6.4 关键日志

| 日志 | 含义 |
|---|---|
| `boot slot A` / `boot slot B` | Bootloader 选定槽并跳转 |
| `trial boot, wait confirm` | 新固件首次运行，等 5 秒确认 |
| `[hb] confirm begin/done` | 试运行确认成功，已清 `trial_pending` |
| `trial failed, rollback` | 连续 3 次启动失败，已回滚 |
| `active invalid, fallback` | active 槽固件损坏，直接回退另一槽 |
| `upgrade ok, reboot to new slot...` | 元数据写成功，即将复位 |
| `ota verify fail! recv=x/y` | 收包数不符或整包 CRC / 向量表校验失败 |
| `[mem] heap free=... min=...` | 堆剩余与历史最低 |
| `[mem] stack left(w): ...` | 各任务栈最小剩余（单位：word） |
| `[FAULT] CFSR=... HFSR=...` | 硬件异常，只打印一次后等 IWDG 复位 |
| `[STACK] overflow task=...` | FreeRTOS 检测到栈溢出 |

### 6.5 回滚验证

1. 正常升级一次 A→B，确认 `[hb] confirm done` 出现，未回滚；
2. 人为让新固件跑不到 5 秒（例如临时屏蔽某个关键任务的 `hb_set`），重启 3 次后应出现 `trial failed, rollback` 并回到旧槽；
3. 回滚后旧固件的 `slot_state` 仍为 VALID，可继续正常使用。

---

## 7. 串口帧协议（F407 自身 OTA）

固定 268 字节：

| 偏移 | 长度 | 内容 |
|---|---|---|
| 0 | 2 | `0x55 0x44` 帧头 |
| 2 | 1 | 总包数（最大 255） |
| 3 | 1 | 当前包号（`0xFF` = 结束帧） |
| 4 | 2 | 实际数据长度（小端，≤256） |
| 6 | 4 | CRC32（小端，覆盖"总包数+包号+长度+数据") |
| 10 | 256 | 数据区，不足补 `0x00` |
| 266 | 2 | `0x33 0x22` 帧尾 |

结束帧的 8 字节负载：`固件总长(4B 小端) + 整包 CRC32(4B 小端)`。

CRC32 用 STM32 硬件 CRC（CRC-32/MPEG-2，按 32 位字、小端组字，尾部不足 4 字节补 `0xFF`）。

**上限**：总包数只有 1 字节，单次最多 `255 × 256 = 65280` 字节。要传更大的固件，必须同时改上位机打包、APP 解析器、帧 CRC 计算三处。

---

## 8. 已知限制

1. **APP 栈/任务**：`lcd_task` 目前空转（演示代码已注释）；`oledmenuTask` 只是占位打印。
2. **`xWifisendTaskHandle` 为 NULL**：`request_upgradeTask` 里对它的通知是空操作，等 TCP 下载任务建起来才有意义。
3. **断点续传未启用**：`dl_state` / `rx_bitmap` 等字段留着但没参与流程。
4. **CAN 只到传输层**：ISO-TP 只有发送端，UDS（`0x10/0x34/0x36/0x37`）还没写，所以还无法对 F103 真正下发镜像；F103 侧接收代码也还没有。
5. **S4 暂存区只到"能存"**：`ota_staging_erase/write/read/verify` 已实现，上电自测默认开启（`OTA_STAGING_SELFTEST_ON_BOOT = 1`），它会擦掉 S4 里已暂存的镜像，正式使用前必须改成 0。下载与 CAN 下发链路尚未实现。

---

## 9. 相关文档

- [OTA问题与踩坑总结.md](../OTA问题与踩坑总结.md)：问题清单、根因复盘、修改元数据的检查清单、448KB → 256KB 分区变更记录。
- [CAN-OTA帧结构与联调说明.md](./CAN-OTA帧结构与联调说明.md)：CAN / ISO-TP / UDS 三层帧结构逐字节说明、完整时序、编译烧录步骤、故障对照表。
