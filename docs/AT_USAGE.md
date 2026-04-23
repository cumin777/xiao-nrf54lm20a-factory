# XIAO nRF54LM20A Factory UART 命令使用文档

## 1. 概述
本固件基于 `UART21`，当前同时支持：

- 治具正式文本命令协议
- 兼容保留的 `AT+...` 调试接口

当前版本说明：
- 默认关闭 `[DBG]` 调试日志
- 源码中仍保留调试链路与串口鲁棒性增强（如空闲超时提交），后续若需继续定位问题，可再切换到诊断模式

- 统一交互口：`UART21`
- 角色：被动执行（不在固件内强制流程顺序）
- 输出原则：仅输出原始测量数据与执行状态，不输出 PASS/FAIL
- 建议串口参数：`115200 8N1`

## 2. 回包规范

### 2.1 测试数据行
测试类指令会输出结构化数据（至少 1 行）：

```text
+TESTDATA:<STATE>,ITEM=<item>,VALUE=<v>,UNIT=<u>,RAW=<raw>,META=<meta>
```

### 2.2 结束状态行
每条指令最后一行为：

- 成功：`OK`
- 失败：`ERROR:<reason>`

常见 `<reason>`：
- `INVALID_PARAM`
- `HW_NOT_READY`
- `PRECONDITION_NOT_MET`
- `STORAGE_IO_FAIL`
- `UNKNOWN_COMMAND`
- `EXECUTION_FAILED`

## 3. 治具文本命令（V3 正式协议）

### 3.1 `help`
- 功能：输出文本命令和兼容 AT 命令清单
- 期望反馈：多行 `+CMDS:`，最后 `OK`

### 3.2 `gpio set gpio<x> <n> <0|1>`
- 功能：按配对映射表把指定 GPIO 配成输出并设置电平，同时把该对中的另一脚切为输入
- 期望反馈示例：

```text
gpio set gpio1 31
OK
```

- 说明：
  - `gpio set` 的回包不再回显 `value`
  - 一次 `set` 会确定该对 GPIO 中“谁是输出侧、谁是输入侧”

### 3.3 `gpio get gpio<x> <n>`
- 功能：读取该映射对中当前被配置为输入模式的那一脚的电平
- 期望反馈示例：

```text
Value:1
OK
```

- 前置条件：
  - 该 GPIO 所属配对必须先执行过一次 `gpio set`
  - 若尚未执行 `gpio set`，会返回 `ERROR:PRECONDITION_NOT_MET`

### 3.3.1 GPIO 配对映射表
- `gpio1 31 <-> gpio1 3`
- `gpio1 30 <-> gpio1 7`
- `gpio0 0 <-> gpio0 3`
- `gpio0 1 <-> gpio0 4`
- `gpio0 2 <-> gpio0 5`
- `gpio1 6 <-> gpio3 10`
- `gpio1 5 <-> gpio3 9`
- `gpio1 4 <-> gpio3 11`
- `gpio3 0 <-> gpio3 7`
- `gpio3 1 <-> gpio3 6`
- `gpio3 2 <-> gpio3 5`
- `gpio3 3 <-> gpio1 1`
- `gpio3 4 <-> gpio1 2`

### 3.4 `bt init`
- 功能：初始化蓝牙协议栈
- 期望反馈示例：

```text
LMP: version 6.0
OK
```

### 3.5 `bt scan on`
- 功能：启动持续扫描会话；收到 `bt scan off` 前不输出扫描摘要
- 期望反馈示例：

```text
OK
```

- 说明：
  - 当前命令只启动扫描并立即返回 `OK`
  - 扫描期间统计第一条命中的设备和 RSSI 最强的设备
  - 最终结果在 `bt scan off` 时统一输出

### 3.6 `bt scan off`
- 功能：停止当前扫描会话并输出扫描结果（幂等）
- 期望反馈示例：

```text
[DEVICE] AA:BB:CC:DD:EE:FF (random) RSSI -42
[DEVICE] 11:22:33:44:55:66 (random) RSSI -35
bt scan off
OK
```

- 说明：
  - 若扫描期间没有命中任何广播包，则只输出 `bt scan off` 和 `OK`

### 3.7 `sleep mode`
- 功能：复用现有 `SLEEPI` 深睡流程，`OK` 后进入 system off
- 期望反馈示例：

```text
+TESTDATA:STATE5,ITEM=SLEEPI,...
sleep mode
OK
```

### 3.8 `ship mode`
- 功能：进入船运模式
- 期望反馈示例：

```text
+TESTDATA:TEXT,ITEM=SHIPMODE,...
ship mode
OK
```

### 3.9 `mic capture <sec>`
- 功能：采集 DMIC 数据并输出统计值
- 说明：`0` 表示执行一次默认采样窗口；`>0` 表示持续采样对应秒数
- 期望反馈示例：

```text
audio data Max:532 Min:-487 Max consecutive:2
OK
```

### 3.10 `imu get`
- 功能：读取单次六轴样本
- 期望反馈示例：

```text
accel data:0.012345,-0.023456,9.801234\rgyro data: 0.001111,-0.002222,0.003333
OK
```

- 失败反馈：若 IMU 驱动或 I2C 访问未能在 `CONFIG_FACTORY_IMU_SAMPLE_TIMEOUT_MS` 内返回，命令循环继续运行并输出 `ERROR:HW_TIMEOUT`。

### 3.11 `imu off`
- 功能：兼容治具停止采样命令
- 期望反馈示例：

```text
imu off
OK
```

### 3.12 `flash <0-255>`
- 功能：将单字节值安全写入 `storage` 分区并回读校验
- 期望反馈示例：

```text
flash 7 OK
OK
```

### 3.13 `bat get`
- 功能：读取电池电压，优先走 `nPM1300` gauge，失败时回退 ADC
- 期望反馈示例：

```text
bat:4012mv
OK
```

### 3.14 `uart20 on`
- 功能：开启 USB 通路上的 `UART20` 通信测试服务
- 行为：
  - 本命令通过 `UART21` 发送
  - 命令成功后，固件会保持监听 `UART20`
  - 当 `UART20` 收到 `whoami\r\n` 时，固件在 `UART20` 回复 `NRF54LM20A\r\n`
- 期望反馈示例（`UART21`）：

```text
+TESTDATA:STATE1,ITEM=UART20TEST,VALUE=1,UNIT=bool,RAW=uart20_test_enabled,META=uart:uart20;trigger:whoami;reply:NRF54LM20A
uart20 on
OK
```

- 期望反馈示例（`UART20`）：

```text
NRF54LM20A
```

- 前置条件：
  - 板级 `uart20` 已启用
  - USB 端通路已经连接到待测板 `uart20`
  - `UART20` 当前波特率为 `115200 8N1`

## 4. 基础系统 AT 指令（兼容接口）

### 4.1 `AT`
- 功能：连通性测试
- 期望反馈：

```text
OK
```

### 4.2 `AT+HELP`
- 功能：输出可用命令清单
- 期望反馈：多行 `+CMDS:`，最后 `OK`

### 4.3 `AT+FLASH?`
- 功能：读取工厂启动标志位（`boot_flag`）
- 期望反馈示例：

```text
+TESTDATA:SYS,ITEM=FLASH_FLAG,VALUE=<0|1|2>,UNIT=raw,RAW=boot_flag,META=from_storage
OK
```

### 4.4 `AT+FLASH=<0|1|2>`
- 功能：设置工厂启动标志位
- 参数：`0` / `1` / `2`
- 期望反馈示例：

```text
+TESTDATA:SYS,ITEM=FLASH_FLAG,VALUE=<0|1|2>,UNIT=raw,RAW=set_by_at_flash,META=persisted
OK
```

### 4.5 `AT+THRESH?`
- 功能：回显当前参数化阈值/配置
- 期望反馈：多行 `+TESTDATA:CFG,...`，最后 `OK`
- 当前回显项包括：
  - `VBUS_USB_MIN_MV`
  - `3V3_MIN_MV`
  - `VBUS_BAT_MAX_MV`
  - `ADC_VBUS_CHANNEL`
  - `ADC_3V3_CHANNEL`
  - `ADC_BATV_CHANNEL`
  - `CHGCUR_REF_MA`
  - `BATV_DELTA_MAX_MV`
  - `SLEEPI_WINDOW_MS`
  - `SLEEPI_REF_UA`
  - `BLE_SCAN_WINDOW_MS`
  - `BLE_RSSI_REF_DBM`

## 5. 单项测试 AT 指令

### 4.1 State1（供电+通信基础）

#### `AT+VBUS`
- 功能：采样 VBUS 电压
- 期望反馈（成功）：

```text
+TESTDATA:STATE1,ITEM=VBUS,VALUE=<mV>,UNIT=mV,RAW=<adc_raw>,META=ch:<idx>;ref_mv:<cfg>
OK
```

#### `AT+V3P3`
- 功能：采样 3V3 电压
- 期望反馈（成功）：

```text
+TESTDATA:STATE1,ITEM=3V3,VALUE=<mV>,UNIT=mV,RAW=<adc_raw>,META=ch:<idx>;ref_mv:<cfg>
OK
```

#### `AT+UART20TEST`
- 功能：开启 `UART20` `whoami -> NRF54LM20A` 通信测试服务
- 期望反馈（成功）：

```text
+TESTDATA:STATE1,ITEM=UART20TEST,VALUE=1,UNIT=bool,RAW=uart20_test_enabled,META=uart:uart20;trigger:whoami;reply:NRF54LM20A
OK
```

#### `AT+UARTLOOP`
- 功能：UART21 状态检查（当前为 ready 检查）
- 期望反馈（成功）：

```text
+TESTDATA:STATE1,ITEM=UARTLOOP,VALUE=1,UNIT=bool,RAW=uart21_ready,META=baud:115200
OK
```

### 4.2 State2（充电+电压）

#### `AT+CHGCUR`
- 功能：读取 PMIC 平均充电电流（`pmic_charger`）
- 期望反馈（成功）：

```text
+TESTDATA:STATE2,ITEM=CHGCUR,VALUE=<mA>,UNIT=mA,RAW=<uA>,META=src:pmic_charger;status:<st>;ref_ma:<cfg>;sensor:gauge_avg_current
OK
```

#### `AT+BATV`
- 功能：读取电池电压（优先 PMIC，失败回退 ADC）
- 期望反馈（成功，PMIC路径）：

```text
+TESTDATA:STATE2,ITEM=BATV,VALUE=<mV>,UNIT=mV,RAW=<uV>,META=src:pmic_charger;ref_delta_mv:<cfg>;sensor:gauge_voltage
OK
```

- 期望反馈（成功，ADC回退路径）：

```text
+TESTDATA:STATE2,ITEM=BATV,VALUE=<mV>,UNIT=mV,RAW=<adc_raw>,META=ch:<idx>;ref_mv:<cfg>
OK
```

### 4.3 State3（无线+IO）

#### `AT+BLESCAN`
- 功能：执行 BLE 主动扫描窗口
- 期望反馈（成功）：

```text
+TESTDATA:STATE3,ITEM=BLESCAN,VALUE=<adv_count>,UNIT=adv,RAW=<best_rssi_dbm>,META=first:<addr>;best:<addr>;window_ms:<cfg>;ref_rssi_dbm:<cfg>;mode:active_scan
OK
```

#### `AT+GPIOLOOP`
- 功能：GPIO 回环采样（D11/D12/D13 -> D18/D17/D16）
- 期望反馈（成功）：

```text
+TESTDATA:STATE3,ITEM=GPIOLOOP,VALUE=<hit_bits>,UNIT=bits,RAW=<bitmask>,META=map:D11->D18;D12->D17;D13->D16
OK
```

#### `AT+NFCLOOP`
- 功能：NFC相关回环采样（D14/D15 -> NFC1/NFC2）
- 期望反馈（成功）：

```text
+TESTDATA:STATE3,ITEM=NFCLOOP,VALUE=<hit_bits>,UNIT=bits,RAW=<bitmask>,META=map:D14->NFC1;D15->NFC2
OK
```

### 4.4 State4（传感器+交互）

#### `AT+IMU6D`
- 功能：读取 IMU 六轴单次样本
- 期望反馈（成功）：

```text
+TESTDATA:STATE4,ITEM=IMU6D,VALUE=1,UNIT=sample,RAW=ax_u:<...>;ay_u:<...>;az_u:<...>;gx_u:<...>;gy_u:<...>;gz_u:<...>,META=mode:single_fetch;src:imu0
OK
```

#### `AT+MICAMP`
- 功能：读取 DMIC 单块音频幅值
- 期望反馈（成功）：

```text
+TESTDATA:STATE4,ITEM=MICAMP,VALUE=<peak_abs16>,UNIT=abs16,RAW=<avg_abs16>,META=samples:<n>;rate:<hz>;mode:single_block;discard:first_block
OK
```

### 4.5 State5（休眠电流）

#### `AT+SLEEPI`
- 功能：准备外置 Flash 低功耗与 `SW0` 唤醒源，然后在 `OK` 后进入深度睡眠（system off）
- 期望反馈（成功）：

```text
+TESTDATA:STATE5,ITEM=SLEEPI,VALUE=1,UNIT=bool,RAW=system_off_armed,META=wakeup:sw0;window_ms:<cfg>;ref_uA:<cfg>;mode:system_off;flash:dpd
OK
```

- 说明：
  - `OK` 发出后设备会很快进入深度睡眠，`UART21` 停止响应。
  - 深睡期间电流应由治具/外部仪表按 `window_ms` 参考窗口测量。
  - 唤醒方式为用户按下 `SW0`，设备会重启。

### 4.6 State6（按键测试）

#### `AT+KEYWAKE`
- 功能：读取 `SW0` 当前电平、运行期中断计数，以及最近一次 `AT+SLEEPI` 后的持久化唤醒结果
- 期望反馈（成功）：

```text
+TESTDATA:STATE6,ITEM=KEYWAKE,VALUE=<0|1>,UNIT=bool,RAW=<wake_count>,META=alias:sw0;level:<0|1>;irq_count:<irq_count>;wake_reset:<gpio|other|none>;reset_cause:<raw>
OK
```

- 字段说明：
  - `VALUE=1` 表示最近一次已记录的深睡唤醒由 `SW0` 触发。
  - `RAW` 为累计深睡唤醒次数。

### 4.7 State7（Flash 首次写入）

#### `AT+FLASHWRITE`
- 功能：执行持久化写入与回读校验（当前为 storage 分区位图位翻转校验）
- 期望反馈（成功）：

```text
+TESTDATA:STATE7,ITEM=FLASHWRITE,VALUE=1,UNIT=bool,RAW=write_readback_ok,META=op:toggle_item_bitmap_bit0
OK
```

### 4.8 船运模式

#### `AT+SHIPMODEA`
- 功能：Phase A 船运模式动作
- 期望反馈（成功）：

```text
+TESTDATA:STATE8A,ITEM=SHIPMODE,VALUE=1,UNIT=bool,RAW=ship_mode_entered,META=phase:A
OK
```

#### `AT+VBUS_B`
- 功能：Phase B VBUS 复检采样
- 期望反馈（成功）：

```text
+TESTDATA:STATE8B,ITEM=VBUS_B,VALUE=<mV>,UNIT=mV,RAW=<adc_raw>,META=ch:<idx>;ref_mv:<cfg>
OK
```

#### `AT+V3P3_B`
- 功能：Phase B 3V3 复检采样
- 期望反馈（成功）：

```text
+TESTDATA:STATE8B,ITEM=3V3_B,VALUE=<mV>,UNIT=mV,RAW=<adc_raw>,META=ch:<idx>;ref_mv:<cfg>
OK
```

#### `AT+SHIPMODEB`
- 功能：Phase B 船运模式动作
- 期望反馈（成功）：

```text
+TESTDATA:STATE9B,ITEM=SHIPMODE,VALUE=1,UNIT=bool,RAW=ship_mode_entered,META=phase:B
OK
```

## 5. State 聚合指令

> 规则：`AT+STATE<id>` 仅按顺序调用该 State 的单项 handler，逐项输出 `+TESTDATA`，最终 `OK/ERROR`。

- `AT+STATE1` -> `AT+VBUS` + `AT+V3P3` + `AT+UARTLOOP`
- `AT+STATE2` -> `AT+CHGCUR` + `AT+BATV`
- `AT+STATE3` -> `AT+BLESCAN` + `AT+GPIOLOOP` + `AT+NFCLOOP`
- `AT+STATE4` -> `AT+IMU6D` + `AT+MICAMP`
- `AT+STATE5` -> `AT+SLEEPI`
- `AT+STATE6` -> `AT+KEYWAKE`
- `AT+STATE7` -> `AT+FLASHWRITE`
- `AT+STATE8A` -> `AT+SHIPMODEA`
- `AT+STATE8B` -> `AT+VBUS_B` + `AT+V3P3_B`
- `AT+STATE9B` -> `AT+SHIPMODEB`

## 6. 推荐手测序列（最小）

```text
AT
AT+HELP
AT+THRESH?
AT+FLASH?
AT+STATE1
AT+STATE2
AT+STATE3
AT+STATE4
AT+STATE5
AT+STATE6
AT+STATE7
AT+STATE8A
AT+STATE8B
AT+STATE9B
```

## 7. 说明
- 本文档描述的是当前代码行为与回包字段。
- 上位机应基于 `VALUE/RAW/META` 做阈值判定与流程控制。
