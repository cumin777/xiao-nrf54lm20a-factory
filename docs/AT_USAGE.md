# XIAO nRF54LM20A Factory AT 使用文档

## 1. 概述
本固件是基于 `UART21` 的 AT 从机。

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

## 3. 基础系统指令

### 3.1 `AT`
- 功能：连通性测试
- 期望反馈：

```text
OK
```

### 3.2 `AT+HELP`
- 功能：输出可用命令清单
- 期望反馈：多行 `+CMDS:`，最后 `OK`

### 3.3 `AT+FLASH?`
- 功能：读取工厂启动标志位（`boot_flag`）
- 期望反馈示例：

```text
+TESTDATA:SYS,ITEM=FLASH_FLAG,VALUE=<0|1|2>,UNIT=raw,RAW=boot_flag,META=from_storage
OK
```

### 3.4 `AT+FLASH=<0|1|2>`
- 功能：设置工厂启动标志位
- 参数：`0` / `1` / `2`
- 期望反馈示例：

```text
+TESTDATA:SYS,ITEM=FLASH_FLAG,VALUE=<0|1|2>,UNIT=raw,RAW=set_by_at_flash,META=persisted
OK
```

### 3.5 `AT+THRESH?`
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

## 4. 单项测试 AT 指令

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
+TESTDATA:STATE4,ITEM=MICAMP,VALUE=<peak_abs16>,UNIT=abs16,RAW=<avg_abs16>,META=samples:<n>;rate:<hz>;mode:single_block
OK
```

### 4.5 State5（休眠电流）

#### `AT+SLEEPI`
- 功能：在窗口前后读取 PMIC 平均电流并上报
- 期望反馈（成功）：

```text
+TESTDATA:STATE5,ITEM=SLEEPI,VALUE=<post_uA>,UNIT=uA,RAW=<pre_uA>,META=delta_uA:<d>;window_ms:<cfg>;ref_uA:<cfg>;mode:pmic_gauge_avg_current
OK
```

### 4.6 State6（按键测试）

#### `AT+KEYWAKE`
- 功能：读取按键状态与中断计数（SW0）
- 期望反馈（成功）：

```text
+TESTDATA:STATE6,ITEM=KEYWAKE,VALUE=<0|1>,UNIT=bool,RAW=<irq_count>,META=alias:sw0;irq_count=<irq_count>
OK
```

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
