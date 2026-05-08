# DMIC 资源清理与 Sleep Mode 优化调查报告

## 问题描述

在执行 mic capture 后，进入 sleep mode（system off）时，DMIC 相关资源未完全清除，可能导致额外的漏电流。

## 硬件架构

```
nRF54LM20A (SoC)
  +-- PDM20 外设 (CLK=P1.13, DIN=P1.14)
  |
  +-- NPM1300 (PMIC, I2C addr 0x6b)
  |     +-- LDO1 = dmic_vdd (3.3V, via I2C)
  |
  +-- power_en = GPIO 固定稳压器 (GPIO1 pin 12, 控制外部 LDO 使能)
```

DMIC 供电链路：**power_en (GPIO1.12)** + **NPM1300 LDO1 (dmic_vdd)** 双路控制。

## 根因分析

### Zephyr DMIC 驱动 (`dmic_nrfx_pdm.c`) 不实现 PM SUSPEND

分析驱动源码确认：

- 该驱动**没有注册 `pm_action_handler`**
- **不支持** `pm_device_action_run(dev, PM_DEVICE_ACTION_SUSPEND)`
- **不支持** `pm_device_action_run(dev, PM_DEVICE_ACTION_TURN_OFF)`
- `dmic_trigger(DMIC_TRIGGER_STOP)` 仅调用 `nrfx_pdm_stop()` 中止 DMA 传输
- PDM 外设 **ENABLE 寄存器保持为 1**，时钟生成器持续运行
- CLK/DIN 引脚仍处于 PDM 功能模式

这意味着在进入 system off 前，必须手动完成全部清理工作。

### `regulator-boot-on` 引用计数问题

overlay 中 LDO1 设置了 `regulator-boot-on`，使启动时引用计数 = 1。
`at_enable_dmic_power()` 调用 `regulator_enable()` 后计数 = 2。
一个 `regulator_disable()` 只将计数降回 1，GPIO 保持 HIGH，DMIC 硬件仍然供电。

当前代码通过直接驱动 GPIO1 pin 12 为 LOW 绕过了这个问题。

## 当前清理序列（at_handler.c:2653 at_prepare_sleepi）

| 步骤 | 操作 | 状态 |
|------|------|------|
| 1 | `dmic_trigger(STOP)` | 正确 |
| 2 | `k_msleep(2)` 等待 DMA 完成 | 正确 |
| 3 | `nrfx_pdm_uninit()` 反初始化 PDM | 正确 |
| 4 | 手动 `gpio_pin_configure(INPUT\|PULL_DOWN)` P1.13/P1.14 | 可改进 -> 用 pinctrl sleep 状态 |
| 5 | `regulator_disable(dmic_vdd)` | 存在引用计数问题 |
| 6 | 直接驱动 GPIO1.12 为 LOW | workaround，可改进 |

## 实施的修复

### 修复 1：用 pinctrl sleep 状态替代手动 GPIO 配置

PDM20 的板级 DTS 已经定义了 `pinctrl-1 = <&pdm20_sleep>` 和 `low-power-enable`，
应通过 `pinctrl_apply_state(pcfg, PINCTRL_STATE_SLEEP)` 利用这些定义，
而不是手动 `gpio_pin_configure()`。

### 修复 2：移除 `regulator-boot-on`

从 overlay 中移除 LDO1 的 `regulator-boot-on`，使 DMIC 默认不供电，
`regulator_enable/disable()` 的引用计数行为正确。

### 修复 3：提取 DMIC 关闭为独立函数

将 DMIC 关闭逻辑提取为 `at_shutdown_dmic()` 函数，提高可读性和可维护性。

## 参考资料

- [Zephyr DMIC nRF PDM 驱动源码](https://github.com/zephyrproject-rtos/zephyr/blob/main/drivers/audio/dmic_nrfx_pdm.c)
- [Zephyr Device Power Management](https://docs.zephyrproject.org/latest/services/pm/device.html)
- [nordic,npm1300-regulator 绑定](https://docs.zephyrproject.org/latest/build/dts/api/bindings/regulator/nordic%252Cnpm1300-regulator.html)
- [nRF54L15 PDM 产品规格书](https://docs.nordicsemi.com/bundle/ps_nrf54L15/page/pdm.html)
- [Nordic DevZone: nRF54L15 System OFF 漏电流](https://devzone.nordicsemi.com/f/nordic-q-a/123511/nrf54l15---system-off-strange-current-peaks)
- [Nordic DevZone: nRF54L15 System OFF 模式实现](https://devzone.nordicsemi.com/f/nordic-q-a/119042/system-off-mode-for-system-power-management-for-nrf54l15)
- [Nordic 电源优化指南](https://docs.nordicsemi.com/bundle/ncs-3.0.2/page/nrf/test_and_optimize/optimizing/power_general.html)
