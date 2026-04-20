# Factory Test Issues

## IMU6D (STATE4) - All zeros

**Status:** Fix pending verification
**Date:** 2026-04-20

### Symptom

```
AT+IMU6D

+TESTDATA:STATE4,ITEM=IMU6D,VALUE=1,UNIT=sample,RAW=ax_u:0;ay_u:0;az_u:0;gx_u:0;gy_u:0;gz_u:0,META=mode:single_fetch;src:imu0
OK
```

All 6-axis readings return 0. VALUE=1 (fetch succeeded) but sensor data is zero.

### Root Cause

1. **ODR not set (direct cause):** LSM6DSL powers up in power-down mode (ODR=0). The factory code calls `sensor_sample_fetch()` without first setting the sampling frequency via `sensor_attr_set(SENSOR_ATTR_SAMPLING_FREQUENCY)`. The sensor never starts sampling, so all reads return zeros.

2. **Missing `imu_vdd` regulator label:** The board DTSI defines LDO1 but without an `imu_vdd` label. `at_enable_imu_power()` checks `DT_NODELABEL(imu_vdd)` which doesn't exist, so LDO1 is never explicitly enabled. (LDO1 has `regulator-boot-on` from the board DTSI, so it may already be on at boot, but explicit control is missing.)

### Reference

Working implementation: `test_plan/11-zephyr-imu/src/main.c`
- `set_sampling_freq()` sets ODR to 26Hz for both accel and gyro
- `enable_imu_power()` uses labeled `imu_vdd` regulator
- Uses `zephyr,deferred-init` on `lsm6ds3tr_c` to defer driver init until after power-on

### Applied Fix

- `src/at_handler.c`: Added `sensor_attr_set()` calls for `SENSOR_CHAN_ACCEL_XYZ` and `SENSOR_CHAN_GYRO_XYZ` with ODR 26Hz before `sensor_sample_fetch()`
- `zephyr/boards/xiao_nrf54lm20a_nrf54lm20a_cpuapp.overlay`: Added `imu_vdd: LDO1` label with 3.3V

---

## MICAMP (STATE4) - dmic_not_ready

**Status:** Fix pending verification
**Date:** 2026-04-20

### Symptom

```
AT+MICAMP

+TESTDATA:STATE4,ITEM=MICAMP,VALUE=0,UNIT=abs16,RAW=dmic_not_ready,META=err:HW_NOT_READY
```

DMIC device not ready, `device_get_binding()` returns NULL or device not ready.

### Root Cause

1. **`pdm20` disabled in devicetree (direct cause):** The SoC DTSI (`nrf54lm20a.dtsi`) sets `pdm20` node to `status = "disabled"`. The factory overlay did not override this, so the DMIC driver was never loaded. `device_get_binding()` returns NULL.

2. **Missing `dmic_vdd` regulator label:** Same issue as IMU - LDO1 (shared power rail for IMU and DMIC) has no `dmic_vdd` label, so `at_enable_dmic_power()` skips LDO1 enable.

3. **Power-on order:** Code obtained device binding before enabling power. Should enable power first.

4. **Insufficient stabilization delay:** 10ms vs 20ms in the working test plan.

### Reference

Working implementation: `test_plan/12-zephyr-dmic-recorder/`
- Overlay: `&pdm20 { status = "okay"; }` and `dmic_vdd: LDO1` with 3.3V
- Code: calls `enable_dmic_power()` before any DMIC operations
- Delay: 20ms after regulator enable

### Applied Fix

- `zephyr/boards/xiao_nrf54lm20a_nrf54lm20a_cpuapp.overlay`: Added `&pdm20 { status = "okay"; }` and `dmic_vdd` label on LDO1 (dual-labeled as `imu_vdd: dmic_vdd: LDO1`)
- `src/at_handler.c`: Reordered power-on before device binding, increased stabilization delay from 10ms to 20ms

---

## BATV (STATE2) - Reading without battery

**Status:** Expected behavior
**Date:** 2026-04-20

### Symptom

```
AT+BATV

+TESTDATA:STATE2,ITEM=BATV,VALUE=43,UNIT=mV,RAW=43000,META=src:pmic_charger;ref_delta_mv:200;sensor:gauge_voltage
```

Reads 43mV / 63mV / 1909mV when no battery is connected. Values are low and unstable.

### Analysis

This is expected. The code reads from the PMIC charger's `SENSOR_CHAN_GAUGE_VOLTAGE`. With no battery connected, the ADC input is floating and picks up residual/leakage voltage. The readings are noise, not a real bug.
