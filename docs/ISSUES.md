# Factory Test Issues

## AT Parser - Intermittent ERROR:UNKNOWN_COMMAND

**Status:** Fix implemented, pending repeated hardware verification
**Date:** 2026-04-20

### Symptom

Repeatedly sending the same valid AT command occasionally returned:

```text
ERROR:UNKNOWN_COMMAND
```

Example observed during `AT+IMU6D` retest:

```text
AT+IMU6D
+TESTDATA:STATE4,ITEM=IMU6D,...
OK

AT+IMU6D
ERROR:UNKNOWN_COMMAND
```

### Root Cause

1. **`UNKNOWN_COMMAND` is a parser failure, not an IMU failure:** The response is only emitted when `at_handler_process_line()` fails to match the received line against any command in `g_item_cmds`/`g_state_cmds`.

2. **The old RX loop accepted every non-CR/LF byte into the command buffer:** In `src/main.c`, any byte other than `\r` or `\n` was appended to `at_buf`. On a shared `UART21` link, a stray leading byte, echo byte, or line noise could turn `AT+IMU6D` into a different string such as `<junk>AT+IMU6D`, which then correctly fell through to `ERROR:UNKNOWN_COMMAND`.

3. **There was no resynchronization to the `AT` prefix:** Once a garbage byte entered an empty buffer, the next otherwise-valid command was parsed as a malformed line instead of being resynced.

### Applied Fix

- `src/main.c`: Added printable-ASCII filtering so non-printable garbage bytes are dropped instead of entering the AT line buffer.
- `src/main.c`: Added RX resynchronization to a clean `AT` prefix. A new command is only assembled after seeing `A` then `T`, which prevents stray leading bytes from poisoning the next command.
- `tests/verify_factory_uart_rx.sh`: Added a regression check for the new UART RX filtering/resync logic.

---

## IMU6D (STATE4) - All zeros

**Status:** Fix implemented, pending hardware verification
**Date:** 2026-04-20

### Symptom

```
AT+IMU6D

+TESTDATA:STATE4,ITEM=IMU6D,VALUE=1,UNIT=sample,RAW=ax_u:0;ay_u:0;az_u:0;gx_u:0;gy_u:0;gz_u:0,META=mode:single_fetch;src:imu0
OK
```

All 6-axis readings return 0. VALUE=1 (fetch succeeded) but sensor data is zero.

### Root Cause

1. **Factory overlay missed the working sample's deferred-init sequence:** `test_plan/11-zephyr-imu` and `platform-seeedboards/examples/zephyr-imu` both add `zephyr,deferred-init` on `&lsm6ds3tr_c`, then call `device_init()` only after `power_en` and `imu_vdd` are enabled. The factory overlay had the regulator label, but did not defer IMU init, so the driver could auto-init before the rail was actually usable.

2. **Single-shot fetch path was weaker than the working sample:** The factory code set ODR, then immediately used one `sensor_sample_fetch()` call. For a single AT transaction this is more fragile than the sample's sequence of `sensor_sample_fetch_chan()` per sensor block after a powered and manually initialized device. If the first frame is read too early, the output registers can remain all zero.

### Reference

Working implementation: `test_plan/11-zephyr-imu/src/main.c`
- `set_sampling_freq()` sets ODR to 26Hz for both accel and gyro
- `enable_imu_power()` uses labeled `imu_vdd` regulator
- Uses `zephyr,deferred-init` on `lsm6ds3tr_c` to defer driver init until after power-on

### Applied Fix

- `zephyr/boards/xiao_nrf54lm20a_nrf54lm20a_cpuapp.overlay`: Added `&lsm6ds3tr_c { zephyr,deferred-init; }` to align the factory firmware with the known-good IMU samples.
- `src/at_handler.c`: Added `at_ensure_imu_ready()` so `AT+IMU6D` enables regulators first and then calls `device_init(g_imu_dev)` when the deferred-init device is still not ready.
- `src/at_handler.c`: Switched the single-shot read flow to sample-style `sensor_sample_fetch_chan()` for accel and gyro separately, with a one-ODR settle delay before the first read.

---

## MICAMP (STATE4) - dmic_not_ready

**Status:** Fix implemented, pending hardware verification
**Date:** 2026-04-21

### Symptom

```
AT+MICAMP

+TESTDATA:STATE4,ITEM=MICAMP,VALUE=0,UNIT=abs16,RAW=dmic_not_ready,META=err:HW_NOT_READY
```

DMIC device was not ready because the build did not materialize an enabled `pdm20` device object for the factory app.

### Root Cause

1. **The factory app was not generating an active `pdm20` device in the final build:** Before the fix, the generated app config showed `CONFIG_NRFX_PDM20` unset and runtime could only fail with `dmic_not_ready`.

2. **The factory overlay was not aligned with the known-good sample fragment:** `test_plan/12-zephyr-dmic-recorder` uses `dmic_dev: &pdm20 { status = "okay"; }` plus `dmic_vdd` on PMIC LDO1. After aligning the overlay to that exact fragment and doing a clean rebuild, the factory app generated `CONFIG_NRFX_PDM20=y` and `CONFIG_AUDIO_DMIC_NRFX_PDM=y`.

3. **The AT handler used `device_get_binding()` instead of the sample's static `DEVICE_DT_GET()` path:** That hid the missing device-object problem until runtime. Switching to `DEVICE_DT_GET(DT_ALIAS(dmic20))` makes the code follow the same path as the working sample and the driver is now linked explicitly.

4. **Capture startup was less robust than the working sample:** The sample discards the first DMIC block after `DMIC_TRIGGER_START`. The factory handler now does the same before computing amplitude metrics.

### Reference

Working implementation: `test_plan/12-zephyr-dmic-recorder/`
- Overlay: `dmic_dev: &pdm20 { status = "okay"; }` and `dmic_vdd: LDO1` with 3.3V
- Code: uses `DEVICE_DT_GET(DT_ALIAS(dmic20))`, powers PMIC first, then configures/starts DMIC
- Capture flow: discard first DMIC block, then process subsequent audio data

### Applied Fix

- `zephyr/boards/xiao_nrf54lm20a_nrf54lm20a_cpuapp.overlay`: Aligned with the sample's `dmic_dev: &pdm20 { status = "okay"; }` fragment, kept `dmic_vdd` on the PMIC LDO1 rail, and ensured `pmic_i2c` is explicitly `okay`.
- `src/at_handler.c`: Switched DMIC access from `device_get_binding()` to `DEVICE_DT_GET(DT_ALIAS(dmic20))`, preserving PMIC power-on sequencing and using a sample-style discard-first-block capture flow.
- Verification after a clean rebuild: generated config now contains `CONFIG_NRFX_PDM20=y` and `CONFIG_AUDIO_DMIC_NRFX_PDM=y`, and the build includes `drivers/audio/dmic_nrfx_pdm.c.o`.

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
