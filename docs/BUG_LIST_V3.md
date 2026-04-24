# Factory Bug List V3

## 1. `bt scan on` may crash or hang the device

- Status: Open
- Priority: High
- Report Date: 2026-04-23

### Symptom

User reported that after sending:

```text
bt scan on
```

the device appears to crash or stop responding.

### Current Understanding

- The command transport path is now able to parse text commands correctly.
- The suspected problem is no longer the UART parser itself.
- The failure is likely inside the BLE enable/scan session path, or in an interaction with runtime state after entering scan mode.

### Reproduction

1. Boot firmware normally
2. Send `bt init\r\n`
3. Send `bt scan on\r\n`
4. Observe whether the device stops replying, resets, or hard-faults

### Evidence To Collect Next

- Full serial log before and after `bt scan on`
- Whether the startup banner reappears after the failure
- Whether `UART21` and `UART20` both go silent
- If available: reset cause after reboot

### Planned Follow-up

- Add a focused BLE crash reproduction checklist
- Instrument the BLE scan start/stop path with minimal crash-safe markers if needed
- Verify whether the fault happens in `bt_enable`, `bt_le_scan_start`, or later callback flow

## 2. `imu get` may hang inside the IMU sampling path

- Status: Mitigated
- Priority: High
- Report Date: 2026-04-23
- Mitigation Date: 2026-04-23

### Symptom

After sending:

```text
imu get
```

the parser completes command recognition, but the device stops making progress before returning IMU data or `ERROR:...`.

### Current Understanding

- Command transport and token parsing are working: `imu get` is received and matched correctly.
- The hang is therefore likely inside:
  - `at_ensure_imu_ready()`
  - `at_fetch_imu_sample()`
  - underlying regulator / I2C / sensor driver access
- The live call chain is:
  - `dispatch_text_command()`
  - `text_handle_imu_get()`
  - `at_get_imu_sample_direct()`
  - `at_ensure_imu_ready()`
  - `at_start_imu_stream_if_needed()`
  - `sensor_trigger_set(SENSOR_TRIG_DATA_READY)`
  - background trigger handler fetches one sample into cache
  - `at_fetch_imu_sample()` copies the cached sample to the text/AT response

### Evidence Seen So Far

The parser reached:

```text
PARSE:text_match=imu get
```

and then no further response was observed.

### Reproduction

1. Boot firmware normally
2. Send `imu get\r\n`
3. Observe whether the device stops replying before any `accel data:` / `gyro data:` output

### Planned Follow-up

- `imu get` and `AT+IMU6D` have been restored to the known-good pre-worker direct sampling path used by the earlier AT implementation.
- The IMU path is now aligned more closely with `test_plan/11-zephyr-imu`: `CONFIG_LSM6DSL_TRIGGER_GLOBAL_THREAD=y`, `CONFIG_MAIN_STACK_SIZE=2048`, `CONFIG_SYSTEM_WORKQUEUE_STACK_SIZE=2048`, and runtime `26 Hz` accel/gyro ODR configured through `sensor_attr_set(...)` before trigger enable.
- Instead of doing a synchronous IMU bus transaction inside `imu get`, the factory firmware now starts the IMU stream once and caches samples from the driver's data-ready callback path. `imu get` only reads the cached sample.
- Diagnostic builds can enable `CONFIG_FACTORY_IMU_TRACE=y` to emit `[IMU]` stage markers before and after regulator enable, device init, stream setup, and cache fetch.
- The failed worker/timeout layer added after the initial V3 text protocol was removed from the active IMU path because hardware logs showed execution stopped before the worker thread ever reached `worker:thread_start`.
- The text response format was corrected to `accel data:<x>,<y>,<z>\rgyro data: <gx>,<gy>,<gz>\r\n`.
- Remaining hardware follow-up: collect one real-board `imu get` log to confirm whether the restored direct IMU path completes normally.
