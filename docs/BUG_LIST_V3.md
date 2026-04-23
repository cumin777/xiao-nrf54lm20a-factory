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

- Status: Open
- Priority: High
- Report Date: 2026-04-23

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

- Add minimal pre/post markers around `at_ensure_imu_ready()` and `at_fetch_imu_sample()`
- Determine whether the block happens in regulator enable, deferred `device_init()`, or sensor fetch
- Decide whether to add timeout/error fallback around IMU access
