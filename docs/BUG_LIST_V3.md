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

1. Boot firmware normally.
2. Send `bt init\r\n`.
3. Send `bt scan on\r\n`.
4. Observe whether the device stops replying, resets, or hard-faults.

### Evidence To Collect Next

- Full serial log before and after `bt scan on`.
- Whether the startup banner reappears after the failure.
- Whether `UART21` and `UART20` both go silent.
- If available: reset cause after reboot.

### Planned Follow-up

- Add a focused BLE crash reproduction checklist.
- Instrument the BLE scan start/stop path with minimal crash-safe markers if needed.
- Verify whether the fault happens in `bt_enable`, `bt_le_scan_start`, or later callback flow.

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

### Planned Follow-up

- `imu get` and `AT+IMU6D` have been restored to the known-good pre-worker direct sampling path used by the earlier AT implementation.
- Remaining hardware follow-up: collect one real-board `imu get` log to confirm whether the restored direct IMU path completes normally.

## 3. `sleep mode` keeps LED on and may not enter clean low-power state

- Status: Mitigated
- Priority: High
- Report Date: 2026-04-24
- Mitigation Date: 2026-04-24

### Symptom

After sending:

```text
sleep mode
```

the command returns normally, but LED behavior suggests the board may not be entering a clean deep-sleep state. User observed that the LED was not turned off.

### Current Understanding

- The firmware path arms `system_off`, configures `SW0` wakeup, suspends external flash into DPD, and then executes deferred `sys_poweroff()`.
- The sleep path now explicitly drives `led0` to `inactive` before returning the `sleep mode` reply, so LED state should no longer remain latched on because of missing firmware action.
- If LED remains on after the reply, one of these is likely true:
  - `sys_poweroff()` is not actually reached.
  - The LED state is being held externally or by another hardware block.
  - The board does enter low power, but the LED circuit behavior makes the state appear misleading.

### Reproduction

1. Boot firmware normally.
2. Send `sleep mode\r\n`.
3. Observe LED state, UART silence, and supply current.
4. Press `SW0` to confirm wake behavior.

### Evidence To Collect Next

- Whether UART stops immediately after the `sleep mode` reply.
- Current measurement after the command.
- Whether pressing `SW0` restarts the board as expected.
- Whether LED remains on continuously or only latches briefly before power-off.

### Planned Follow-up

- Confirm on real hardware that the new `led0 -> inactive` action takes effect before `system_off`.
- Confirm whether the deferred `sys_poweroff()` path is actually executed on real hardware.
- Compare measured current against the expected deep-sleep baseline instead of relying on LED state alone.

## 4. `ship mode` wake response is incomplete and post-wake services become unavailable

- Status: Open
- Priority: High
- Report Date: 2026-04-24

### Symptom

After `ship mode`, the wake-up response is incomplete. Following the `ship mode` reply, other system functions are unusable; at minimum, `UART20` is reported as abnormal after wake.

### Current Understanding

- The current ship-mode flow likely lacks enough protection around post-wake reinitialization.
- The issue may involve partial wake, incomplete regulator/peripheral restore, or stale UART service state after returning from ship-related power transitions.
- `UART20` behavior is the first confirmed externally visible casualty, but the problem may affect a broader subset of services.

### Reproduction

1. Boot firmware normally.
2. Send `ship mode\r\n`.
3. Perform the expected wake operation.
4. Check whether the wake response is complete and whether `UART20` / other services still function.

### Evidence To Collect Next

- Full serial log before ship mode, during wake, and after wake.
- Whether the startup banner reappears after wake.
- Whether `UART21` remains usable while `UART20` fails.
- Reset cause and any persisted wake flags after recovery.

### Planned Follow-up

- Protect the ship-mode exit / wake path with explicit reinitialization checks.
- Verify whether UART services need to be re-enabled after ship-mode wake.
- Narrow whether the failure is limited to `UART20` or indicates a larger post-wake platform init gap.

## 5. `bt scan` discovers far fewer devices than a phone

- Status: Open
- Priority: Medium
- Report Date: 2026-04-24

### Symptom

`bt scan` can discover only a small subset of nearby devices. The number of discovered devices is far lower than what a phone can see in the same environment.

### Current Understanding

- This issue is distinct from the possible `bt scan on` crash/hang path.
- Current firmware uses a constrained BLE scan reporting flow, and the gap may come from one or more of:
  - passive scan vs. phone behavior
  - current scan window / interval settings
  - deduping / caching policy
  - periodic summary output showing only a reduced subset

### Reproduction

1. Boot firmware normally.
2. Send `bt init\r\n`.
3. Send `bt scan on\r\n`.
4. Compare the discovered device count and visible addresses against a phone in the same location.

### Evidence To Collect Next

- Raw device list seen by phone at the same time.
- Raw serial output from the board during the same scan window.
- Whether the missing devices are all weak-RSSI devices or include strong nearby devices.

### Planned Follow-up

- Compare passive and active scan behavior.
- Revisit scan parameters and report/filter policy.
- Add a raw-count / raw-address diagnostic mode if needed.

## 6. GPIO flip coverage is incomplete without the fixture

- Status: Open
- Priority: Medium
- Report Date: 2026-04-24

### Symptom

GPIO toggling has not been fully validated end-to-end. Without the production fixture connected, not all GPIO paths have actually been exercised.

### Current Understanding

- The firmware contains GPIO pair mapping and text command support, but full production coverage depends on the fixture wiring.
- Current validation is therefore incomplete for the real jig-connected matrix.

### Reproduction

1. Connect the production fixture.
2. Execute the full GPIO set/get sequence across all mapped pairs.
3. Compare actual loopback behavior against the expected fixture wiring.

### Evidence To Collect Next

- Which GPIO pairs have already been manually verified.
- Full fixture-side test log for all mapped GPIO pairs.
- Any pair that passes in direct bench testing but fails on the real jig.

### Planned Follow-up

- Run the complete GPIO matrix on the real fixture.
- Record any pair-specific failures separately from protocol-level issues.
- Do not treat current bench-only GPIO verification as full production closure.
