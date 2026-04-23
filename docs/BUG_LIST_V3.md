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
