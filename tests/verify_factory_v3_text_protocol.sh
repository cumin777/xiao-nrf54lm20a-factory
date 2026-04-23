#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$repo_root"

rg -n "dispatch_text_command|tokenize_text_command|FACTORY_TEXT_CMD_MAX_TOKENS" src/at_handler.c >/dev/null

rg -n "\"gpio\", \"set\"|text_handle_gpio_set" src/at_handler.c >/dev/null
rg -n "\"gpio\", \"get\"|text_handle_gpio_get" src/at_handler.c >/dev/null
rg -n "text_handle_bt_init|text_handle_bt_scan_on|text_handle_bt_scan_off" src/at_handler.c >/dev/null
rg -n "text_handle_uart20_on|uart20 on" src/at_handler.c >/dev/null
rg -n "g_ble_text_scan_active|at_ble_scan_start_session|at_ble_scan_stop_session|emit_text_ble_scan_results" src/at_handler.c >/dev/null
rg -n "text_handle_sleep_mode|text_handle_ship_mode" src/at_handler.c >/dev/null
rg -n "text_handle_mic_capture|text_handle_imu_get|text_handle_imu_off" src/at_handler.c >/dev/null
rg -n "text_handle_flash_write|text_handle_bat_get" src/at_handler.c >/dev/null

rg -n "legacy_flash_value" src/factory_storage.h src/factory_storage.c src/at_handler.c >/dev/null
rg -n "LMP: version 6.0|\\[DEVICE\\]|audio data Max:|accel data:|gyro data:|Value:|bat:" src/at_handler.c >/dev/null

echo "verify_factory_v3_text_protocol: PASS"
