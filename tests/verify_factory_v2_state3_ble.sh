#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$repo_root"

# State3 BLE should use real Zephyr BT scan path
rg -n "bt_enable|bt_le_scan_start|bt_le_scan_stop|BT_GAP_ADV_TYPE_ADV_IND" src/at_handler.c >/dev/null
rg -n "CONFIG_FACTORY_BLE_SCAN_WINDOW_MS|CONFIG_FACTORY_BLE_RSSI_REF_DBM" src/at_handler.c >/dev/null

# State3 BLE placeholder must be removed
if rg -n "STATE3\", \"BLESCAN\", \"err:BLE_NOT_READY" src/at_handler.c >/dev/null; then
  echo "State3 BLE handler still NOT_IMPLEMENTED"
  exit 1
fi

# Required Kconfig and build options
rg -n "FACTORY_BLE_SCAN_WINDOW_MS|FACTORY_BLE_RSSI_REF_DBM" zephyr/Kconfig >/dev/null
rg -n "CONFIG_BT=y|CONFIG_BT_OBSERVER=y" zephyr/prj.conf >/dev/null

echo "verify_factory_v2_state3_ble: PASS"
