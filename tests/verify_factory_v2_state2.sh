#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$repo_root"

# State2 should use real PMIC charger/ADC paths
rg -n "SENSOR_CHAN_GAUGE_AVG_CURRENT|SENSOR_CHAN_GAUGE_VOLTAGE|pmic_charger|DT_NODELABEL\(pmic_charger\)" src/at_handler.c >/dev/null
rg -n "CONFIG_FACTORY_CHGCUR_REF_MA|CONFIG_FACTORY_BATV_DELTA_MAX_MV|CONFIG_FACTORY_ADC_BATV_CHANNEL" src/at_handler.c >/dev/null

# State2 placeholders must be removed
if rg -n "STATE2\", \"CHGCUR\", \"err:SENSOR_NOT_READY|STATE2\", \"BATV\", \"err:ADC_NOT_READY" src/at_handler.c >/dev/null; then
  echo "State2 handlers still NOT_IMPLEMENTED"
  exit 1
fi

# Required Kconfig items
rg -n "FACTORY_CHGCUR_REF_MA|FACTORY_BATV_DELTA_MAX_MV|FACTORY_ADC_BATV_CHANNEL" zephyr/Kconfig >/dev/null

echo "verify_factory_v2_state2: PASS"
