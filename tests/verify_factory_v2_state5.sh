#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$repo_root"

# State5 should use real current sampling path
rg -n "at_pmic_charger_fetch|SENSOR_CHAN_GAUGE_AVG_CURRENT|CONFIG_FACTORY_SLEEPI_WINDOW_MS|CONFIG_FACTORY_SLEEPI_REF_UA" src/at_handler.c >/dev/null

# State5 placeholder must be removed
if rg -n "STATE5\", \"SLEEPI\", \"err:MEASURE_PATH_TBD" src/at_handler.c >/dev/null; then
  echo "State5 handler still NOT_IMPLEMENTED"
  exit 1
fi

# Required Kconfig items
rg -n "FACTORY_SLEEPI_WINDOW_MS|FACTORY_SLEEPI_REF_UA" zephyr/Kconfig >/dev/null

echo "verify_factory_v2_state5: PASS"
