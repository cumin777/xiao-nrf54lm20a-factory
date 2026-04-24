#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$repo_root"

# State5 should use real system-off path with flash suspend and SW0 wake
rg -n "sys_poweroff|PM_DEVICE_ACTION_SUSPEND|hwinfo_clear_reset_cause" src/at_handler.c >/dev/null
rg -n "GPIO_INT_LEVEL_ACTIVE" src/at_handler.c >/dev/null
rg -n "DT_NODELABEL\\(py25q64\\)|flash_suspend_failed|system_off_armed" src/at_handler.c >/dev/null
rg -n "at_sleepi_force_led_off|DT_ALIAS\\(led0\\)|gpio_pin_set_dt\\(&g_led0, 0\\)" src/at_handler.c >/dev/null

# State5 placeholder must be removed
if rg -n "STATE5\", \"SLEEPI\", \"err:MEASURE_PATH_TBD" src/at_handler.c >/dev/null; then
  echo "State5 handler still NOT_IMPLEMENTED"
  exit 1
fi

# Required Kconfig/config items
rg -n "FACTORY_SLEEPI_WINDOW_MS|FACTORY_SLEEPI_REF_UA" zephyr/Kconfig >/dev/null
rg -n "CONFIG_SPI=y" zephyr/prj.conf >/dev/null
rg -n "CONFIG_SPI_NOR=y" zephyr/prj.conf >/dev/null
rg -n "CONFIG_PM_DEVICE=y" zephyr/prj.conf >/dev/null
rg -n "CONFIG_POWEROFF=y" zephyr/prj.conf >/dev/null
rg -n "CONFIG_HWINFO=y" zephyr/prj.conf >/dev/null
rg -n "&py25q64" zephyr/boards/xiao_nrf54lm20a_nrf54lm20a_cpuapp.overlay >/dev/null

echo "verify_factory_v2_state5: PASS"
