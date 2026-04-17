#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$repo_root"

# Kconfig parameterization must exist
test -f zephyr/Kconfig
rg -n "config FACTORY_VBUS_USB_MIN_MV" zephyr/Kconfig >/dev/null
rg -n "config FACTORY_3V3_MIN_MV" zephyr/Kconfig >/dev/null
rg -n "config FACTORY_VBUS_BAT_MAX_MV" zephyr/Kconfig >/dev/null
rg -n "config FACTORY_ADC_VBUS_CHANNEL" zephyr/Kconfig >/dev/null
rg -n "config FACTORY_ADC_3V3_CHANNEL" zephyr/Kconfig >/dev/null

# ADC must be enabled in app config
rg -n "CONFIG_ADC=y" zephyr/prj.conf >/dev/null

# overlay should expose ADC io-channels for app access
rg -n "zephyr,user" zephyr/boards/xiao_nrf54lm20a_nrf54lm20a_cpuapp.overlay >/dev/null
rg -n "io-channels" zephyr/boards/xiao_nrf54lm20a_nrf54lm20a_cpuapp.overlay >/dev/null

# State1/8B ADC handlers should use real ADC read path and avoid NOT_IMPLEMENTED stub pattern
rg -n "adc_read_dt|adc_channel_setup_dt" src/at_handler.c >/dev/null
if rg -n "STATE1\", \"VBUS\", \"err:ADC_NOT_READY|STATE1\", \"3V3\", \"err:ADC_NOT_READY|STATE8B\", \"VBUS_B\", \"err:ADC_NOT_READY|STATE8B\", \"3V3_B\", \"err:ADC_NOT_READY" src/at_handler.c >/dev/null; then
  echo "ADC handlers still return NOT_IMPLEMENTED placeholders"
  exit 1
fi

# threshold query AT command should exist
rg -n "AT\+THRESH\?" src/at_handler.c >/dev/null

echo "verify_factory_v2_state1: PASS"
