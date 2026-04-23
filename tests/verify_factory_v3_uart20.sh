#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$repo_root"

rg -n "&uart20" zephyr/boards/xiao_nrf54lm20a_nrf54lm20a_cpuapp.overlay >/dev/null
rg -n "at_handler_poll_background" src/main.c src/at_handler.h src/at_handler.c >/dev/null
rg -n "AT\\+UART20TEST|uart20 on|whoami|NRF54LM20A|UART20TEST" src/at_handler.c docs/AT_USAGE.md >/dev/null

echo "verify_factory_v3_uart20: PASS"
