#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$repo_root"

rg -n "CONFIG_FACTORY_IMU_SAMPLE_TIMEOUT_MS|at_get_imu_sample_with_timeout|at_imu_worker_thread_fn" src/at_handler.c zephyr/Kconfig >/dev/null
rg -n "ERROR:HW_TIMEOUT|HW_TIMEOUT|HW_BUSY" src/at_handler.c docs/AT_USAGE.md >/dev/null
rg -n "accel data:.*gyro data: " src/at_handler.c docs/AT_USAGE.md >/dev/null
rg -n "g_parser_debug_logging_enabled" src/at_handler.c >/dev/null
rg -n "PARSE:text_argv" src/at_handler.c >/dev/null
rg -n "!g_uart_debug_logging_enabled" src/main.c >/dev/null

echo "verify_factory_v3_imu_safe: PASS"
