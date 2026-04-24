#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$repo_root"

rg -n "at_get_imu_sample_direct|at_ensure_imu_ready|at_fetch_imu_sample" src/at_handler.c >/dev/null
! rg -n "at_get_imu_sample_with_timeout|at_imu_worker_thread_fn|g_imu_worker" src/at_handler.c >/dev/null
rg -n "sensor_trigger_set\\(|at_imu_trigger_handler|at_imu_store_latest_sample|g_imu_sample_ready" src/at_handler.c >/dev/null
rg -n "sensor_attr_set\\(|at_imu_set_sampling_freq" src/at_handler.c >/dev/null
rg -n "CONFIG_LSM6DSL_TRIGGER_GLOBAL_THREAD=y|CONFIG_SYSTEM_WORKQUEUE_STACK_SIZE=2048|CONFIG_MAIN_STACK_SIZE=2048" zephyr/prj.conf >/dev/null
! rg -n "CONFIG_LSM6DSL_ACCEL_ODR=2|CONFIG_LSM6DSL_GYRO_ODR=2" zephyr/prj.conf >/dev/null
rg -n "accel data:.*gyro data: " src/at_handler.c docs/AT_USAGE.md >/dev/null
rg -n "CONFIG_FACTORY_IMU_TRACE|imu_trace_line|imu_trace_rc|\\[IMU\\]" src/at_handler.c zephyr/Kconfig >/dev/null
! rg -n "CONFIG_FACTORY_IMU_TRACE=y" zephyr/prj.conf >/dev/null
rg -n "g_parser_debug_logging_enabled" src/at_handler.c >/dev/null
rg -n "PARSE:text_argv" src/at_handler.c >/dev/null
rg -n "!g_uart_debug_logging_enabled" src/main.c >/dev/null

echo "verify_factory_v3_imu_safe: PASS"
