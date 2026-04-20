#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$repo_root"

# State4 should use real IMU/DMIC APIs and the IMU deferred-init sequence from the working sample
rg -n "device_init\\(g_imu_dev\\)" src/at_handler.c >/dev/null
rg -n "sensor_sample_fetch_chan\\(g_imu_dev, SENSOR_CHAN_ACCEL_XYZ\\)" src/at_handler.c >/dev/null
rg -n "sensor_sample_fetch_chan\\(g_imu_dev, SENSOR_CHAN_GYRO_XYZ\\)" src/at_handler.c >/dev/null
rg -n "zephyr,deferred-init" zephyr/boards/xiao_nrf54lm20a_nrf54lm20a_cpuapp.overlay >/dev/null
rg -n "dmic_configure|dmic_trigger|dmic_read|AUDIO_DMIC" src/at_handler.c >/dev/null

# State4 placeholders must be removed
if rg -n "STATE4\", \"IMU6D\", \"err:IMU_NOT_READY|STATE4\", \"MICAMP\", \"err:DMIC_NOT_READY" src/at_handler.c >/dev/null; then
  echo "State4 handlers still NOT_IMPLEMENTED"
  exit 1
fi

# Required configs
rg -n "CONFIG_SENSOR=y" zephyr/prj.conf >/dev/null
rg -n "CONFIG_LSM6DSL=y" zephyr/prj.conf >/dev/null
rg -n "CONFIG_AUDIO=y" zephyr/prj.conf >/dev/null
rg -n "CONFIG_AUDIO_DMIC=y" zephyr/prj.conf >/dev/null

echo "verify_factory_v2_state4: PASS"
