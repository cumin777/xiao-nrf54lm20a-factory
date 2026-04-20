#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$repo_root"

# State4 should use real IMU/DMIC APIs
rg -n "sensor_sample_fetch|sensor_channel_get|SENSOR_CHAN_ACCEL_X|SENSOR_CHAN_GYRO_X" src/at_handler.c >/dev/null
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
