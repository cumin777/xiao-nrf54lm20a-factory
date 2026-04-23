#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$repo_root"

rg -n "static bool g_led_ready" src/main.c >/dev/null
rg -n "if \\(g_led_ready\\)" src/main.c >/dev/null

if rg -n "if \\(!gpio_is_ready_dt\\(&led\\)\\)" src/main.c >/dev/null; then
  echo "main loop still depends on LED readiness"
  exit 1
fi

echo "verify_factory_v3_startup: PASS"
