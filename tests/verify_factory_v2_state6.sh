#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$repo_root"

# State6 should use SW0 alias + gpio callback based keywake path
rg -n "DT_ALIAS\(sw0\)|gpio_callback|gpio_pin_interrupt_configure_dt" src/at_handler.c >/dev/null

# KEYWAKE should not remain NOT_IMPLEMENTED placeholder
if rg -n "STATE6\", \"KEYWAKE\", \"err:KEY_PATH_TBD" src/at_handler.c >/dev/null; then
  echo "KEYWAKE still NOT_IMPLEMENTED"
  exit 1
fi

# command should remain exposed
rg -n "AT\+KEYWAKE|AT\+STATE6" src/at_handler.c >/dev/null

echo "verify_factory_v2_state6: PASS"
