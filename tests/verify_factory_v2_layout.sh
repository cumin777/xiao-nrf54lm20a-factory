#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$repo_root"

# Required module split
test -f src/at_handler.c
test -f src/at_handler.h
test -f src/factory_storage.c
test -f src/factory_storage.h

# Hardcoded unsafe flash address must be removed
if rg -n "FACTORY_FLASH_OFFSET|0xFE000" src >/dev/null; then
  echo "Unsafe hardcoded flash offset detected"
  exit 1
fi

# Required state commands must exist
rg -n "AT\+STATE1" src/at_handler.c >/dev/null
rg -n "AT\+STATE8A" src/at_handler.c >/dev/null
rg -n "AT\+STATE8B" src/at_handler.c >/dev/null
rg -n "AT\+STATE9B" src/at_handler.c >/dev/null

# AT+FLASH support must exist in handler
rg -n "AT\+FLASH" src/at_handler.c >/dev/null

# Main should call AT handler module
rg -n "at_handler_process_line|at_handler_init" src/main.c >/dev/null

echo "verify_factory_v2_layout: PASS"
