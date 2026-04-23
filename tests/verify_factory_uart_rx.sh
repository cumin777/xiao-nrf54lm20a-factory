#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$repo_root"

rg -n "static bool uart_rx_is_printable" src/main.c >/dev/null
rg -n "static void uart_rx_reset" src/main.c >/dev/null
rg -n "if \\(!uart_rx_is_printable\\(ch\\)\\)" src/main.c >/dev/null
if rg -n "if \\(ch != 'A'\\)|if \\(ch != 'T'\\)" src/main.c >/dev/null; then
  echo "AT-only UART gating still present"
  exit 1
fi

echo "verify_factory_uart_rx: PASS"
