#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$repo_root"

rg -n "static bool at_rx_is_printable" src/main.c >/dev/null
rg -n "static void at_rx_reset" src/main.c >/dev/null
rg -n "if \\(!at_rx_is_printable\\(ch\\)\\)" src/main.c >/dev/null
rg -n "if \\(at_len == 0U\\) \\{" src/main.c >/dev/null
rg -n "if \\(ch != 'A'\\)" src/main.c >/dev/null
rg -n "if \\(ch != 'T'\\)" src/main.c >/dev/null

echo "verify_factory_uart_rx: PASS"
