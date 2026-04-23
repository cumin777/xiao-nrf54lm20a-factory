#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$repo_root"

rg -n "debug_send_label|debug_send_line|debug_send_u32|debug_send_hex8|debug_send_rx_line" src/main.c src/at_handler.c >/dev/null
rg -n "PARSE:raw=|PARSE:trimmed=|PARSE:dispatch_table=|PARSE:final_reason=" src/at_handler.c >/dev/null
rg -n "BOOT:boot_flag=|BOOT:path=|RX_LINE:" src/main.c >/dev/null

echo "verify_factory_v3_uart_debug: PASS"
