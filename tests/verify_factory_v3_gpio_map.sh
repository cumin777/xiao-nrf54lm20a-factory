#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$repo_root"

rg -n "g_gpio_text_pairs|g_gpio_text_pair_state|gpio_text_find_pair|gpio_text_configure_endpoint|gpio_text_read_endpoint" src/at_handler.c >/dev/null
rg -n "PRECONDITION_NOT_MET" docs/AT_USAGE.md >/dev/null

rg -n "\\{ \\{ 1, 31 \\}, \\{ 1, 3 \\} \\}" src/at_handler.c >/dev/null
rg -n "\\{ \\{ 1, 30 \\}, \\{ 1, 7 \\} \\}" src/at_handler.c >/dev/null
rg -n "\\{ \\{ 0, 0 \\}, \\{ 0, 3 \\} \\}" src/at_handler.c >/dev/null
rg -n "\\{ \\{ 0, 1 \\}, \\{ 0, 4 \\} \\}" src/at_handler.c >/dev/null
rg -n "\\{ \\{ 0, 2 \\}, \\{ 0, 5 \\} \\}" src/at_handler.c >/dev/null
rg -n "\\{ \\{ 1, 6 \\}, \\{ 3, 10 \\} \\}" src/at_handler.c >/dev/null
rg -n "\\{ \\{ 1, 5 \\}, \\{ 3, 9 \\} \\}" src/at_handler.c >/dev/null
rg -n "\\{ \\{ 1, 4 \\}, \\{ 3, 11 \\} \\}" src/at_handler.c >/dev/null
rg -n "\\{ \\{ 3, 0 \\}, \\{ 3, 7 \\} \\}" src/at_handler.c >/dev/null
rg -n "\\{ \\{ 3, 1 \\}, \\{ 3, 6 \\} \\}" src/at_handler.c >/dev/null
rg -n "\\{ \\{ 3, 2 \\}, \\{ 3, 5 \\} \\}" src/at_handler.c >/dev/null
rg -n "\\{ \\{ 3, 3 \\}, \\{ 1, 1 \\} \\}" src/at_handler.c >/dev/null
rg -n "\\{ \\{ 3, 4 \\}, \\{ 1, 2 \\} \\}" src/at_handler.c >/dev/null

echo "verify_factory_v3_gpio_map: PASS"
