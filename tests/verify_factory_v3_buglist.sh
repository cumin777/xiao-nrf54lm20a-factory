#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$repo_root"

test -f docs/BUG_LIST_V3.md
rg -n "bt scan on|may crash or hang the device|Status: Open" docs/BUG_LIST_V3.md >/dev/null

echo "verify_factory_v3_buglist: PASS"
