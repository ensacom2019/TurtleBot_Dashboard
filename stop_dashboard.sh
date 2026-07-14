#!/usr/bin/env bash
set -euo pipefail

pkill -f "python3 .*server.py" 2>/dev/null || true
pkill -f "python .*server.py" 2>/dev/null || true
echo "Stopped dashboard server processes."
