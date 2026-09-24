#!/bin/zsh
cd -- "$(dirname "$0")" || exit 1
.venv/bin/python scripts/sync_game_data.py --offline-ok || exit 1
exec .venv/bin/python scripts/altitude_launch.py --open
