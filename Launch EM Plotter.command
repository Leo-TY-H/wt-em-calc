#!/bin/zsh
cd -- "$(dirname "$0")" || exit 1
.venv/bin/python scripts/build_em_backend.py || exit 1
.venv/bin/python scripts/sync_game_data.py --offline-ok || exit 1
exec .venv/bin/python scripts/em_server.py --open
