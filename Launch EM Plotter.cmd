@echo off
setlocal
cd /d "%~dp0"
set PYTHONUTF8=1
set OPENBLAS_NUM_THREADS=1
set OMP_NUM_THREADS=1
if not exist ".venv\Scripts\python.exe" (
    echo Run Setup Windows.cmd first.
    pause
    exit /b 1
)
".venv\Scripts\python.exe" scripts\sync_game_data.py --offline-ok
if errorlevel 1 (
    pause
    exit /b 1
)
".venv\Scripts\python.exe" scripts\em_server.py --open
if errorlevel 1 pause
