@echo off
setlocal
cd /d "%~dp0"
set PYTHONUTF8=1
set PYTHONHOME=
set PYTHONPATH=
set PYTHONNOUSERSITE=1
set "APP_PYTHON=.venv\Scripts\python.exe"
if exist "runtime\python.exe" set "APP_PYTHON=runtime\python.exe"
"%APP_PYTHON%" scripts\sync_game_data.py --force
pause
