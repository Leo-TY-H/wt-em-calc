@echo off
setlocal
cd /d "%~dp0"
set PYTHONUTF8=1
".venv\Scripts\python.exe" scripts\sync_game_data.py --force
if errorlevel 1 goto done
".venv\Scripts\python.exe" scripts\update_pages_snapshot.py --offline
:done
pause
