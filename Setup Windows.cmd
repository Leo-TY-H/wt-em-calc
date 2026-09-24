@echo off
setlocal
cd /d "%~dp0"
set PYTHONUTF8=1
if not exist ".venv\Scripts\python.exe" (
    py -3.11 -m venv .venv
    if errorlevel 1 goto failed
)
".venv\Scripts\python.exe" -m pip install -r requirements-plotter.txt
if errorlevel 1 goto failed
".venv\Scripts\python.exe" scripts\build_em_backend.py
if errorlevel 1 goto failed
".venv\Scripts\python.exe" scripts\verify_portable_runtime.py
if errorlevel 1 goto failed
echo Setup and runtime checks passed. Use Launch EM Plotter.cmd or Launch Altitude Plotter.cmd.
pause
exit /b 0
:failed
echo Setup failed. See WINDOWS.md for Python and C++ compiler requirements.
pause
exit /b 1
