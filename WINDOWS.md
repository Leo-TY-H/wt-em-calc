# Windows setup and transfer

1. Copy `WT-FM-Reconstruction-Windows-2026-09-24.zip` and extract it to a short
   local path such as `C:\WT-FM`. Do not run launchers inside the ZIP viewer.
2. Install **64-bit Python 3.11** from [python.org](https://www.python.org/downloads/windows/),
   including the Python launcher (`py`). The pinned dependencies target this
   version. Do not copy the Mac `.venv` or compiled `.native_em` directory.
3. Install [Visual Studio Build Tools](https://visualstudio.microsoft.com/visual-cpp-build-tools/)
   with **Desktop development with C++**, the MSVC x64 compiler and Windows SDK.
   The calculator builds Cython extensions for useful runtime speed.
4. Double-click `Setup Windows.cmd`. It creates `.venv`, installs the pinned
   requirements, compiles the equations and runs the installation checks.
5. Double-click `Launch EM Plotter.cmd` or `Launch Altitude Plotter.cmd`. Leave
   the console open while using the local browser application. Close it with
   Ctrl+C to stop the server.

If `py -3.11` is unavailable, install Python 3.11 with the launcher, or create
`.venv` with an existing 64-bit Python 3.11 before running setup. On this device
the environment has already been created from the `aero-ai` Conda environment;
the C++ tools, compiled backend and runtime checks have passed. See
[the Windows continuation](research/WINDOWS_INSTRUCTOR_PROGRESS.md).
If setup reports a Microsoft C++ compiler error, install the workload above and
rerun setup; if discovery still fails, run it from the x64 Native Tools command
prompt. Internet is required for the initial dependency installation; bundled
source data support local calculations afterward.

To rerun checks from Command Prompt in the project folder:

```bat
set PYTHONUTF8=1
.venv\Scripts\python.exe scripts\verify_portable_runtime.py
```

The launchers enable UTF-8 and limit numerical library threads. Windows uses
spawned calculation workers and a Windows build lock. The altitude launcher
runs directly from this folder, avoiding privileged symlinks; stop the server
before editing equation files. The native x86 Mach-O reference is input to the
Unicorn emulator: it is not launched as a Windows program, and no installed Mac
game is needed for the included native checks.

MSVC uses `/O2 /fp:strict`; the existing Mac/Linux flags remain unchanged.
[`/fp:strict` documentation](https://learn.microsoft.com/en-us/cpp/build/reference/fp-specify-floating-point-behavior?view=msvc-170)
explains its preservation of floating-point ordering and exclusion of fused
contractions. Windows compilation, spawned workers and bounded numerical checks
have now passed on the destination device. These checks do not certify a global
Instructor boundary; Windows browser interaction and runtime speed have not
been benchmarked in this continuation.

For diagnosis only, `set WT_EM_BACKEND=python` explicitly selects the slower
reference equations. Normal setup and launch expect the compiled backend; the
installation smoke check deliberately rejects a Python-only backend.

Read `HANDOFF.md` when continuing development. `TRANSFER_MANIFEST.json` records
the files and SHA-256 hashes in the original transfer snapshot; data updates
will change those files. This folder is now connected to `Leo-TY-H/wt-em-calc`.
Use `Push to GitHub.cmd` for local changes and see `DATA_SYNC.md` for automatic
game-data updates. Retired experiments remain in the Mac workspace/archive.
