# Windows Instructor work — 2026-09-24

## Target and current status

The user resumed Instructor research after the Windows transfer and selected
**maximum balanced turn with full pitch input** as the boundary definition.
This supersedes the earlier instruction to wait for a new research decision.
It does not validate any withdrawn method or the existing approximation.

The Windows compiled calculator now runs, and a real missing branch affecting
11 catalog records has been repaired. The all-aircraft maximum boundary is
**not established**. Retain the experimental designation in the application.
Matching isolated controller calls cannot establish which retained states are
attainable in a balanced turn, or justify classifying every point below one cap
as permitted. See [RETIRED_APPROACHES.md](RETIRED_APPROACHES.md).

## Changes

- Fixed `build_em_backend.py` for both Python 3.11 and newer `ast.unparse`
  spellings of tuple targets. The generator in a generated `cpdef` is replaced
  with the same two-coordinate predicate; no arithmetic policy changed.
- Fixed `InstructorNative.setup` to initialize the property pointer for every
  prepared engine. A twin-engine test previously dereferenced a null pointer.
  These remain prescribed healthy, non-VTOL fixtures, not live engine states.
- Recovered reduced Instructor wake modes 0 and 1 from the pinned original
  predictor. Mode 0 omits wash; mode 1 uses the raw-angle left-wing CL and its
  own propwash/swirl convention. Neither is interchangeable with mode 2 or
  the detailed aircraft aerodynamic wake. Pitch prediction, auto-trim and the
  active experimental forward balance now share the recovered dispatch.
- Added a read-only AMD64 PE locator and an offline harness for two reviewed
  Instructor stages from the installed Windows executable. The harness rejects
  any different binary hash. No game executable was modified or launched.
- Added native regression coverage and optional separate report destinations.
  Existing historical validation artifacts were not overwritten.

Legacy records are `go229_v3`, `f1m2`, `o3u_1`, `fw_200c_1`, `il-10`,
`il-10_1946`, `il-10_1946_china`, `il-10_1946_hungary`,
`uav_inf_fpv_strike_drone`, `uav_inf_recon_drone` and `uav_quadcopter`.
The other 1,301 supported records use wake mode 2. Wake type is obtained from
prepared aircraft data; no aircraft-name exceptions enter the equations.

## Executable and data provenance

Installed Windows executable:
`D:\New folder\Games\War Thunder\win64\aces.exe`, version **2.59.0.31**,
SHA-256 `eaee1800caef7a2012cbe028d4830d0ffc529d282c9f7f57a3f84b9bc2973048`.

The bundled x64 Mach-O oracle has SHA-256
`820fee4a55601ffa459e2635da4ce1436cfc7adcc703a7c799380c95d136ebd5`.
It is emulated offline on Windows; it is not executed by Windows. Existing
aircraft data are independently pinned, including datamine version 2.59.0.13.
This work does not establish patch equality of the installed game and those
data, or port the complete Windows predictor/owner implementation.

Reviewed preferred Windows addresses (valid only for the hash above):

| Purpose | Address |
| --- | --- |
| Instructor property loader | `0x143093050` |
| Instructor update | `0x1430933b0` |
| Instructor defaults | `0x143099fa0` |
| Auto-trim wrapper | `0x1430b6210` |
| Pitch predictor wrapper | `0x1430b6a00` |
| Pitch-clamp test span, end exclusive | `0x14309635b`–`0x143096560` |
| Conditional trim-writeback test span, end exclusive | `0x143093c49`–`0x143093c8f` |

The PE locator uses exception-directory ranges and instruction-checked byte
candidates. It deliberately leaves gaps/leaf functions unattributed. It is not
a complete control-flow analysis or proof that unreported references are absent.

## Observed checks

Reports and logs are in `analysis/windows-instructor/`.

| Check | Observed result | Scope |
| --- | --- | --- |
| MSVC backend build | 45 extensions built | Python 3.11.15, Cython 3.1.4, MSVC 14.44; `/fp:strict` |
| Portable runtime | PASS | Spawned compiled workers, F-16XL Instructor on/off, J6K1 off, exact F-16XL native aero replay, positive/negative stall policy |
| Propulsion kernels | 25,392 blade and 5,704 owner cases; zero differences | Compiled/reference equality across 713 pinned graphs, both engine-control modes |
| Installed Windows stages | 2,400 cases; zero differences | 2,000 pitch clamps, 200 successful and 200 failed trim writebacks; no substituted calls within tested spans |
| Legacy wake plus three mode-2 controls | 336 calls each in Python and compiled; zero differences | All 13 predictor outputs, success/failure status and history; 224 mode-1 and 112 mode-0 calls per backend |
| Full jet predictor stress | 1,628 calls; zero differences | Bundled Mach-O oracle; independent prepared inputs, one/two-engine fixtures |
| Supported fleet predictors | 15,744 calls across 1,312 records; zero differences | Four prescribed scenarios each; 10,496 mode-1 and 5,248 mode-0 calls; see backend breakdown in `fleet-predictor-validation.json` |
| Legacy production boundary probes | Five balanced boundary points at 300 km/h | Horten 229, F1M2, O3U-1, IL-10 and Fw 200; experimental model only |
| Full sampling columns | Three columns with resolved boundaries and no numerical gaps | Horten 229, F1M2 and IL-10 at 300 km/h; experimental boundary/interior machinery |
| PE parser/reference regressions | Four tests passed | File/range bounds, invalid/truncated images, gap attribution, overlapping opcode candidates |

The legacy tests prescribe force/wash, four speeds, four heights, three flap
positions and both direction flags over eight scenarios. They are not actual
engine/trajectory integrations. Native predictor failure is a legitimate output:
the test compares it and the associated retained history, rather than counting
every failed native solve as a mismatch.

Of the fleet mode-0 comparisons, 5,244 used the translated symmetric branch and
four used the existing asymmetric machine-code fallback. Those four verify
fixture/backend integration, not independent equations. All 112 mode-0 calls in
each eight-scenario legacy suite used the translated symmetric branch.

The five boundary probes have force closure below `5e-6 g`. Their serialized
`verified limit` status means a bracket/constraint of the **experimental model**,
not a verified in-game full-pitch maximum. They do not certify a whole chart.

## Reproduce on this device

The project `.venv` was created with the already installed
`C:\Users\guowe\anaconda3\envs\aero-ai\python.exe` (3.11.15). Dependencies are
from `requirements-plotter.txt`. Visual Studio 2022 Build Tools with the C++
workload and Windows SDK were installed. The normal launchers now use this
environment. Do not copy `.venv` or `.native_em` to another device.

PowerShell, from the project root:

```powershell
$env:PYTHONUTF8 = '1'
& '.\.venv\Scripts\python.exe' scripts/build_em_backend.py
& '.\.venv\Scripts\python.exe' scripts/verify_portable_runtime.py
& '.\.venv\Scripts\python.exe' scripts/verify_pe_scan.py
& '.\.venv\Scripts\python.exe' scripts/verify_instructor_legacy_wake.py
& '.\.venv\Scripts\python.exe' scripts/verify_instructor_legacy_wake.py --compiled --out analysis/windows-instructor/legacy-wake-compiled-validation.json
& '.\.venv\Scripts\python.exe' scripts/verify_instructor_legacy_wake.py --compiled --all-aircraft --cases 4 --out analysis/windows-instructor/fleet-predictor-validation.json
& '.\.venv\Scripts\python.exe' scripts/verify_windows_instructor.py --binary 'D:\New folder\Games\War Thunder\win64\aces.exe'
```

The Windows hash guard must not be removed after a game update. Rediscover and
review the instruction spans and source-field mapping before accepting another
hash. Existing raw-source/native-helper substitutions in the older Mach-O
harness retain their original scope; these are separate from the two Windows
spans tested without substituted calls.

## Remaining accuracy gates

1. Establish full-pitch permission with requested/actual trim, failed auto-trim
   retention, overload timing, authority adaptation and owner look-ahead/restore
   semantics. A local stage match or a freely allocated trim is insufficient.
2. Define and validate attainable controller histories for the maximum balanced
   turn. A stationary-memory requirement and a one-g initialization are not
   established by the selected boundary definition. Preserve the J6K1 30%-flap
   counterexample near 771 km/h and disconnected F-16XL permission evidence.
3. Validate the current Windows complete predictor and owner against current
   aircraft/configuration data, then physical boundaries and interior points.
4. Resolve unsupported data/physics records rather than fabricating geometry or
   engine behavior. The starting catalog has 1,348 records, 1,312 supported:
   29 lack matching collision geometry, four lack advanced-mass geometry, two
   lack a validated upgraded-vehicle mapping, and one lacks an RPM lifecycle.
   Exact-name inspection of 1,163 collision resources in the installed base
   aircraft logic packages found no matches for the missing propeller resources.
   `installed-collision-availability.json` records searched package hashes and
   every unsupported record. Some records are test/unused variants; this is a
   source-catalog census, not a claim that all are currently playable vehicles.

All further work must preserve force/moment closure, fixed intact flaps,
negative-SEP permission, positive-stall enforcement, the explicit negative-stall
exclusion policy and consistent boundary/interior treatment.
