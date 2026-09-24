# Fixed-configuration balanced-turn continuation

The selected target is **maximum balanced turn with full pitch input**.
The user clarified that configuration and control mode are prescribed before
plotting and remain fixed. Earlier flap changes or Instructor mode switches
must not be introduced to improve the result. Internal requested/actual trim
and controller history must be derived under that fixed configuration.

This continuation executes the installed Windows reduced predictor and finds
conditional full-pitch force/moment roots. **It does not yet establish the
maximum boundary across attainable internal states.** The application's
Instructor approximation remains experimental.

## Data and prepared assets

Verified all 2,879 source files at datamine **2.59.0.34**, commit
`2e2b2e050d80802a64dc1e155d16e088cf2cf122`: 1,226 FMs, 1,649 vehicle
records and four shared configuration files.

SB-25J Netherlands propulsion and mass assets were regenerated from its current
FM and exact installed collision resource. The original property/mass producer
ran again; stored FM hashes were not simply relabeled. Native and independent
mass consumers agree at 0%, 30%, 70% and 100% fuel. Its full experimental
sampling column also passes, and the website catalog enables it.

The catalog contains 1,467 entries, of which 1,312 are supported. All 1,312 were
included in the Windows predictor comparison. The 155 unsupported entries
comprise 119 without prepared propulsion assets (helicopter/zeppelin records),
29 without matching collision geometry, four without advanced-mass component
geometry, two without validated upgrade mappings, and one without a running
RPM lifecycle. Catalog completeness does not imply every entry is playable.

`refresh_prop_assets.py` publishes regenerated propulsion/mass JSON and the
merged propulsion manifest together after native mass checks. It requires the
local pinned oracle and exact installed geometry. Launch-time and six-hour
GitHub raw-data updates remain configured. Prepared geometry and executable
behavior have independent version pins; source downloads cannot certify them.

## Windows predictor execution

Installed executable version **2.59.0.31**, SHA-256
`eaee1800caef7a2012cbe028d4830d0ffc529d282c9f7f57a3f84b9bc2973048`.
Its version is separate from the newer datamine version above.

`windows_instructor_native.py` executes the complete common reduced predictor
at `0x1430b06c0`, called by the pitch and auto-trim wrappers at `0x1430b6a00`
and `0x1430b6210`. Windows polar, control and math consumers execute original
instructions. The prepared sweep provider (`0x142fdae80`) and flap packer
(`0x143140ce0`) are explicit adapters; releasing their fixture-owned arrays
is a no-op. The bundled Mach-O fixture builder supplies prepared source fields.
This is not a Windows game-state initializer or complete moving owner replay.

Four scenarios per supported entry exercise speed, altitude, flaps, sweep where
available, wake and both predictor modes:

| Result | Observed value |
| --- | --- |
| Complete Windows calls | 15,744 across 1,312 entries |
| Mode 1 / mode 0 | 10,496 / 5,248 |
| Execution errors / decision mismatches / history-write flag mismatches | 0 / 0 / 0 |
| Bit-exact calls / calls with float differences | 5,448 / 10,296 |
| Largest command difference | 0.00009518861770629883, Saab B17BS mode 0 |
| Largest angle difference | 0.000152587890625 degrees, same case |

These are explicit Windows-versus-port numerical differences, not exact
equivalence. Of the mode-0 comparisons, 5,244 use the translated branch and
four use the existing asymmetric Mach-O fallback. Those four do not validate
independent equations. This sample grid does not prove agreement arbitrarily
close to every controller transition.

## Coupled conditional roots

`probe_windows_balanced_turn.py` uses physical equilibria as candidates, then
feeds their source state into the keyboard controller with original Windows
predictors. Roll and yaw requests balance their required delivered controls;
pitch request is +1. The physical candidate solver's free control coordinates
do not grant free trim to the Instructor: requested/actual entry trim is
explicit and held during each conditional query. Root queries do not advance
controller history. Source solving requests tighter numerical closure.

J6K1: 771.03162178 km/h TAS, sea level, 30% fuel, 30% fixed intact flaps,
WEP command, automatic engine control, no payload, RB torque/gyro off, 48 Hz.
No configuration or mode switch is used. Diagnostic histories have zero
overload timer, unit authority factor and explicit predictor/angle histories.

| Conditional entry pitch trim | Full-pitch root | AoA | Force error | Angular error |
| --- | --- | --- | --- | --- |
| 0 | 11.342124939 g | 7.148774990 degrees | 0.0000148441 g | 0.00000768815 rad/s² |
| -0.937431693 | 7.138725281 g | 1.072938676 degrees | 0.00000326490 g | 0.00000128698 rad/s² |

All three delivered-control residuals are below 0.00000027. Both roots meet
the existing physical acceptance criteria; the errors above are measured
closure, not the tighter solver's requested stopping target. Failed auto-trim
retains the specified request in both cases.

These are **not two user-selectable maximum boundaries**, nor evidence that
either entry history is reachable at that point. Actual trim is held during
the conditional query. Its advancement inside moving prediction and
persistence through owner restores are not yet replayed by this driver.

All sampled sign-changing intervals between 5 and 13 g were refined. The
physical interval from 12 to 13 g remains unresolved. Tangencies and narrower
components are not excluded. Even this single speed column is not a global
maximum certificate. Do not promote these conditional roots into the UI.

## Validation and reproduction

The compact [validation record](windows-balanced-turn-validation.json) retains
measured roots, configuration, counts, limitations and SHA-256 hashes of full
local reports under `analysis/windows-instructor/`.

Additional passing checks: eight data-sync tests; compiled keyboard regression
on F-16XL, JAS39C and Horten 229 (12 updates, zero differences); portable runtime
with spawned compiled workers, native aero and positive/negative stall policy;
and website metadata regeneration. Windows source rebuilds now publish
extensions in a signature-specific directory so an open plotter does not lock
the next source generation's DLLs. The manifest switches after all 45
extensions build successfully. Open plotters need a restart to load new data.

From the project root in PowerShell:

```powershell
$env:PYTHONUTF8 = '1'
& '.\.venv\Scripts\python.exe' scripts/sync_game_data.py --force
& '.\.venv\Scripts\python.exe' scripts/refresh_prop_assets.py sb_25j_netherlands --game 'D:\New folder\Games\War Thunder'
& '.\.venv\Scripts\python.exe' scripts/update_pages_snapshot.py --offline
& '.\.venv\Scripts\python.exe' scripts/verify_windows_predictor.py --binary 'D:\New folder\Games\War Thunder\win64\aces.exe' --all-aircraft --out analysis/windows-instructor/windows-fleet-predictor-validation.json
& '.\.venv\Scripts\python.exe' scripts/probe_windows_balanced_turn.py --binary 'D:\New folder\Games\War Thunder\win64\aces.exe'
```

Next required work is the Windows moving prediction/owner trim path and a
justified projection onto maximum balanced flight under fixed configuration.
It must preserve retained state across prediction restores, exclude unresolved
physical solves, and handle disconnected permission regions. Requiring all
memories to become stationary or assuming a valid one-g starting point remains
unjustified; see [withdrawn approaches](RETIRED_APPROACHES.md).
