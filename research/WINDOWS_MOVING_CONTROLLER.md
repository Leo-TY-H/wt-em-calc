# Windows moving-controller continuation — 2026-09-24

The requested chart definition remains **maximum balanced turn with full pitch
input**, under the configuration and control mode selected before plotting.
No configuration switching, freely chosen entry trim, or stationary-history
assumption has been added to that definition. The production Instructor chart
remains experimental. These results close native-execution gaps; they do not
certify the maximum boundary.

## Catalog and data

The 36 user-approved unsupported fixed-wing IDs are omitted through
[`aircraft-exclusions.json`](../references/aircraft-exclusions.json). The list
survives data refreshes and enters the catalog cache signature. All 1,312
previously supported entries remain. Local and static website metadata contain
1,431 entries: those 1,312 supported aircraft and 119 unsupported helicopter or
zeppelin records. Shared raw FM sources are retained for other vehicle aliases.

A fresh upstream check verified all 2,879 source files at datamine **2.59.0.34**,
commit `2e2b2e050d80802a64dc1e155d16e088cf2cf122`. Installed Windows code is
**2.59.0.31**, SHA-256
`eaee1800caef7a2012cbe028d4830d0ffc529d282c9f7f57a3f84b9bc2973048`.
Current data and the pinned executable are distinct versions.

## Original Windows execution

`windows_instructor_controller.py` executes full update `1430933b0`, including
reduced predictors, disabled-output MouseAim internals, moving body prediction,
actuator trim updates and snapshot restores. Arithmetic consumers execute
original instructions. It remains an explicit prepared free-air fixture:

- The inherited Mac fixture supplies source FM properties and initialization.
- Sweep/polar property providers supply prepared authored data. Native flap
  lookup `142fc1a40` executes against separately packed value/index arrays.
- Empty actor-owned collections and absent optional components are explicit
  interface values; control-history/publication predicates are prescribed true.
- Scratch allocation is bounded and fixture-owned; free is a no-op. Seven
  fleet entries exercise dynamic MouseAim allocation in the tested case.
- Propulsion inputs are held. Contact handling uses the native disabled gate.
  Scene, spawning, network clocks and whole-engine evolution are not recovered.

The named Windows Instructor/MouseAim groups are initialized separately from
adjacent atmospheric globals. Detailed-body property requests use prescribed
sweep, rather than a stale reduced-predictor stack pointer.

The Windows polar constructor groups the high-side parabola numerator as
`critical - (slope * line + offset)` at `143149ebd..143149eca`; the Mac port
groups it as `(critical - offset) - slope * line`. Float32 rounding differs.
`windows_instructor_source.py` independently reproduces the Windows expression
from source coefficients. It does not read native intermediate results. The
existing portable calculator equations retain their previous behavior.

## Measured checks

The compact [validation record](windows-moving-controller-validation.json)
contains counts, configuration, exclusions, limitations and full-report hashes.

| Check | Coverage | Result |
| --- | --- | --- |
| Moving full-pitch controller | All 1,312 supported entries, one prescribed case each | Zero execution errors or command/history/predictor-input differences |
| Native pitch-trim slew | 105,562 updates in that fleet run | Every observed update matches the independent float32 slew rule |
| Internal restore retention | 4,660 fleet restores | Requested/actual/cache trim preserved |
| Varied controller cases | Six aircraft, four conditions, moving and static: 48 calls | Zero comparison differences; 4,078 trim updates and 316 restores |
| Owner final restore/publication | Six aircraft × eight cases, 22 selected fields | Zero failures; full delivered record and final trim replacement checked |
| Owner look-ahead count cap | 2,000 prescribed clock/tick cases | Zero differences |
| Owner loop call protocol | 120 cases with 0–20 look-ahead ticks | Zero differences in order, arguments, pilot-request restoration and final dt |

The fleet condition is 250 m/s, 18-degree body AoA, 1,500 m altitude, 30% fuel,
30% fixed flaps, zero sweep, full pitch +1, roll .15, yaw -.05, RB gyro/torque
off and 48 Hz. Entry requested/actual/cached trim are explicit test vectors.
This is kernel coverage, not a feasible balanced operating point for every
aircraft. The varied cases cover 0/30/37/100% flaps, available sweep, auto-trim
enabled/disabled, both gyro modes and differing speeds/heights/angles.

The command/history comparison uses a readable controller with independently
called original Windows reduced predictors and independent Windows polar
rounding. It does not substitute captured results into the update under test.
Moving actual trim is checked separately against each native actuator write.
The optional protection-stage observation executes on 1,304 fleet branches;
all observed polar fields and angle limits match. Eight alternative branches
are covered by the complete command/history and predictor-input comparisons.

Owner final restore `140e53e75..140e53f56` executes native snapshot restore,
record copy, cached-trim publication and cleanup. Actor transform publication
is a recording callback. Owner scheduling tests use recording doubles for
loop children, so those tests certify the call protocol only.

Eight data-sync tests, compiled portable runtime checks with spawned workers,
native aerodynamic comparison, positive/negative stall-policy checks and
website metadata regeneration also pass. The two earlier conditional J6K1
roots are unchanged after applying Windows polar rounding.

## Remaining boundary gates

1. Complete caller/physics initialization and composition. The full outer
   look-ahead loop reaches additional physics dependencies beyond the inner
   Instructor fixture. A direct prepared-loop probe stops at integer division
   `142fbe7d2`: its simulation-parameter interval at parameter `+10` is not
   initialized. This is a missing fixture input, not evidence of a game bug.
   Derive this and subsequent owner/engine/scene inputs from their native
   producers; do not guess values merely to pass execution.
2. Determine which retained controller states can contribute to a balanced
   maximum with the chosen configuration fixed. Native trim retention is now
   checked on Windows, but does not make arbitrary entry trim attainable.
3. Couple that state evolution to physical force/moment closure, and search
   disconnected valid regions without treating failed solves as rejection.
   The J6K1 physical interval from 12 to 13 g remains unresolved.
4. Validate the resulting boundary and its interpretation across aircraft,
   configurations and speed columns before replacing the experimental chart.

No new maximum boundary is published by these changes. The conditional-root
driver still holds entry actual trim; the moving fixture is not silently
substituted into it without a validated balanced-source/owner adapter.

## Reproduction

From the project root with the local executable oracles installed:

```powershell
$env:PYTHONUTF8 = '1'
& '.\.venv\Scripts\python.exe' scripts/verify_windows_controller.py --binary 'D:\New folder\Games\War Thunder\win64\aces.exe' --all-aircraft --moving-only --cases 1 --out analysis/windows-instructor/windows-fleet-controller-validation.json
& '.\.venv\Scripts\python.exe' scripts/verify_windows_controller.py --binary 'D:\New folder\Games\War Thunder\win64\aces.exe'
& '.\.venv\Scripts\python.exe' scripts/verify_windows_owner_restore.py --binary 'D:\New folder\Games\War Thunder\win64\aces.exe'
& '.\.venv\Scripts\python.exe' scripts/verify_windows_owner_protocol.py --binary 'D:\New folder\Games\War Thunder\win64\aces.exe'
```

Open plotters cache their catalog and need a restart to load the removals.
