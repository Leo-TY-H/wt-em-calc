The replacement stationary solver is the default for new API and browser
calculations. It solves aircraft equilibrium, retained wing history and propeller
closure through one evaluator and one shared numerical corrector.

Charts use the **alpha-march chart solver** by default (`chart_solver="march"`).
At each speed it starts from admitted level flight and raises alpha until the
first physical limit; that state is the envelope point. A speed tracer places
the columns (corners, edges and withheld midpoint checks). The selected SEP
levels are then solved directly from the same columns and checked between
knots with withheld exact solves. Every drawn state is an admitted full-model
state with the unchanged physical checks, at zero prescribed sideslip. The
envelope tolerance is 0.15 deg/s at standard precision and the curve tolerance
is three times the chart curve tolerance. A compiled evaluate-and-correct loop
(`scripts/march_native`, built into `.native_march/`) reproduces the Python
evaluation bit for bit; it is used only while the Python sources it was
verified against are unchanged (`scripts/verify_march_native.py --stamp`
re-verifies and records them). Otherwise the same chart is computed in Python,
more slowly. Unresolved speeds stay gaps in the outline, and only the calculated
SEP levels are available; another level needs a new calculation.

Configurations outside the march solver's validated domain use the previous
adaptive speed/load sampler with PCHIP interpolation for that aircraft:
manual engine controls, unavailable wing sweep in the speed range,
non-default load caps, and the diagnostic trim, sampling and model-experiment
modes. The same applies to any aircraft on which the march fails. The chart
records the reason (`chart_solver_fallback`). `chart_solver="baseline"` selects
the previous sampler for a request, and `WT_EM_CHART_SOLVER=baseline` makes it
the server default (`stationary_execution="baseline"`).

**Enable exports** is off by default. Normal calculations prepare the interactive
chart and point inspection while skipping downloadable-data conversion. Enable
the switch before calculating to prepare exports immediately, or turn it on
afterward and download the existing result without another solve. The first
download then prepares the export data once. API requests can opt in with
`enable_exports: true`; explicit command-line file exports remain available.

The aircraft picker follows the [War Thunder Wiki aviation roster](https://wiki.warthunder.com/aviation),
including playable premium, squadron, event and hidden aircraft. Drones, Zeppelin,
ship-launched scouts, mission/tutorial aircraft and unlisted internal variants
are excluded. The curated list in `references/aircraft-exclusions.json` is kept
in Git and survives game-data refreshes. Exclusions also apply to the displayed
variants and armament choices of aircraft sharing a flight model. Source assets
remain available because retained aircraft can share them.

### Solver and model settings

Browser calculations use the stationary solver, solver-default Mach arithmetic,
retained aerodynamic history, full trim and a fixed 48 Hz simulation rate.
These options have no browser controls. Copying or loading old conditions applies
these fixed choices to the next calculation; existing results keep their actual
recorded settings. **Include default ammunition** starts enabled for aircraft
with supported ammunition data and can still be unchecked. It includes the
standard gun belts and countermeasures in the mass model. **Additional settings**
retains ammunition, armament variant and additional mass controls.

Instructor assistance applies the game's authored payload multiplier when
calculating centre of gravity and inertia; total payload mass is unchanged.
This follows the assistance setting independently of manual trim, torque/gyro
and Flutter. Aircraft using advanced mass or an explicit full-payload CG rule
retain their native mass behavior.

The diagnostic API still accepts explicit solver/model settings.
`trim_solver_mode="nested"` selects the original nested correction algorithm
with current propulsion conventions, retained history and full trim.
`trim_solver_mode="off"` performs an uncorrected trial through `POST /api/trial`,
which takes `settings`, `speed_kmh`, `load_g`, optional `coordinates` and optional
propeller `request_indices`. Residuals are reported; admission is unchecked.
Chart requests reject off mode before scheduling work.

New stationary settings use `mach_curve_mode="stable"`: evaluate the existing
stored mode-3 Mach polynomial pieces in binary64 with a final binary32
multiplier round. Recentered polynomial evaluation reduces cancellation while
preserving the generated coefficients, breakpoints and genuine joins. It does
not refit or smooth the curves. Other arithmetic remains native. Explicit
`"native"` settings preserve the original rounded arithmetic; baseline/off
settings without an explicit arithmetic choice default to native. Saved settings
record the choice, and saved-point replay checks it. The legacy `"continuous"`
experiment remains available only as an explicit smoothing approximation.

The API-only instantaneous-aerodynamics and lateral-trim experiments retain
their declared modeling effects. Explicit API settings and saved-point replay
can use them; the browser always uses retained history and full trim.
Instantaneous aerodynamics uses current flow/controls/wash for wing angles and
current lift for downwash, removing history lag and spin memory. Ideal symmetry
sets side force to zero with ideal roll/yaw constraint reactions and neutral
aileron/rudder; it does not establish physical roll/yaw control authority.
The zero-sideslip option retains physical roll/yaw balance. Exported settings
and labels identify these experiments.

### Numerical method and cost

`JointProblem` owns equilibrium and physical evaluation. Operating points and
boundaries share a bounded Newton/trust-region corrector with scaled equations,
branch-aware differences and protected steps across native rounding jumps.
Production correction stops at the existing componentwise closure ceilings,
followed by the final physical checks. Boundaries and conditional history use
their declared row requirements without an additional raw precision target.
Stalled full-aircraft trials far outside closure stop unresolved after two fresh
linearizations without useful progress; trials within ten times the closure
scales retain the longer six-iteration recovery window. Inner propulsion/history
and boundary corrections retain their existing recovery window. Reusing a previous
Jacobian does not consume the fresh-linearization allowance. Numerical closure
and physical admission thresholds are unchanged. Optimizer success alone never
admits a point. Explicit research diagnostics can request tighter precision
without imposing it on ordinary charts.

Accepted steps update the local Jacobian with scaled Broyden secants. Nearby
points reuse that model as a hint, checking the actual equations and branch
before using it; poor predictions trigger fresh differences. Conditional history
probes start from nearby solved histories. Exact samples are reused within a
correction instead of evaluating the same coordinates again for reporting.
Load and speed sampling preserve one nearby admitted propeller/history state
and request identity alongside the aircraft prediction. Internal states from
different roots are not averaged, and every continued point is recorrected.
Each configured solver keeps a bounded cache of admitted complete states for
nearby requests. After finding a valid interior, the sampler tries downward
continuation through earlier unclosed load trials once before refining a lower
edge. A balanced physical rejection still stops that continuation.

Derivative probes defer final reporting and reuse exact aircraft preparation
when only propulsion or retained history changes. Structural propulsion
dependencies avoid irrelevant nominal-stencil comparisons without removing
rounded-sensitivity safeguards. Previews keep dense surfaces as arrays; final
export validates and converts each array once to the existing list/null format.

An admitted continuation or ordinary candidate ends the ordinary search.
Alternative starts and bounded propeller branch discovery are failure recovery;
they do not run again after success. Boundary retries reuse active-face/cap
corrections while their starting state is unchanged and reuse identical
midpoints. Source hashing, interval proofs and whole-domain searches are outside
the production point path. These rules are shared by every aircraft; there are
no aircraft IDs, fitted per-aircraft tolerances or special speed bands in the
optimization policy.

The baseline load sampler establishes a plot cap or observed rejection bracket
before asking for stationary boundary correction. It does not pretrace all
potential physical limits at every speed column. Shared boundary correction,
fold localization and physical admission remain available on observed brackets.

Near a narrow pitch-response limit, stationary command probes contract when the
initial step overshoots the rising load branch. A balanced state already exactly
at a manual control stop supplies that reached boundary instead of searching for
an inset copy. The preceding-branch checks, force/moment closure, physical guards
and fallback sideslip recovery remain in use.

When balanced load corrections approach pitch-response reversal and it becomes
the nearest constraint, boundary recovery tries a command-coordinate bracket
before general continuation. This requires a balanced rejected endpoint and
the existing preceding-branch checks; an unsuccessful attempt leaves general
continuation and sideslip recovery available. Numerical failures alone never
establish a physical limit.

Instructor one-g autotrim initializes tail force from pitch-moment balance and
updates the wing-angle estimate once to account for that force's lift. The
existing three-equation correction and physical acceptance limits still apply.
Boundary ordering permits up to **0.001 g** backward load and **0.0001Â°** backward
AoA after a candidate passes physical admission; forward extent is unchanged.

The default numerical SEP allowance is **0.05 m/s**. Physical limits remain
separate from numerical error and chart interpolation tolerances. Refinement
comparisons include both samples' numerical allowances; contour and interpolation
inversions stop at output accuracy rather than invisible decimal precision. The solver
retains strict forward-flow, bank, stall, Instructor, control and structural
checks, bounded governor equilibrium, manual-pitch shaft closure, and signed
propeller absorbed power. Continuation selects an admitted local branch;
bounded discovery supplies seeds when ordinary correction fails. Full governor
ordering and aircraft preference across all competing branches remain unproved.

Stationary refinement stops below the configured speed/turn display tolerances.
This includes low-speed level-flight localization. At a propeller pitch stop,
the normal stationary governor solves RPM from torque balance: underspeed at
minimum pitch or overspeed at maximum pitch. The same reduced coordinate
changes pitch at requested RPM between the stops. Actual engine over-rev is
rejected and located as an RPM boundary in the default stationary mode.
Pitch stops are mode transitions, not aircraft stall boundaries. The API-only
`stationary_governor_mode="exact_rpm"` retains the former restricted model for
diagnostics; the default is `"pitch_stops"`. Torque, wake and aircraft balance
must still close in either mode. Source-join columns reuse nearby admitted
internal states and recorrect them with the equations on the new side. Local
load probes retain a balanced physical rejection for boundary localization.
A governor mode change alone does not require extra quarter-point probes when
its checked midpoint already meets chart accuracy. Failed samples remain
explicit gaps; locating a transition to chart resolution is not a coverage proof.

Chart altitude sets atmospheric conditions; the aerodynamic model assumes flight
outside ground effect, including charts at sea-level altitude.

Wing data loading supports explicit flap-polar tables and nested `Polar.NoFlaps`
and `Polar.FullFlaps` endpoints. Each endpoint inherits shared polar properties
before defaults are applied, preserving the declared clean and deployed-flap
lift, drag and stall angles.

At known aerodynamic formula switches, one-sided chart limits are displayed at
the switch when their sampled speed bracket is narrower than chart resolution.
The curves remain separate across the jump; physical samples and unresolved
numerical gaps are preserved. A localized trim maximum takes precedence over a
later stall or control-limit hint. A rejected post-stall root is checked against
continuation of the admitted branch before it can limit that branch's load.

Locally admitted results have `valid=true`, `status="admitted"` and can be
charted. Failed searches remain unresolved, never proved infeasible. These
results do not certify complete global branch coverage or selection; separate
certification flags remain false. Actual sampling gaps and unresolved boundaries
remain visible in chart metadata. Ordinary charts do not require global proofs.

Optional research modules and the explicit `"columns"`/`"serial_cells"`
executions remain for investigation. Their local interval certificates and
incomplete root censuses are not production acceptance gates. The original
universal milestone remains unproved; the user's amended delivery goal is an
efficient practical replacement. See `SOLVER-WORK.md` for assumptions, historical
proof work, and the bounded final validation record.

The latest full default F-16 chart completed in **16.312 s** versus the saved
native nested baseline's **17.312 s**, including previews and chart preparation,
with 12 workers and identical chart settings. Retained-column aircraft evaluations
fell by 35.2%. Build/syntax and 137 affected checks per backend passed. Saved
boundary/contour overlays are essentially coincident at full chart scale. These
individual runs do not establish a universal runtime or reliability guarantee.
Exact settings, limitations and preserved reports are in `SOLVER-WORK.md`.

# neothunderism

[Open the website](https://neothunderism.pages.dev) or [download the Windows app](https://github.com/Leo-TY-H/wt-em-calc/releases/latest/download/neothunderism-windows-x64.zip).

On the website, add aircraft, choose flight conditions, and click **Calculate diagram**. Hover over the plot to inspect results; use the download buttons to save charts or data.

The EM calculator keeps its equation version for the current session. **Calculate** does not check for updates or restart the app. After local equation edits, refresh the browser to load the changes before calculating. A refresh during an active calculation keeps that calculation's version; refresh again after it finishes to apply updates.

For propeller aircraft, enable **Manual engine controls** to set the available propeller pitch, oil radiator and radiator controls per entry. Pitch uses the aircraft's direct blade-angle or RPM-governor behavior; automatic propeller control can be retained. Radiator opening changes drag. The calculation stops at the first predicted **Over-rev boundary**, using steady axial flow at the selected altitude and throttle. The RPM search stops at VNE (or an earlier enforced speed limit), and no over-rev line is drawn if no crossing occurs before it. An entry already over-revving at the minimum plotted speed is skipped; unresolved RPM searches stop further sampling and are reported. Individual turning points retain their own RPM diagnostics. Thermal endurance, engine damage and transient overspeed are not simulated.

The propeller slider identifies what it controls: **Propeller RPM setting** selects within the aircraft's governor RPM range (including type-1 Spitfire and type-2 governors); **Propeller blade-angle setting** directly selects blade pitch on type-6/type-8 systems such as the Bf 109. A manual RPM setting retains pitch regulation up to the physical blade-pitch stops. Its percentage is a position within the configured RPM range, not a percentage of maximum RPM.

The baseline solver's quasi-steady propeller governor follows the first torque-balanced branch reached from minimum blade pitch. It settles induced airflow and brackets shaft torque separately, so previous trim guesses cannot select a different stalled-blade equilibrium. Unresolved branch searches are reported as propulsion failures. This defines a repeatable steady chart; transient governor hysteresis and alternative equilibria retained by prior flight history are not simulated. Exported propulsion diagnostics include the branch policy, search resolution and pitch bracket. The replacement stationary solver instead follows the locally admitted continuation branch as described above.

During that baseline search, fixed target RPM allows engine torque and atmospheric conditions to be reused within each flight condition. Only the independent shaft's propellers are resampled, with bounded secant acceleration of wake closure. A shared torque/wake kernel avoids assembling unused reporting outputs during the search; blade lift/drag evaluation skips unused section pitching-moment table lookups. The complete propulsion equations still validate the result at the original tolerances. Diagnostics count these propeller samples separately from complete engine frames. For diagnostic comparisons, `WT_EM_PREPARED_GOVERNOR=0` and `WT_EM_GOVERNOR_ACCELERATION=0` disable prepared evaluation and wake acceleration without changing branch policy.

Blade geometry and Mach-independent polar parameters are prepared once per flight condition. The compiled wake loop and evaluator reuse private residual, flow and coefficient buffers; Mach-dependent coefficients are recalculated at every trial. Returned results remain detached from scratch storage. The Python backend runs the same wake algorithm, with the same convergence, cancellation and work-limit checks.

Flap charts show a dashed **Flap deployment limit** when inhibited outward travel prevents deployment before the selected extension's hold limit. Above the dashed line, the chart assumes prior deployment; the envelope still ends at the applicable automatic-retraction or structural limit. Limits are converted to true airspeed at the selected altitude. The chart's flap percentage is actual extension: automatic IAS/Mach schedules clamp the requested target before the flaps move. For example, a MiG-29 (9-12A) takeoff request can deploy only 20% at high speed, starting from retracted; it does not require full takeoff extension first. No separate deployment line is drawn when deployment and hold limits coincide.

The **Missile simulator** tab offers starter scenarios, missile search and a live setup preview. Enter the launcher aircraft and target aircraft states at release, using km/h for speed and metres for position. Aircraft face their travel direction, with zero roll. The launcher uses Launch angle; the target uses Climb angle. The simulator applies the selected missile's release processing and ignition/guidance timing, plots the engagement, supports timeline playback and exports JSON with SI units. The target follows constant velocity; ideal visibility and radar support retain geometric seeker and flight limits. The model is experimental, and a point-proximity event does not assert aircraft damage.

After a simulation, hover over the missile or its traveled path for speed, Mach, altitude, range and force data. Expand **Propulsion & aerodynamic plots** to select thrust, mass, propellant consumed, drag/lift, drag coefficient, angle of attack, dynamic pressure or net acceleration. Click a chart point to move the 3D timeline there. Chart toolbars save PNGs; **Flight data JSON** includes the numerical telemetry. Missing dynamics at an exact proximity event remain blank.

Missiles are listed as **Display name [missile_id]**, excluding DEFAULT profiles and duplicate display names. Use the same search-and-add menu as the EM aircraft picker and click **Add** beside a missile to compare up to eight missiles with the same aircraft setup. Up to four independent flights calculate concurrently and share a color-coded 3D view, timeline and telemetry plots. Use **Inspect missile result** or a colored legend button for each outcome. Comparison JSON includes every completed flight and identifies incomplete or cancelled comparisons. The target is visual only: its path and marker do not show hover data or attract selection.

The missile simulator and launch envelope use a standalone C++ calculation engine by default (`WT_MISSILE_ENGINE=native`). Body physics, motors, guidance, optical/radar tracking and managers, session stepping, termination, numerical telemetry, envelope setup and search all execute in the native process. Python is used to build the engine and serve or transport application requests; the calculation executable does not load a Python interpreter. The recovered game model remains the source specification.

Envelope evaluations now use typed C++ state throughout the flight loop. Profile validation, property preparation and typed property conversion run once per missile and worker; the constructed and launched body and the seeker reset are kept per launcher state, which every sample of an envelope shares; only target-dependent seeker initialization runs for each geometry. The final comparison covered all 227 profiles, 908 flights and 490,982 steps with identical initial states, stored step fields and results, including 681 warm-cache cases and zero fallback. The full simulator continues to use the accepted standalone native implementation.

With 15 workers, three-run median times for launcher-centred envelopes (launcher at 6,000 m and 900 km/h, target hot at 900 km/h, five layers from 2,000 to 10,000 m) were **0.50 / 0.65 / 0.86 s** for AIM-9L at Quick / Standard / Detailed (66 / 103 / 136 directions) and **2.0 / 3.4 / 5.4 s** for AIM-120C-5 (79 / 139 / 230 directions). Every evaluation is bit-identical to the 2026.10.25.1 engine. Timings depend on the scenario and machine load; the figures above use the original validation machine and are not server response-time guarantees.

Build the engine in a source checkout with `python scripts/build_missile_engine.py`, using Python with setuptools and a C++17 compiler (MSVC on Windows). The executable and build manifest are written under `.native_missile_engine/`. A missing or stale native build produces an error; it never silently switches to Python calculations. **Restart the app/server after upgrading** so both services use native setup and job handling. `WT_MISSILE_WORKERS` limits concurrent simulator jobs to 1â€“4.

For reference comparisons, explicitly set `WT_MISSILE_ENGINE=python`; `WT_MISSILE_BACKEND=reference` selects the original reference backend within that path. The older `WT_MISSILE_NATIVE` and compiled-helper settings apply only to the legacy Python path. The standalone engine also accepts a JSON request on standard input with `--mode simulator` or `--mode envelope` and streams NDJSON progress and results. The build manifest records source hashes and the executable identity.

The **Launch envelope** tab draws where a missile can hit, in 3D, around the launcher: the launcher sits at the centre flying along +X, and each point is a target position at release, one altitude layer at a time. The target flies straight and level toward, away from, parallel to or at a fixed heading from the launcher. A launch counts as a hit when the missile's proximity fuze fires, exactly as in the Missile simulator, so fuze arming and radius shape the envelope together with the motor, mass, drag, lift, control, guidance and seeker limits. Along horizontal directions from the launcher the launch distance is bisected between hits and misses with complete simulator runs. Directions start on a regular grid (30°, 15° or 10° for Quick, Standard and Detailed); between two finished neighbours another direction is added where one hits and the other does not (down to 1°, 0.5° or 0.25°), and where their gaps differ or the boundary bends, so the outline is resolved where the envelope ends or changes while empty sky costs little. New directions start from the boundaries their neighbours predict. The page draws the hit region as a smooth filled outline through every searched boundary (monotone spline interpolation between directions and layers) in the 3D envelope, a top view of one layer and a side profile; **Directions** and **Samples** show what was searched. Click the outline to list that direction's launches, then **Fly this shot in the simulator** to replay the farthest hit in the Missile simulator tab. Exports include every searched direction (CSV) and full JSON. Near a boundary, hits and narrow misses can alternate with very small changes in the launch; each search keeps the changes it brackets. The rays searched, and therefore the envelope, are the same for any number of threads. `WT_ENVELOPE_WORKERS` sets the native simulation thread count (default: logical CPUs − 1, bounded to 1–16; an explicit setting is bounded to 1–61).

For Windows 10/11 x64:

1. Extract the entire Windows ZIP into a writable folder.
2. Run **Launch EM Plotter.cmd** or **Launch Altitude Plotter.cmd**. Keep its console window open while using the app.
3. Launchers open using bundled data without checking for updates or requiring internet access. Run **Update Game Data.cmd** to refresh aircraft data when GitHub is accessible.

No Python installation is needed. To update the app, close it and extract the new download into a new folder.
