# Speed–altitude SEP plot

Double-click **Launch Altitude Plotter.command**, or run:

```sh
.venv/bin/python scripts/altitude_launch.py --open
```

The application opens at <http://127.0.0.1:8766>. Choose an aircraft, fuel,
throttle, fixed sweep where available, and speed/altitude ranges. The default
**Fast** sampling checks SEP interpolation to 1 m/s. Smooth and Detailed
request 0.5 and 0.15 m/s checks and take longer.

This is a clean, Instructor-independent calculation. There is no RB/SB switch
or requested-flaps control. Flaps are retracted and torque/gyro assistance is
fixed off. Legacy API conditions normalize to this same calculation and cache
key, regardless of their Instructor/flap settings.

Contours retain negative SEP and emphasize zero SEP. Aircraft speed limits,
sample markers, unresolved-region markers and point inspection are available.
SVG, PNG, PDF, CSV and JSON exports use the same result. Completed results can
be reopened using their `?job=...` URL.

## 2D envelope and 3D surface

Use **2D envelope / 3D surface** to switch views without recalculating the
flight model. 2D remains the default. The 3D view loads the same checked grid
on demand, preserving its masked values. Height and color show SEP, white
marks zero SEP, and gold shows the best-SEP guide. Drag to orbit, scroll to
zoom, or reset the view. Each view preserves its camera/zoom while switching.
Samples and operating-point inspection work in both views. Unresolved markers
appear on the 3D floor, with SEP explicitly unavailable.

**3D PNG** saves the current camera view. The links under the chart export the
2D envelope (SVG/PNG/PDF) and data (CSV/JSON). Saved result URLs preserve the
selected view with `view=3d`. Older saved surfaces also support the new view
and guide without rerunning their flight-model calculations.

## Continuous best-SEP guide

The gold overlay spans the minimum through maximum plotted altitude and
tracks high-SEP speeds. SEP is the rate of total energy-height gain:
`Ps = d(h + V²/(2g))/dt`. Both speed and altitude contribute to energy.
There is no duration, start-speed input, target-altitude input, or terminal
zoom/minimum-climb-time objective. Guide CSV exports altitude, TAS, SEP and
specific power (W/kg).

The guide includes every local speed maximum at every surface altitude row,
plus regularly spaced connecting candidates and the edges of checked regions.
A graph selects a connected schedule with the highest trapezoidal
altitude-averaged SEP among these candidates. When row maxima can be connected,
it follows them; when disconnected, the full-range connected branch matters.
Every connecting cell must be checked. Negative SEP remains valid (the best
available rate may still mean energy loss). If no checked connection spans the
full requested altitude range, the interface explains why no guide is drawn.
Intermediate guide points follow the surface rather than cutting a 3D chord.

This is a continuous speed schedule, not a dynamically feasible flight
trajectory or a guarantee of instantaneous global optimality between grid
rows. Acceleration limits, pitch transitions and climb-angle drag are not
modeled. Rapid switches between preferred speeds therefore require judgment.
The former minimum-time climb modules and their reports remain historical
analysis; the current feature does not invoke their route planners.

The current full-range F-16 Fast result took **6.8 seconds** from Calculate to
rendered 2D plot, including the guide. Browser checks measured **0.7 seconds**
for the first 3D switch and **0.4 seconds** for a subsequent switch. These are
workstation-specific measurements; see `energy-performance.json` and
`surface-browser-validation.json` in `analysis/altitude-envelope`.

## Earlier performance measurements

Measured uncached browser Calculate-to-render times on this workstation,
including result transfer and plotting, at 100–1,600 km/h and Fast sampling:

| Aircraft | Altitude range | Calculate to chart |
| --- | --- | --- |
| F-16A ADF | 0–16 km | 5.1 s |
| F-16A ADF | 0–18 km | 6.1 s |
| Gripen C | 0–16 km | 3.2 s |

The service prepares six persistent calculation workers once during startup
(about 6–8 seconds measured). A changed equation backend may require a separate
one-time private compile before startup. Completed results are cached. These
measurements cover the listed jets, not every aircraft or sampling setting.
Propeller support is experimental and can be slower.

## Isolation from the EM investigation

The existing EM page, server, solver, sampler, launcher and cache were not
edited by this feature. Its files are `app/altitude.*` and
`scripts/altitude_*.py`; its results are in `outputs/altitude`.

The launcher copies the Python code and compiled backend to a private snapshot
under `outputs/altitude-runtime`. Aircraft data and the interface are read from
the workspace. If its copied backend is stale, it rebuilds only inside that
private snapshot. It never rebuilds the workspace's shared native backend.
Direct diagnostic scripts use Python when the shared backend is stale.

Separate persistent workers keep progress, cancellation and downloads
responsive. The existing EM service on port 8765 remains independent. Restart
the altitude service to adopt later equation fixes. Saved artifacts retain
their equation fingerprint. This feature has not been published.

## Calculation and limits

Each sample solves the existing TrimSolver at 1 g and zero turn rate, with
Instructor disabled. All original equilibrium, propulsion-settling, control,
stall and enabled structural checks remain in force. Nearby Jacobians and
accepted solutions supply initial search directions; final acceptance still
uses the unchanged solver. A small bank can balance asymmetric forces.

SEP is native finite-step energy-height change at a horizontal trimmed
operating point, not a simulated climb. The native high-altitude correction
remains active. Its one-sided rounding-sensitive rejection gets a bounded
search for a slightly nonascending seed, followed by the full acceptance
checks. The search target is -0.000002 g, within the original 0.0002 g force
tolerance; no tolerance or physical exclusion is weakened.

Speed curves use piecewise cubic interpolation checked by additional actual
trim solves. Independent solves between altitude rows check the second
coordinate (midpoints in Fast; quarter points and midpoints in Smooth/Detailed).
Checks evaluate speed polynomials directly, avoiding a fixed display-grid
error floor. Fine settings resolve valid speed curvature below 1 km/h.
Coarse numerical edge failures are revisited when closer accepted seeds
become available, using a bounded recovery search with unchanged trim limits.
Checked linear and fixed-Mach alternatives handle sharp changes without
transporting boundary noise into interior contours. Altitude interpolation uses atmospheric density and splits at the
aircraft's engine-table altitude knots and the atmospheric transition. Failed
checks trigger more rows. Accepted altitude interpolation stays fixed while
neighboring bands refine. The interpolation and rendering grid follow both
feasible speed edges; IAS/Mach and plot-limit intersections are explicit
altitude knots. Isolated numerical holes get neighboring-seed/full-search
retries with unchanged acceptance. Unresolved intervals and rejected trim stay
masked locally, rather than removing an entire altitude band. Additional
speed probes narrow these masks; passing mask-boundary samples remain visible.
The contour display grid adapts to curve curvature and includes exact mask
boundaries, so a display cell cannot erase a neighboring checked contour.
Dense rendering nodes are interpolated values, never presented as solved
operating points. CSV and the Samples overlay contain the actual solves.

The selected tolerances are sampled local checks, not global error bounds.
Negative SEP is not a trim failure; blank areas and plot-range edges are not
certified ceilings. Fuel, boost supply and health are frozen. Aircraft are
fully upgraded with closed radiators and no inferred stores/ammunition.

## Verification

The dedicated checks write to `analysis/altitude-envelope`:

```sh
.venv/bin/python scripts/verify_altitude.py
.venv/bin/python scripts/verify_altitude_energy.py
.venv/bin/python scripts/verify_altitude_fine.py
WT_EM_BACKEND=python .venv/bin/python scripts/verify_altitude_native.py
.venv/bin/python scripts/verify_altitude_surface.py PATH_TO_RESULT/data.json
.venv/bin/python scripts/verify_altitude_browser.py --job COMPLETED_JOB_ID
.venv/bin/python scripts/verify_altitude_3d.py --job COMPLETED_JOB_ID
```

Checks cover analytic linear/closed nonlinear contours using the production
sampler, masked failures, cancellation and worker recovery, mode-independent
cache keys, input validation, exports, browser rendering/inspection and mobile
layout. Additional real solves check exported contours. Native tests replay
selected F-16 and Gripen engine/aero/body states at 0, 8, 16 and 18 km against
the pinned original executable, using the existing harness's explicit adapters
and frozen-fuel/health scope. This is not live-flight or catalog-wide validation.
