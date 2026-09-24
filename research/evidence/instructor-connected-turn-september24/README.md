# Trial: first restriction when progressively tightening a balanced turn

The user selected the connected-turn definition on September 24, 2026,
replacing the requirement to find the highest permitted separated component.
This trial implements that selection in an isolated continuation calculation.
**It is not installed in production, and full AoA-versus-speed curves are not
yet validated.** No plots were generated or application files changed.

## Definition tested

At each fixed speed, altitude, mass, configuration and power, start at a
permitted 1 g equilibrium. Increase the turn coordinate `u = sqrt(n² - 1)`
and correct each neighboring state with the full aircraft equations. Retain
nonzero turn rates, negative SEP, fixed intact flaps, normal pitch response,
stall, structural and delivered-control limits. Stop at the first restriction
encountered along this branch. Higher permitted components do not extend it.

The endpoint's body AoA is the proposed AoA-versus-speed value. A missing or
unresolved 1 g starting equilibrium leaves this definition unresolved at that
speed; the calculation does not silently start on a higher-load island.
This is a static equilibrium sequence, not a simulated constant-speed maneuver
or a proof of dynamic reachability. Controller history does not advance with
the numerical continuation steps.

Controller assumptions and the delivered-control predicate are unchanged from
the [static-reference investigation](../instructor-static-reference-september24/README.md):
ascending canonical native one-g auto-trim observations, retained trim on
failure, settled actual trim, steady adjusted-angle history, native rate
feedback, and overload timer one through the authored mapping. The two
controller modules here are copies of that archived candidate, explicitly
selected only by these trial scripts.

## Implementation and limits of the numerical experiment

`connected_trace.py` adapts the existing level-flight continuation. It removes
the recovery paths that resume above failed intervals, limits the trial turn
step to `0.1 * resolution`, limits accepted AoA increments to
`0.2° * resolution`, and refines observed balanced rejection brackets.
It disables the production permission-triggered search for alternative rounded
equilibria. Every tested source is first a corrected physical state and then
classified by the same native delivered-control predicate.

The ordinary run uses existing force/moment acceptance. The polished study
requests tighter correction on **every** solve, before permission is known:
targets 1e-6 g and 2e-6 rad/s², with the production final acceptance tolerances
unchanged at 2e-4 g and 5e-5 rad/s². It does not selectively improve rejected
sources until they become permitted.

An `envelope_limit` and `unresolved=False` in these research JSON files certify
an observed local endpoint bracket, **not absence of earlier narrow exclusions**.
The continuation and bracket routines still use finite samples; active-constraint
fallbacks do not prove coverage between them. The strict numerical title does
not confer a mathematical connected-component certificate. Resolution studies
are therefore essential, and production integration is intentionally deferred.

## Results

At the selected standard conditions (sea level, 30% fuel, 110% selected power,
RB torque/gyro convention), three resolutions 1, 1/2 and 1/4 produced:

| Aircraft and speed | Endpoint body AoA | Load | Restriction |
|---|---:|---:|---|
| F-16XL, 475 km/h | 26.17165–26.17170° | 8.46702–8.46704 g | Instructor |
| F-16XL, 1000 km/h | 8.37234–8.37237° | 13.25636–13.25639 g | Instructor |
| J35D, 800 km/h | 17.98814–17.98823° | 12.98008–12.98016 g | Instructor |
| F/A-18E, 800 km/h | 26.05139–26.05349° | 14.71721–14.71795 g | Wing force |
| Yak-3, 400 km/h | 13.88816–13.88820° | 5.54988–5.54989 g | Instructor |

These are individual speed columns, not complete curves. Jet columns cost
roughly 0.5–1.8 seconds and Yak-3 roughly 2.0–4.8 seconds across those resolutions,
excluding interpreter/startup. They are not cold full-chart benchmarks.

### Low-speed F-16XL

At 157.91807776 km/h, ordinary resolutions 1 and 1/4 missed an earlier narrow
rejection and returned approximately 26.56°. Resolution 1/2 found 25.6793°.
Native replay confirms the earlier rejected endpoint at 1.3090643 g / 25.6825°
has a command margin around -0.04321. It is not a port mismatch or a margin
barely below zero. Predictor commands change abruptly between nearby sources.

With uniform physical polishing, resolutions 1/8, 1/16, 1/32 and 1/64 find
25.68196°, 25.67972°, 25.67618° and 25.68031°, respectively, all near 1.309 g.
This is useful numerical agreement around **25.68°**, with a 0.00579° spread,
but does not rule out even narrower earlier rejections. The finest run uses
2,711 continuation points and about 7.82 seconds for this single speed.
Ordinary, unpolished resolution 1/32 instead stops on an unresolved numerical
interval at approximately 1.07085 g; that failed run is retained.

Thus the revised definition removes the need to search for a higher admissible
island, but still requires finding the first narrow exclusion reliably and
economically. It must not be implemented as a coarse first failed sample.

### J6K1, fixed 30% flaps

At 770.9 and 771.03162178 km/h, the 1 g initialization fails physical closure,
with force errors around 0.00626 and 0.01030 g. These errors exceed acceptance;
they are not Instructor exclusions. No connected endpoint is returned.

An independent descending-load check from a known gentle-turn seed finds
balanced, permitted states down to 1.01 g at 770.9 km/h and 1.02 g at
771.03162 km/h. Smaller tested loads again lose physical closure near the
negative-angle stall region. This does **not** prove that every possible 1 g
root is absent. It establishes that the current starting-equilibrium search is
unresolved and cannot certify connection to level flight.

The earlier 7.13893 g / 1.07316° local high-load crossing remains valid under
its static reference, but this trial does not establish its connection to a
permitted 1 g start. It must not be presented as the new connected boundary.

## Native and physical verification

`check_native.py` replays saved endpoint coordinates without re-trimming to a
different root. Eighteen original-controller/actuator/delivery comparisons
cover accepted and rejected bracket endpoints for F-16XL, J35D and Yak-3.
Four additional comparisons cover the polished low-speed F-16XL endpoints at
resolutions 1/8 and 1/64. **All 22 match**, including the permission decisions;
accepted native deliveries satisfy the unchanged full-aircraft closure checks.
The wing-force F/A-18E endpoint is a physical check, not one of these native
Instructor endpoint comparisons.

Binary SHA-256:
`820fee4a55601ffa459e2635da4ce1436cfc7adcc703a7c799380c95d136ebd5`.

## Remaining work

1. Resolve the J6K1 starting equilibrium; distinguish a demonstrated physical
   minimum-load edge from a failed root search. Starting at a minimum-load
   gentle turn instead of 1 g would be a further explicit definition choice.
2. Detect/refine narrow first exclusions without applying thousands of samples
   to every speed. Preserve numerical uncertainty where coverage is unresolved.
3. Integrate the selected endpoint and the same connected-branch exclusion into
   every boundary/interior path, including speed probes and inherited seeds.
4. Validate complete curves, interpolation and cold runtime before production
   promotion. Stable isolated columns and matching native updates do not fulfill
   that requirement.

The new definition is useful and the experiment produces repeatable local
results. It has not yet completed the original implementation task under the
revised definition.

Reproduce the principal trials:

```sh
.venv/bin/python analysis/instructor-connected-turn-september24/run_trial.py
.venv/bin/python analysis/instructor-connected-turn-september24/check_native.py
.venv/bin/python analysis/instructor-connected-turn-september24/check_level.py
```

`run_trial.run(tag, resolution, polished=True)` runs the uniform-polishing
variant. `summary.json`, logs and full source-point JSON retain all successful
and unresolved outcomes. Production Python and pre-existing application edits
were not modified by this trial.
