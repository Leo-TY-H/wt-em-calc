# Retained-trim projection: exact local test, unresolved maximum

**The request to fully solve the continuous maximum balanced-turn boundary is
not completed.** This continuation derives an exact, guarded reduction of native
pitch permission and checks candidate auto-trim entry surfaces. It does not
establish the state set that determines the maximum, and no production
replacement is made.

The balanced-turn chart definition is already accepted. These experiments do
not change it to a maneuver simulation, require all memories to settle, or make
exhaustive flight-history optimization a prerequisite. An algebraic invariance
or dominance argument could still establish the projection. That argument has
not been found here.

## Exact permission reduction

For raw elevator demand `d` before `InvertElevator`, symmetric native authority
`[-a,a]`, matching requested/actual pitch trim `t`, inactive recovery, fixed
native predictor commands `p_low,p_high`, and fixed native authority factor
`k` in `(0,1]`, the forward trim map simplifies to

```
M(v,t) = v + (1 - abs(v))*t.
```

The native delivered interval is

```
lower = max(-a + (1-a)*t, min(p_low,t))
upper = min( a + (1-a)*t, t + k*(max(p_high,t)-t)).
```

Consequently the condition `lower <= d <= upper` reduces to membership in one
closed trim interval. `trim_interval.py` calculates that interval, intersecting
the following constraints with `[-1,1]`:

* For `a < 1`, mechanical authority gives
  `((d-a)/(1-a), (d+a)/(1-a))`, with both endpoints included.
* If `d < p_low`, lower protection requires `t <= d`.
* For `k = 1`, upper protection adds `t >= d` only when `d > p_high`.
* For `k < 1`, upper protection requires `t >= d` when `d >= p_high`,
  and otherwise `t >= (d-k*p_high)/(1-k)`.

Full authority `a=1` imposes no additional trim restriction if `d` is in
`[-1,1]`. The formula uses real arithmetic; the executable uses float32.
Native comparison tolerances remain explicit, and the formula is not used
outside its guards. In particular, it does not substitute requested trim for
moving actual trim or discard the retained inner-prediction actuator updates
documented in the preceding report.

At the previously checked high J6K1 point, the trim interval starts at about
**−0.3428746**. Retained trim near **−0.9374** is outside it. This is a necessary
and sufficient local permission test within the stated guards, **not a proof
that any member of the interval can be obtained by the game at that source**.

`verify_trim_interval.py` passes 600 randomized original native clamp
comparisons and 34 reductions of previously native-checked physical cases.
The synthetic forward delivery uses the separately verified actuator port;
these are not 600 new complete native owner/actuator trajectories. Native
classification agrees away from the explicitly excluded 3e-7 command margin.

## Entry-surface experiments

`entry_surface.py` samples physical J6K1 sources with the selected fixed 30%
flaps and original power setting at seven altitudes. Two scans attempt 350
physical sources in total: one at 1 g and another using 7 g for high-speed
samples. Failed physical solves remain failures; they do not become evidence
of failed native auto-trim. Successful auto-trim numerical seeds are carried
spatially, which is explicitly not elapsed flight.

The scans show altitude-dependent success/failure regions. A coarse preceding
successful sample is not the last successful state at the actual transition.
`entry_fronts.py` therefore refines local brackets with immutable query history:
root-query order never advances the controller. Three 7-g brackets resolve:

| Altitude, m | Successful speed, km/h | Failed speed, km/h | Successful pitch trim |
|---:|---:|---:|---:|
| 0 | 770.912130801 | 770.912190310 | −0.937418580 |
| 1,000 | 766.405679461 | 766.405738471 | −0.935023546 |
| 2,000 | 762.349642719 | 762.349705702 | −0.932169855 |

These are local brackets for the sampled sources and specified predictor
history, not unique event speeds for all aircraft histories. Successful trim
is the value eligible for retention; the different calculated trim on the
failed side must not be adopted.

The 1-g front refinements at 6,000, 8,000 and 10,000 m remain **unresolved**:
their intermediate physical solves fail trim convergence and vertical-step
closure. The evidence is preserved in `entry-fronts.json`. No inference about
an auto-trim transition is made from those failed physical queries.

`verify_entries.py` reconstructs 43 sources, including all six endpoints of
the resolved brackets, and runs the original native mode-0 predictor. Every
output field, success flag and written history matches the independent port.
Archived and reconstructed success flags agree. Reconstructed sources meet
the existing force and angular closure tolerances. The executable is pinned to
SHA-256 `820fee4a55601ffa459e2635da4ce1436cfc7adcc703a7c799380c95d136ebd5`.

## What this does and does not resolve

The local maximum calculation can test whether an **established admissible
retained-state set** intersects the required trim interval. It does not need
to optimize trim as an unconstrained aircraft coordinate. If all such trims
give the same active limit, the state can be eliminated; if another physical
restriction binds first, trim is nonbinding. Otherwise the boundary depends
on the retained state.

What is missing is an argument selecting or eliminating that set for the
requested maximum. A spatial path through auto-trim source space is not proof
of an attainable flight path. Different conditional limits do not by
themselves prove that a maximum over attainable states is undefined or
discontinuous. Conversely, a smooth conditional branch does not prove a global
maximum. Neither conclusion is asserted here.

An asynchronous question asked whether earlier configuration/mode changes are
allowed. No answer was available during these tests; the exploratory scans
kept configuration and mode fixed. That question is not about the already
accepted balanced-turn definition, and answering it alone would not supply
the missing projection proof.

Production Python sources, the server and application outputs are unchanged.
Existing application work is preserved. No fitted parameters, blend, plot,
flap-damage mask, arbitrary failed-trim reset, or all-memory fixed-point
requirement was introduced.

## Reproduction

From the workspace root:

```sh
.venv/bin/python analysis/instructor-reachable-trim-september24/verify_trim_interval.py
.venv/bin/python analysis/instructor-reachable-trim-september24/entry_surface.py
.venv/bin/python analysis/instructor-reachable-trim-september24/entry_surface.py --supported
.venv/bin/python analysis/instructor-reachable-trim-september24/entry_fronts.py
.venv/bin/python analysis/instructor-reachable-trim-september24/verify_entries.py
```

`source-audit.json` compares all 381 baseline production Python sources.
`summary.json` records the counts and unresolved status. The separate checkpoint
archive contains this investigation and the updated handoff.
