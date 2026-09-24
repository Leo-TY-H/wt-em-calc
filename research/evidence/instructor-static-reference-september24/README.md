# Static native permission: scalar-cap counterexample; replacement not completed

**The requested production maximum balanced-turn boundary is not completed.**
A candidate was implemented in the production call path and exercised through
complete chart calculations. It was withdrawn after maximum-selection and
runtime validation failed. All 381 pre-existing production Python sources now
match the preceding source manifest exactly. The pre-existing application edits
are preserved. The server was not restarted. No plots were generated.

This investigation does **not** make unrestricted flight-history optimization a
prerequisite. It specifies a static reference and finds a concrete difficulty
inside that reference: native permission is not always a downward-closed set of
balanced AoAs. A maximum endpoint alone cannot classify its interior.

## Restriction derived from the native calculation

Let `d` be the raw elevator required by the **full** balanced aircraft, before
`InvertElevator`. Full force balance and the discrete angular-rate balance find
this command; negative SEP is allowed and the actual nonzero turn rates remain
in the aircraft and mode-1 predictor inputs.

Let `p−, p+` be the commands returned by the two native mode-1 calculations,
including their finite iteration, saturation, tail-angle bounds and output on
failure. Native command protection consumes these outputs even when the
predictor reports failure. Mode-0 success/failure has a *different* consequence:
only successful automatic trim writes its requested trim and cache.

For symmetric authority `[-a,a]`, equal requested and actual trim `t`, unit
authority adaptation, retained pitch trim and no effective post-stall recovery,
the native composition reduces, apart from float32 rounding, to:

```
M(v,t) = v + (1 - abs(v))*t
L = max(-a + (1-a)*t, min(p−,t))
U = min( a + (1-a)*t, max(p+,t))
permitted <=> L <= d <= U
```

This follows by applying the native requested-trim inverse in
`protected_pitch_command`, clamping the pilot input, mapping authority, and
applying `trimmed_targets` with actual trim. For unequal authority endpoints or
unequal requested/actual trim, use that original composition, not the symmetric
formula. The candidate does so. It also checks roll/yaw mechanical authority
with the native trim-availability gates. Reversed clamp endpoints retain the
native `min(max(request, low), high)` ordering.

Trim cancels from the upper bound only when `t <= p+ <= M(a,t)`. It does not
cancel when hardware clips, or when native protection includes the trim request
above the predictor output. With negative trim and reduced authority,
`M(a,t)=a+(1-a)t`; retaining more negative trim decreases available pull.

The existing forward reduced-moment correction remains a useful equation
identity. For a delivered elevator, it computes the reduced acceleration

```
b(d) = (F_tail_required_at_zero_reduced_acceleration - F_tail_at_d)
       * reduced_tail_lever / pitch_inertia
```

It is generally nonzero at a full-aircraft equilibrium. The full aircraft has
its own component flows, controls, moments and turn-rate terms; setting this
reduced acceleration to zero loses their difference. The PD demand must use the
native adjusted wing angles, target-angle clamps, rate feedback and overload
mapping.

But replacing returned predictor-command permission with an inequality on
`b(d)` requires additional conditions: the same monotone inverse branch,
inactive tail-angle/command saturation, and convergence of the finite predictor
to that inverse. Even there, native trim inclusion and mechanical clipping must
still be applied. The previous production reduction did not establish those
conditions generally, optimized trim freely, and disabled overload behavior.
Selected forward-balance matches do not establish their global validity.

## Explicit static reference tested

The archived candidate defines the following reference, not an unrestricted
live-controller envelope:

* At each selected aircraft/mass/altitude/configuration/power, start native
  mode-0 request/cache at zero and visit axial-freestream propulsion observations
  in **ascending** speed at 25, 50, 75, ... km/h. Mode-0 itself solves its native
  simplified horizontal one-g condition. These are prescribed reference inputs,
  not a flown path or high-g trim allocation.
* Apply one native auto-trim update per reference observation. Success carries
  the returned request and predictor history. Failure retains both; its proposed
  output is never adopted. Refine a success-to-failure bracket from its successful
  side to 0.001 km/h, carrying only successful updates.
* Append the exact queried speed, then apply one auto-trim update at the actual
  balanced-turn source. This last source includes actual turn-source propulsion.
  On failure it retains the canonical reference request/cache.
* Root-query endpoints never become predecessors of other queries. Only the
  fixed lattice supplies reusable predecessors. Propeller reference observations
  use an isolated engine ensemble and canonical initialization. The tested J6K1
  queries in forward and reverse order give identical reference dictionaries.
* Actual trim and primary actuators are **assumed settled at the resulting
  request**. This does not assert that one zero-time call settles an actuator.
  It explicitly excludes trim-lag assistance, including the inner predictor's
  retained actual-trim motion. Balanced physics alone does not require this
  assumption; it is part of this reference.
* Previous adjusted angles equal current adjusted angles. Thus no transient
  angle extrapolation is manufactured. Turn rates and native rate feedback are
  retained. Mode-1 histories start at zero; this port's mode-1 command iteration
  starts from zero regardless of the retained history fields.
* Authority adaptation is one. Overload memory is **retained at one**, evaluated
  at zero elapsed time through native `angle_targets`. This is a specified
  memory endpoint, not an all-memory fixed point or a claim of flight
  reachability. It preserves authored mappings such as J35's unreleased reserve.
* On the accepted pre-stall sources recovery's axis mix is zero; a settled
  recovery filter leaves protected delivery unchanged. The candidate guards the
  critical-angle condition rather than extending this argument post-stall.

The 25-km/h reference cadence is an explicit numerical reference convention,
not a recovered flight cadence, unique native history, or fitted coefficient.
It must not be promoted as a universally preferred controller initialization.
The successful mode-0 iteration can itself depend on the incoming cache.
No convergence of every controller memory is demanded.

## Minimal native-checked conflict with a pure upper AoA cap

F-16XL, sea level, 30% fuel, selected 110% throttle/afterburner, no flaps,
RB torque/gyro setting, and the static reference above:

| Load | Body AoA, approximately | Required delivered elevator | Native delivered interval | Permission |
|---|---:|---:|---|---|
| 1.340 g | 26.3998° | −0.04635 | [−0.36750, +0.42729] | permitted |
| 1.355 g | 26.7564° | −0.05972 | [−0.34736, −0.07471] | rejected |
| 1.380 g | 27.3560° | −0.10731 | [−0.31362, −0.07465] | permitted |

The rejected middle state is a physically balanced, pre-stall state, with normal
pitch response, no structural exclusion and nonzero turn rates. Its required
command exceeds the native upper endpoint by about **0.01499**; this is not a
near-zero classification ambiguity. Required command on this balanced turn
branch cannot simply be assumed monotone in body AoA.

`verify_native.py` executes the pinned original controller, native actuator and
native delivery kernels. It compares the original clamp and complete relevant
controller outputs to the port, and checks physical closure after permitted
control delivery. The three source force errors are below 2.6e-6 g; their
angular-rate closure errors are below 1.1e-5 rad/s² (ordinary acceptance remains
2e-4 g / 5e-5 rad/s²). Body pitch rates are approximately −0.1335, −0.1387 and
−0.1473 rad/s, respectively. The JSON retains exact states and tolerances.

Any rule consisting solely of `alpha <= alpha_cap(speed)` that admits the third
state must admit the rejected second state. Thus it cannot equal the native
pointwise restriction **in this reference**. This does not prove that a maximum
endpoint is undefined, or that every imaginable reference has this topology.
It proves that the selected reference's interior cannot be inferred solely from
its upper endpoint.

The smallest representational adjustment is to retain an upper endpoint for the
chart outline **and explicitly represent native-permitted/excluded interior
intervals**, using the same delivered-control predicate for both. It requires
searching above the first loss of permission. A continuous conditional crossing
alone does not establish that highest endpoint.

Additional high-angle F-16XL samples show native finite-iteration sensitivity to
small differences between balanced source representations. A first expectation
that a freshly re-trimmed 9.4-g source would reproduce a previously permitted
475-km/h sample failed; that failed assertion remains in `native-validation.log`.
It was an expectation about source-family classification, not a native/port
output mismatch. Do not use the earlier source as proof that every re-trim of
that load is permitted. `representations.json` preserves a small perturbation
check; it does not certify all possible neighboring states.

## J6K1 trim-failure regression

With fixed, intact 30% flaps, the ascending reference retains approximately
**−0.93741727** requested pitch trim after its last successful reference update
near 770.91224 km/h. Forward/reverse query order agrees.

At **771.03162178 km/h**, the candidate's local balanced upper crossing is about
**7.13893 g / 1.07316° body AoA**. The ±0.001-g neighbors have respectively
positive and negative native-composition permission margins. The previously high
11.34-g / approximately 7.144° point is rejected by about 0.308 delivered-command
units. At that high source, authority is approximately 0.4815, and the negative
retained trim makes the mechanical endpoint binding. Trim does not cancel.

Eight local roots from 770.8 through 780 km/h and their ±0.001-g neighbors close
the full aircraft equations. The reference at 770.8 has a materially different
successful trim output; no interpolation or smoothing is applied to that change.
The first failed turn-source query need not coincide exactly with the canonical
axial reference's failure event. Neither failed calculation's proposed output is
substituted. These are **local conditional roots**, not a proof of a global
maximum over live histories or over every separated balanced component.

## Overload evidence

Eight original-controller checks cover timers 0, 0.899, 0.901 and 1. At F-16XL
800 km/h / 13 g, the native upper angle target changes from 18.30372° at timer zero
to 28.12000° at one, with native predictor output changing as well. At J35
800 km/h / 10 g, the authored mapping keeps the upper target at 19.44581° for all
four tested timer values. Consequently disabling overload and selecting timer
one are not interchangeable, even in a static reference.

## Implementation and chart results

The withdrawn implementation replaced `instructor_aoa.controller_limits`, so the
ordinary production boundary and interior callers used the **same** native
mode-0/mode-1/clamp/delivery composition. It did not change aircraft forces,
moments, stall, structural limits, flap policy or SEP definitions.

A second implementation searched downward from an independently solved physical
upper boundary, instead of accepting the first Instructor rejection. Its body
angle scan used 0.25° probes and local physical brackets. That finite scan does
**not** certify absence of narrower permitted components between rejected probes.
Some physical probes also remained unresolved. The inherited `verified limit`
label certifies a local constraint crossing, not a certified global maximum;
this is one reason the candidate was withdrawn.

| Experiment | Full calculation / total time | Outstanding chart failures |
|---|---:|---|
| First direct-permission F-16XL | 13.00 / 15.79 s | 20 unresolved speed intervals; first-component selection can miss higher permission |
| First direct-permission J6K1 flaps | 26.99 / 28.34 s | no unresolved speed/load intervals or blank columns in this run |
| Upper-component search F-16XL | 80.98 / 92.63 s | 129 unresolved speed intervals; 349 reported interior gaps; incomplete maximum certification |
| Upper-component search J6K1 flaps | 36.77 / 38.13 s | no unresolved speed/load intervals or blank columns in this run |
| Attempt to find upper component before surface construction, F-16XL | 122.92 / 139.85 s | 392 unresolved speed intervals, 42 blank rendered columns, two detached contour ends |

Not every reported gap was independently shown to be a native exclusion.
Numerical failures and true native exclusions must not be relabeled collectively
as physical holes. The first experiment also ran F/A-18E, J35D, Yak-3, F-14B at
50% sweep, Spitfire full flaps and F-16XL Instructor-off. Complete per-case results
and unsuccessful experiments are retained in `summary.json` and the logs.

The last optimization is rejected, not described as a performance improvement.
No default/server replacement remains installed. `candidate/` contains the
withdrawn production-path implementation for review. `source-audit.json` verifies
restoration of all 381 baseline Python sources; only this report, test artifacts,
archived candidate and handoff remain newly saved by this investigation.

## What remains necessary

1. Select a numerically justified highest component across the entire physical
   balanced branch. The 0.25° scan is not sufficient proof; neither is continuation
   through the J6K1 trim event.
2. Represent and validate interior exclusions explicitly, distinguishing native
   rejection from physical-solver failure and interpolation uncertainty.
3. Resolve the maximum/source-representation sensitivity without smoothing,
   unsupported monotonicity assumptions or optimizing retained trim.
4. Bring that complete calculation back within acceptable chart runtime, then
   promote it and rerun boundary/interior/native validation. This work has not
   achieved those requirements.

The scalar-cap obstruction is concrete and native-checked. The remaining
implementation is a one-dimensional balanced-branch topology/accuracy problem
under declared controller-state assumptions, not a demand to enumerate all
possible flight histories.

## Reproduction

The following scripts explicitly opt into the archived candidate; they do not
install it in production:

```sh
.venv/bin/python analysis/instructor-static-reference-september24/verify_native.py
.venv/bin/python analysis/instructor-static-reference-september24/validate_reference.py
.venv/bin/python analysis/instructor-static-reference-september24/scan_topology.py
.venv/bin/python analysis/instructor-static-reference-september24/probe_representations.py
# Expensive and known not to satisfy completion criteria:
.venv/bin/python analysis/instructor-static-reference-september24/run_charts.py f16xl j6k_flaps
```

The archived chart runner uses spawn to make candidate-module selection explicit
after restoration; the reported in-production-path timings used the normal
forkserver worker setup. Do not compare startup overhead as equation performance.
The original executable used for new native checks has SHA-256
`820fee4a55601ffa459e2635da4ce1436cfc7adcc703a7c799380c95d136ebd5`.
