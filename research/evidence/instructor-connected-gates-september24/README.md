# Connected-turn gates: repeatability passes; 1 g starting point does not

September 24, 2026. The user requested two gates, followed by representative
runtime benchmarks **if successful**. Both gates were exercised. The numerical
repeatability gate passes at the two difficult F-16XL speeds tested. The J6K1
starting-equilibrium gate does not pass. Consequently no fleet runtime benchmark
was run, no acceptable full-curve runtime is claimed, and production is unchanged.
No plots, server restarts or changes to existing application work were made.

## Gate 1: smaller steps and independent starting guesses

Acceptance was declared before the final trials: all runs must resolve their
endpoint, with an AoA spread no greater than 0.02 degrees and a load spread no
greater than 0.002 g. Original physical acceptance remains 2e-4 g force error and
5e-5 rad/s² angular error; every source requests the same tighter physical
correction before its controller permission is known.

Each speed receives nine cold solver runs: three initial AoA/elevator guesses
times continuation resolutions 1/32, 1/64 and 1/128. These correspond to maximum
accepted AoA increments of 0.00625, 0.003125 and 0.0015625 degrees. The full
balanced equations correct every point. Failed controller permission is not
used to search for a different favorable rounded equilibrium.

| F-16XL speed | Endpoint AoA range | Load spread | AoA spread | Result |
|---|---|---:|---:|---|
| 157.91807776 km/h | 25.67617–25.68532° | 0.0004022 g | 0.009149° | Pass |
| 158.34307140 km/h | 25.66950–25.67822° | 0.0003846 g | 0.008722° | Pass |

All 18 final runs start from valid 1 g equilibria and close a local balanced
accept/reject bracket. Native replay of both bracket endpoints from every
finest-resolution starting-guess run gives 12 matching controller, actuator and
delivery comparisons. Accepted native deliveries close the aircraft equations.

This passes the specified **numerical repeatability** test. It is not an
analytic proof that no narrower earlier excluded interval exists between
samples. Nor does it certify the entire speed range or fleet.

### Numerical defects corrected in the trial

The old tracer treated an absolutely small continuation step as a possible
failed correction even when the corrected point was valid. At fine resolutions
this could stop an ordinary accepted interior sequence. The local tracer now
scales that guard with requested resolution and applies it only to a failed
corrector. Its iteration guard is raised from 4,000 to 16,000 so the finest
test can actually reach the candidate boundary. Physical tolerances are unchanged.

The initial gate harness called the raw root solver without the existing
`solve_level` retry. Its high-angle cold starts fell into an unbalanced root
basin. The final harness uses that established retry for each supplied guess.
Default and high-angle guesses recover the same half-angle starting root;
the low-angle/elevator guess converges separately. This tests different
initializations, not three different physical level-flight branches.

Initial failures are preserved in `initial-attempt/` and
`without-level-retry/`. The corrected tracer remains local to this experiment.
It has not been installed in production or integrated into surface sampling.

## Gate 2: J6K1 starting equilibrium

At 770.9 and 771.03162178 km/h with fixed 30% flaps, five initial AoA/elevator
guesses at zero sideslip and six additional sideslip guesses (-2, -1, -0.5,
+0.5, +1, +2 degrees) find **no physically valid 1 g equilibrium**. These are
failed physical closures, not evidence of an Instructor permission failure.
The zero-sideslip force residuals remain roughly 0.00626 and 0.01030 g,
respectively, far beyond the 0.0002 g acceptance tolerance.

The direct lower-stall solve at 771.03162178 km/h is more informative than those
failed starts. Full force/moment balance plus a small positive stall margin gives:

| Stall margin | Load | Body AoA |
|---:|---:|---:|
| 0.002001° | 1.0118203 g | −11.64424° |
| 0.000499° | 1.0118051 g | −11.64570° |
| 0.000101° | 1.0118010 g | −11.64609° |

Force errors are below 2.1e-7 g and angular errors below 5.7e-7 rad/s². These
states approach a physical lower edge **above 1 g** on the traced branch.
All three are also slightly rejected by the static Instructor reference;
original-controller comparisons confirm margins approximately −0.000515,
−0.000630 and −0.000661. Thus even the physical lower edge must not simply be
declared a permitted starting point. Prior tested gentle turns at 1.02 g are
permitted, but connection from a permitted 1 g start remains unestablished.

At 770.9 km/h, the first direct solve closes near a 0.01-degree stall inset and
1.00754 g. Attempts at smaller insets do not reliably satisfy the active stall
equation. The files explicitly record that failure; balanced roots whose stall
margin misses its target are not accepted as lower-edge certificates.

An additional diagnostic solves pitch moment at zero bank, sideslip and angular
rates while varying AoA, using canonically initialized propulsion. Near the
negative critical angle, all sampled pre-stall pitch-balanced states still
produce excess vertical force (about 0.00636 g at 770.9 km/h and 0.01039 g at
771.03 km/h for body AoA −11.7 degrees). This supports the lower-edge explanation
on the symmetric sheet. It is a sampled diagnostic, **not a global proof of
nonexistence of every possible 1 g root**.

The strict starting-equilibrium gate therefore fails. The earlier high-load
J6K1 crossing at approximately 7.139 g cannot be certified as connected to a
permitted 1 g state by this work. Starting from the lowest permitted gentle turn
when 1 g is unavailable is a possible explicit adjustment, but it has not been
silently substituted or validated here.

## Native evidence and runtime scope

All **15** new original-controller/actuator/delivery comparisons match: twelve
F-16XL endpoint checks and three J6K1 lower-edge checks. Binary SHA-256 is
`820fee4a55601ffa459e2635da4ce1436cfc7adcc703a7c799380c95d136ebd5`.
This controller evidence does not by itself prove the absent J6K1 level root.

Measured individual F-16XL columns take roughly **4.3–14.4 seconds**, excluding
interpreter startup, at these validation resolutions. The finest runs contain
over 5,400 continuation points each. These figures are diagnostic costs, not
acceptable production chart benchmarks. Full cold curves, representative
aircraft/configurations, interpolation and end-to-end runtime remain untested
for this candidate because gate 2 did not pass.

`summary.json` contains machine-readable gate decisions. Reproduction:

```sh
.venv/bin/python analysis/instructor-connected-gates-september24/gate_repeatability.py
# The neighboring speed uses the same run(speed, start, resolution) helper;
# its nine rows are retained separately in neighbor-repeatability.json.
.venv/bin/python analysis/instructor-connected-gates-september24/gate_level.py
.venv/bin/python analysis/instructor-connected-gates-september24/resolve_lower_edge.py
.venv/bin/python analysis/instructor-connected-gates-september24/level_force_curve.py
.venv/bin/python analysis/instructor-connected-gates-september24/native_gate.py
.venv/bin/python analysis/instructor-connected-gates-september24/summarize.py
```

`native_gate.py` needs both saved repeatability sets. All artifacts are isolated
under this directory and use the previously documented static controller
reference. The overall production implementation remains incomplete.
