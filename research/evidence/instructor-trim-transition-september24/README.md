# Trim retention through the balanced-turn transition

The user correctly distinguished **a remaining boundary-projection gap** from
known implementation errors. This investigation traces the missing trim path
and establishes when its effect cancels, is nonbinding, or limits a balanced
turn. **It does not establish a global maximum over attainable retained states.**
Production, the server, application outputs and previous evidence are unchanged.
No fitting, blending, plots, flap-damage policy or new default history is added.

## Findings

1. Failed auto-trim preserves requested pitch trim and its cache. The pitch
   clamp uses that **requested** trim; the actuator uses **actual** trim.
2. Actual trim also survives the moving inner prediction's restores. Its slew
   advances during the internal search windows and analytical stopping steps,
   not just the subsequent physical actuator update. Treating that lag as only
   one real-time timestep per controller call misses a consequential path.
3. Trim cancellation is conditional. With matching requested/actual trim,
   symmetric authority, unit adaptation and no recovery, the inverse and forward
   maps cancel while the requested control remains inside mechanical authority.
   Once authority clips, retained trim changes the delivered elevator endpoint
   and therefore the limiting balanced AoA. Different requested/actual trim also
   prevents that cancellation.
4. J6K1 with 30% fixed, intact flaps is a binding counterexample. At
   771.031622 km/h, zero requested/actual trim permits the checked 7.148767°,
   11.342131-g balanced point. A native last-success request of −0.937431693,
   with matching actual trim, gives a local upper crossing at **1.072929°,
   7.138721 g**. Both use the same aircraft equations, power and configuration.
5. Retaining this last-success request yields a continuous local conditional
   branch through prediction failure. It removes the earlier switch to an
   independently supplied zero prior. This is **a conditional branch**, not
   proof that it is the unique or maximum boundary across possible histories.

## Original data flow

Addresses refer to the pinned executable SHA-256
`820fee4a55601ffa459e2635da4ce1436cfc7adcc703a7c799380c95d136ebd5`.

| Native location | Effect |
|---|---|
| `101a92c9b..92ce7` | Call one-g auto-trim; false return skips the pitch/roll setters and cache writes. |
| `101a94b5f` | Pitch-clamp inverse reads requested pitch trim at `FM+87f8`. |
| `101a4b499..4f0` | Slew actual pitch trim at `FM+a294` toward the request, using the native rate at `FM+8060`. |
| `101a4b4f8..632` | Clamp pilot command, map authority, then mix with actual trim. |
| Inner predictor `101a98590`, window restore `101a9b09a` | Search replays the actuator; actual trim persists across the restores. |
| Owner `104f708ee..709b3` | Preserve final trim cache, restore aircraft/record, publish cached trim into delivered record. Requested/actual trim and controller histories survive. |

The source listings are in `analysis/disassembly/0000000101a92670.asm`,
`analysis/disassembly/0000000101a4b230.asm` and
`analysis/instructor-inner-state-september24/field-disassembly/0000000104f6af60.asm`.
The prior owner/inner-state reports describe broader restore semantics.

## Where cancellation stops

Use raw pitch units before `InvertElevator`. For trim `t` and authority-mapped
stick value `v`, the native forward trim map, apart from float32 rounding, is

```
M(v,t) = t + (1 - abs(t) * sign(v*t)) * v.
```

For symmetric pitch authority `[-a,a]`, matching requested/actual trim `t`, unit
authority adaptation, and inactive recovery, let `p−,p+` be the two native
mode-1 predictor commands. The delivered pitch interval reduces to

```
lower = max(M(-a,t), min(p−,t))
upper = min(M(+a,t), max(p+,t)).
```

Thus even the reduced interval retains both the hardware endpoints and the
native inclusion of trim between the two predictor bounds. Simply intersecting
`[p−,p+]` with mechanical limits would omit those `min/max` operations.

When `t <= p+ <= M(a,t)`, the upper limit is `p+`: trim cancels. When
`M(a,t) < p+`, full stick cannot supply that prediction; the upper limit is
`M(a,t)`. With negative trim, positive authority and positive stick,
`M(a,t) = a + (1-a)*t`, so decreasing retained trim reduces the endpoint whenever
`a < 1`. If another physical restriction is reached before this endpoint, the
trim restriction is nonbinding for that state.

The native comparisons with matching trim verify this reduction within
1.79e−7 delivered-command units. F-14B's failed-auto-trim source at
195.91328 km/h has full mechanical pitch authority: trims −0.9, −0.3, 0 and
+0.3 all give upper protected pitch about 0.774571, above the required
0.707215. J6K1's reduced authority supplies the opposite, binding example.

For unequal requested and actual trim, unequal signed authority, nonunit
adaptation or recovery, keep the original composition. Compute the clamp using
the requested trim and retained controller state, then map its permitted
commands through authority and the actual actuator trim. Do not apply the
symmetric cancellation outside its assumptions.

## Actual trim inside native prediction

`trace_inner_trim.py` executes the original moving inner predictor with the
correct nonzero FM timestep, fixed requested flaps and an empty free-air scene.
It independently checks every trim slew update, verifies that actual trim does
not roll back between windows, compares the other controller outputs to the
port, and executes the original final owner restore.

Twelve source/cadence/initial-trim cases pass **1,363 internal actuator trim
updates**. At the failed high J6K1 source, a 1/48-second controller call starting
with actual trim zero and retained request −0.937431693 ends at actual trim
−0.270833433. Its eight search windows execute 104 actuator calls, totaling
2.166667 seconds of internal actuator time. This is simulated prediction time,
not an assertion that 2.17 seconds elapsed in flight.

A separate seven-call prescribed sequence independently carries requested trim,
actual trim, response parameters and all checked controller histories. It passes
**694 internal actuator trim updates**:

| Call | Auto-trim | Requested pitch trim afterward | Actual pitch trim afterward |
|---:|---|---:|---:|
| 0, successful source | success | −0.933341324 | −0.312499940 |
| 1, failed source | failure | −0.933341324 | −0.348958135 |
| 2, failed source | failure | −0.933341324 | −0.585937381 |
| 3, failed source | failure | −0.933341324 | −0.929690003 |
| 4, failed source | failure | −0.933341324 | −0.933341324 |
| 5, failed source | failure | −0.933341324 | −0.933341324 |
| 6, successful source | success | −0.930118024 | −0.930118024 |

At the high source, positive raw authority is 0.481528908 and the balanced point
requires elevator 0.303758353. The actual-trim threshold for mechanical support
is approximately **−0.342874574**. The first failed call above already leaves
actual trim below that threshold: its maximum mechanical elevator is about
0.3006042. Thus that particular carried state cannot support the old high point,
even before considering additional protection.

This sequence uses prescribed physical observations; the jump between its low
and high balanced sources is not a flown trajectory. The timing and thresholds
are diagnostic results, not a fitted flight time or a global history policy.
Native body prediction executes with prepared held propulsion inputs; a complete
independent propeller-owner trajectory comparison is not claimed.

## Local balance does not require all memories to settle

`verify_permission.py` checks actual control delivery and re-evaluates the
aircraft equations using the same held propulsion sample. At the same high
J6K1 physical point and retained request −0.937431693:

| Actual trim before next ordinary actuator update | Maximum delivered target afterward | Required balanced elevator still delivered? |
|---:|---:|---|
| 0 | 0.480179 | yes |
| −0.30 | 0.324637 | yes |
| −0.34 | 0.303899 | yes |
| −0.35 | 0.298714 | no |
| −0.937432 | −0.004502 | no |

The permitted rows satisfy the existing force/moment tolerances after the
original controller/actuator/delivery calls. Trim is moving in the first rows;
an appropriate permitted pilot command compensates for that movement for the
checked update. These are local permission witnesses, not proof that a complete
prediction/flight sequence can maintain them. In particular, the moving inner
trace above shows why external-time slew alone overstates the available lag.

The **44 native local checks** also include the mechanically rejected actual
trims from the carried inner sequence, all eight conditional boundary roots and
their neighboring loads, and the F-14B cancellation examples. Native output
parity passes even where delivery is correctly rejected.

## Conditional continuation without a fallback jump

`verify_transition.py` first checks a small continuous sequence of independently
balanced 7-g observations, carrying trim and controller state through success,
failure and return. Both dispatch branches pass 37 calls each. Each call checks
the original controller, final owner restore, ordinary actuator and delivery.
This fixture skips moving inner prediction to isolate those stages; the moving
inner tests above are separate.

Its last successful request at 770.9 km/h is −0.937431693. `boundaries.py` keeps
that exact retained request on failure and solves for a local upper balanced
crossing, with matching actual trim as an explicit conditional assumption:

| Speed, km/h | Auto-trim | Body AoA, degrees | Load, g |
|---:|---|---:|---:|
| 770.900000 | success | 1.078491 | 7.142021 |
| 770.920000 | failure | 1.077624 | 7.141500 |
| 770.960000 | failure | 1.075975 | 7.140534 |
| 771.031622 | failure | 1.072929 | 7.138721 |
| 772 | failure | 1.032258 | 7.114698 |
| 775 | failure | 0.907412 | 7.041468 |
| 780 | failure | 0.703363 | 6.923717 |

All eight solved roots and their ±0.001-g neighbors have valid physical sources.
Permission is positive below and negative above each root. No interpolation,
blending, change to equations, or failed-auto-trim success gate is used.
Root queries receive immutable histories; they do not advance the controller.

`isolated-prior-boundaries.json` preserves the separate −0.933341324 experiment
seeded from the earlier 770.865607-km/h source. It gives 1.108899° and 7.171434 g
at 771.031622 km/h. This difference is retained-state sensitivity, not a choice
of which numerical answer looks smoother. A first conditional family that
reuses an older prior after a later success is not a carried traversal; the
event-seeded table above avoids that mistake locally.

The wider branch scan contains 50 physical solves / 100 explicit trim-context
checks. Every source meets existing physical tolerances. It verifies that the
observed difference is not caused by the archived invalid 779.996418-km/h point;
new solves are used here.

## Implementation consequence and remaining gap

The trim data flow is now concrete: success updates request/cache; failure
preserves them; moving prediction and ordinary actuators advance actual trim;
restoration retains it; protection and delivery consume different trim fields.
A correct retained-state evaluator can constrain the balanced aircraft using
the resulting delivered-control interval, without optimizing trim, resetting
failed requests or demanding a fixed point for every memory.

**This still does not select the state set over which the maximum capability
boundary should be taken.** The user's balanced-turn definition is already
accepted and is not reopened. Local balance, one successful predecessor, or a
continuous conditional branch cannot prove that all maximizing retained states
are attainable, nor that none permits a higher balanced turn. A speed-only
replacement still needs that projection established. The findings rule out
universal trim cancellation and show that the gap is not merely implementation.

The next evidence requirement is a state-consistent maximum/reachability
argument that includes the retained actual-trim updates inside prediction.
Neither the older fully stationary architecture nor the new local conditional
branch should be silently promoted as that answer. Full-chart reruns would not
resolve this missing argument, so no production replacement or chart run is
performed here.

## Reproduction

From the workspace root, using `.venv/bin/python`:

```sh
.venv/bin/python analysis/instructor-trim-transition-september24/probe.py
.venv/bin/python analysis/instructor-trim-transition-september24/verify_transition.py
.venv/bin/python analysis/instructor-trim-transition-september24/boundaries.py
.venv/bin/python analysis/instructor-trim-transition-september24/trace_inner_trim.py
.venv/bin/python analysis/instructor-trim-transition-september24/verify_permission.py
# Optional reproduction of the separately preserved older-prior family:
.venv/bin/python analysis/instructor-trim-transition-september24/boundaries.py --isolated
```

`source-audit.json` verifies production Python source preservation.
`summary.json` collects the separate test counts; these are not to be added as
a count of complete game frames. `research-checkpoint.tar.gz` preserves this
investigation separately from preceding checkpoints.
