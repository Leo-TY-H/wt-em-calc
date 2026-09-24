# Negative-AoA stall no longer excludes EM chart points

Implemented September 24, 2026, following the user's instruction to ignore
negative-AoA stall generally. This is now the production policy for all aircraft
and both Instructor settings, including boundary and interior calculations.

`em_operating.stall_margin` is the positive critical wing angle minus the maximum
wing angle over all retained aircraft phases. All existing physical rejection,
boundary solving and sampling paths consume that same margin. The lower critical
angle is retained only as the diagnostic `negative_stall_margin`, exported on
points as `negative_stall_margin_deg`.

The negative critical angle no longer clips the initial numerical guess, and
the obsolete lower/upper stall-side selection in `em_sampling` has been removed.
Positive stall, control authority, normal pitch response, structural limits,
Instructor permission and force/moment convergence requirements remain active.

Native polars, negative-angle force/moment calculations, aerodynamic history,
spin and control calculations are unchanged. Ignoring negative stall as a chart
exclusion does not extend the linear lift curve or manufacture equilibria.
The method text and exported assumptions state this policy. Existing unrelated
application changes are preserved.

## Validation

- `scripts/verify_em_stall_side.py`: positive stall retained and negative stall
  ignored in RB and SB, including checks that failed force balance stays invalid.
- `scripts/verify_em_negative_alpha.py`: all 16 existing negative-AoA equilibria
  pass, along with its propulsion averaging checks.
- `verify_change.py`: three explicit source comparisons against the saved
  previous operating-point function preserve force, moment, residual, history and
  SEP outputs exactly. F-16XL's positive-stall margin stays negative. Negative-
  critical-angle J6K1 and full-flap Spitfire states gain positive active stall
  margins while retaining their negative diagnostic margins.
- Three ordinary production speed columns complete with verified local limits:
  F-16XL at 475 km/h Instructor off; J6K1 with 30% flaps at 771.03162 km/h with
  Instructor off and on. Every `post-stall` classification in those columns
  agrees with the positive stall margin. These are integration checks of the
  current production model, not validation of the unfinished connected Instructor
  replacement or a fleet runtime benchmark.
- The compiled backend rebuilt successfully. The idle local plotter on port
  8765 was restarted after validation; health and served policy text pass. Its
  new runtime fingerprint does not reuse the previous cached chart as latest.

## J6K1 starting-point result

At 770.9 and 771.03162178 km/h with fixed 30% flaps, the production 1 g solve
still fails force balance, both with Instructor off and on. Its sole rejection
reason is now `trim did not converge`, with force errors approximately 0.006259
and 0.010303 g, compared with the required 0.0002 g tolerance. It is no longer
rejected for negative stall. The starting-equilibrium gate therefore remains
unpassed, and the conditional representative fleet benchmark is not triggered.

The requested negative-stall policy change is complete. The broader static
connected-boundary implementation remains unfinished; the previously archived
native-permission candidate has not been installed by this change.

`baseline/` saves the preceding sources; `source/` saves the changed production
sources. `source-audit.json` records their scope. Earlier lower-stall research
used the old two-sided margin; its active-edge scripts must not be interpreted
as lower-stall solvers if rerun against the new positive-only `stall_margin`.

No plots were generated. Numerical evidence is in `validation.json`,
`columns.json`, their logs and `server-validation.json`.
