# Development handoff — 2026-09-24

## Windows continuation

Newest: [Windows moving-controller checks](research/WINDOWS_MOVING_CONTROLLER.md).
The 36 user-approved unsupported fixed-wing entries have been removed through
an explicit persistent catalog exclusion list; all 1,312 supported entries remain.
Windows controller execution now includes moving prediction, native actuator
trim, authored flap lookup tables and explicit scratch allocation. The Windows
wing-polar high-side parabola uses a different float32 expression grouping from
the Mac constructor; `windows_instructor_source.py` supplies that reviewed order
to Windows research drivers. Read the validation scope before promoting results.
The maximum boundary remains unresolved; fixed configuration does not grant
arbitrary entry trim, stationary histories or a connected one-g starting state.

Latest: [fixed-configuration Windows continuation](research/WINDOWS_BALANCED_TURN_CONTINUATION.md)
records data 2.59.0.34, the complete Windows reduced predictor comparison,
regenerated SB-25J assets and conditional full-pitch roots. The user requires
the prescribed configuration/control mode to remain fixed. A global maximum
over attainable internal controller states is still unresolved.

The user has now resumed Instructor research and selected **maximum balanced
turn with full pitch input**. Read
[research/WINDOWS_INSTRUCTOR_PROGRESS.md](research/WINDOWS_INSTRUCTOR_PROGRESS.md)
for the Windows setup, repaired legacy wake branches, installed-executable
evidence and remaining accuracy gates. The earlier pause below is historical;
the experimental designation and withdrawn-method cautions still apply.

## User decision and current scope

The user withdrew the latest Instructor AoA-versus-speed research approach and
requested removal of proven-flawed alternatives, cleanup and a Windows handoff.
They explicitly chose: **Keep Instructor limits available but clearly experimental.**
Do not resume the abandoned boundary search automatically or claim that the
underlying AoA-versus-speed problem has been solved.

The previous original task and its long chronological handoff are superseded
by this cleanup decision. Original files and all bulk experiment results were
preserved outside the working directory in the sibling folder:
`WT FM Reconstruction Archive 2026-09-24`.

## What remains active

- Full aircraft force/moment equations, native low-level controller ports,
  aircraft catalogs, propeller physics, reference executable and flight logs.
- Existing web interface changes including RB/SB conditions, entries and the
  altitude application. These were user work and were preserved.
- `instructor_aoa.py` and `instructor_aoa_balance.py`: the pre-existing static
  approximation, now explicitly experimental in UI, profile and exports.
- `instructor_chart_inputs.py`: source-field preparation extracted unchanged
  from the retired held-pitch module. It does not implement that old solver.
- Negative-AoA stall does not mask chart points, for both RB and SB. Native
  force polars are unchanged; positive stall and other physical limits remain.

## Unresolved Instructor issue

Free trim allocation and omitted overload memory are assumptions of the active
approximation. Native auto-trim solves a simplified level condition, not the
high-g candidate. On failure it retains the previous request. Pitch protection
uses requested trim, while delivered controls depend on actual trim. Actual
trim can advance inside prediction and survive restores. Trim cancellation is
conditional. Matching individual native controller calls is not validation of
a chart-wide boundary or all interior points.

The J6K1 regression is fixed 30% flaps near 771 km/h, with other configuration
fields taken from the selected evidence reports. Ignoring negative stall did
not cure its physical one-g force/moment closure failure. Do not reinterpret an
unbalanced numerical iterate as a valid lower starting state.

Static-reference permission can have disconnected permitted regions. A single
upper cap cannot represent all points in such a reference. The connected-turn
replacement also failed the general initialization gate; a broad representative
runtime benchmark was therefore not justified. See the retired-approaches note.

## Cleanup and portability changes

The held-pitch, stationary-history, historical upper-boundary and prolonged
boundary implementations and dependent probes were removed from `scripts/`.
Recent static-reference/connected-turn code was in archived analysis folders
and is not included in the transfer. The list is in `research/cleanup.json`.
Full pre-cleanup scripts/app and the existing app Git diff are in the archive.

Generated analysis/output bulk and downloaded Mac tools were moved out. Compact
native fixtures and selected reports remain. Some historical research utilities
need archived datasets or an installed game's resource files; they are not
part of the installation smoke check. Do not treat every old utility as a
self-contained supported command. Original flight logs and references remain.

`references/prop-native-config.json` now holds the runtime census previously
mixed with generated analysis. Callers and Docker inputs were updated. Cache
identity includes this census and gameplay inputs. The native test default now
uses the bundled pinned executable instead of `/Applications/...`.

Windows additions: `.cmd` setup/launchers, platform-aware backend signatures,
portable build locking, MSVC floating-point flags, and direct altitude launch
without symlinks. Keep UTF-8 enabled when running tools on Windows.

## Checks and next work

Read `research/CLEANUP_VALIDATION.md` for the final observed checks. Run
`Setup Windows.cmd` on the destination machine and address any platform-specific
build or numeric mismatch before trusting Windows output. The supplied smoke
check is bounded and does not certify a global envelope or production readiness.

No production deployment, upload, push or Git commit was performed. The Mac Git
repository is website-only; the ZIP is the complete working-source transfer.
Choose the next research problem with the user before replacing the experimental
Instructor approximation. Preserve force/moment closure, fixed intact flaps,
negative-SEP permission, consistent boundary/interior treatment and the explicit
negative-stall exclusion policy.

## Follow-up application check

Both apps passed real Chrome calculation, rendering, inspection, exports and
cancel/recalculate checks. An altitude reload bug (110% throttle restoring above
the HTML maximum due to floating-point roundoff) was fixed without changing the
physics throttle. See research/APP_FUNCTIONAL_VALIDATION.md. Restart the local
server after editing equation sources; an older running EM server was restarted
during this verification. Windows/MSVC was untested at transfer time; the
subsequent Windows checks are recorded in the continuation linked above.
