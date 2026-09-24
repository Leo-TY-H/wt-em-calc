# War Thunder flight-model reconstruction

Start here after transferring the project. The working calculator and the
altitude application are retained. The Instructor option is **experimental**:
its static AoA approximation is not a validated reconstruction of native
balanced-turn permission.

The Windows continuation repaired legacy Instructor wake branches affecting
11 catalog records and verified the compiled runtime on this device. See
[Windows Instructor progress](research/WINDOWS_INSTRUCTOR_PROGRESS.md) for
test evidence and the unresolved full-pitch boundary work.

The [latest continuation](research/WINDOWS_BALANCED_TURN_CONTINUATION.md)
adds current-data Windows predictor comparisons and conditional full-pitch
balance checks. A validated maximum boundary is still under investigation.

- **Windows:** follow [WINDOWS.md](WINDOWS.md), then run `Launch EM Plotter.cmd`
  or `Launch Altitude Plotter.cmd`.
- **macOS:** the existing `.command` launchers remain available. For a fresh
  environment, create `.venv`, install `requirements-plotter.txt`, and run
  `python scripts/build_em_backend.py` using that environment.
- **Continue the research:** read [HANDOFF.md](HANDOFF.md) before changing the
  Instructor model. [research/RETIRED_APPROACHES.md](research/RETIRED_APPROACHES.md)
  records approaches withdrawn from the active project and the reasons.
- **Check an installation:** run `python scripts/verify_portable_runtime.py`
  inside the environment. It uses spawned workers, both aircraft catalogs,
  compiled equations, an original-code aerodynamic comparison, and stall-policy checks.

## Project contents

`scripts/` contains the calculator, recovered native calculations and research
utilities. `app/` contains both interfaces. `references/` contains aircraft data,
prepared physics inputs, and the pinned executable used as a read-only test
oracle. `FlightTestData/` contains the original recorded flight data.
`analysis/` retains compact native fixtures and historical reports; `research/`
contains the cleanup record and selected Instructor evidence. Historical reports
are evidence with their original scope, not current validation claims.

`outputs/`, `.native_em/`, and `.venv/` are generated locally. Rebuild them on the
other device. The portable ZIP excludes these, `.git`, tool installations and
retired experiments. Preserve the included reference data when extracting it.

The GitHub repository now tracks the application, Python source and runtime
aircraft data. A clone can run the calculator after installing dependencies.
Executable oracles, raw collision packages, analysis and flight logs remain
local research assets; they are not needed for normal calculator use.

## Automatic data updates

Both launchers check the latest datamine before starting, at most once per six
hours. GitHub Actions refreshes the same inputs every six hours, even when your
computer is off. The source version is in `references/data-version.json`.

- **Update Game Data.cmd** checks immediately and rebuilds the website catalog.
- **Push to GitHub.cmd** updates data, commits local project changes, reconciles
  remote commits, and pushes to GitHub.
- Command-line equivalent: `python scripts/push_to_github.py`.

Close both plotters before refreshing data or pulling changes; running apps
cache their models. See [DATA_SYNC.md](DATA_SYNC.md) for scope and asset limits,
and [DEPLOYING.md](DEPLOYING.md) to enable website publication when ready.

## Current interpretation

The calculator balances aircraft forces and moments in a turn; negative SEP is
allowed. Selected flaps are fixed and intact. Positive-AoA stall, structural and
control limits remain active. Negative-AoA stall is a diagnostic only and does
not exclude chart points; the force polars are unchanged.

RB pairs the experimental Instructor approximation with propeller torque/gyro
off; SB pairs Instructor off with torque/gyro on. The same experimental AoA
constraint is applied at the boundary and in the interior. No replacement
connected-turn or static-reference permission search has been promoted.

Propeller EM and the altitude tools retain their existing limitations. See
[ALTITUDE_ENVELOPE.md](ALTITUDE_ENVELOPE.md) for altitude methodology; historical
performance numbers in older documents refer to the original Mac, not Windows.
Deployment documents are retained for reference; this cleanup does not deploy
or change a hosted service.
