# Live missile implementation report

The interactive game-target simulator is implemented in `WT-FM-LIVE` on branch
`live-missile`, based on `origin/main` 11ddd6e5. Keyboard/touch controls, unrestricted
attitude steering, eight concurrent browser sessions, cameras, pause/slow motion,
canonical replay, playback and export share the existing missile engine.

Verification began after the initial implementation pass, as requested. All functional checks below passed, including the final 720-case canonical
Python reference sweep.
Performance has one reported limitation: eight missiles under Chromium 4x CPU
throttling exceed the planned 4 ms p99 engine-time target. No Worker was added.
No release stamp, tag, merge, backend deployment or Pages deployment was performed.

Preview: http://127.0.0.1:8799/ (server from this worktree).
Evidence and desktop/phone screenshots: `outputs/live-missile/`.

## Acceptance results

| Plan item | Result and retained evidence |
|---|---|
| 1 Existing results | PASS: 681/681 profile/geometry evaluations, 24/24 simulator/escape/early-exit variants, and another 720/720 corpus setups equal the retained release. Standard envelope ray sets, plans and finals also match in all 20 alternating runs. `baseline.json`, `zero-input.json`, `envelope-performance.json`. |
| 2 Zero input | PASS: 720/720 release documents equal zero-input steering after removing only the prescribed fields. Every removed target quaternion equals the setup quaternion. `zero-input.json`. |
| 3 Reference / translated / typed | PASS: full reference/live sweep 720/720, with 744 typed compare reports and no declines. The strict canonical JSON sweep also passed 720/720 with 720 successful typed comparisons. `corpus.json`, `canonical-reference.json`. |
| 4 Native live / replay | PASS: 720/720 on Windows and 720/720 on Linux; 108,678 steps per corpus, including stopped runs. `wasm-corpus.json`, `linux-corpus.json`. |
| 5 No look-ahead | PASS: Python rejects queries after the final input; the production typed lookup rejects four ahead queries and permits two legal ones. Corpus runs have no look-ahead errors. `test_missile_live.py`, `test_missile_steered.cpp`. |
| 6 WASM / native | PASS on the specified outcome gate: 720/720 outcomes and initial target quaternions match on each OS; maximum retained missile-position difference is 0 m. Full documents: 681/720 identical on Windows, 603/720 on Linux. Differences are detailed below. |
| 7 Performance | REPORTED LIMIT: Chromium p99 0.8 ms normally, 5.8 ms at 4x throttle (target 4 ms). Firefox completed 14,400 steps for each of eight missiles. Envelope median differences are unresolved within run-to-run spread. `performance.json`, `envelope-performance.json`. |
| 8 Page logic | PASS: 7 grouped core checks, including ten simulated minutes at all three speed scales; 17 playback checks. Browser event checks cover pause/key clearing, touch release, restart, setup errors and separate instances/routing for two fixture build labels. `events.json`. |
| 9 Real browser | PASS: WebGL available; 50 runs, zero handles after each, zero allocated-heap growth from run 5 to 50, one live renderer, no page errors, 84 chase-camera attitude checks. Two exports replay with identical trajectories and outcomes. Refreshed desktop/390px phone screenshots inspected. `browser.json`. |
| 10 Existing regressions | PASS: envelope search 37 tests, launcher aircraft 13, profile loading 6; UI envelope/launcher checks, models 16 checks (112 missiles/79 meshes), playback 17, plots 10. All 227 generated profiles unchanged. |
| 11 Steering geometry | PASS: 7 grouped Python tests; all 12,498 steps in the 60 corpus logs plus 14,400 random steps checked for frame orthogonality and segment continuity. Loop diameter/return, later-loop f32-time tolerance, inverted right turn, roll, vertical setup, 2,000 quaternion frames and 10,001 polynomial samples pass. |

HTTP checks also passed: immutable hash inputs, two malformed requests rejected,
and a 201,875-byte replay request accepted and stopped at the requested step.

## Numerical differences

The WASM corpus has 39 non-identical documents against Windows and 117 against Linux.
All target positions, target attitudes, missile positions and outcome codes are equal.
Only derived telemetry angles and one closest-distance scalar differ:

- Maximum angle-of-attack difference: 1.4210854715202004e-14 degrees.
- Maximum thrust-vector telemetry difference: 1.7763568394002505e-15 degrees on
  Windows; 2.220446049250313e-16 degrees on Linux.
- Maximum closest-distance difference: 1.8189894035458565e-12 m (one case per OS).

`wasm-differences.json` retains field counts, maxima and ten examples per OS.
These are consistent with platform math-library rounding in derived scalar values;
there is no measured flight-position or target-attitude divergence in this corpus.

## Browser size and timing

Emscripten 4.0.10, assertion/stack-check build; Playwright 1.51.1, Chromium 134 and
Firefox 135. Chromium uses headless WebGL with SwiftShader. Only one timing job ran
at a time; this worktree's reference sweep was suspended during timing and resumed.

| Asset | Raw bytes | Brotli bytes |
|---|---:|---:|
| missile-engine.wasm | 22,298,691 | 2,347,165 |
| missile-engine.mjs | 94,344 | 24,138 |

The WASM is below the 25 MiB file-size gate.

| Eight-session Chromium test | Frames | Engine steps | Median / frame | p99 / frame | Total engine time / step |
|---|---:|---:|---:|---:|---:|
| Normal CPU | 1,001 | 6,488 | 0.2 ms | 0.8 ms | 0.0329 ms |
| 4x CPU throttle | 1,001 | 8,928 | 1.2 ms | 5.8 ms | 0.1724 ms |

Local WASM download: 64.1 ms normally / 127.5 ms throttled. Combined download and
streaming compilation: 71.0 / 169.4 ms. Compilation overlaps download, so an isolated
compiler duration is not inferred. Firefox totals were 914-1,252 ms per 14,400 steps
(0.0635-0.0869 ms/step), restarting sessions after natural termination as recorded.

The initial bundle measurement was 0.7 / 6.9 ms p99; the final retest was
0.8 / 5.8 ms. Both throttled measurements exceed the target. The initial scalar
measurements remain in `initial-timing.json`.

Standard envelope, four workers, five alternating repetitions per build:

| Missile | Release median | Candidate median | Release / candidate spread | Conclusion |
|---|---:|---:|---:|---|
| AIM-9L | 0.43057 s | 0.42187 s | 0.06004 / 0.05130 s | Unresolved within spread |
| AIM-120C-5 | 2.38329 s | 2.38810 s | 0.23996 / 0.16591 s | Unresolved within spread |

## Reproduction and provenance

Native signature: `34c53d4d5f4b8939759dddc08d531906814c55f414954b4d420defd6cc855f07`.
WASM fingerprint: `69526f9dc4811ad8a41ec6dacb3d54a120e4c96b3bf08d293b23115aafa791db`.
WASM SHA-256: `47d993c614deb6dc1a2de56cf3ad64a953a4869cf3fabe59035130e27a4f2200`.
Windows executable SHA-256: `da389c0a2c85d2b456f64bbcf89dd29f1a6c60817a0c6ea7770916aac7554d51`.
Linux executable SHA-256: `f2e30a4823cef661ac759a44709604994aaa628ffcca4ff9f16a64632cb648bd`.
Baseline executable SHA-256: `47782fb6e5c35179407fc4557343857ffafbb8ed5b7f10a718f6f2800a9fe77b`.

The retained release baseline is `../WT-FM-app` at 11ddd6e5. Baseline execution was
deferred until final verification, using that unchanged executable. Sources and the
seeded input generator are versioned; input hashes and retained paths are recorded
in `outputs/live-missile/manifest.json`. No repository or checkpoint copy was made.

Key commands (run from this worktree, using the shared Python environment):

```text
python scripts/build_missile_engine.py
python scripts/build_missile_wasm.py --assertions
python scripts/verify_missile_live_baseline.py
python scripts/verify_missile_live_zero.py
python scripts/verify_missile_live.py --reference
python scripts/verify_missile_live_reference.py
python scripts/verify_missile_live.py --wasm --report wasm-corpus.json
python scripts/verify_missile_live.py --wasm --linux-native --report linux-corpus.json
python scripts/inspect_missile_wasm_differences.py
python scripts/test_missile_live.py
node scripts/test_missile_live_ui.cjs
node scripts/test_missile_live_browser.cjs
node scripts/test_missile_live_events.cjs
python scripts/run_live_performance.py
```

`build_missile_linux.py` runs under WSL against the existing generated sources.
The typed look-ahead unit compiles with `g++ -std=c++17 -O2 -ffp-contract=off
-I scripts/missile_native -I .native_missile_engine/generated
scripts/test_missile_steered.cpp -o .linux_missile_engine/test-steered`.
Browser scripts use `../tools/live-browser/node_modules/playwright` and
`PLAYWRIGHT_BROWSERS_PATH=../tools/live-browser/browsers` (absolute path on Windows).
The timing supervisor uses the existing default Python's psutil installation.

## Integration and remaining decisions

- The 4x-throttle p99 exceeds the planned threshold. Leo can decide whether a Worker
  follow-up is warranted; the current implementation remains on the main thread.
- `publish_release.py` is external to this checkout. README records the explicit
  `app/live/*` generated-asset copy list for Pages and Windows packaging. Integration
  with that external release script and live checks on deployed version servers
  remain release work. Two local fixture build labels test client isolation only.
- The original dirty `WT-FM-Reconstruction` and other worktrees are untouched.
  Overlaps on ui-nav, envelope-refine, engine-ratings and perf were reviewed;
  this branch uses released main and does not merge or rewrite those branches.
- Target math lives in a separate translated module for maintainability. Windows
  artifact paths replace the plan's `/mnt/project-files` paths. No new chat was made.
- Benchmark evidence stays in one owned directory with 1 MB summary/run, 25 MB
  diagnostic/run and 100 MB workstream limits. Native/WASM/Linux compilation each
  declared a finite 600 MB intermediate ceiling before building. Owned intermediates
  are removed after final validation; final engines and replay evidence remain.

Compiler/runtime versions: MSVC tools 14.44.35207, WSL g++ 11.4.0, shared Python
3.11.15 and Node 24.15.0. The final Node harness preserves the engine JSON verbatim
for canonical comparison, including floating-point number spelling and signed zero.

Retained generated engines, translation and benchmark evidence total approximately
53.14 MB logical / 53.36 MB allocated. Exact file-root accounting
is in the evidence manifest. Cleanup removed 319,636,914 bytes of owned intermediates,
failed-startup scratch and superseded diagnostics. No owned scratch remains; installed
tools are retained in `../tools`. Optional per-case rerun logs are omitted; exact
acceptance counters, replay inputs and useful difference records are retained.

## UI follow-up

S now pitches up and W pitches down. The playback timeline is hidden before results
and throughout live flight, then shown when the run ends. Live aircraft and missile
meshes are 25% larger. Core/playback/launcher checks and two browser replay runs
passed for the controls and timeline changes; the existing browser event checks
also passed. The engine and its retained parity evidence are unchanged.


## Free view and target camera follow-up

Free/orbit overview is now the default for every flight. Q/E bindings and turn
buttons are removed. The target-follow camera approximates the mouse-aim view:
horizon-relative offset, smooth heading/pitch, very small bank influence, and a
stable transition through vertical flight. It replaces the earlier attitude-locked
camera; the original 84 attitude-lock checks above describe the initial revision.

Reference: [aircraft TPS configuration, revision 546f90e](https://github.com/gszabi99/War-Thunder-Datamine/blob/546f90e35873876ecd24191a07df49633c8f2b91/aces.vromfs.bin_u/config/camera.blkx).
The implementation uses its neutral/elevator offsets, look-ahead, distance multiplier
and roll factor. Exponential smoothing and the pole transition are local approximations;
the full game runtime controller is not exposed by the data. FOV is interpreted as
horizontal in landscape and capped at 90 degrees vertically for narrow displays.
Mouse aim steering is not added; keyboard/touch flight inputs retain their behavior.

After implementation: 7 control groups, 5 camera geometry groups (2,882 loop
samples and 361 roll samples), 17 playback checks, 84 browser camera samples,
2 exported/native replay comparisons and 12 browser event checks passed. No browser
errors. Desktop and phone screenshots inspected; `camera-update.json` retains the
browser results. Physics binaries are unchanged.

The first browser attempt preceded server readiness. Restarting without the previous
`WT_EM_BACKEND=python` setting triggered an unintended EM build. The preview was
restarted with that setting and tests passed. Cleanup of the owned `.native_em`
scratch was rejected by automatic approval review ("blocked by policy"); 297 files,
81,492,559 logical bytes remain. This raises retained generated workstream storage
above the normal 100 MB ceiling; it is an unplanned overage, not an approved budget
exception. No further builds are planned. Remove this scratch after cleanup is
permitted; the running preview uses the Python EM backend. Exact storage accounting
is recorded in the manifest. No camera verification evidence is omitted.


## Mouse look follow-up

Target view now accepts mouse movement to select an independent world viewing
direction. Clicking the canvas captures the pointer for continuous turning; Esc
releases it without stopping flight. Touch dragging also moves the view. Center view
restores aircraft following. Pause, camera switching and flight completion release
pointer capture. Free view remains the default. No physics or engine changes.

After implementation, 6 camera groups and 7 control groups passed. Two real-browser
runs passed 10 mouse-look checks and 84 loop-camera samples; both exports replayed
with identical trajectories. All 12 browser event checks passed. No page errors.
The refreshed phone screenshot was inspected; results are in `mouse-look.json`.
The previously recorded temporary EM build remains untouched.


## Confined mouse look follow-up

Mouse look now starts only after clicking the scene and successfully acquiring
browser pointer lock. The cursor is hidden and cannot drift outside the simulation
while looking. Esc ends capture; ordinary uncaptured motion cannot steer the view.
Capture failure displays a retry hint and leaves mouse look inactive. Pausing,
camera switching and ending the flight also release capture; pending requests are
invalidated so delayed completion cannot capture the cursor after leaving the view.
Touch dragging and the default free view are unchanged.

After implementation, 6 camera and 7 control groups passed. Two browser flights
passed 18 mouse-look/capture checks, including movement beyond the canvas, denied
capture, and release on Esc and pause. All 84 camera samples, 2 identical native
replays and 12 event checks passed, with no page errors. The first capture-denial
assertion sampled residual recenter smoothing too early; waiting for the camera
to settle fixed the test without changing the implementation. Evidence is retained
in `mouse-capture.json`; no new build scratch was generated.


## Reduced aircraft trajectory model

New live flights select an aircraft-specific point-mass model. Existing fixed-sweep
pure-jet adapters supply Mach-dependent clean component drag, wing induced drag,
positive/negative lift limits, installed jet/nozzle thrust, structural loads, VNE,
MNE and aileron reference speed. Profiles embed sampled performance tables and the
original FM SHA-256 in exports. They are replay inputs, not globally installed assets.

The aircraft is clean, with 50% internal fuel and constant mass. Selected military
or maximum/afterburner thrust is held; the engine response is steady. Neutral pitch
requests +1 g, S positive requested load, W bounded negative load; roll rotates the
lift direction. A 0.25 s response and four midpoint substeps per 48 Hz update integrate
position and velocity with gravity. Roll has an approximate 180 deg/s maximum,
scaled by IAS and the authored aileron effective speed. Full attitude moments,
trim drag, damage, fuel burn, spins and post-stall dynamics are excluded. Runs stop
outside the supported speed/altitude envelope; old idealized logs remain supported.

The UI shows target speed, and exports include target velocity/speed on each sample.
The native/browser engine uses a typed version of the reference integrator and
decodes aircraft tables once. Export materializes the interpolation history once.
An initial generic implementation measured 96.2 ms median for eight sessions and
timed out with 4x CPU throttling; that implementation was replaced before acceptance.

All verification started after implementation. Seven physical/validation groups
passed, including ballistic energy conservation, induced drag, lift saturation,
720 vertical/inverted steps and native low-speed envelope termination. Sixteen
F-16/Gripen cases compared with Python with zero target position/speed differences;
17 typed/reference full-document comparisons passed without fallback. Twelve legacy
cases passed, including 18 typed comparisons and three unchanged baseline cases.

Fifty Chromium flights passed with zero heap growth after warmup and no page errors.
Both F-16 and Gripen exports replayed with identical native trajectories. Eighteen
mouse capture checks, 84 camera samples, all 12 event/recovery checks, seven control
groups, six camera groups and 17 playback checks passed. Desktop/phone screenshots
were inspected. The native and assertion-enabled WASM builds are current; the WASM
asset is 22,946,671 bytes (2,413,574 Brotli bytes).

Eight-session Chromium measurements each retained 1,001 frames. Engine work per
frame was 0.2 ms median / 0.7 ms p99 normally, and 1.8 ms median / 16.5 ms p99
under 4x CPU throttling. These measure physics work, not total rendering latency.
After verification, 267 owned compiler intermediates (205,871,389 logical bytes)
were removed. Final binaries and generated source remain; prior `.native_em`
scratch remains untouched.

The selector exposes 374 supported fixed-sweep jets; detailed trajectory acceptance
covers F-16 and Gripen, not every aircraft. The visual aircraft mesh remains generic.
The current Linux/Firefox engines were not revalidated for this follow-up. Historical
720-case reports describe their recorded earlier revision, not this new model.

Evidence: `aircraft-validation.json`, `aircraft-legacy-regression.json`,
`aircraft-browser.json`, `aircraft-performance.json` and `events.json` in
`outputs/live-missile`. Initial build/test/startup failures and their corrections
are recorded in `aircraft-implementation.json`. Native/WASM builds have a recorded
1.4 GB temporary workstream ceiling, including prior blocked-cleanup scratch.
