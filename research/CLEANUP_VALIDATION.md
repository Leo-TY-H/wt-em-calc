# Cleanup validation — 2026-09-24

Observed on the original macOS arm64 machine, using Python 3.9 and pinned local
dependencies. These checks do not establish Windows/MSVC equivalence or runtime
speed and do not validate the experimental Instructor boundary.

- Parsed all remaining Python source files; no imports of the 27 retired modules.
- Confirmed the extracted `source_state` function is AST-identical to its original.
- Confirmed unrelated CSS and all altitude app files are byte-for-byte preserved;
  app.js and index.html cleanup changes are confined to experimental labeling.
- JavaScript syntax checks passed for app.js, altitude.js and altitude_surface.js.
- Current project compiled backend: portable-runtime smoke check passed.
- Fresh relocated source copy, without a venv or compiled cache: built 45 native
  extensions from source and passed the same runtime smoke check using the Mac
  Python dependency environment. Reference and code paths resolved in the new folder.
- Spawned-worker solves passed force, angular-rate and history closure for
  F-16XL at 600 km/h, 2 g with Instructor on and off; J6K1 at 400 km/h, 2 g with
  Instructor off. This is a small installation smoke set, not a fleet benchmark.
- Original-code replay matched F-16XL component forces, application points and
  raw aerodynamic moment exactly for the selected SB point.
- Positive stall retained / negative stall ignored: four RB/SB checks passed,
  including preservation of invalid status for an unbalanced J6K1 iterate.
- Native F-16XL mode-1 predictor subset: four cases, eight calls, zero output or
  preparation failures, using the bundled pinned executable.
- EM API health, metadata (1,348 aircraft) and UI returned HTTP 200.
- Altitude health, `/api/altitude/meta` (1,348 aircraft) and UI returned HTTP 200.
- Portable input inventory passed Windows reserved-name, case-collision, path
  length and symlink checks. UTF-8 and CRLF launchers are included.

The zip is separately checked against TRANSFER_MANIFEST.json, including file
contents, expected membership and excluded cache/history directories. Its
SHA-256 is in the adjacent .sha256 file. Full build/runtime logs are retained
in the sibling archive. Run Setup Windows.cmd to validate the destination.

Follow-up: full browser checks found and fixed saved-altitude throttle input
roundoff. See APP_FUNCTIONAL_VALIDATION.md; the initial cleanup preservation
check above predates this intentional UI fix.
