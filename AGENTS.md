# Repository rules for agents

These rules apply throughout this repository, including benchmarks, research,
checkpoints, packaging, and work delegated to other agents. Read them before
generating artifacts. Pass them explicitly to agents using separate worktrees
that do not yet contain this file. Historical plans and old scripts may describe
larger snapshots; they are not permission to recreate those outputs.

## Output budgets

- Default to compact summaries: settings, source/input hashes, command, timings,
  exact work counters, validation outcomes, and concise failure records.
- Use these default limits (MB means 1,000,000 bytes):
  - Summary data: at most **1 MB per run**, across all summary files.
  - Optional diagnostic data: at most **25 MB compressed per run**.
  - All retained generated benchmark artifacts: at most **100 MB per workstream**,
    including snapshots, profiles, logs, and temporary output. A workstream means
    the entire experiment across its runs, agents, and output directories.
- These are storage limits, never limits on numerical work or accuracy. Keep all
  physical checks, tolerances, failure classifications, and acceptance gates.
  Do not drop validation cases to fit an output budget.
- Before a broad run, inspect the writer and run a small representative case in
  an owned directory. Measure bytes and estimate the full batch, including
  workers and repetitions. Existing scripts are not automatically budget-safe.
  Add bounded output controls before scaling up a verbose legacy benchmark.
- Use `scripts/benchmark_output.py` for benchmark writes. See
  [BENCHMARK-OUTPUTS.md](BENCHMARK-OUTPUTS.md) for enforced defaults, shared batch
  accounting, and command-line overrides. Larger justified studies are allowed
  without an approval prompt; record a reason and a finite byte ceiling.
- Writers must enforce diagnostic byte/record caps while writing, including
  compressed buffers and worker totals. Compression alone is not a size cap.
  Do not write an unlimited dump and compress it afterward.
- At the diagnostic cap, stop saving optional detail, continue the calculation
  and exact counters, and record omitted counts and `detail_truncated: true`.
  If required evidence cannot be retained, mark evidence incomplete; do not claim
  acceptance. Redesign the representation or document a justified exception.
- Any exception must be recorded before the run: purpose, owner, explicit byte
  ceiling, retained evidence, and cleanup condition. First reduce duplicated
  data. Do not silently increase caps or split runs to evade the workstream cap.

## Benchmark evidence and retention

- Timing repetitions retain scalar timings and counters. Save a replayable state
  once per distinct case/configuration, with references from later repetitions.
  Do not embed repeated model assets, engine tables, histories, or full charts.
- Full traces and per-worker profiles are opt-in for a named unresolved issue
  and a small reproducer. Stream bounded gzip JSONL or another compact format;
  avoid pretty-printed large JSON and repeated cumulative dumps.
- Preserve enough final state/configuration to replay physical checks and compare
  results. Hashes identify evidence; they do not replace unavailable source or
  required replay data. Retain final acceptance evidence and useful failures.
- Use one owned output directory per workstream. Keep a small manifest recording
  owner/task, status (active/final/superseded), command, source revision plus dirty
  changes, input hashes/locations, budget, retained paths, and cleanup condition.
- Reuse the same immutable baseline across repetitions and cooperating agents.
  Replace only your own superseded scratch results once useful evidence is saved.
  Avoid accumulating numbered copies of successful runs and reports.
- Handoffs report bytes added, retained artifact paths, known evidence omissions,
  and remaining scratch files. Measure logical bytes as well as disk allocation;
  filesystem compression and hardlinks must not hide excessive generated output.

## Checkpoints and reference data

- Never recursively copy the repository, research tree, output tree, model data,
  runtime, virtual environment, or another checkpoint into a new checkpoint.
- Record a Git revision, the relevant dirty patch/untracked source, exact run
  configuration, dependencies, and an input hash manifest. Reuse one available
  immutable reference dataset by path and hash. Preserve unique source that Git
  does not track; a revision alone does not reproduce a dirty workspace.
- If an executable snapshot is necessary, copy only an explicit minimal file
  allowlist. Inspect its size first; reject nested checkpoints, caches, output
  trees, junctions, and symlinks that escape the intended input set. Reuse it for
  the whole experiment instead of copying it for each run or agent.
- Historical checkpoint payloads may be read-only NTFS hardlinks. Never clear
  read-only protection to modify them in place. Make an independent copy of only
  the files being changed. Never hardlink mutable source or working outputs.
- Required live references and historical evidence with consumers must remain
  available. Check code, manifests, and handoff references before retiring data.
  Store each large reference once; refer to its hash instead of embedding copies.

## Shared workspace and cleanup

- Check Git status, active work, and ownership before edits or cleanup. Preserve
  other agents' dirty files, outputs, processes, and baselines. An ignored or old
  file is not automatically disposable. Unknown ownership means leave it alone.
- Clean only your own temporary files after success or failure. Packaging should
  use one temporary staging directory; remove it after archive integrity and
  required smoke checks pass. Preserve the final archive and its manifest.
- Keep generated artifacts out of Git; do not broadly unignore output directories.
  Keep these instructions versionable. Never delete Git objects or rewrite shared
  history as routine storage cleanup.
- Do not run repository-wide cleanup, deduplication, or automatic age-based
  deletion during another task. Coordinate changes to shared benchmark writers;
  these rules do not authorize overwriting another agent's implementation.
