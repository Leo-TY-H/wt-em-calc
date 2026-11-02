# Benchmark output controls

The limits bound saved artifacts, not solver iterations, repetitions, physical
checks, or accuracy. Use larger limits when an investigation needs them; no
approval prompt is required. Record a reason so the extra storage is intentional.

| Option | Default | Scope |
|---|---:|---|
| `--output-summary-mb` | 1 | All summary files for one benchmark invocation |
| `--output-diagnostic-mb` | 25 | All result payloads, traces, profiles and logs for that invocation |
| `--output-total-mb` | 100 | Entire workstream directory, including existing files and temporary writes |
| `--output-budget-root` | Output directory (or the output file's parent) | Shared directory for a batch and its workers |
| `--output-budget-reason` | None | Required when increasing defaults or changing an existing shared cap |
| `--full-output` | Off | Include optional solver histories in legacy/research results; caps still apply |

MB means 1,000,000 bytes. Repetitions in one invocation share its allowance;
suite cases have separate invocation allowances and share the workstream cap.
Gzip files count by compressed size. Plain JSON, profiles and logs count by their
actual file size. Up to 2 MB of the total is reserved for bookkeeping and its
atomic replacement. Numerical output can exceed 1 MB: it is diagnostic/replay
data, not a summary. Limits are finite but can be set as high as the work needs.

For a larger, explicitly bounded study, add these options to the usual command:

```text
--output-summary-mb 5 --output-diagnostic-mb 100 --output-total-mb 500
--output-budget-root outputs/my-study
--output-budget-reason "Full acceptance corpus; retain final evidence, retire intermediate traces after review"
```

Keep every output path, including `--results-directory`, below that root. Use
one fresh directory per workstream; do not point at the repository root or all
historical outputs. Cooperating processes must use the same root. Batch runners
propagate the settings to children through `WT_BENCHMARK_OUTPUT`. Separate manual
commands should pass the same flags explicitly. Moving files to fresh roots to
reset the budget defeats the workstream policy.

## What happens at a limit

Writes are bounded while being produced. A partial result never replaces the
previous valid file. Optional logs/profiles can be omitted; child stdout is still
drained so a full log cannot deadlock or terminate a calculation. The output
ledger records `omitted` writes; log metadata also records bytes seen and saved.

Required output that cannot fit is marked missing in `.benchmark-output.json`.
Calculations and validation continue; the benchmark returns nonzero (normally 2)
instead of silently treating the evidence as complete. A previous incremental
result may remain at the requested path: always check the process exit status
and ledger before using it. A missing artifact is not a physical failure and is
not a passing acceptance result. Increase the appropriate limit with a reason
and rerun if that evidence is needed.

The ledger and lock coordinate writers across processes. Do not remove them to
reset counters. Writers reject unowned existing files, files belonging to another
run, and writes outside the root or through links/junctions. They do not prune
anything, change historical checkpoints, or terminate other agents' processes.
Existing historical folders without a ledger should be treated as read-only;
start a fresh output directory.

## Compact defaults and compatibility

- SEP and active-boundary experiments retain numerical states, certificates,
  residuals, work counters and all validation records. Default serialization
  excludes verbose search/correction histories listed in `benchmark_output.py`.
  `--full-output` retains those histories when needed. This filtering happens
  when saving, after computation; it never changes the solver's in-memory state.
- Branch benchmarks save complete comparison results as `result.json.gz`, reuse
  identical comparison inputs across repetitions, and accept `--reuse-result`
  for an existing correctness result. Each calculation and physical replay still
  runs. Reuse requires the existing comparison fingerprint to match exactly.
  Acceptance and reference-audit readers accept both plain and gzip JSON.
- Branch task counters remain exact even when optional task logs fill up.
  Missing task identities make determinism evidence incomplete. The runner
  cannot certify acceptance from truncated task evidence.
- The trim redesign's `--snapshot` saves a bounded compressed source bundle,
  not recursively copied directories. The direct SEP snapshot uses an explicit
  flat source/native-library allowlist and preflights its size. Existing snapshots
  must be reused, not overwritten; large reference assets stay shared inputs.
- The PowerShell reporting wrapper exposes `-OutputSummaryMB`,
  `-OutputDiagnosticMB`, `-OutputTotalMB` and `-OutputBudgetReason` and uses the
  bounded subprocess logger. Shell redirections added outside these tools are
  outside their control.

Coverage includes `scripts/benchmark_*.py`, the reporting wrapper and its point
study delegates, branch benchmark/acceptance/test/reference tools, and the direct
SEP/active-boundary research benchmarks and their two validation runners. Frozen
historical runtimes remain unchanged. New writers must use `OutputBudget`; these
controls are not a filesystem quota for unrelated programs or arbitrary scripts.

The default compact writer reduced a saved 39.46 MB boundary benchmark to
13.19 MB while preserving all 72 validation records, 144 work-counter records,
and boundary classifications. The original result was not modified.
