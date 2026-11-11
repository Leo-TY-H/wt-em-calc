# Changelog

## 2026.11.11.5

- All FM versions use one shared HTTP service and the same compiled solver. Version inputs and catalogs are prepared ahead of use, with a bounded pool of warm calculation workers.
- Short calculations reach the final chart without downloading an unnecessary preview first. Completed charts and point details survive worker reuse and input updates.
- Newly built inputs become available after verification; fresh pages continue to default to the newest available version.

## 2026.11.11.4

- Short-release aircraft (release start at most 10 seconds) use the released Instructor pull boundary everywhere, without reapplying the wing-protection cap when the in-game release conditions are absent. Other limits and longer-release aircraft retain their existing behavior.


## 2026.11.11.3

- Aircraft added or duplicated in Engine Performance now keep valid, unique entry IDs when switching to EM or altitude diagrams, so those calculations accept the shared aircraft list.

## 2026.11.11.2

- Fresh pages default to the newest available game version. EM, engine, and missile calculators load that version's catalog and send calculations to its server.
- Manually selected historical versions keep their own data and calculation routes. A default version's server starts again when needed.
