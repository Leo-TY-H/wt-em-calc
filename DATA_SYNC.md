# Game data synchronization

The source is `gszabi99/War-Thunder-Datamine`, following `master`, configured in
`references/data-sync-config.json`. Each refresh resolves one commit, downloads
from that exact revision, and verifies Git blob hashes before replacing files.
The version and hash inventory are recorded in `references/data-version.json`.

## Scope and schedule

The updater downloads all aircraft FMs, vehicle definitions and FM/model
mappings, plus gameplay, game parameters, flight-model configuration and
modification definitions used by this calculator. It does not mirror unrelated
tank, ship, texture or sound files. Wiki display names remain a separate cache;
run `python scripts/fetch_vehicle_names.py` to refresh labels. New vehicles
without wiki labels are still listed by ID.

Both launchers check automatically with a six-hour local cache. Network outages
allow launch using existing data; integrity errors or local edits stop the update.
Close both plotters before updating, pulling or pushing: running servers cache
their catalogs. `Update Game Data.cmd` checks immediately and rebuilds the
static website catalog without needing a running server.

GitHub **Actions → Sync game data** runs every six hours, on relevant pushes to
`main`, and on demand. Changed data and website metadata are committed to `main`.
The workflow requires `contents: write` and branch rules permitting bot pushes.
Failed downloads or tests never produce an automatic data commit. GitHub controls
the exact schedule timing; successful refreshes do not imply a deployed API.

## Push local work

Double-click `Push to GitHub.cmd`, or run:

```text
python scripts/push_to_github.py --message "Describe your changes"
```

This fetches remote history, refreshes game data and metadata, commits all
non-ignored changes, rebases onto the remote branch, and performs a normal push.
On conflicts, it aborts the rebase and preserves your local commit for manual
resolution. A concurrent remote push is rejected safely; retry the command.
Authentication uses Git's credential manager; no credentials are stored here.
To receive remote work without publishing, use `git pull --ff-only` with a clean
working tree. The launchers update data, not your Python/frontend source files.

Local source edits are never silently overwritten. Keep custom FM experiments
separately and restore the source before auto-updating. The first migration uses
the old FM/vehicle manifests; shared configurations without old hashes are
adopted on that first refresh, then protected by the full hash inventory.

## Prepared assets and limits

Propeller propulsion and collision/mass assets require exact FM hash matches.
After an FM change, the aircraft remains listed but cannot calculate until its
assets are regenerated and validated. New prop aircraft show a missing-asset
reason. Automatic downloads cannot reconstruct installed collision geometry or
validate a new executable's behavior.

Existing research producers are `scripts/prepare_propulsion_assets.py` and
`scripts/prepare_prop_geometry.py`; they require local native/collision research
inputs. Never change stored hashes to bypass validation. The recovered equations
remain at their researched version; current data is not proof of current-game
numerical accuracy.

The user-approved removal of 36 unsupported fixed-wing records is maintained in
`references/aircraft-exclusions.json`. The unified local catalog, worker snapshots
and generated website metadata omit those IDs. Refreshing upstream data preserves
the exclusions. Raw source FMs remain synchronized because retained vehicle aliases
may share them. This is an explicit list, not an automatic deletion of any aircraft
that becomes temporarily unsupported after an update.

For one vehicle with an exact installed collision resource, the transactional
producer is `python scripts/refresh_prop_assets.py VEHICLE --game GAME_DIRECTORY`.
It reruns the pinned native loaders and checks mass at four fuel selections
before publishing both assets and updating the propulsion manifest. Rebuild
website metadata afterward with `python scripts/update_pages_snapshot.py --offline`.

Historical `fm-2.59.0.13` fixtures are retained for research. Runtime jets all
read the updated catalog, including the two reference jets. Runtime data,
prepared JSON assets and code are tracked in Git; executable oracles, analysis,
raw collision packages, environments and generated results remain local.

## Verification

```text
python -m unittest discover -s tests -v
python scripts/sync_game_data.py --force
python scripts/update_pages_snapshot.py --offline
```

Tests cover bad downloads, local edits, additions/removals, publication rollback,
deterministic no-change updates, check caching and stale prepared assets.
