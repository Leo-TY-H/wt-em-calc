# Shared FM calculation service

The public API runs one Node process. `/v/<version>/api/...` selects immutable
inputs; all versions use the same compiled solver. Three private calculation
workers share a warm, single-threaded library template. They have no HTTP ports.
An idle worker can be reassigned while its completed charts and point details
remain in bounded memory. Busy workers are never reassigned.

Set `WT_EM_DATASETS=/datasets`, `WT_EM_CATALOGS=/catalogs`, and
`WT_EM_PRIMARY` to the historical unversioned dataset. Mount the host datasets
with `ro,rslave`, catalogs read-only, and the version index at
`/srv/wt-em/.data-sync`. Pinning the unversioned dataset keeps existing tabs on
their selected inputs. The browser defaults to the highest available numeric
version. Without dataset/catalog settings, the image retains the ordinary
single-dataset server used by local installations.

On the host, compile `scripts/dataset_pack.cpp` and `scripts/dataset_check.cpp`
with C++17 and libgit2 into `shared-service/scripts/bin`. Install the publisher
and catalog preparation scripts beside them. `dataset-mounts.json` maps mount
names to immutable SquashFS filenames under `shared/dataset-images`.
The datasets directory is a shared bind mount, with images under `.mounts` and
relative version symlinks. The initial image stores identical Git blobs once.
The native checker verifies all files and shared objects against their Git trees.

Install these systemd units and the Docker drop-in. The input mount service must
run before Docker on reboot. The publication timer also prepares new versions;
the existing compiler/backfill can invoke it immediately after storing inputs.
Publication uses the running image, verifies a native-built input image, then
publishes the catalog manifest last. The API hides versions until both are ready.
No calculation request extracts files, builds catalogs, or starts a container.
After refreshing the current compiler inputs, invoke the publisher with
`--primary` while holding the compiler update lock. It preserves the exact new
input tree under `refs/shared-inputs/`, including updated geometry, and publishes
an immutable generation. Running jobs and completed point URLs retain their
original generation; new jobs use the updated inputs without restarting HTTP.

Route all API paths through the shared container, preserving the `/v/` prefix.
Disable the old `wt-em-versions.path` and `wt-em-versions.timer`. The input refresh
hook must preserve the shared process when updating the compiler's mutable input
workspace; it publishes prepared immutable inputs separately. Never replace a
published version's input tree in place.

Runtime result storage remains ephemeral. Each worker keeps the existing two
completed EM results; compressed inactive worker states have a total 512 MB cap.
The publisher caps each new input image at 500 MB and each catalog at 5 MB, and
never retains unbounded logs. These are publication limits, not solver limits.

Run `node --test scripts/test_game_versions.cjs scripts/test_result_poll.cjs`
for routing and preview checks. With a private candidate running, use
`WT_EM_TEST_URL=http://127.0.0.1:18872 node scripts/test_shared_service.cjs CONFIG`
to check every available catalog, multiple versions, saved points, cancellation,
and worker limits. `CONFIG` is a saved ordinary EM request. Set
`WT_EM_REFERENCE_URL` to an isolated server with identical inputs to additionally
compare chart values exactly. Run the existing native equivalence gate on the
combined release before public deployment.
