# Website deployment

Game-data synchronization runs independently of website publication. The
repository includes the application, Python source and runtime reference data.
Push local work with `Push to GitHub.cmd`; see [DATA_SYNC.md](DATA_SYNC.md).

## Enable GitHub Pages when ready

1. In repository **Settings → Pages**, choose **GitHub Actions** as the source.
2. In **Settings → Secrets and variables → Actions → Variables**, set
   `ENABLE_PAGES` to `true` for automatic deployment.
3. Run **Actions → Deploy website → Run workflow** for first publication.

The workflow publishes only `app/`, after frontend changes and successful data
synchronization. Its `workflow_run` trigger handles bot data commits, which do
not trigger ordinary push workflows. See [GitHub's event documentation](https://docs.github.com/en/actions/reference/workflows-and-actions/events-that-trigger-workflows).
Manual execution deploys even if `ENABLE_PAGES` is unset; otherwise publication
stays disabled until that variable is enabled.

The expected project URL is `https://leo-ty-h.github.io/wt-em-calc/`.

## Calculation service

GitHub Pages cannot run the Python solver. Without an `API_BASE_URL` repository
variable, the site shows the interface and static aircraft catalog as a preview.
For calculations, deploy the included Docker container and set `API_BASE_URL`
to its HTTPS URL; see [CALCULATION_SERVER.md](CALCULATION_SERVER.md).

The Pages workflow writes this URL into the deployed `config.js`; local use
continues with a same-origin API. Redeploy the calculation container after
solver/data commits. API deployment is separate from automatic data updates.

To rebuild metadata without a running server:

```text
python scripts/update_pages_snapshot.py --offline
```

The existing `--url http://127.0.0.1:8765/api/meta` option can still export from
a running calculator.
