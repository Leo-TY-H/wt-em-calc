# Public calculation server

The website and calculation service deploy independently:

- Once enabled in [DEPLOYING.md](DEPLOYING.md), GitHub Pages publishes `app/`
  after frontend pushes and successful data synchronization.
- A container VM runs `scripts/em_server.py` and retains calculated results on
  a persistent volume.
- The repository variable `API_BASE_URL` connects Pages to the API.

The server accepts browser requests only from origins listed in
`WT_EM_ALLOWED_ORIGINS`. It processes one calculation at a time and limits the
waiting queue to prevent unbounded public compute use.

## Recommended first host: Fly.io

The included `fly.toml` requests two shared CPUs, 2 GB RAM, one always-running
Machine, and a persistent `/data` volume. Always-on operation prevents a long
calculation from being stopped after its browser stops polling.

After installing `flyctl` and signing in:

    fly apps create YOUR_UNIQUE_API_NAME
    fly volumes create em_data --app YOUR_UNIQUE_API_NAME --region iad --size 5
    fly secrets set WT_EM_ALLOWED_ORIGINS=https://YOUR_GITHUB_NAME.github.io --app YOUR_UNIQUE_API_NAME
    fly deploy --app YOUR_UNIQUE_API_NAME

If the Pages site uses a custom domain, use that exact origin instead. Multiple
origins are comma-separated. Do not include repository paths; browser origins
contain only scheme and host.

Check the deployment:

    curl https://YOUR_UNIQUE_API_NAME.fly.dev/api/health

It should return `{"status":"ok"}`.

In the GitHub repository, create an Actions variable named `API_BASE_URL` with
this value:

    https://YOUR_UNIQUE_API_NAME.fly.dev

Run the Pages workflow again. It writes the URL into the deployed `config.js`;
the committed local configuration remains same-origin for local development.

## Updating

- Push frontend changes to `main`; Pages updates automatically.
- Run `fly deploy --app YOUR_UNIQUE_API_NAME` for solver, API, dependency, or
  aircraft-data changes.
- Increase the volume before it fills. Cached results make repeat requests much
  faster, but the cache grows over time.

## Production controls

- Exact CORS allowlist; unknown browser origins are rejected.
- Maximum request body of 32 KB.
- At most two queued/running calculations, with one compute worker.
- Non-root container process.
- `/api/health` health check.
- Persistent calculation and export cache at `/data/em`.

This is suitable for a limited public preview. Before promoting it to a large
audience, add per-client rate limiting and monitoring to control abuse and cost.
