# Oracle Always Free EM deployment

Provision an **Always Free** Ubuntu `VM.Standard.A1.Flex` VM in the Oracle account's home region with the local Oracle API profile. The tool checks existing VM and volume use, creates a 2-OCPU/12-GB Arm VM with a 50-GB boot disk, and adds a public network with inbound SSH from the current IP and HTTP/HTTPS from anywhere. It never selects an AMD Micro VM or a paid shape. Oracle can still reject the request when Always Free capacity is unavailable.

```powershell
& "$env:USERPROFILE\.oci\venv\Scripts\python.exe" deploy/oracle/provision.py --plan
& "$env:USERPROFILE\.oci\venv\Scripts\python.exe" deploy/oracle/provision.py --apply
```

The provisioning command prints the VM's public IP and the deployment command to run. Re-running it reuses resources with this tool's names.

From this repository on Windows, run:

```powershell
python deploy/oracle/deploy.py --host YOUR_PUBLIC_IP --key C:\path\to\oracle_private_key
```

The deployment command validates the VM's Arm shape and free CPU/memory size, installs Docker, builds the current source on the VM, and starts the EM app behind Caddy HTTPS. It prints a test URL using the free `sslip.io` DNS service. The server checks game data every six hours with a systemd timer and restarts after successful updates. Re-run the same command after changing the source code. The command bundles source files from the working tree, including uncommitted edits; it does not upload local reference data, results, or credentials.

For a safe preflight that does not contact Oracle:

```powershell
python deploy/oracle/deploy.py --host YOUR_PUBLIC_IP --key C:\path\to\oracle_private_key --dry-run
```

Saved calculations and downloaded aircraft references live under `/opt/wt-em/shared` on the VM. They survive app updates. The script does not delete old results automatically; check disk usage periodically. To inspect the data timer, run `sudo systemctl status wt-em-sync.timer` on the VM.

The Cloudflare Pages frontend at `https://neothunderism.pages.dev` calls this VM's HTTPS API. The Pages hostname is included in `WT_EM_ALLOWED_ORIGINS` when this deployment script runs. The `app/config.js` file selects the VM API only on that production Pages hostname; local and Oracle-hosted copies keep using their own origin.

The Pages project uses Cloudflare Direct Upload. After a frontend change, publish the `app` directory with `npx wrangler pages deploy app --project-name neothunderism --branch main` from the repository root while logged in to Cloudflare with Wrangler. This uploads only the interface. The Oracle data-sync timer refreshes the flight-model data independently, so data updates do not require a Pages deployment.
