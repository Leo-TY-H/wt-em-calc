"""Install or update the EM website on an existing Oracle Always Free Arm VM.

Run from any directory with: python deploy/oracle/deploy.py --host PUBLIC_IP
    --key PATH_TO_ORACLE_PRIVATE_KEY
The script sends source files only; local references, results, and credentials stay local.
"""

import argparse
import io
import ipaddress
import os
from pathlib import Path
import re
import subprocess
import sys
import tarfile
import time
from urllib.request import urlopen


ROOT = Path(__file__).resolve().parents[2]
SETUP = Path(__file__).with_name("remote_setup.sh")


def git(*args):
    return subprocess.check_output(["git", *args], cwd=ROOT)


def source_archive():
    paths = [p.decode("utf-8") for p in git("ls-files", "-z", "--cached", "--others", "--exclude-standard").split(b"\0") if p]
    if not paths or "deploy/oracle/remote_setup.sh" not in paths:
        raise RuntimeError("Deployment source files are missing from the archive")
    archive = io.BytesIO()
    with tarfile.open(fileobj=archive, mode="w:gz") as output:
        for name in paths:
            path = ROOT / name
            if not path.is_file() or path.is_symlink():
                raise RuntimeError(f"Unsafe source path: {name}")
            output.add(path, arcname=name, recursive=False)
    return archive.getvalue(), paths


def ssh_args(user, host, key):
    return ["ssh", "-i", str(key), "-o", "BatchMode=yes", "-o", "StrictHostKeyChecking=accept-new", f"{user}@{host}"]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", required=True, help="Oracle VM public IPv4 address")
    parser.add_argument("--key", type=Path, required=True, help="Oracle VM SSH private key")
    parser.add_argument("--user", default="ubuntu", help="VM SSH user (default: ubuntu)")
    parser.add_argument("--dry-run", action="store_true", help="Validate the source bundle without connecting")
    args = parser.parse_args()
    address = ipaddress.ip_address(args.host)
    if address.version != 4 or not address.is_global:
        parser.error("--host must be a public IPv4 address")
    if not re.fullmatch(r"[a-z_][a-z0-9_-]*", args.user):
        parser.error("--user must be a Linux account name")
    if not args.dry_run and not args.key.is_file():
        parser.error("SSH private key not found")
    bundle, paths = source_archive()
    release = git("rev-parse", "--short", "HEAD").decode().strip() + "-" + str(int(time.time()))
    hostname = f"{address}.sslip.io"
    print(f"Prepared {len(paths)} source files ({len(bundle) / 1048576:.1f} MiB) for {hostname}", flush=True)
    if args.dry_run:
        return
    remote = ssh_args(args.user, args.host, args.key)
    remote_archive = f"/tmp/wt-em-{release}.tar.gz"
    subprocess.run([*remote, "cat > " + remote_archive], input=bundle, check=True)
    try:
        subprocess.run([*remote, "bash -s -- " + args.host + " " + release + " " + remote_archive],
                       input=SETUP.read_bytes(), check=True)
    finally:
        subprocess.run([*remote, "rm -f " + remote_archive], check=False,
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    url = f"https://{hostname}/api/health"
    for attempt in range(20):
        try:
            with urlopen(url, timeout=10) as response:
                if response.status == 200 and b'"ok"' in response.read():
                    print(f"Live website: https://{hostname}/")
                    return
        except Exception:
            if attempt == 19:
                raise RuntimeError(f"VM setup finished, but HTTPS health check failed: {url}")
            time.sleep(6)


if __name__ == "__main__":
    try:
        main()
    except (OSError, subprocess.CalledProcessError, RuntimeError) as error:
        print(f"Deployment stopped: {error}", file=sys.stderr)
        raise SystemExit(1)
