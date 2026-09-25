#!/usr/bin/env bash
set -euo pipefail

host_ip="$1"
release="$2"
archive="$3"
site_host="${host_ip}.sslip.io"
base=/opt/wt-em
release_dir="${base}/releases/${release}"

if [[ "$(uname -m)" != aarch64 ]]; then
  echo 'Oracle Always Free Ampere A1 (Arm64) is required.' >&2
  exit 1
fi
read -r shape ocpus memory < <(python3 - <<'PY'
import json
from urllib.request import Request, urlopen
base = 'http://169.254.169.254/opc/v2/instance/'
def get(path):
    with urlopen(Request(base + path, headers={'Authorization': 'Bearer Oracle'}), timeout=5) as response:
        return json.load(response)
instance, config = get(''), get('shapeConfig')
print(instance.get('shape', ''), config.get('ocpus', ''), config.get('memoryInGBs', ''))
PY
)
if [[ "$shape" != VM.Standard.A1.Flex ]] || \
   ! python3 -c 'import sys; c,m=map(float,sys.argv[1:]); assert 0<c<=2 and 0<m<=12' "$ocpus" "$memory"; then
  echo "Refusing VM outside one Always Free A1 allowance: $shape, $ocpus OCPU, $memory GB" >&2
  exit 1
fi

if ! command -v docker >/dev/null || ! sudo docker compose version >/dev/null 2>&1; then
  sudo apt-get update
  sudo apt-get install -y ca-certificates curl
  sudo install -m 0755 -d /etc/apt/keyrings
  sudo curl -fsSL https://download.docker.com/linux/ubuntu/gpg -o /etc/apt/keyrings/docker.asc
  sudo chmod a+r /etc/apt/keyrings/docker.asc
  . /etc/os-release
  sudo tee /etc/apt/sources.list.d/docker.sources >/dev/null <<EOF
Types: deb
URIs: https://download.docker.com/linux/ubuntu
Suites: ${UBUNTU_CODENAME:-$VERSION_CODENAME}
Components: stable
Architectures: $(dpkg --print-architecture)
Signed-By: /etc/apt/keyrings/docker.asc
EOF
  sudo apt-get update
  sudo apt-get install -y docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
fi

sudo install -d -o "$(id -u)" -g "$(id -g)" "$release_dir"
tar -xzf "$archive" -C "$release_dir"
cd "$release_dir"
sudo docker build -t "wt-em:${release}" .

sudo install -d "${base}/shared/references" "${base}/shared/data-sync" "${base}/shared/em"
if [[ ! -f "${base}/shared/references/data-version.json" ]]; then
  seed="wt-em-seed-${release}"
  sudo docker create --name "$seed" "wt-em:${release}" >/dev/null
  sudo docker cp "${seed}:/srv/wt-em/references/." "${base}/shared/references/"
  sudo docker rm "$seed" >/dev/null
fi
sudo chown -R 10001:10001 "${base}/shared"
sudo docker network inspect wt-em-net >/dev/null 2>&1 || sudo docker network create wt-em-net >/dev/null

sudo docker rm -f wt-em-app >/dev/null 2>&1 || true
sudo docker run -d --name wt-em-app --restart unless-stopped --network wt-em-net \
  -e PORT=8080 -e WT_EM_HOST=0.0.0.0 -e WT_EM_OUTPUT_DIR=/data/em \
  -e WT_EM_MAX_PENDING_JOBS=2 -e "WT_EM_ALLOWED_ORIGINS=https://${site_host},https://neothunderism.pages.dev" \
  -v "${base}/shared/references:/srv/wt-em/references" \
  -v "${base}/shared/data-sync:/srv/wt-em/.data-sync" \
  -v "${base}/shared/em:/data/em" "wt-em:${release}" >/dev/null

sudo install -d "${base}/caddy"
printf '%s {\n  encode zstd gzip\n  reverse_proxy wt-em-app:8080\n}\n' "$site_host" | \
  sudo tee "${base}/caddy/Caddyfile" >/dev/null
if ! sudo docker container inspect wt-em-caddy >/dev/null 2>&1; then
  sudo docker run -d --name wt-em-caddy --restart unless-stopped --network wt-em-net \
    -p 80:80 -p 443:443 \
    -v "${base}/caddy/Caddyfile:/etc/caddy/Caddyfile:ro" \
    -v wt-em-caddy-data:/data -v wt-em-caddy-config:/config caddy:2 >/dev/null
else
  sudo docker restart wt-em-caddy >/dev/null
fi

sudo tee "${base}/sync.sh" >/dev/null <<'EOF'
#!/usr/bin/env bash
set -euo pipefail
version=/opt/wt-em/shared/references/data-version.json
before="$(sha256sum "$version" | cut -d' ' -f1)"
docker exec wt-em-app python scripts/sync_game_data.py --force
after="$(sha256sum "$version" | cut -d' ' -f1)"
if [[ "$before" != "$after" ]]; then
  docker restart wt-em-app >/dev/null
  echo 'Game data updated; calculation server restarted.'
fi
EOF
sudo chmod 755 "${base}/sync.sh"
sudo tee /etc/systemd/system/wt-em-sync.service >/dev/null <<'EOF'
[Unit]
Description=Update War Thunder aircraft data for EM website
After=docker.service
[Service]
Type=oneshot
ExecStart=/opt/wt-em/sync.sh
EOF
sudo tee /etc/systemd/system/wt-em-sync.timer >/dev/null <<'EOF'
[Unit]
Description=Check EM website aircraft data every six hours
[Timer]
OnCalendar=*-*-* 00,06,12,18:15:00
Persistent=true
[Install]
WantedBy=timers.target
EOF
sudo systemctl daemon-reload
sudo systemctl enable --now wt-em-sync.timer
sudo systemctl start wt-em-sync.service

for attempt in $(seq 1 20); do
  if sudo docker exec wt-em-app python -c \
    'from urllib.request import urlopen; assert urlopen("http://127.0.0.1:8080/api/health",timeout=4).status==200' >/dev/null 2>&1; then
    echo "Website ready at https://${site_host}/"
    exit 0
  fi
  sleep 3
done
echo 'Server health check failed; inspect docker logs wt-em-app.' >&2
exit 1
