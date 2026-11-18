#!/usr/bin/env bash
# Ubuntu/Linux: Cloudflare Quick Tunnel (temporary public HTTPS URL)
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cd "$ROOT"
if ! command -v python3 >/dev/null 2>&1; then echo 'python3 is not installed'; exit 1; fi
CLIENT=''
if command -v cloudflared >/dev/null 2>&1; then
  CLIENT="$(command -v cloudflared)"
elif [ -x "$ROOT/.bin/cloudflared" ]; then
  CLIENT="$ROOT/.bin/cloudflared"
else
  case "$(uname -m)" in
    x86_64|amd64) ARCH='amd64';;
    aarch64|arm64) ARCH='arm64';;
    *) echo 'Unsupported CPU architecture. Please install cloudflared manually.'; exit 1;;
  esac
  if command -v curl >/dev/null 2>&1; then DOWNLOADER=curl
  elif command -v wget >/dev/null 2>&1; then DOWNLOADER=wget
  else echo 'curl or wget is required to download cloudflared.'; exit 1; fi
  echo 'Downloading cloudflared from its official GitHub release...'
  mkdir -p "$ROOT/.bin"
  URL="https://github.com/cloudflare/cloudflared/releases/latest/download/cloudflared-linux-${ARCH}"
  if [ "$DOWNLOADER" = curl ]; then curl --fail --location --retry 2 "$URL" -o "$ROOT/.bin/cloudflared.download"
  else wget -O "$ROOT/.bin/cloudflared.download" "$URL"; fi
  mv "$ROOT/.bin/cloudflared.download" "$ROOT/.bin/cloudflared"
  chmod 755 "$ROOT/.bin/cloudflared"
  CLIENT="$ROOT/.bin/cloudflared"
fi
STARTED_SERVER=0
if ! python3 -c 'import urllib.request; urllib.request.urlopen("http://127.0.0.1:8080/api/health", timeout=2).read()' >/dev/null 2>&1; then
  echo 'Starting the local HTTP server...'
  python3 "$ROOT/server.py" --port 8080 &
  SERVER_PID=$!
  STARTED_SERVER=1
  sleep 1
  if ! kill -0 "$SERVER_PID" 2>/dev/null; then echo 'The HTTP server did not start.'; exit 1; fi
fi
cleanup() {
  if [ "$STARTED_SERVER" -eq 1 ]; then kill "$SERVER_PID" 2>/dev/null || true; fi
}
trap cleanup EXIT INT TERM
printf '\n%s\n' '════════════════════════════════════════════════════════════'
echo 'Public tunnel is starting. Look for a https://*.trycloudflare.com URL.'
echo 'Share that HTTPS URL with anyone, including people on other networks.'
echo 'Visitors share the scene; anyone with the URL can edit it.'
echo 'The temporary URL expires when the tunnel stops and may change each run.'
echo 'Press Ctrl+C to stop sharing.'
printf '%s\n\n' '════════════════════════════════════════════════════════════'
"$CLIENT" tunnel --url http://127.0.0.1:8080
