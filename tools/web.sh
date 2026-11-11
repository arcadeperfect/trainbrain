#!/usr/bin/env bash
# The web build: the layout app compiled to wasm, in a page (web/index.html),
# written to target/web. Then, optionally, deployed to Cloudflare Pages.
#
#   tools/web.sh            build into target/web
#   tools/web.sh serve      build, then serve it on http://localhost:8080
#   tools/web.sh deploy     build, then `wrangler pages deploy` (project
#                           $PAGES_PROJECT, default trainbrain)
#
# Needs: rustup target wasm32-unknown-unknown, and wasm-bindgen-cli at the
# version trainbrain-app pins (cargo install wasm-bindgen-cli --version 0.2.127).
# wasm-opt (binaryen) is used if it is on PATH. Deploying needs wrangler
# (npx wrangler) and a Cloudflare login (wrangler login).
set -euo pipefail
cd "$(dirname "$0")/.."

out=target/web
app=$(ls -d crates/*-app); app=${app#crates/}
bin=$(grep -A1 '^\[\[bin\]\]' "crates/$app/Cargo.toml" | sed -n 's/^name = "\(.*\)"$/\1/p')
cargo build --release -p "$app" --target wasm32-unknown-unknown
rm -rf "$out" && mkdir -p "$out"
wasm-bindgen --target web --no-typescript --out-dir "$out" --out-name layout \
  "target/wasm32-unknown-unknown/release/$bin.wasm"
if command -v wasm-opt >/dev/null; then
  wasm-opt -Oz --enable-bulk-memory --enable-nontrapping-float-to-int -o "$out/layout_bg.wasm" "$out/layout_bg.wasm"
fi
cp web/index.html "$out/"
ls -la "$out"

case "${1:-}" in
  serve) cd "$out" && python3 -m http.server 8080 ;;
  deploy) npx --yes wrangler pages deploy "$out" --project-name "${PAGES_PROJECT:-trainbrain}" --branch main --commit-dirty=true ;;
esac
