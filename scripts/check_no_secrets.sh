#!/usr/bin/env bash
set -euo pipefail

ROOT="${1:-.}"
cd "$ROOT"

fail=0

# Files that should never exist in a public tree (examples/docs excluded by name).
while IFS= read -r -d '' f; do
  case "$f" in
    */.git/*|*/.venv/*|*/venv/*) continue ;;
    */python/certs/.gitignore|*/python/certs/README.md|*/.env.example) continue ;;
  esac
  echo "ERROR: sensitive-looking file present: $f" >&2
  fail=1
done < <(find . -type f \( \
  -name '*.key' -o -name '*.pem' -o -name '*.crt' -o -name '*.p12' -o \
  -name '*.pfx' -o -name '.env' -o -name '.env.*' \
\) -print0 2>/dev/null)

# Private-key headers and common AWS access-key IDs.
if grep -RInE \
  --exclude-dir=.git --exclude-dir=.venv --exclude-dir=venv \
  --exclude='.env.example' \
  '-----BEGIN ([A-Z0-9 ]+ )?PRIVATE KEY-----|AKIA[0-9A-Z]{16}|ASIA[0-9A-Z]{16}' \
  . 2>/dev/null; then
  echo "ERROR: private-key or AWS access-key pattern detected." >&2
  fail=1
fi

# Account-specific IoT endpoints should not be committed to reusable public source.
# Placeholder endpoints containing YOUR_ are allowed.
if grep -RInE \
  --exclude-dir=.git --exclude-dir=.venv --exclude-dir=venv \
  --exclude='.env.example' \
  '[a-zA-Z0-9-]{8,}-ats\.iot\.[a-z0-9-]+\.amazonaws\.com' \
  . 2>/dev/null | grep -v 'YOUR_'; then
  echo "ERROR: account-specific AWS IoT endpoint detected." >&2
  fail=1
fi

if [[ "$fail" -ne 0 ]]; then
  echo "Secret-safety scan FAILED." >&2
  exit 1
fi

echo "Secret-safety scan passed."
