#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
DOCS_DIR="${REPO_ROOT}/docs"

if ! /usr/bin/python3 -c "import sphinx, furo" >/dev/null 2>&1; then
  echo "Missing Sphinx/Furo. Install with:" >&2
  echo "  /usr/bin/python3 -m pip install --user -r ${DOCS_DIR}/requirements.txt" >&2
  exit 1
fi

exec /usr/bin/python3 -m sphinx -b html \
  "${DOCS_DIR}/source" \
  "${DOCS_DIR}/build/html"
