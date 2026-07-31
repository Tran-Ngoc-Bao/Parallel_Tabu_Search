#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BIN="${SCRIPT_DIR}/target/release/sequence-tabu-search"

DEFAULT_DATA_PREFIX="50"
PROBLEM_FILE="${1:-${SCRIPT_DIR}/../../data/soict-2025/${DEFAULT_DATA_PREFIX}.40.2.txt}"

OUTPUTS_DIR="${OUTPUTS_DIR:-outputs}"
COMPACT_OUTPUT="${COMPACT_OUTPUT:-1}"
RUN_ID="${RUN_ID:-}"

CMD=(
  "${BIN}" run
  "${PROBLEM_FILE}"
  --outputs "${OUTPUTS_DIR}"
)

if [ "${COMPACT_OUTPUT}" = "1" ]; then
  CMD+=(--compact-output)
fi

if [ -n "${RUN_ID}" ]; then
  CMD+=(--run-id "${RUN_ID}")
fi

"${CMD[@]}"
