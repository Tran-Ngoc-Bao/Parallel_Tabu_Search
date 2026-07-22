#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BENCHMARK_RUN="${SCRIPT_DIR}/benchmark_runs.sh"

while read -r prefix ai seg strat minpull pool rand prefer_pulled workers; do
  echo "RUN CASE: $prefix $ai $seg $strat $minpull $pool $rand $prefer_pulled $workers"

  DEFAULT_DATA_PREFIX="$prefix" \
  ADAPTIVE_ITERATIONS="$ai" \
  ADAPTIVE_PULL_ELITE_SEGMENTS="$seg" \
  ELITE_PULL_STRATEGY="$strat" \
  MIN_PULL_ELITES_PER_WORKER_FACTOR="$minpull" \
  ELITE_POOL_FACTOR="$pool" \
  RANDOMIZE_WORKER_HYPERPARAMS="$rand" \
  PREFER_PULLED="$prefer_pulled" \
  NUM_WORKERS="$workers" \
  OUTPUTS_DIR="${SCRIPT_DIR}/outputs/num-workers/${strat}-factor${minpull}-np${workers}" \
  bash "${BENCHMARK_RUN}" </dev/null
done <<'EOF'
200 12 4 rank 10 0.02 0 1 4
200 12 4 rank 10 0.02 0 1 5
200 12 4 rank 10 0.02 0 1 6
200 12 4 rank 10 0.02 0 1 7
200 12 4 rank 10 0.02 0 1 8
200 12 4 rank 10 0.02 0 1 9
200 12 4 rank 10 0.02 0 1 10
EOF