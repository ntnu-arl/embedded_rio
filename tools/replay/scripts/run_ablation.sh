#!/usr/bin/env bash
# Build + run the baro×underweighting ablation, then plot it.
#
# Builds rio_replay 4× into separate build dirs (one per flag combo, via the
# RIO_BARO_AIDING / RIO_RADAR_UNDERWEIGHT CMake options), replays $LOG through
# each into a labeled run dir with a config.json, then calls compare_ablation.py.
#
# Required env:  LOG (input CSV)   ULG (PX4 .ulg, EKF2 reference / mocap sync)
# Optional env:  NAME (default = LOG basename)  SKIP (skip-seconds, default 0)
#               MOCAP (ROS1 bag → use as ground truth instead of EKF2)
#               MOCAP_OFFSET / MOCAP_FRAME (forwarded to compare_ablation.py)
#               NO_MOUNT (non-empty → --no-mount-correction: skip the q_m
#               mount tilt when transforming replay to NED)
#               PY (python, default .venv/bin/python3)  JOBS (build parallelism)
#
# Layout produced:  runs/<NAME>/ablation/<tag>/   (per-config data + config.json)
#                   runs/<NAME>/ablation/cmp/      (overlay + RMSE plots)
set -euo pipefail

REPO="${REPO:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)}"
cd "$REPO"

: "${LOG:?set LOG=path/to/input.CSV}"
: "${ULG:?set ULG=path/to/flight.ulg}"
NAME="${NAME:-$(basename "${LOG%.*}")}"
SKIP="${SKIP:-0}"
PY="${PY:-.venv/bin/python3}"
JOBS="${JOBS:-$(nproc 2>/dev/null || echo 4)}"
[ -x "$PY" ] || PY=python3

# tag | baro | underweight | label   (baseline first → gets the blue color)
combos=(
  "baro_uw|ON|ON|UBRIO"
  "baro_nouw|ON|OFF|BRIO"
  "nobaro_uw|OFF|ON|URIO"
  "none|OFF|OFF|RIO"
)

run_dirs=()
for combo in "${combos[@]}"; do
  IFS='|' read -r tag baro uw label <<< "$combo"
  bdir="build/replay_abl_${tag}"
  rdir="runs/${NAME}/ablation/${tag}"
  echo "── ablation [$label]  (baro=$baro underweight=$uw) ───────────────────"
  cmake -S tools/replay -B "$bdir" -DCMAKE_BUILD_TYPE=Release \
        -DRIO_BARO_AIDING="$baro" -DRIO_RADAR_UNDERWEIGHT="$uw" >/dev/null
  cmake --build "$bdir" -j "$JOBS" >/dev/null
  mkdir -p "$rdir"
  "$bdir/rio_replay" "$LOG" "$rdir" 2>&1 | tee "$rdir/run.log"
  baro_json=$([ "$baro" = ON ] && echo true || echo false)
  uw_json=$([ "$uw" = ON ] && echo true || echo false)
  cat > "$rdir/config.json" <<EOF
{
  "label": "$label",
  "baro": $baro_json,
  "underweight": $uw_json,
  "log": "$LOG",
  "ulg": "$ULG"
}
EOF
  run_dirs+=("$rdir")
done

echo "── plotting ablation ─────────────────────────────────────────────────"
plot_args=(--ulg "$ULG" --skip-seconds "$SKIP" --out "runs/${NAME}/ablation/cmp")
[ -n "${NO_MOUNT:-}" ] && plot_args+=(--no-mount-correction)
if [ -n "${MOCAP:-}" ]; then
  plot_args+=(--mocap "$MOCAP")
  [ -n "${MOCAP_OFFSET:-}" ] && plot_args+=(--mocap-offset "$MOCAP_OFFSET")
  [ -n "${MOCAP_FRAME:-}" ]  && plot_args+=(--mocap-body-frame "$MOCAP_FRAME")
fi
"$PY" tools/replay/scripts/compare_ablation.py "${run_dirs[@]}" "${plot_args[@]}"
