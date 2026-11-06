#!/bin/bash
# Runs the whole benchmark campaign: every (scenario, controller, repetition) in a fresh
# simulation, one after the other. Already completed runs are skipped, so an interrupted
# campaign can be resumed by running the same command again.
#
#   ros2 run mpc_benchmark run_benchmark.sh [output_dir]
#
# Environment overrides: REPS, CONTROLLERS, SCENARIOS, SIMULATOR, RUN_TIMEOUT.
set -u

OUT="${1:-${HOME}/mpc_benchmark_results}"
REPS="${REPS:-5}"
CONTROLLERS="${CONTROLLERS:-mpc_rti mpc_ltv baseline dwb rpp mppi}"
SCENARIOS="${SCENARIOS:-s1_tracking s2_waypoints s3_unknown_obstacles}"
SIMULATOR="${SIMULATOR:-gazebo}"
RUN_TIMEOUT="${RUN_TIMEOUT:-480}"
mkdir -p "${OUT}/logs"

# Processes that may survive a launch shutdown (e.g. the Nav2 container needs SIGKILL).
cleanup() {
  local pattern='^(/opt/ros/[a-z]+/lib/|gz sim|ruby .*gz sim)'
  for sig in INT TERM KILL; do
    pgrep -f -- "${pattern}" >/dev/null || break
    pkill -"${sig}" -f -- "${pattern}"
    sleep 2
  done
  rm -f /dev/shm/fastdds_* /dev/shm/sem.fastdds_* 2>/dev/null || true
}

total=0
failed=0
for scenario in ${SCENARIOS}; do
  for controller in ${CONTROLLERS}; do
    for rep in $(seq 1 "${REPS}"); do
      run_id="${scenario}__${controller}__${rep}"
      if [ -f "${OUT}/${run_id}_summary.csv" ]; then
        echo "[skip] ${run_id}"
        continue
      fi
      total=$((total + 1))
      echo "[run ] ${run_id}"
      start=$(date +%s)
      timeout --signal=INT --kill-after=30 "${RUN_TIMEOUT}" \
        ros2 launch mpc_benchmark benchmark.launch.py \
          controller:="${controller}" scenario:="${scenario}" run_id:="${run_id}" \
          output_dir:="${OUT}" simulator:="${SIMULATOR}" \
        > "${OUT}/logs/${run_id}.log" 2>&1
      code=$?
      cleanup
      if [ ! -f "${OUT}/${run_id}_summary.csv" ]; then
        failed=$((failed + 1))
        echo "[fail] ${run_id} (exit ${code}), see ${OUT}/logs/${run_id}.log"
      else
        echo "[done] ${run_id} in $(( $(date +%s) - start )) s"
      fi
    done
  done
done
echo "Finished: ${total} runs, ${failed} without results. Results in ${OUT}"
