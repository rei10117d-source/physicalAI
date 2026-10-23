#!/usr/bin/env bash
# Run Gazebo + AI, record /odom, and plot the resulting path.
set -Eeuo pipefail
ROOT="$HOME/pai_ws"
TOOLS="$HOME/pai_trajectory_tools"
WORLD="$ROOT/src/pai_nav/worlds/obstacle_course.sdf"
MODE="${1:-ai}"
if [[ "$MODE" != "ai" && "$MODE" != "baseline" ]]; then
  echo "Usage: bash run_ai_test.sh [ai|baseline]" >&2; exit 2
fi
for file in "$TOOLS/record_trajectory.py" "$TOOLS/plot_trajectory.py" "$WORLD" "$ROOT/install/setup.bash"; do
  if [[ ! -f "$file" ]]; then echo "Missing: $file" >&2; exit 2; fi
done
set +u
source /opt/ros/humble/setup.bash
source "$ROOT/install/setup.bash"
mkdir -p "$ROOT/results"
RUN_ID="${MODE}_$(date +%Y%m%d_%H%M%S)"
CSV="$ROOT/results/${RUN_ID}.csv"
PNG="$ROOT/results/${RUN_ID}.png"
RECLOG="$ROOT/results/${RUN_ID}_recorder.log"
LAUNCHLOG="$ROOT/results/${RUN_ID}_launch.log"
rec_pid=''
launch_pid=''
cleanup() {
  trap - INT TERM EXIT
  if [[ -n "$launch_pid" ]] && kill -0 "$launch_pid" 2>/dev/null; then
    kill -INT "$launch_pid" 2>/dev/null || true
    sleep 2
    kill -TERM "$launch_pid" 2>/dev/null || true
    wait "$launch_pid" 2>/dev/null || true
  fi
  if [[ -n "$rec_pid" ]] && kill -0 "$rec_pid" 2>/dev/null; then
    kill -INT "$rec_pid" 2>/dev/null || true
    sleep 1
    kill -TERM "$rec_pid" 2>/dev/null || true
    wait "$rec_pid" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM
printf 'Experiment: %s\nCSV: %s\n' "$RUN_ID" "$CSV"
python3 "$TOOLS/record_trajectory.py" --output "$CSV" >"$RECLOG" 2>&1 &
rec_pid=$!
sleep 2
ros2 launch pai_nav demo.launch.py "mode:=$MODE" >"$LAUNCHLOG" 2>&1 &
launch_pid=$!
# Allow for slow WSL/Gazebo startup; exit on goal, launch failure, or timeout.
START=$(date +%s)
LIMIT="${PAI_MAX_WALL_SECONDS:-900}"
while kill -0 "$rec_pid" 2>/dev/null; do
  if ! kill -0 "$launch_pid" 2>/dev/null; then
    echo 'ROS launch ended before recorder finished.' >&2; break
  fi
  if (( $(date +%s) - START > LIMIT )); then
    echo "Timeout after ${LIMIT} wall-clock seconds." >&2; break
  fi
  sleep 2
done
# Capture the result before stopping Gazebo.
if grep -q 'GOAL DETECTED:' "$RECLOG"; then
  echo 'Goal reached according to /odom recorder.'
else
  echo 'Goal not confirmed; graph will show partial trajectory.'
fi
cleanup
trap - EXIT INT TERM
if [[ ! -s "$CSV" ]]; then echo 'No CSV produced.' >&2; exit 1; fi
python3 "$TOOLS/plot_trajectory.py" "$CSV" --world "$WORLD" --output "$PNG"
echo "CSV: $CSV"
echo "PNG: $PNG"
echo "Recorder log: $RECLOG"
echo "Navigation log: $LAUNCHLOG"
echo 'Run done. Check the navigation log for GOAL REACHED too.'
