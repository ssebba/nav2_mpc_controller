#!/bin/bash
# Records the README GIF: benchmark scenario S3 (three boxes on the route, unknown to the
# planner) driven by the MPC. Gazebo runs headless; RViz renders in a virtual X display with
# software OpenGL, captured with ffmpeg and converted to an optimized GIF.
#
#   tools/record_demo.sh [output.gif] [controller] [duration_s]
#
# Needs Xvfb and ffmpeg; run from a sourced workspace.
set -euo pipefail

OUT="${1:-docs/media/demo.gif}"
CONTROLLER="${2:-mpc_rti}"
DURATION="${3:-36}"
WORK="$(mktemp -d)"
mkdir -p "$(dirname "${OUT}")"

DISPLAY_NUM=99
while [ -e "/tmp/.X${DISPLAY_NUM}-lock" ]; do
  DISPLAY_NUM=$((DISPLAY_NUM + 1))
done
Xvfb ":${DISPLAY_NUM}" -screen 0 1280x720x24 -nolisten tcp +extension GLX \
  > "${WORK}/xvfb.log" 2>&1 &
XVFB_PID=$!
cleanup() {
  kill -INT "${RVIZ_PID:-0}" "${LAUNCH_PID:-0}" 2>/dev/null || true
  sleep 5
  pkill -KILL -f '^(/opt/ros/[a-z]+/lib/|gz sim|ruby .*gz sim)' 2>/dev/null || true
  kill "${XVFB_PID}" 2>/dev/null || true
}
trap cleanup EXIT

# The scenario runner sends the goal as soon as Nav2 is active.
ros2 launch mpc_benchmark benchmark.launch.py \
  scenario:=s3_unknown_obstacles controller:="${CONTROLLER}" output_dir:="${WORK}" \
  use_rviz:=False headless:=True > "${WORK}/launch.log" 2>&1 &
LAUNCH_PID=$!

# RViz on its own (a crash must not stop the simulation): X11 platform, no Wayland, no GPU.
env -u WAYLAND_DISPLAY DISPLAY=":${DISPLAY_NUM}" QT_QPA_PLATFORM=xcb LIBGL_ALWAYS_SOFTWARE=1 \
  ros2 run rviz2 rviz2 --fullscreen \
  -d "$(ros2 pkg prefix mpc_bringup)/share/mpc_bringup/rviz/mpc_record_s3.rviz" \
  --ros-args -p use_sim_time:=true > "${WORK}/rviz.log" 2>&1 &
RVIZ_PID=$!

echo "Waiting for the robot to move..."
until ros2 topic echo --once /cmd_vel geometry_msgs/msg/TwistStamped 2>/dev/null \
  | grep -A1 'linear:' | grep -qE 'x: [0-9.]*[1-9]'; do
  sleep 1
done

ffmpeg -loglevel error -y -f x11grab -draw_mouse 0 -framerate 15 -video_size 1280x720 \
  -i ":${DISPLAY_NUM}" -t "${DURATION}" "${WORK}/demo.mp4"

# Two-pass palette GIF, 10 fps, 800 px wide, sped up 2x.
ffmpeg -loglevel error -y -i "${WORK}/demo.mp4" \
  -vf "setpts=0.5*PTS,fps=10,scale=800:-1:flags=lanczos,palettegen=stats_mode=diff" "${WORK}/palette.png"
ffmpeg -loglevel error -y -i "${WORK}/demo.mp4" -i "${WORK}/palette.png" \
  -lavfi "setpts=0.5*PTS,fps=10,scale=800:-1:flags=lanczos[x];[x][1:v]paletteuse=dither=bayer:bayer_scale=5" \
  "${OUT}"
echo "Wrote ${OUT} ($(du -h "${OUT}" | cut -f1))"
