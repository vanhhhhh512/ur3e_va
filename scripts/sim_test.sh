#!/usr/bin/env bash
# Kiem tra toan bo chuoi ve ma khong can man hinh that (dung man hinh ao Xvfb).
#
#   1. stroke_preview  -> kiem tra bo tri + xuat quy dao du dinh (khong can Gazebo)
#   2. bringup_sim     -> Ignition headless + MoveIt + RViz tren man hinh ao
#   3. draw            -> ve ba hinh, ghi vet but thuc te ra CSV
#   4. anh chup RViz + video + do sai so bam quy dao
#
# Can: xvfb, ffmpeg, imagemagick (sudo apt install -y xvfb ffmpeg imagemagick).
# Ket qua nam trong thu muc OUT (mac dinh <repo>/out).
# Khong bat 'set -u': cac file setup.bash cua ROS tham chieu bien chua dat.
set -o pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUT="${OUT:-$REPO/out}"
GAZEBO_GUI="${GAZEBO_GUI:-false}"
RECORD="${RECORD:-true}"
SCREEN="1600x1000"

source /opt/ros/humble/setup.bash
if [ ! -f "$REPO/install/setup.bash" ]; then
  echo "!! Chua build workspace nen khong co package nao de chay."
  echo "   Chay hai lenh nay truoc:"
  echo "     cd $REPO"
  echo "     colcon build --symlink-install"
  exit 1
fi
source "$REPO/install/setup.bash"
mkdir -p "$OUT"

cleanup() {
  [ -n "${FFMPEG_PID:-}" ] && kill -INT "$FFMPEG_PID" 2>/dev/null
  [ -n "${BRINGUP_PID:-}" ] && kill -INT "$BRINGUP_PID" 2>/dev/null
  sleep 3
  pkill -f "ign gazebo" 2>/dev/null
  pkill -f ruby 2>/dev/null
  [ -n "${XVFB_PID:-}" ] && kill "$XVFB_PID" 2>/dev/null
  return 0
}
trap cleanup EXIT

wait_for() {  # $1 mo ta | $2 lenh kiem tra | $3 timeout giay
  local waited=0
  until eval "$2" >/dev/null 2>&1; do
    sleep 2
    waited=$((waited + 2))
    if [ "$waited" -ge "$3" ]; then
      echo "!! TIMEOUT sau ${3}s khi cho: $1"
      return 1
    fi
  done
  echo ">> San sang sau ${waited}s: $1"
}

echo "===== 1/4 Kiem tra bo tri cac hinh ====="
ros2 run ur_pen_plotter stroke_preview --ros-args \
  --params-file "$REPO/src/ur_pen_plotter/config/canvas.yaml" \
  -p planned_csv:="$OUT/planned.csv" 2>&1 | tee "$OUT/preview.log"
PREVIEW_RC=${PIPESTATUS[0]}
if [ "$PREVIEW_RC" -ne 0 ]; then
  echo "!! Bo tri khong hop le, dung lai."
  exit "$PREVIEW_RC"
fi

echo "===== 2/4 Khoi dong mo phong (Ignition headless + MoveIt + RViz) ====="
mkdir -p /tmp/.X11-unix
Xvfb :99 -screen 0 "${SCREEN}x24" >/dev/null 2>&1 &
XVFB_PID=$!
export DISPLAY=:99
export LIBGL_ALWAYS_SOFTWARE="${LIBGL_ALWAYS_SOFTWARE:-1}"
sleep 2

ros2 launch ur_pen_plotter bringup_sim.launch.py \
  gazebo_gui:="$GAZEBO_GUI" launch_rviz:=true > "$OUT/bringup.log" 2>&1 &
BRINGUP_PID=$!

wait_for "node move_group" "ros2 node list | grep -q move_group" 180 || exit 1
wait_for "action follow_joint_trajectory" \
  "ros2 action list | grep -q joint_trajectory_controller/follow_joint_trajectory" 180 || exit 1
sleep 5

if [ "$RECORD" = "true" ]; then
  # preset ultrafast + 10 fps de viec ghi man hinh khong tranh CPU voi mo phong.
  ffmpeg -nostdin -loglevel error -y -f x11grab -video_size "$SCREEN" -framerate 10 -i :99 \
    -c:v libx264 -preset ultrafast -crf 28 -pix_fmt yuv420p -threads 2 "$OUT/demo.mp4" &
  FFMPEG_PID=$!
fi

echo "===== 3/4 Ve hinh tron, chu V va chu A ====="
ros2 launch ur_pen_plotter draw.launch.py \
  trail_csv:="$OUT/trail.csv" 2>&1 | tee "$OUT/draw.log"
DRAW_RC=${PIPESTATUS[0]}

sleep 3
import -window root "$OUT/rviz.png" 2>/dev/null || echo "(khong chup duoc man hinh)"
if [ -n "${FFMPEG_PID:-}" ]; then
  kill -INT "$FFMPEG_PID" 2>/dev/null
  wait "$FFMPEG_PID" 2>/dev/null
  FFMPEG_PID=""
fi

echo "===== 4/4 Doi chieu quy dao du dinh vs vet but thuc te ====="
if [ -s "$OUT/trail.csv" ]; then
  python3 "$REPO/scripts/plot_trail.py" --planned "$OUT/planned.csv" --actual "$OUT/trail.csv" \
    --out "$OUT/ban_ve.png" 2>&1 | tee "$OUT/sai_so.log"
else
  echo "!! Khong co vet but nao duoc ghi lai."
  DRAW_RC=1
fi

echo "===== Ket qua nam trong $OUT ====="
ls -la "$OUT"
exit "$DRAW_RC"
