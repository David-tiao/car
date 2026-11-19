#!/usr/bin/env bash
# Four independent groups; Sim2Real is an EXTERNAL standalone project.
# This script never moves or changes ~/Desktop/car/sim2real.
set -Eeuo pipefail
HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
WS="$(cd -- "$HERE/.." && pwd -P)"
REPO="$(cd -- "$WS/../.." && pwd -P)"
SIM="${CAR_SIM2REAL_ROOT:-$REPO/sim2real}"
CMD="${1:-doctor}"
if (($#)); then shift; fi

require_site() {
  [[ -d "$SIM" && -f "$SIM/launcher.py" && -f "$SIM/server.py" && -f "$SIM/web/index.html" ]] || {
    echo "[ERROR] Standalone Sim2Real site missing: $SIM" >&2; return 2;
  }
}
source_ros() {
  [[ -f /opt/ros/humble/setup.bash ]] || { echo '[ERROR] ROS2 Humble not found' >&2; return 2; }
  local had_u=0
  case "$-" in *u*) had_u=1;; esac
  set +u
  source /opt/ros/humble/setup.bash
  [[ ! -f "$WS/install/setup.bash" ]] || source "$WS/install/setup.bash"
  if ((had_u)); then set -u; fi
}
case "$CMD" in
  sim|site)
    require_site
    exec /usr/bin/python3 "$SIM/launcher.py" "$@"
    ;;
  test)
    source_ros
    echo '[TEST] Mock MCU -> CAR-LINK -> ROS2; web test API is NOT yet connected.'
    echo '[TEST] No motor commands; not considered hardware telemetry.'
    exec ros2 launch car_serial_link offline_serial_demo.launch.xml "$@"
    ;;
  real)
    require_site
    source_ros
    if ros2 node list -a 2>/dev/null | grep -Eq '(^|/)car_serial_mock_mcu($|[[:space:]])'; then
      echo '[ERROR] Mock STM32 node detected; stop it before hardware-only monitoring.' >&2
      exit 3
    fi
    echo '[REAL] READ ONLY: ROS2 -> existing Sim2Real HTTP API.'
    echo '[REAL] Source provenance still requires human verification; no motor control.'
    exec /usr/bin/python3 "$SIM/ros2_visualization_bridge.py" \
      --plan /car/navigation/preview_path \
      --road-path /competition/mission_road_path "$@"
    ;;
  build)
    [[ -f /opt/ros/humble/setup.bash ]] || { echo '[ERROR] ROS2 Humble not found' >&2; exit 2; }
    set +u
    source /opt/ros/humble/setup.bash
    set -u
    cd "$WS"
    exec colcon build --packages-up-to car_web_test car_web_real car_web_sim "$@"
    ;;
  doctor)
    require_site
    echo "[PASS] External Sim2Real remains at: $SIM"
    if [[ -L "$REPO/sim2real" ]]; then
      echo '[WARN] sim2real is still a symlink. Run organizer v2 for a physical directory.' >&2
      exit 3
    fi
    for pkg in car_web_test car_web_real car_web_sim; do
      [[ -f "$WS/src/$pkg/package.xml" && -f "$WS/src/$pkg/CMakeLists.txt" ]] || {
        echo "[FAIL] ROS2 adapter package incomplete: $pkg" >&2; exit 3;
      }
      echo "[PASS] ROS2 adapter: $pkg"
    done
    [[ -f "$WS/src/car_web_keyboard/COLCON_IGNORE" ]] && echo '[PASS] Keyboard group ignored (not implemented)'
    echo '[INFO] This check does not claim real hardware is connected.'
    ;;
  tree)
    echo "Sim2Real standalone: $SIM"
    echo 'ROS2 integration packages:'
    for pkg in car_web_test car_web_real car_web_sim car_web_keyboard; do
      ls -ld "$WS/src/$pkg" 2>/dev/null || true
    done
    ;;
  *)
    echo 'Usage: web.sh {doctor|build|sim|test|real|tree} [arguments]' >&2
    exit 2
    ;;
esac
