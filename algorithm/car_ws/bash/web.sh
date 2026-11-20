#!/usr/bin/env bash
# Sim2Real V13 four-mode launcher. Original `car` shortcut is unchanged.
# sim: browser-only; test: Mock MCU -> ROS2 -> HTTP; real: verified serial -> ROS2 -> HTTP.
# keyboard: existing WASDQE ROS2 local loopback; actual STM32 motor TX stays disabled.
set -Eeuo pipefail
HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
WS="$(cd -- "$HERE/.." && pwd -P)"
REPO="$(cd -- "$WS/../.." && pwd -P)"
SIM="${CAR_SIM2REAL_ROOT:-$REPO/sim2real}"
CMD="${1:-doctor}"
if (($#)); then shift; fi

require_site() {
  [[ -f "$SIM/launcher.py" && -f "$SIM/server.py" && -f "$SIM/web_mode_gateway.py" \
     && -f "$SIM/web_mode_runner.py" && -f "$SIM/web/index.html" \
     && -f "$SIM/web/mode_router.js" ]] || {
    echo "[ERROR] Four-mode Sim2Real files missing under $SIM. Apply patch package first." >&2; return 2;
  }
}
source_ros() {
  [[ -f /opt/ros/humble/setup.bash ]] || { echo '[ERROR] ROS2 Humble not found at /opt/ros/humble' >&2; return 2; }
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
  test|real)
    require_site
    source_ros
    exec /usr/bin/python3 "$SIM/web_mode_runner.py" "$CMD" "$@"
    ;;
  keyboard)
    require_site
    echo '[KEYBOARD] Local /keyboard opens existing ROS2 PC pub+sub; no STM32 motor TX.'
    echo "[PAGE] Open http://127.0.0.1:${CAR_PORT:-8080}/keyboard after site starts."
    exec /usr/bin/python3 "$SIM/launcher.py" "$@"
    ;;
  build)
    source_ros
    cd "$WS"
    exec colcon build --packages-up-to car_web_test car_web_real car_web_sim "$@"
    ;;
  doctor)
    require_site
    echo "[PASS] Standalone Sim2Real: $SIM"
    [[ -f "$SIM/keyboard_local_ros2_autostart.py" && -f "$SIM/keyboard_mapping_core.py" && -f "$SIM/web/keyboard_preview_motion.js" ]] || {
      echo '[FAIL] Keyboard mapping/autostart/animation files missing' >&2; exit 3;
    }
    for pkg in car_web_test car_web_real car_web_sim; do
      [[ -f "$WS/src/$pkg/package.xml" && -f "$WS/src/$pkg/CMakeLists.txt" ]] || {
        echo "[FAIL] ROS2 adapter package incomplete: $pkg" >&2; exit 3;
      }
      echo "[PASS] ROS2 adapter: $pkg"
    done
    [[ -f "$WS/src/car_web_keyboard/COLCON_IGNORE" ]] && echo '[INFO] Keyboard ROS2 PC loopback is standalone; STM32 motor TX disabled'
    /usr/bin/python3 -m py_compile "$SIM/server.py" "$SIM/web_mode_gateway.py" "$SIM/web_mode_runner.py" "$SIM/ros2_visualization_bridge.py" "$SIM/keyboard_local_ros2_autostart.py" "$SIM/keyboard_mapping_core.py" "$SIM/keyboard_ros2_pc.py" "$SIM/keyboard_ros2_web_feedback.py"
    /usr/bin/python3 -m py_compile "$SIM/test_inspection.py" "$SIM/test_telemetry_bridge.py"
    echo '[PASS] Python syntax; original car shortcut unaffected'
    echo '[INFO] doctor does not validate physical STM32, laser, OpenMV or ROS2 runtime.'
    ;;
  tree)
    echo "Standalone web frontend: $SIM/web"
    printf '%s\n' '  /sim      (browser-only simulation)' '  /test     (Mock ROS2 bridge)' \
      '  /real     (hardware ROS2 telemetry)' '  /keyboard (local PC ROS2 WASDQE mapping; no STM32 TX)'
    for pkg in car_web_test car_web_real car_web_sim car_web_keyboard; do
      ls -ld "$WS/src/$pkg" 2>/dev/null || true
    done
    ;;
  *)
    echo 'Usage: bash web.sh {doctor|build|sim|test|real|keyboard|tree} [args]' >&2
    exit 2
    ;;
esac
