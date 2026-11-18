SIM2REAL OPERATOR CONSOLE V13.0.0 — CHASSIS DIAGNOSTICS & MANUAL TRAINING
=============================================================================

WHAT'S NEW
- Two main modes retained: SIMULATION and LIVE. LIVE is read-only.
- Two fully collapsible workspaces near the top: MANUAL TRAINING and CHASSIS DIAGNOSTICS.
- Manual training is SIMULATION ONLY. Click a safe map point to plan and move;
  W/A/S/D translate, Q/E rotate, G toggle gripper, R extend/retract, SPACE stop.
  No actual STM32, ROS2 /cmd_vel or HTTP command endpoint is connected.
- Real-time analytics compares map-aligned /odom position against planned
  route polyline: cross-track RMS/P95/max, heading RMS and a clearly-labelled
  simulated reference speed comparison. This is NOT absolute position accuracy.
- Full original uploaded 'kart_log_view_mecanum_v4(1).html' retained under
  web/tools/mecanum_debugger.html (six original tabs, all waveform/PID/quality
  tools preserved). Supports importing original ULog43 CSV as before.
- Realtime /odom samples can be exported as named-column CSV or sent to the
  embedded legacy debugger. Missing wheel/PID channels remain missing.
- ROS2 navigation view and camera dashboard can be collapsed to reduce clutter.
- No new server dependency. Only browser JS and CSS added.

INSTALL INTO THE USER'S NEW DESKTOP LOCATION (BACK UP FIRST IF CUSTOMIZED)
  # Stop the old 'car' server with Ctrl+C first.
  cd ~/Downloads
  python3 -m zipfile -e competition_simulator_server_v13.zip .
  mkdir -p "$HOME/Desktop/car/sim2real"
  cp -a competition_simulator_server/. "$HOME/Desktop/car/sim2real/"
  bash "$HOME/Desktop/car/sim2real/start_server.sh"

EXISTING 'car' ALIAS SHOULD POINT TO:
  car() { bash "$HOME/Desktop/car/sim2real/start_server.sh"; }

OPEN: http://127.0.0.1:8080/?build=13.0.0
CHECK: curl -s http://127.0.0.1:8080/api/build

REAL VEHICLE DATA FLOW
  STM32F4 -> Pi5 serial protocol parser (NOT YET PROVIDED) -> Pi5 ROS2 /odom
  -> Ubuntu DDS -> start_ros_visualization.sh -> localhost POST /api/telemetry
  -> browser 140ms polling -> LIVE monitor and realtime error comparison.
  Do not run both visualization and legacy odom bridges simultaneously.
  OpenMV and Pi5 cameras upload frames to the existing authenticated APIs.

LIMITATIONS / SAFETY
- Manual controls are restricted to browser-only simulation; physical motion
  requires protocol review, emergency stop, command timeout, safety limits,
  on-site supervision, and a separate explicitly authorized controller.
- Competition rules prohibit remote control while the robot is running.
- Error measurements require matching map frames. Without map->odom
  localization, odometry can drift. The browser applies only initial alignment.
- Rotation swept footprint and extension-arm collisions are still simplified.
- Live data channels do not magically contain wheel speeds, PID terms or
  mechanical actuator feedback. The legacy analyzer shows missing fields.
- Preserve data/shared_scene.json and data/camera_token.txt during upgrades.

--- PREVIOUS V12 README BELOW ---

COMPETITION CAR SIMULATOR SERVER V12.0.0 — ROS2 / NAV2 / LIDAR DASHBOARD
===========================================================================

WHAT WAS ADDED
- Built-in RViz-style 2D navigation viewer within the existing SIMULATION / LIVE website.
- Browser layers: /map, /scan, /global_costmap/costmap, /local_costmap/costmap,
  /plan, /local_plan, /competition/road_path, /competition/road_graph,
  /competition_start_zones, TF map->odom->base_link->laser_frame, /odom, /cmd_vel,
  /diagnostics and /navigate_to_pose/_action/status.
- ROS2 topic discovery, message rates and TF availability readouts.
- User can switch between world map view (TF required) and raw laser-frame view
  (works with /scan alone); frame-mismatched data is NOT forcibly overlaid.
- The bridge is READ ONLY and never commands a robot or starts Nav2.
- The entire website and all download / script filenames remain in ENGLISH.
- Original Stage 6 RViz2 config from uploaded algorithm.zip included as a fallback
  for native desktop RViz2. Browser viewer is NOT a complete 3D RViz2 replacement.

FIRST UPDATE / INSTALL ON UBUNTU
  cd ~/Downloads && python3 -m zipfile -e competition_simulator_server_v12.zip . && bash competition_simulator_server/start_server.sh

SUBSEQUENT WEBSITE START
  car

BROWSER
  http://127.0.0.1:8080/?build=12.0.0
  On local Wi-Fi, use the IP printed by the Ubuntu web server, not 127.0.0.1.

CONNECT REAL ROS2 DATA (SECOND TERMINAL ON UBUNTU)
  source /opt/ros/humble/setup.bash
  ros2 topic list -t
  cd ~/Downloads/competition_simulator_server
  bash start_ros_visualization.sh

The new script subscribes to ROS2 topics and also forwards REAL /odom to the
existing website telemetry interface (max 15 Hz). You usually do NOT need to
run start_ros_bridge.sh at the same time: it can duplicate samples.
The simulator does not launch a lidar driver, map server, SLAM, or Nav2.
Those nodes must already be running on the Pi5 and/or Ubuntu ROS2 network.

CUSTOM TOPIC NAMES (EXAMPLES)
  bash start_ros_visualization.sh --scan /scan --map /map \
    --global-costmap /global_costmap/costmap --local-costmap /local_costmap/costmap \
    --plan /plan --local-plan /local_plan --odom /odom

  bash start_ros_visualization.sh --help

NETWORK PI5 -> UBUNTU ROS2
  Both machines must share compatible ROS_DOMAIN_ID, ROS_LOCALHOST_ONLY=0,
  same ROS2 communication setup and working DDS discovery over the network.
  Verify: ros2 topic echo /scan --once
          ros2 topic echo /odom --once
          ros2 topic echo /map --once --qos-durability transient_local
  Check TF: ros2 run tf2_ros tf2_echo map base_link
            ros2 run tf2_ros tf2_echo base_link laser_frame

BROWSER INSTRUCTIONS
  Open the ROS2 navigation and lidar section near the TOP of the webpage.
  Choose world map or local laser-frame view, toggle grid layers and paths,
  and inspect topic Hz / age / node graph / diagnostics.
  If there is no map->laser_frame TF, laser points are NEVER drawn in map view.
  Select local scan view to inspect their raw sensor-frame shape instead.
  If /odom exists but map->base_link is absent, the robot pose in the browser
  map viewer remains hidden; the separate LIVE odom panel can still show
  odometry relative to its own initial field registration.

WEBSITE API CHECKS
  curl -s http://127.0.0.1:8080/api/build
  curl -s http://127.0.0.1:8080/api/ros/state
  curl -s http://127.0.0.1:8080/api/ros/grid/map
  curl -s http://127.0.0.1:8080/api/telemetry

Only local ROS bridge software can POST to the ROS APIs. Other web visitors
may READ live ROS data and cameras; use trusted networks, do not expose
private telemetry or video to untrusted public tunnels.

ORIGINAL DESKTOP RVIZ2 (OPTIONAL)
  cd ~/Downloads/competition_simulator_server
  bash start_native_rviz.sh

The native profile is copied from your Stage 6 uploaded algorithm.zip and can
render native RViz2 MarkerArray plugins and further 3D plugins; it is a separate
window. Browser V12 supports basic 2D markers and Nav2 navigation layers,
not RViz2 3D RobotModel, PointCloud2, URDF and arbitrary plugin rendering.

LIMITATIONS
- No actual Pi5/STM32/lidar connected during package validation.
- No control commands are published; ROS2 /cmd_vel is only displayed as a
  commanded velocity, NOT treated as measured chassis movement.
- This bridge sends periodic full-grid snapshots when /map or costmap updates
  are received. Nav2 incremental OccupancyGridUpdate-only topics are not yet
  reconstructed; standard full OccupancyGrid messages are supported.
- LaserScan points are sampled (up to 720), global / local paths (up to 500)
  and marker sets are reduced for browser bandwidth.
- Missing or incompatible TF is explicitly shown. ROS2 /odom is not
  equivalent to global map pose until localization supplies map->odom.
- You can adjust command-line topic names if your launch files remap them.

---- PREVIOUS V11 INSTRUCTIONS BELOW ----

V11.0.0 — TWO RUN MODES + TWO LIVE CAMERAS + SETTINGS DIRECTORY

- Top bar: choose SIMULATION or LIVE ROS2 ODOM.
- Sidebar: 9 directory buttons. Click to show only that settings group.
- SIMULATION displays planned motion and material-handling animation.
- LIVE mode displays real /odom velocity, pose, yellow actual track and two real camera feeds. Does not command STM32F4.
- Simulator never treats /cmd_vel commands as measured motion.
- Existing 5x5 numbered grid, two toggle obstacles, A* / Dijkstra / D* Lite, QR mission, rotor and manipulator controls are retained.

UBUNTU INSTALL / REPLACE
  cd ~/Downloads && python3 -m zipfile -e competition_simulator_server_v11.zip . && bash competition_simulator_server/start_server.sh
After installation: car
Web browser: http://127.0.0.1:8080/?build=11.0.0
Check: curl -s http://127.0.0.1:8080/api/build

LIVE DATA
1. Make sure actual ROS2 nav_msgs/msg/Odometry messages are published to /odom.
2. Start bridge in separate terminal:
  cd ~/Downloads/competition_simulator_server && bash start_ros_bridge.sh
3. Select LIVE mode at top of page.
4. A fresh telemetry stream (not replay data) is required to display online status.
5. Map location requires initial pose calibration. Encoders alone can drift.

IMPORTANT
- Both modes are VIEW / SIMULATION ONLY. No motor control is sent.
- Per-client live/simulation selection is local. Shared obstacle scene remains synchronized.
- The server allows local telemetry POST and other LAN users to read.
- Existing scene saved under data/shared_scene.json is not overwritten by packaging.

V9.0.0 - FIXED 5x5 SNAKE-NUMBERED GRID (0-24)

The grid follows the user-supplied reference image:
- 2400 x 2400 mm field, five columns x five rows, cell 480 x 480 mm.
- Bottom row #0 to #4; next row #9 to #5; middle #10 to #14; next #19 to #15; top #20 to #24.
- Each click toggles one occupied cell; two obstacles maximum.
- Yellow static restricted zones overlay the grid; occupied red cells are blocked for routing.
- A* / Dijkstra / D* Lite and the full mission route are retained.
- Live ROS2 /odom bridge, staged handling, material stack simulation and rotor controls are retained.
- Fine 50 mm navigation grid is retained for vehicle footprint clearance, regardless of visible 480 mm cell.
- Real field orientation remains the official uploaded V5 drawing; only the grid topology follows the reference.

START (Ubuntu):
cd ~/Downloads && python3 -m zipfile -e competition_simulator_server_v9.zip . && bash competition_simulator_server/start_server.sh

LATER:
car

Open http://127.0.0.1:8080/?build=9.0.0 and verify /api/build returns 9.0.0.

COMPETITION SIMULATOR SERVER V8.0.0
==================================

1) Ubuntu English edition: download competition_simulator_server_v8.zip to ~/Downloads.
2) Stop the running car service with Ctrl+C (or the launcher replaces the previous car service).
3) Extract and start:
   cd ~/Downloads && python3 -m zipfile -e competition_simulator_server_v8.zip . && bash competition_simulator_server/start_server.sh
4) Open http://127.0.0.1:8080/?build=8.0.0
5) Next time use your existing `car` shortcut. All paths and downloaded files have English names.

OFFICIAL FIXTURE POSITION CONTROL
- Tap a grid cell to block it, tap that same cell again to unblock (no erase-tool required).
- Raw turntable: centre X=1100..1300 mm; Y=2250..2550 mm (experimental ±150 mm from 2400 mm; NOT an official position tolerance).
- QR board: centre Y=1100..1300 mm along EAST boundary; X fixed at nominal right-edge position.
- Adjust using bounded sliders or the Raw / QR map tools; global mission waypoints update.
- Fig. 1 shows nominal horizontal/vertical ranges. Exact field installation may vary.
- Shared scene stores both fixture locations and validates bounds on the server.

GRID / PLANNING
- Default fieldwide grid: 300x300 mm, 8x8 cells over 2.4x2.4 m.
- User-selectable 200mm / 300mm / 400mm cells, clearing old obstacles on change.
- Max TWO static obstacles, one red blocked cell per obstacle, no concentric inflation colors.
- A*/Dijkstra/D* Lite use 50mm search and rectangular 260x290mm robot-footprint checks.
- Global ordered task path is planned over the full fixed obstacle configuration.
- User-selected experimental destination is the OTHER start/finish area; official rules may differ.

TURNTABLE / MATERIALS
- Rotor delay 0..15s, angular speed 36..60 deg/s (10..6 s/rev), step 5..180deg. Initial: 4s / 45deg/s / 120deg. Materials spaced 120deg.
- Existing QR task simulation and material transfer are preserved.

LIVE MODE (actual robot /odom)
- Start server with `car` in terminal #1.
- Publish REAL nav_msgs/msg/Odometry on /odom from STM32F4 / your ROS2 odometry node.
- In terminal #2:
  cd ~/Downloads/competition_simulator_server && bash start_ros_bridge.sh
- For a custom odometry topic:
  bash start_ros_bridge.sh --topic /your/odom --hz 15
- Browser choose: Run data source -> "Real live / ROS2 odom".
- Yellow line = observed /odom track. Planned green line remains separate.
- The first received odom pose is aligned to user-adjustable field start X/Y/yaw.
- No odom feed means DISCONNECTED. ROS bridge does not command motors or subscribe /cmd_vel.
- Odometry alone drifts. For correct positioning on competition map, integrate ROS TF map->odom->base_link
  or provide a real localized pose with the correct frame; calibrate initial pose carefully.
- /api/telemetry POST is LOCALHOST ONLY. Other LAN users can READ the feed.
- The existing kart_log_view_mecanum_v4 HTML only plays CSV; it is not a live source by itself.

OPTIONAL GROWING ULOG43 FILE (if your logging program appends records in real time)
  python3 ulog43_follow.py ~/path/to/currently_growing.csv
  - Only new appended rows are sent; opening a static log sends NOTHING.
  - This matches the 43-channel mappings in the uploaded mecanum telemetry viewer.
  - Requires the log writer to output all 43 columns as CSV during actual driving.
  - Does not need /odom, but the logged X/Y/yaw must be valid real measurements.

OPTIONAL CSV REPLAY (for testing WITHOUT the vehicle, not true real-time measurement)
  python3 ulog43_replay.py ~/Downloads/log0_kart.csv --max-seconds 20
  Choose "real live" in browser to visualize the replay.
  This follows ULog43: timestamp ch5 ms; x/y ch32/33 m; yaw ch16 deg;
  wheels ch28-31 pulses/s converted via 40mm wheel radius, PPR 440, wheelbase 186mm, track 220mm.
  Check the calibration matches your hardware before trusting calculated speed.

PUBLIC SECURITY
- Anybody who accesses the simulator URL can edit the shared scene.
- The site has no login or TLS on LAN. Do not expose raw LAN HTTP to public networks without protections.
- Telemetry POST works ONLY from loopback 127.0.0.1.
- Replays and live telemetries only display robot status; this website does not send motor commands.

DIAGNOSTIC
  curl -s http://127.0.0.1:8080/api/build
  curl -s http://127.0.0.1:8080/api/telemetry

Control-C stops HTTP server (or the separate ROS bridge).

V8.0 COMPETITION ROTOR / RAW Y ADJUSTMENT
- Three material slots are spaced at 120 degrees as per the official field diagram.
- Rotor angular speed adjustable 36..60 degrees/s = 10..6 seconds per revolution.
- Rotor step angle defaults to 120 degrees but remains adjustable; dwell time adjustable per venue.
- Raw turntable X: 1.1..1.3 m (diagram range). Raw turntable Y: 2.25..2.55 m (experimental +/- 150 mm based on disc radius, NOT official positioning tolerance).
- Disc radius 0.15 m, material spots, station waypoint, collision check and sharing all follow raw X/Y.
- QR board Y: 1.1..1.3 m (diagram range). Click same obstacle cell twice to unblock.
- Old scenario rotor step/rate values are migrated to the new defaults to avoid preserving stale 30 degree setup.

V8.0 TELESCOPIC GRIPPER + STATION MICRO-ALIGNMENT
- Dashboard now contains a collapsible section "Station alignment / telescopic gripper".
- At QR, raw material, process and storage stations, simulated chassis aligns its X/Y and yaw before working.
- QR scan aligns only; no manipulator extension for camera scanning.
- At material handling stations: align -> extend -> grasp/place -> retract -> move to next ring if needed -> return to nominal waypoint -> drive.
- Process ring centres X=1.05/1.20/1.35 m and storage ring centres Y=1.34/1.20/1.06 m are followed individually when enabled.
- Controls: micro offset 0..40 mm (default 15 mm), alignment translation 0.01..0.12m/s (default 0.04), yaw nudge 0..10 deg (default 2), arm reach 0..150mm (default 90), arm speed 0.02..0.20m/s (default .10).
- On movement collision the simulated station operation STOPS instead of silently moving through a blocked grid cell.
- Rotating chassis sweep and arm-end swept volume are NOT collision checked; this is not proof of real hardware clearance.
- All new settings are saved to shared scenario + exported planning JSON.
- In LIVE mode, only ROS2 odom actually moves the body; manipulator remains STOWED unless real actuator telemetry is added.

============================================================
V11 - TWO REAL CAMERA STREAMS (OpenMV + Raspberry Pi 5)
============================================================
1) Start CAR HTTP website as usual in Ubuntu:
   car
   Open http://127.0.0.1:8080/ ; choose LIVE mode.
   Click Settings > Camera Monitor (item 09).

2) OpenMV H7 Plus connected to Ubuntu via USB:
   In OpenMV IDE, install openmv_camera_usb_main.py as main.py.
   IMPORTANT: this example REPLACES your original OpenMV visual recognition
   code. If your OpenMV already scans QR / colours and talks to STM32F4,
   INTEGRATE the USB VCP snap handler into THAT script instead of replacing it.
   Close OpenMV IDE / Serial Terminal so it releases USB VCP.
   Check ls /dev/ttyACM* and run:
       sudo apt install python3-serial
       cd ~/Downloads/competition_simulator_server
       bash start_openmv_camera.sh --port /dev/ttyACM0
   To use a different path: --port /dev/ttyACM1

3) Raspberry Pi 5 77-degree camera (CSI / Picamera2):
   On Raspberry Pi 5, verify the camera:
       rpicam-hello --list-cameras
   On Raspberry Pi OS:
       sudo apt update && sudo apt install python3-picamera2
   Copy pi5_camera_sender.py to the Raspberry Pi (USB stick or scp).
   On UBUNTU server, get upload TOKEN (do not share publicly):
       cat ~/Downloads/competition_simulator_server/data/camera_token.txt
   On RASPBERRY PI, paste the actual Ubuntu LAN IP and TOKEN:
       python3 pi5_camera_sender.py --server http://UBUNTU-LAN-IP:8080 --token TOKEN
   USB OpenMV may alternatively be connected to Raspberry Pi: then install
   python3-serial on Pi, copy openmv_usb_sender.py, supply --server / --token.

4) Status endpoint (safe to read by anyone viewing this website):
       curl http://127.0.0.1:8080/api/cameras
   Only devices knowing the camera token can publish; this does NOT prevent
   all website visitors from viewing both live camera images.

SECURITY: Camera upload uses plain HTTP on LAN; only use on trusted Wi-Fi.
For remote upload use a VPN/SSH tunnel rather than sending the camera token
via an untrusted network. If you enable public site sharing, viewers can also
see both camera images. HTTPS tunnel helps viewer transport but DOES NOT
restrict who can watch a publicly shared address.

Camera feed is independent of /odom. Live robot motion still requires:
   bash start_ros_bridge.sh
The website shows only camera pixels and odometry; it does not send motor
commands and does not yet publish OpenMV QR/color detections to the browser.
Cameras must be physically attached and sending to appear online.
