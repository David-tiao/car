# ROS2 + standalone Sim2Real directory layout

```text
~/Desktop/car/
├── STM32F4/                          # Unchanged
├── sim2real/                         # STANDALONE website & simulator, never moved
│   ├── launcher.py / server.py
│   ├── web/ / data/ / rviz_profiles/
│   └── ros2_visualization_bridge.py
└── algorithm/car_ws/
    ├── src/
    │   ├── car_web_test/             # Mock UART to ROS2 (web API pending)
    │   ├── car_web_real/             # Read-only hardware-monitor adapter
    │   ├── car_web_sim/              # Lightweight launcher/adapter ONLY
    │   ├── car_web_keyboard/         # Reserved; COLCON_IGNORE
    │   ├── car_serial_link/          # Existing; unchanged
    │   ├── car_navigation/           # Existing; unchanged
    │   └── ...                       # Existing packages; unchanged
    ├── bash/web.sh                   # One place for starting groups
    └── build/ install/ log/          # Usual colcon outputs
```

- Original command `car` is kept compatible (sim2real/launcher.py stays put).
- `car_ws/bash/web.sh sim` calls standalone `sim2real/launcher.py`.
- `car_ws/bash/web.sh real` calls standalone `sim2real/ros2_visualization_bridge.py` (read-only).
- `car_ws/bash/web.sh test` calls the existing `car_serial_link` mock demo.
- `car_ws/bash/web.sh build` builds three small ROS2 adapter packages in one workspace.
- In-browser pure simulation stays in `sim2real/web/`; it does not require ROS2 or HTTP test APIs.
- No keyboard-to-car mapping, motor control, STM32 changes, or new webpage APIs in this organizer.
- The previous v1 organizer must NOT be rerun: it intentionally moved the website.
