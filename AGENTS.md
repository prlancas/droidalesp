# droidalesp — Agent Guide

ESP32 firmware that drives a hoverboard-based robot ("Droidal") through an
**ODrive clone** motor controller. Built with PlatformIO + Arduino framework.

## Hardware / wiring
- **MCU:** ESP32 dev board (`board = esp32dev`).
- **Motor controller:** ODrive clone, spoken to over UART using the ODrive
  ASCII protocol (`w`, `r`, `t`, `v` commands). Old ODrive 0.5.x style firmware.
- **Serial link:** `HardwareSerial odriveSerial(2)` on `RX=16`, `TX=17`,
  115200 baud.
- **Drive layout:** differential drive. Two axes (`axis0`, `axis1`) on opposite
  sides, so one axis is inverted relative to the other (note the sign flips in
  the code).

## Control paths
There are two independent ways to command the robot:
1. **Web GUI** (`AsyncWebServer` on port 80) — the primary manual remote. Phone-
   friendly D-pad + sliders. This is the intended day-to-day control surface.
2. **micro-ROS** (`micro_ros_arduino`) — subscribes to `cmd_vel`
   (`geometry_msgs/Twist`) for autonomous/ROS control and publishes
   `odrive_status` (`std_msgs/String`) with encoder positions. Talks to a
   micro-ROS agent over WiFi UDP (`AGENT_IP:AGENT_PORT`).

The GUI drives in **trajectory position mode** (`control_mode 3`, `input_mode
5`): each button click nudges a target position. micro-ROS drives in **velocity
mode** (`control_mode 2`, `input_mode 1`).

## Odometry vs. console (important)
`handleOdriveTraffic()` continuously polls `axis*.encoder.pos_estimate` and
parses the replies into `actualPos0/1`. Any *other* serial reply (e.g. from a
raw debug command) corrupts that parsing. To avoid this, the in-browser
**ODrive console** sets `consoleMode = true`, which **pauses odometry polling**
and routes all serial lines to a log the browser polls. There is intentionally
**no telnet server** — it was removed because its replies fought the odometry
parser.

## Safety model
This is a heavy, powerful machine. Behaviour is deliberately tame by default:
- `applySensitivity()` scales ODrive `vel_limit`, `accel_limit`, **and the
  closed-loop `pos_gain` / `vel_gain` / `vel_integrator_gain`** off one slider.
  Low sensitivity = soft gains = no overshoot/oscillation. Turn it up only when
  in control. The growing oscillation seen at full battery is a stiff-gain
  instability, which is why the gains (not just speed limits) scale here.
- Startup applies a **low** default sensitivity.
- A separate **step-size** slider scales how far one button click travels.
- **`/cmd_vel` watchdog:** velocity-mode drive holds a speed until the next
  message, so a dropped link mid-move would run away. `handleCmdVelWatchdog()`
  commands zero velocity if no `/cmd_vel` arrives within `CMDVEL_TIMEOUT_MS`
  (400 ms). Any manual web command clears `cmdVelActive` so the watchdog never
  fights position-mode driving.

## Build / flash
```bash
# pio lives in pyenv 3.10.10 on this machine
~/.pyenv/versions/3.10.10/bin/pio run                 # build
~/.pyenv/versions/3.10.10/bin/pio run -t upload       # USB upload
# OTA upload (uncomment the espota block in platformio.ini first)
```

## Layout
- `src/main.cpp` — all firmware logic + the embedded web UI (`index_html`).
- `src/config.h` — secrets (WiFi, ROS agent, OTA). **gitignored**; copy from
  `src/config.example.h`.
- `lib/micro_ros_arduino/` — prebuilt micro-ROS library (gitignored, must exist
  locally to link; `libmicroros.a` under `src/esp32`).
- `platformio.ini` — env, lib deps, micro-ROS link flags.

## Gotchas
- Keep the web UI ASCII-only; use HTML entities for glyphs (arrows) so they
  don't render as mojibake on some clients.
- `setup()` has a `delay(15000)` before sending ODrive `initString` to let the
  controller boot — don't remove it casually.
- `config.h` is required to compile; CI/fresh clones need it created first.
