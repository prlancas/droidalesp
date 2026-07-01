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
   (`geometry_msgs/Twist`) for autonomous/ROS control (Nav2 drives this) and
   publishes `odrive_status` (`std_msgs/String`) with encoder positions. Talks to
   a micro-ROS agent over WiFi UDP (`AGENT_IP:AGENT_PORT`).

   **Auto-reconnect:** `handleMicroRos()` runs a ping-based state machine
   (`WAITING_AGENT -> AGENT_AVAILABLE -> AGENT_CONNECTED -> AGENT_DISCONNECTED`)
   via `create_entities()` / `destroy_entities()`. If the agent (its container)
   restarts, the ESP32 re-creates its ROS entities and resumes on its own — no
   power-cycle needed. `setup()` only sets the transport; it no longer creates
   entities inline.

The GUI drives in **trajectory position mode** (`control_mode 3`, `input_mode
5`): each button click nudges a target position. micro-ROS drives in **velocity
mode** (`control_mode 2`, `input_mode 1`).

### `/cmd_vel` contract (REP-103, SI) — the base is Nav2-ready
`subscription_callback` treats `linear.x` as **m/s (+ = forward)** and
`angular.z` as **rad/s (+ = CCW / turn left)**, per REP-103. It:
- mixes diff-drive: `left = v - w*(base/2)`, `right = v + w*(base/2)` (m/s), then
  converts to ODrive **turns/s** via `TURNS_PER_M = 1/(2*pi*WHEEL_RADIUS_M)`
  (`WHEEL_RADIUS_M = 0.095`, `WHEEL_BASE_M = 0.43` — **keep in sync with
  `droidal_viz.py`**);
- maps to axes: **axis 1 = LEFT** wheel (`+cmd` forward), **axis 0 = RIGHT**
  wheel (mirrored, `-cmd` forward). Flip a single axis's sign here if a direction
  comes out reversed after reflashing.
- **switches to velocity mode ONCE** (guarded by `velModeActive`) instead of
  rewriting `control_mode`/`input_mode` every message — that per-message rewrite
  was the "tracing steps" jerkiness. Any manual web command clears `velModeActive`
  (and `cmdVelActive`) so the next `/cmd_vel` re-arms velocity mode.

Historical: before this, `+angular.z` turned the robot CW (yaw decreased), so the
old ROS-side `goto_goal` negated it. That hack is gone now that the firmware is
correct — but it means changing these signs **invalidates the saved SLAM map**
(rebuild it), and firmware + `droidal_viz` odometry signs must always match.

## Odometry vs. console (important)
`handleOdriveTraffic()` continuously polls `axis*.encoder.pos_estimate` and
parses the replies into `actualPos0/1`. Any *other* serial reply (e.g. from a
raw debug command) corrupts that parsing. To avoid this, the in-browser
**ODrive console** sets `consoleMode = true`, which **pauses odometry polling**
and routes all serial lines to a log the browser polls. There is intentionally
**no telnet server** — it was removed because its replies fought the odometry
parser.

## Battery voltage (read / report / compensate)
`handleOdriveTraffic()` polls `r vbus_voltage` after the two encoder reads
(feedback state 0->1->2->3->0) into `batteryVoltage`. It's:
- **reported** three ways: appended to `/odrive_status` as a `V:<volts>` token
  (droidal_viz republishes it as `/battery_voltage`, `std_msgs/Float32`), shown
  live in the web UI (`/volt` endpoint, polled every 2 s), and logged on the USB
  serial.
- **compensated for**: `voltageGainScale()` returns `VOLT_GAIN_REF / vbus`
  (clamped `[VOLT_COMP_MIN, 1.0]`) and `applySensitivity()` multiplies the
  closed-loop gains by it. Fully charged (~42 V) the base has far more torque
  headroom and rings; scaling gains down keeps the feel constant as the pack
  drains. Gains are re-applied automatically when `vbus` drifts > 0.5 V. Tune
  `VOLT_GAIN_REF` (default 34 V) if it's still too lively when full.
- Speed is capped **regardless** of voltage: trajectory `vel_limit` (web/position
  mode) and a hard `MAX_WHEEL_TURNS` clamp in `subscription_callback` (velocity
  mode). Future use: low-`/battery_voltage` return-to-base for auto-charging.

## Safety model
This is a heavy, powerful machine. Behaviour is deliberately tame by default:
- `applySensitivity()` scales ODrive `vel_limit`, `accel_limit`, **and the
  closed-loop `pos_gain` / `vel_gain` / `vel_integrator_gain`** off one slider,
  then multiplies the gains by the battery-voltage compensation above. Low
  sensitivity = soft gains = no overshoot/oscillation. Turn it up only when in
  control.
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
