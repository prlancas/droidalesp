#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <HardwareSerial.h>
#include <ESPAsyncWebServer.h>

// Include your private configuration
#include "config.h"

// micro-ROS imports
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/string.h>
#include <geometry_msgs/msg/twist.h>
#include <rmw_microros/rmw_microros.h>   // rmw_uros_ping_agent for reconnection

// ODrive & Network Objects
AsyncWebServer webServer(80);
HardwareSerial odriveSerial(2);

// ODrive State
float currentTargetPos0 = 0;
float currentTargetPos1 = 0;
float actualPos0 = 0;
float actualPos1 = 0;
// Latest ODrive DC bus (battery) voltage, polled alongside the encoders.
float batteryVoltage = 0.0f;
// Voltage at which sensitivity was last (re)applied, so we only recompute the
// gain compensation when the pack has drifted noticeably.
float lastAppliedVoltage = 0.0f;
// Default sensitivity (0.01 to 1.0). Kept intentionally LOW for safety:
// the robot is heavy and powerful, so it starts tame and is turned up by hand.
float globalSensitivity = 0.10f;
// Scales how far a single button click travels (0.05 = tiny nudge, 1.0 = full step).
float stepScale = 1.0f;
String odriveResponseBuffer = "";

// In-browser ODrive console. When enabled, odometry polling is paused so that
// raw command replies don't corrupt the encoder-position parser.
bool consoleMode = false;
String consoleLog = "";

// --- /cmd_vel safety watchdog ---------------------------------------------
// Velocity-mode drive (from /cmd_vel) holds a speed until the next message, so
// a dropped connection while moving would leave the robot running away. Track
// the last /cmd_vel and, if it goes stale, command zero velocity. The web UI
// drives in position mode and clears cmdVelActive so it isn't affected.
const unsigned long CMDVEL_TIMEOUT_MS = 400;
unsigned long lastCmdVelMs = 0;
bool cmdVelActive = false;
// True once the ODrive axes have been switched into velocity mode for /cmd_vel.
// We only reconfigure the controller when this is false (entering velocity drive)
// instead of on every message, which is what made the drive stutter.
bool velModeActive = false;

void appendConsoleLog(const String &line) {
  consoleLog += line;
  consoleLog += '\n';
  // Keep the buffer bounded so it can't grow without limit.
  if (consoleLog.length() > 4000) {
    consoleLog = consoleLog.substring(consoleLog.length() - 3000);
  }
}

// micro-ROS Objects
rcl_subscription_t subscriber;
geometry_msgs__msg__Twist msg_sub;
rcl_publisher_t publisher;
std_msgs__msg__String msg_pub;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

bool ros_initialized = false;
char status_buffer[64];

// --- micro-ROS agent reconnection state machine ---------------------------
// Previously ROS was initialised once in setup(); if the agent (the
// micro-ros-agent container) restarted, the ESP32 silently went mute until a
// manual power-cycle. This state machine pings the agent, (re)creates the ROS
// entities when it appears, and destroys them when it disappears, so the robot
// reconnects on its own after any agent/stack restart.
enum AgentState { WAITING_AGENT, AGENT_AVAILABLE, AGENT_CONNECTED, AGENT_DISCONNECTED };
AgentState agentState = WAITING_AGENT;

// Run statement X at most once every MS milliseconds (each call site keeps its
// own timer). Used to throttle agent pings without blocking the main loop.
#define EXECUTE_EVERY_N_MS(MS, X) do {                 \
  static volatile unsigned long _last_run = 0;         \
  if (millis() - _last_run > (unsigned long)(MS)) {    \
    X;                                                 \
    _last_run = millis();                              \
  }                                                    \
} while (0)

unsigned long lastUpdate = 0;
int feedbackState = 0; 

// Initial safe configuration
const char* initString = "w axis0.controller.config.control_mode 3\n\
w axis0.controller.config.input_mode 5\n\
w axis0.trap_traj.config.vel_limit 1.0\n\
w axis0.trap_traj.config.accel_limit 0.2\n\
w axis0.trap_traj.config.decel_limit 0.2\n\
w axis0.requested_state 8\n\
w axis1.controller.config.control_mode 3\n\
w axis1.controller.config.input_mode 5\n\
w axis1.trap_traj.config.vel_limit 1.0\n\
w axis1.trap_traj.config.accel_limit 0.2\n\
w axis1.trap_traj.config.decel_limit 0.2\n\
w axis1.requested_state 8\n";

const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html><head><title>Droidal Remote</title>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0, maximum-scale=1.0, user-scalable=no">
<style>
  body { font-family: sans-serif; display: flex; flex-direction: column; align-items: center; height: 100vh; background: #121212; color: white; margin: 0; padding: 16px 0; box-sizing: border-box; overflow-y: auto; }
  .grid { display: grid; grid-template-columns: repeat(3, 80px); grid-template-rows: repeat(3, 80px); gap: 20px; margin-bottom: 24px;}
  button { width: 100%; height: 100%; border: none; border-radius: 15px; font-size: 24px; font-weight: bold; cursor: pointer; transition: transform 0.1s, background 0.2s; touch-action: manipulation; }
  button:active { transform: scale(0.9); }
  .dir { background: #333; color: white; }
  .stop { background: #ff3b30; color: white; grid-column: 2; grid-row: 2; font-size: 32px; }
  .slider-container { width: 280px; text-align: center; background: #1e1e1e; padding: 16px 20px; border-radius: 20px; margin-bottom: 16px; }
  input[type=range] { width: 100%; margin: 12px 0; }
  h2 { margin: 0 0 6px 0; color: #aaa; font-size: 18px; }
  .val { color: #4caf50; font-weight: bold; }
  label { font-size: 13px; color: #888; }
  .console { width: 280px; background: #1e1e1e; padding: 16px 20px; border-radius: 20px; }
  #consoleToggle { background: #555; color: #fff; height: 44px; font-size: 16px; border-radius: 12px; margin-bottom: 12px; }
  #consoleToggle.on { background: #ff9500; }
  #log { background: #000; color: #0f0; font-family: monospace; font-size: 12px; text-align: left; height: 160px; overflow-y: auto; padding: 8px; border-radius: 8px; white-space: pre-wrap; word-break: break-word; }
  .row { display: flex; gap: 8px; margin-top: 8px; }
  #cmdIn { flex: 1; border-radius: 8px; border: none; padding: 8px; font-size: 14px; }
  #sendBtn { width: 70px; height: 38px; font-size: 14px; border-radius: 8px; background: #0a84ff; color: #fff; }
  .hidden { display: none; }
</style></head>
<body>
  <div class="grid">
    <button class="dir" style="grid-column:2;grid-row:1" onclick="send('f')">&#9650;</button>
    <button class="dir" style="grid-column:1;grid-row:2" onclick="send('l')">&#9664;</button>
    <button class="stop" onclick="send('s')">STOP</button>
    <button class="dir" style="grid-column:3;grid-row:2" onclick="send('r')">&#9654;</button>
    <button class="dir" style="grid-column:2;grid-row:3" onclick="send('b')">&#9660;</button>
  </div>

  <div class="slider-container">
    <h2>Drive Power <span class="val" id="sensVal">10%</span></h2>
    <input type="range" min="1" max="100" value="10" id="sensSlider" oninput="updateSens(this.value)">
    <label>Tame &amp; Safe &larr;&nbsp;&nbsp;&rarr; Aggressive (Full Power)</label>
  </div>

  <div class="slider-container">
    <h2>Step Size <span class="val" id="stepVal">100%</span></h2>
    <input type="range" min="5" max="100" value="100" id="stepSlider" oninput="updateStep(this.value)">
    <label>Tiny Nudge &larr;&nbsp;&nbsp;&rarr; Full Step Per Click</label>
  </div>

  <div class="slider-container">
    <h2>Battery <span class="val" id="voltVal">--.- V</span></h2>
    <label>ODrive DC bus voltage</label>
  </div>

  <div class="console">
    <button id="consoleToggle" onclick="toggleConsole()">ODrive Console: OFF</button>
    <div id="consoleBody" class="hidden">
      <div id="log"></div>
      <div class="row">
        <input id="cmdIn" placeholder="e.g. r vbus_voltage" onkeydown="if(event.key=='Enter')sendCmd()">
        <button id="sendBtn" onclick="sendCmd()">Send</button>
      </div>
      <label>Odometry is paused while the console is ON.</label>
    </div>
  </div>

<script>
  function send(cmd) { fetch('/cmd?v=' + cmd).catch(e => {}); }
  function updateSens(val) {
    document.getElementById('sensVal').textContent = val + '%';
    fetch('/sens?v=' + (val/100)).catch(e => {});
  }
  function updateStep(val) {
    document.getElementById('stepVal').textContent = val + '%';
    fetch('/step?v=' + (val/100)).catch(e => {});
  }
  function pollVolt() {
    fetch('/volt').then(r => r.text()).then(t => {
      document.getElementById('voltVal').textContent = t + ' V';
    }).catch(e => {});
  }
  setInterval(pollVolt, 2000); pollVolt();

  var consoleOn = false, logTimer = null;
  function toggleConsole() {
    consoleOn = !consoleOn;
    var btn = document.getElementById('consoleToggle');
    btn.textContent = 'ODrive Console: ' + (consoleOn ? 'ON' : 'OFF');
    btn.classList.toggle('on', consoleOn);
    document.getElementById('consoleBody').classList.toggle('hidden', !consoleOn);
    fetch('/console/toggle?on=' + (consoleOn ? '1' : '0')).catch(e => {});
    if (consoleOn) { logTimer = setInterval(pollLog, 400); }
    else { clearInterval(logTimer); }
  }
  function pollLog() {
    fetch('/console/log').then(r => r.text()).then(t => {
      if (t.length) {
        var el = document.getElementById('log');
        el.textContent += t;
        el.scrollTop = el.scrollHeight;
      }
    }).catch(e => {});
  }
  function sendCmd() {
    var i = document.getElementById('cmdIn');
    if (!i.value.trim()) return;
    fetch('/console/send?cmd=' + encodeURIComponent(i.value)).catch(e => {});
    i.value = '';
  }
</script>
</body></html>
)rawliteral";

// Battery-voltage gain compensation. At low battery the motor voltage-saturates
// so the drive feels tame; fully charged (~42 V) the same gains have far more
// torque headroom and the base overshoots/oscillates. Scale the closed-loop
// gains by (VOLT_GAIN_REF / vbus) so behaviour stays consistent as the pack
// drains. Lower VOLT_GAIN_REF (or VOLT_COMP_MIN) if it's still too lively when
// fully charged.
const float VOLT_GAIN_REF = 34.0f;   // volts where the base tuning feels right
const float VOLT_COMP_MIN = 0.5f;    // never cut gains below 50%

float voltageGainScale() {
  if (batteryVoltage < 1.0f) return 1.0f;   // no reading yet: don't compensate
  float comp = VOLT_GAIN_REF / batteryVoltage;
  if (comp > 1.0f) comp = 1.0f;             // don't boost gains above baseline
  if (comp < VOLT_COMP_MIN) comp = VOLT_COMP_MIN;
  return comp;
}

void applySensitivity(float sens) {
  if (sens < 0.01f) sens = 0.01f;
  if (sens > 1.0f) sens = 1.0f;
  globalSensitivity = sens;

  // Trajectory limits: how fast/hard the planned move is. Independent of the
  // battery so top speed is capped no matter how charged the pack is.
  float vel   = 0.5f + sens * 4.0f;
  float accel = 0.3f + sens * 2.0f;

  // Closed-loop gains: THIS is what actually causes the "overshoot then
  // oscillate harder and harder" behaviour at full battery. A stiff pos_gain
  // fights the robot's momentum and rings. Scaling the gains down at low
  // power keeps the response soft and stable; turn the slider up only when
  // you're in control and want a snappier (stiffer) response. The extra
  // voltage compensation keeps that feel constant across the charge range.
  float gscale     = voltageGainScale();
  float pos_gain   = (1.0f + sens * 14.0f) * gscale;   // soft (~1) .. firm (~15)
  float vel_gain   = (0.04f + sens * 0.12f) * gscale;
  float vel_igain  = (0.05f + sens * 0.20f) * gscale;
  lastAppliedVoltage = batteryVoltage;

  for (int a = 0; a <= 1; a++) {
    odriveSerial.printf("w axis%d.trap_traj.config.vel_limit %.2f\n", a, vel);
    odriveSerial.printf("w axis%d.trap_traj.config.accel_limit %.2f\n", a, accel);
    odriveSerial.printf("w axis%d.trap_traj.config.decel_limit %.2f\n", a, accel);
    odriveSerial.printf("w axis%d.controller.config.pos_gain %.3f\n", a, pos_gain);
    odriveSerial.printf("w axis%d.controller.config.vel_gain %.3f\n", a, vel_gain);
    odriveSerial.printf("w axis%d.controller.config.vel_integrator_gain %.3f\n", a, vel_igain);
  }

  Serial.printf("Sensitivity %.2f (vbus=%.1f comp=%.2f) -> vel=%.2f accel=%.2f pos_gain=%.2f vel_gain=%.3f\n",
                sens, batteryVoltage, gscale, vel, accel, pos_gain, vel_gain);
}

void processCommand(String cmd) {
  cmd.trim();
  if (cmd.length() == 0) return;

  // Manual web control uses position mode; hand control back from any active
  // /cmd_vel velocity drive so the watchdog doesn't fight it. Also clear
  // velModeActive so the next /cmd_vel re-arms velocity mode.
  cmdVelActive = false;
  velModeActive = false;

  // Stop and raw-passthrough commands don't need the position-mode dance.
  if (cmd == "s") {
    odriveSerial.println("v 0 0");
    odriveSerial.println("v 1 0");
    return;
  }
  if (cmd != "f" && cmd != "b" && cmd != "l" && cmd != "r") {
    odriveSerial.println(cmd);   // raw ODrive command passthrough
    return;
  }

  // Fresh baseline from the live encoder estimate (polled continuously, even
  // during Nav2 velocity drive), so every manual move is *relative to where the
  // robot actually is now* rather than some old absolute position.
  currentTargetPos0 = actualPos0;
  currentTargetPos1 = actualPos1;

  // Enter position mode SAFELY. Order matters: after a Nav2 velocity drive the
  // ODrive is in velocity mode with input_mode 1 (passthrough) and still holds
  // a stale input_pos from the previous manual session. If control_mode is
  // flipped to position first, the axis briefly servos to that stale setpoint at
  // full effort -- the robot lurches back to where it used to be. So we:
  //   1) select trap-traj input_mode 5 (velocity/accel limited) FIRST,
  //   2) pin the trajectory target to the current position (overwrite the stale
  //      input_pos so there is nothing to rush toward),
  //   3) only then enable position control_mode 3.
  odriveSerial.println("w axis0.controller.config.input_mode 5");
  odriveSerial.println("w axis1.controller.config.input_mode 5");
  odriveSerial.printf("t 0 %.3f\n", currentTargetPos0);
  odriveSerial.printf("t 1 %.3f\n", currentTargetPos1);
  odriveSerial.println("w axis0.controller.config.control_mode 3");
  odriveSerial.println("w axis1.controller.config.control_mode 3");

  // Base distance per click, then scaled by the Step Size slider so one click
  // can be a full move (1.0) or a tiny nudge (down to 0.05).
  float step = (0.2f + (globalSensitivity * 0.8f)) * stepScale;
  float turn = (0.1f + (globalSensitivity * 0.2f)) * stepScale;

  if (cmd == "f") { currentTargetPos0 -= step; currentTargetPos1 += step; }
  else if (cmd == "b") { currentTargetPos0 += step; currentTargetPos1 -= step; }
  else if (cmd == "l") { currentTargetPos0 -= turn; currentTargetPos1 -= turn; }
  else if (cmd == "r") { currentTargetPos0 += turn; currentTargetPos1 += turn; }

  odriveSerial.printf("t 0 %.3f\n", currentTargetPos0);
  odriveSerial.printf("t 1 %.3f\n", currentTargetPos1);
}

void subscription_callback(const void * msgin) {
  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;

  // Switch to velocity mode ONCE, not on every message. Rewriting control_mode /
  // input_mode at the ~15 Hz command rate disturbed the ODrive velocity loop and
  // made the base stutter ("tracing steps"). Only reconfigure when we're entering
  // velocity-mode /cmd_vel control (e.g. after the web UI ran in position mode).
  if (!velModeActive) {
    odriveSerial.println("w axis0.controller.config.control_mode 2");
    odriveSerial.println("w axis0.controller.config.input_mode 1");
    odriveSerial.println("w axis1.controller.config.control_mode 2");
    odriveSerial.println("w axis1.controller.config.input_mode 1");
    velModeActive = true;
  }

  // Differential-drive mixing, REP-103 convention and proper SI units so it
  // matches the metric odometry (droidal_viz) and Nav2's output:
  //   linear.x  is m/s (+ = forward), angular.z is rad/s (+ = CCW / turn LEFT).
  // Wheel geometry (keep in sync with droidal_viz.py): radius 0.095 m, base 0.43 m.
  const float WHEEL_RADIUS_M = 0.095f;
  const float WHEEL_BASE_M   = 0.43f;
  const float TURNS_PER_M    = 1.0f / (2.0f * 3.14159265f * WHEEL_RADIUS_M);

  float v = msg->linear.x;   // m/s
  float w = msg->angular.z;  // rad/s
  float left_ms  = v - w * (WHEEL_BASE_M * 0.5f);   // left wheel ground speed, m/s
  float right_ms = v + w * (WHEEL_BASE_M * 0.5f);   // right wheel ground speed, m/s
  float left  = left_ms  * TURNS_PER_M;             // ODrive velocity, turns/s
  float right = right_ms * TURNS_PER_M;

  // Hard safety ceiling on wheel speed, independent of the commanded value or
  // battery voltage (~2 turns/s = ~1.2 m/s wheel surface). Nav2 commands far
  // less; this only catches a runaway/bad command.
  const float MAX_WHEEL_TURNS = 2.0f;
  if (left  >  MAX_WHEEL_TURNS) left  =  MAX_WHEEL_TURNS;
  if (left  < -MAX_WHEEL_TURNS) left  = -MAX_WHEEL_TURNS;
  if (right >  MAX_WHEEL_TURNS) right =  MAX_WHEEL_TURNS;
  if (right < -MAX_WHEEL_TURNS) right = -MAX_WHEEL_TURNS;

  // Map wheel speeds to ODrive axes: axis 1 = LEFT wheel (forward = +cmd),
  // axis 0 = RIGHT wheel (mounted mirrored, forward = -cmd). If a single
  // direction comes out reversed on the robot, flip that axis's sign here.
  odriveSerial.printf("v 1 %.3f\n",  left);
  odriveSerial.printf("v 0 %.3f\n", -right);

  // Feed the watchdog: a fresh /cmd_vel keeps velocity drive alive.
  lastCmdVelMs = millis();
  cmdVelActive = true;
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  if (timer != NULL && ros_initialized) {
    snprintf(status_buffer, sizeof(status_buffer), "A0:%.2f A1:%.2f V:%.2f",
             actualPos0, actualPos1, batteryVoltage);
    msg_pub.data.size = strlen(status_buffer);
    rcl_publish(&publisher, &msg_pub, NULL);
  }
}

// Create all ROS entities. Returns false (and leaves cleanup to the caller) if
// any step fails, e.g. the agent vanished mid-handshake.
bool create_entities() {
  allocator = rcl_get_default_allocator();
  if (rclc_support_init(&support, 0, NULL, &allocator) != RCL_RET_OK) return false;
  if (rclc_node_init_default(&node, "droidal", "", &support) != RCL_RET_OK) return false;
  if (rclc_subscription_init_default(&subscriber, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd_vel") != RCL_RET_OK) return false;
  if (rclc_publisher_init_default(&publisher, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "odrive_status") != RCL_RET_OK) return false;
  if (rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(100), timer_callback) != RCL_RET_OK) return false;

  executor = rclc_executor_get_zero_initialized_executor();
  if (rclc_executor_init(&executor, &support.context, 2, &allocator) != RCL_RET_OK) return false;
  rclc_executor_add_subscription(&executor, &subscriber, &msg_sub, &subscription_callback, ON_NEW_DATA);
  rclc_executor_add_timer(&executor, &timer);

  msg_pub.data.data = status_buffer;
  msg_pub.data.capacity = sizeof(status_buffer);
  msg_pub.data.size = 0;

  ros_initialized = true;
  return true;
}

// Tear down all ROS entities. Sets the session-destroy timeout to 0 first so we
// don't block trying to cleanly close a session whose agent is already gone.
void destroy_entities() {
  ros_initialized = false;
  rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
  (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  rcl_timer_fini(&timer);
  rclc_executor_fini(&executor);
  rcl_subscription_fini(&subscriber, &node);
  rcl_publisher_fini(&publisher, &node);
  rcl_node_fini(&node);
  rclc_support_fini(&support);
}

// Non-blocking agent connect/spin/reconnect, driven each loop().
void handleMicroRos() {
  switch (agentState) {
    case WAITING_AGENT:
      // Poll for an agent; cheap ping so the web UI/odometry stay responsive.
      EXECUTE_EVERY_N_MS(500,
        agentState = (rmw_uros_ping_agent(100, 1) == RMW_RET_OK) ? AGENT_AVAILABLE : WAITING_AGENT;);
      break;
    case AGENT_AVAILABLE:
      agentState = create_entities() ? AGENT_CONNECTED : WAITING_AGENT;
      if (agentState == WAITING_AGENT) destroy_entities();  // roll back partial setup
      break;
    case AGENT_CONNECTED:
      EXECUTE_EVERY_N_MS(1000,
        agentState = (rmw_uros_ping_agent(150, 1) == RMW_RET_OK) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
      if (agentState == AGENT_CONNECTED) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
      }
      break;
    case AGENT_DISCONNECTED:
      destroy_entities();
      agentState = WAITING_AGENT;
      break;
  }
}

void handleCmdVelWatchdog() {
  // If we're under velocity-mode drive and /cmd_vel has gone stale, stop.
  if (cmdVelActive && (millis() - lastCmdVelMs > CMDVEL_TIMEOUT_MS)) {
    odriveSerial.println("v 0 0");
    odriveSerial.println("v 1 0");
    cmdVelActive = false;
  }
}

void handleOdriveTraffic() {
  // Pause odometry polling while the web console is active so that raw command
  // replies don't get misread as encoder positions.
  if (!consoleMode && millis() - lastUpdate > 100 && feedbackState == 0) {
    odriveSerial.println("r axis0.encoder.pos_estimate");
    feedbackState = 1;
    lastUpdate = millis();
  }

  while (odriveSerial.available()) {
    char c = odriveSerial.read();
    if (c == '\n' || c == '\r') {
      if (odriveResponseBuffer.length() > 0) {
        if (consoleMode) {
          appendConsoleLog(odriveResponseBuffer);
        } else if (feedbackState == 1) {
          actualPos0 = odriveResponseBuffer.toFloat();
          odriveSerial.println("r axis1.encoder.pos_estimate");
          feedbackState = 2;
        } else if (feedbackState == 2) {
          actualPos1 = odriveResponseBuffer.toFloat();
          odriveSerial.println("r vbus_voltage");
          feedbackState = 3;
        } else if (feedbackState == 3) {
          float v = odriveResponseBuffer.toFloat();
          if (v > 1.0f) batteryVoltage = v;   // ignore junk/empty replies
          // Re-apply gains when the pack has drifted enough that the voltage
          // compensation would change (battery moves slowly, so this is rare).
          if (fabsf(batteryVoltage - lastAppliedVoltage) > 0.5f) {
            applySensitivity(globalSensitivity);
          }
          feedbackState = 0;
        }
        odriveResponseBuffer = "";
      }
    } else {
      odriveResponseBuffer += c;
    }
  }
}

void setup() {
  Serial.begin(115200);
  odriveSerial.begin(115200, SERIAL_8N1, 16, 17);
  
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED) { delay(500); }

  ArduinoOTA.begin();
  // Set up the micro-ROS transport once. ROS entity creation and reconnection
  // are handled by the agent state machine in loop() (handleMicroRos), so the
  // robot survives the agent/stack restarting without a power-cycle.
  set_microros_wifi_transports((char*)WIFI_SSID, (char*)WIFI_PASS, (char*)AGENT_IP, AGENT_PORT);
  agentState = WAITING_AGENT;
  
  webServer.on("/", HTTP_GET, [](AsyncWebServerRequest *request){ 
    request->send_P(200, "text/html", index_html); 
  });
  
  webServer.on("/cmd", HTTP_GET, [](AsyncWebServerRequest *request){
    if (request->hasParam("v")) { processCommand(request->getParam("v")->value()); request->send(200, "text/plain", "OK"); }
  });

  webServer.on("/sens", HTTP_GET, [](AsyncWebServerRequest *request){
    if (request->hasParam("v")) { 
      applySensitivity(request->getParam("v")->value().toFloat()); 
      request->send(200, "text/plain", "OK"); 
    }
  });

  webServer.on("/volt", HTTP_GET, [](AsyncWebServerRequest *request){
    char b[16];
    snprintf(b, sizeof(b), "%.2f", batteryVoltage);
    request->send(200, "text/plain", b);
  });

  webServer.on("/step", HTTP_GET, [](AsyncWebServerRequest *request){
    if (request->hasParam("v")) {
      float s = request->getParam("v")->value().toFloat();
      if (s < 0.05f) s = 0.05f;
      if (s > 1.0f) s = 1.0f;
      stepScale = s;
    }
    request->send(200, "text/plain", "OK");
  });

  // --- ODrive web console (replaces the old telnet passthrough) ---
  webServer.on("/console/toggle", HTTP_GET, [](AsyncWebServerRequest *request){
    if (request->hasParam("on")) {
      consoleMode = (request->getParam("on")->value() == "1");
      // Reset the odometry parser state so it resyncs cleanly afterwards.
      feedbackState = 0;
      odriveResponseBuffer = "";
      appendConsoleLog(consoleMode ? "--- Console ON (odometry paused) ---"
                                   : "--- Console OFF ---");
    }
    request->send(200, "text/plain", consoleMode ? "1" : "0");
  });

  webServer.on("/console/send", HTTP_GET, [](AsyncWebServerRequest *request){
    if (consoleMode && request->hasParam("cmd")) {
      String c = request->getParam("cmd")->value();
      appendConsoleLog("> " + c);
      odriveSerial.println(c);
    }
    request->send(200, "text/plain", "OK");
  });

  webServer.on("/console/log", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(200, "text/plain", consoleLog);
    consoleLog = "";
  });

  webServer.begin();
  
  delay(15000); 
  odriveSerial.print(initString);
  // Start tame: apply the low default power/gains so the robot is safe on boot.
  applySensitivity(globalSensitivity);
}

void loop() {
  ArduinoOTA.handle();
  handleMicroRos();   // connect to / spin / reconnect the micro-ROS agent

  handleCmdVelWatchdog();
  handleOdriveTraffic();
}