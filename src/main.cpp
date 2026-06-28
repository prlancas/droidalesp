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

// ODrive & Network Objects
AsyncWebServer webServer(80);
HardwareSerial odriveSerial(2);

// ODrive State
float currentTargetPos0 = 0;
float currentTargetPos1 = 0;
float actualPos0 = 0;
float actualPos1 = 0;
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

void applySensitivity(float sens) {
  if (sens < 0.01f) sens = 0.01f;
  if (sens > 1.0f) sens = 1.0f;
  globalSensitivity = sens;

  // Trajectory limits: how fast/hard the planned move is.
  float vel   = 0.5f + sens * 4.0f;
  float accel = 0.3f + sens * 2.0f;

  // Closed-loop gains: THIS is what actually causes the "overshoot then
  // oscillate harder and harder" behaviour at full battery. A stiff pos_gain
  // fights the robot's momentum and rings. Scaling the gains down at low
  // power keeps the response soft and stable; turn the slider up only when
  // you're in control and want a snappier (stiffer) response.
  float pos_gain   = 1.0f + sens * 14.0f;   // soft (~1) .. firm (~15)
  float vel_gain   = 0.04f + sens * 0.12f;
  float vel_igain  = 0.05f + sens * 0.20f;

  for (int a = 0; a <= 1; a++) {
    odriveSerial.printf("w axis%d.trap_traj.config.vel_limit %.2f\n", a, vel);
    odriveSerial.printf("w axis%d.trap_traj.config.accel_limit %.2f\n", a, accel);
    odriveSerial.printf("w axis%d.trap_traj.config.decel_limit %.2f\n", a, accel);
    odriveSerial.printf("w axis%d.controller.config.pos_gain %.3f\n", a, pos_gain);
    odriveSerial.printf("w axis%d.controller.config.vel_gain %.3f\n", a, vel_gain);
    odriveSerial.printf("w axis%d.controller.config.vel_integrator_gain %.3f\n", a, vel_igain);
  }

  Serial.printf("Sensitivity %.2f -> vel=%.2f accel=%.2f pos_gain=%.2f vel_gain=%.3f\n",
                sens, vel, accel, pos_gain, vel_gain);
}

void processCommand(String cmd) {
  cmd.trim();
  if (cmd.length() == 0) return;

  currentTargetPos0 = actualPos0;
  currentTargetPos1 = actualPos1;

  odriveSerial.println("w axis0.controller.config.control_mode 3");
  odriveSerial.println("w axis0.controller.config.input_mode 5");
  odriveSerial.println("w axis1.controller.config.control_mode 3");
  odriveSerial.println("w axis1.controller.config.input_mode 5");

  // Base distance per click, then scaled by the Step Size slider so one click
  // can be a full move (1.0) or a tiny nudge (down to 0.05).
  float step = (0.2f + (globalSensitivity * 0.8f)) * stepScale;
  float turn = (0.1f + (globalSensitivity * 0.2f)) * stepScale;

  if (cmd == "f") { currentTargetPos0 -= step; currentTargetPos1 += step; } 
  else if (cmd == "b") { currentTargetPos0 += step; currentTargetPos1 -= step; } 
  else if (cmd == "l") { currentTargetPos0 -= turn; currentTargetPos1 -= turn; }
  else if (cmd == "r") { currentTargetPos0 += turn; currentTargetPos1 += turn; }
  else if (cmd == "s") { 
    odriveSerial.println("v 0 0"); 
    odriveSerial.println("v 1 0"); 
    return; 
  } 
  else { odriveSerial.println(cmd); return; }

  odriveSerial.printf("t 0 %.2f\n", currentTargetPos0);
  odriveSerial.printf("t 1 %.2f\n", currentTargetPos1);
}

void subscription_callback(const void * msgin) {
  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
  odriveSerial.println("w axis0.controller.config.control_mode 2");
  odriveSerial.println("w axis0.controller.config.input_mode 1");
  odriveSerial.println("w axis1.controller.config.control_mode 2");
  odriveSerial.println("w axis1.controller.config.input_mode 1");

  float left_vel = msg->linear.x - msg->angular.z;
  float right_vel = msg->linear.x + msg->angular.z;
  odriveSerial.printf("v 0 %.2f\n", -left_vel);
  odriveSerial.printf("v 1 %.2f\n", right_vel);
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  if (timer != NULL && ros_initialized) {
    snprintf(status_buffer, sizeof(status_buffer), "A0:%.2f A1:%.2f", actualPos0, actualPos1);
    msg_pub.data.size = strlen(status_buffer);
    rcl_publish(&publisher, &msg_pub, NULL);
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
  set_microros_wifi_transports((char*)WIFI_SSID, (char*)WIFI_PASS, (char*)AGENT_IP, AGENT_PORT);

  allocator = rcl_get_default_allocator();
  if (rclc_support_init(&support, 0, NULL, &allocator) == RCL_RET_OK) {
      if (rclc_node_init_default(&node, "droidal", "", &support) == RCL_RET_OK) {
          rclc_subscription_init_default(&subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd_vel");
          rclc_publisher_init_default(&publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "odrive_status");
          msg_pub.data.data = status_buffer;
          msg_pub.data.capacity = sizeof(status_buffer);
          rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(100), timer_callback);
          rclc_executor_init(&executor, &support.context, 2, &allocator);
          rclc_executor_add_subscription(&executor, &subscriber, &msg_sub, &subscription_callback, ON_NEW_DATA);
          rclc_executor_add_timer(&executor, &timer);
          ros_initialized = true;
      }
  }
  
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
  if (ros_initialized) { rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)); }

  handleOdriveTraffic();
}