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
WiFiServer telnetServer(23);
WiFiClient telnetClient;
AsyncWebServer webServer(80);
HardwareSerial odriveSerial(2);

// ODrive State
float currentTargetPos0 = 0;
float currentTargetPos1 = 0;
float actualPos0 = 0;
float actualPos1 = 0;
float globalSensitivity = 0.5; // Default sensitivity (0.1 to 1.0)
String inputBuffer = "";
String odriveResponseBuffer = "";

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
<html><head><title>Drodal Remote</title>
<meta name="viewport" content="width=device-width, initial-scale=1.0, maximum-scale=1.0, user-scalable=no">
<style>
  body { font-family: sans-serif; display: flex; flex-direction: column; align-items: center; justify-content: center; height: 100vh; background: #121212; color: white; margin: 0; overflow: hidden; }
  .grid { display: grid; grid-template-columns: repeat(3, 80px); grid-template-rows: repeat(3, 80px); gap: 20px; margin-bottom: 30px;}
  button { width: 100%; height: 100%; border: none; border-radius: 15px; font-size: 24px; font-weight: bold; cursor: pointer; transition: transform 0.1s, background 0.2s; touch-action: manipulation; }
  button:active { transform: scale(0.9); }
  .dir { background: #333; color: white; }
  .stop { background: #ff3b30; color: white; grid-column: 2; grid-row: 2; font-size: 32px; }
  .slider-container { width: 280px; text-align: center; background: #1e1e1e; padding: 20px; border-radius: 20px; }
  input[type=range] { width: 100%; margin: 15px 0; }
  h2 { margin: 0 0 10px 0; color: #aaa; font-size: 18px; }
  label { font-size: 14px; color: #888; }
</style></head>
<body>
  <div class="grid">
    <button class="dir" style="grid-column:2;grid-row:1" onclick="send('f')">▲</button>
    <button class="dir" style="grid-column:1;grid-row:2" onclick="send('l')">◀</button>
    <button class="stop" onclick="send('s')">S</button>
    <button class="dir" style="grid-column:3;grid-row:2" onclick="send('r')">▶</button>
    <button class="dir" style="grid-column:2;grid-row:3" onclick="send('b')">▼</button>
  </div>
  <div class="slider-container">
    <h2>Drive Sensitivity</h2>
    <input type="range" min="1" max="100" value="50" id="sensSlider" onchange="updateSens(this.value)">
    <label>Tame (Low Volt) &larr; &rarr; Aggressive (Full Volt)</label>
  </div>
<script>
  function send(cmd) { fetch(`/cmd?v=${cmd}`).catch(err => console.log(err)); }
  function updateSens(val) { fetch(`/sens?v=${val/100}`).catch(err => console.log(err)); }
</script>
</body></html>
)rawliteral";

void applySensitivity(float sens) {
  globalSensitivity = sens;
  float vel = sens * 4.0f;     
  float accel = sens * 1.5f;   
  
  odriveSerial.printf("w axis0.trap_traj.config.vel_limit %.2f\n", vel);
  odriveSerial.printf("w axis0.trap_traj.config.accel_limit %.2f\n", accel);
  odriveSerial.printf("w axis0.trap_traj.config.decel_limit %.2f\n", accel);
  odriveSerial.printf("w axis1.trap_traj.config.vel_limit %.2f\n", vel);
  odriveSerial.printf("w axis1.trap_traj.config.accel_limit %.2f\n", accel);
  odriveSerial.printf("w axis1.trap_traj.config.decel_limit %.2f\n", accel);
  
  Serial.printf("Sensitivity updated: Vel=%.2f, Accel=%.2f\n", vel, accel);
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

  float step = 0.2f + (globalSensitivity * 0.8f); 
  float turn = 0.1f + (globalSensitivity * 0.2f);

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
  if (millis() - lastUpdate > 100 && feedbackState == 0) {
    odriveSerial.println("r axis0.encoder.pos_estimate");
    feedbackState = 1;
    lastUpdate = millis();
  }

  while (odriveSerial.available()) {
    char c = odriveSerial.read();
    if (c == '\n' || c == '\r') {
      if (odriveResponseBuffer.length() > 0) {
        if (feedbackState == 1) {
          actualPos0 = odriveResponseBuffer.toFloat();
          odriveSerial.println("r axis1.encoder.pos_estimate");
          feedbackState = 2;
        } else if (feedbackState == 2) {
          actualPos1 = odriveResponseBuffer.toFloat();
          feedbackState = 0;
        } else {
          if (telnetClient && telnetClient.connected()) {
            telnetClient.println(odriveResponseBuffer);
          }
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

  webServer.begin();
  telnetServer.begin();
  
  delay(15000); 
  odriveSerial.print(initString);
  applySensitivity(0.5); 
}

void loop() {
  ArduinoOTA.handle();
  if (ros_initialized) { rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)); }

  handleOdriveTraffic();

  if (telnetServer.hasClient()) {
    if (!telnetClient || !telnetClient.connected()) {
      if (telnetClient) telnetClient.stop();
      telnetClient = telnetServer.available();
    } else { telnetServer.available().stop(); }
  }

  if (telnetClient && telnetClient.connected() && telnetClient.available()) {
    while (telnetClient.available()) {
      char c = telnetClient.read();
      if (c == '\n' || c == '\r') {
        if (inputBuffer.length() > 0) {
          feedbackState = 0; 
          odriveSerial.println(inputBuffer);
          inputBuffer = "";
        }
      } else { inputBuffer += c; }
    }
  }
}