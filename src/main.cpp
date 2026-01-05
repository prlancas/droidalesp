#include <Arduino.h>
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
String inputBuffer = "";

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

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){Serial.printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){Serial.printf("Soft error on line %d: %d.\n",__LINE__,(int)temp_rc);}}

const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
    <title>Drodal Remote</title>
    <meta name="viewport" content="width=device-width, initial-scale=1.0, maximum-scale=1.0, user-scalable=no">
    <style>
        body { font-family: sans-serif; display: flex; flex-direction: column; align-items: center; justify-content: center; height: 100vh; background: #121212; color: white; margin: 0; overflow: hidden; }
        .grid { display: grid; grid-template-columns: repeat(3, 80px); grid-template-rows: repeat(3, 80px); gap: 20px; }
        button { width: 100%; height: 100%; border: none; border-radius: 15px; font-size: 24px; font-weight: bold; cursor: pointer; transition: transform 0.1s, background 0.2s; touch-action: manipulation; }
        button:active { transform: scale(0.9); }
        .dir { background: #333; color: white; }
        .stop { background: #ff3b30; color: white; grid-column: 2; grid-row: 2; font-size: 32px; }
        .up { grid-column: 2; grid-row: 1; }
        .down { grid-column: 2; grid-row: 3; }
        .left { grid-column: 1; grid-row: 2; }
        .right { grid-column: 3; grid-row: 2; }
        h2 { margin-bottom: 20px; color: #aaa; }
    </style>
</head>
<body>
    <h2>ODrive Control</h2>
    <div class="grid">
        <button class="dir up" onclick="send('f')">▲</button>
        <button class="dir left" onclick="send('l')">◀</button>
        <button class="stop" onclick="send('s')">S</button>
        <button class="dir right" onclick="send('r')">▶</button>
        <button class="dir down" onclick="send('b')">▼</button>
    </div>
    <script>
        function send(cmd) { fetch(`/cmd?v=${cmd}`).catch(err => console.log(err)); }
        document.addEventListener('keydown', (e) => {
            const key = e.key.toLowerCase();
            if (key === "arrowup") send('f');
            if (key === "arrowdown") send('b');
            if (key === "arrowleft") send('l');
            if (key === "arrowright") send('r');
            if (key === "s") send('s');
        });
    </script>
</body>
</html>
)rawliteral";

void processCommand(String cmd) {
  cmd.trim();
  if (cmd.length() == 0) return;

  if (cmd == "f") {
    currentTargetPos0 += 1.0; currentTargetPos1 += 1.0;
  } else if (cmd == "b") {
    currentTargetPos0 -= 1.0; currentTargetPos1 -= 1.0;
  } else if (cmd == "l") {
    currentTargetPos0 -= 0.5; currentTargetPos1 += 0.5;
  } else if (cmd == "r") {
    currentTargetPos0 += 0.5; currentTargetPos1 -= 0.5;
  } else if (cmd == "s") {
    odriveSerial.println("v 0 0"); odriveSerial.println("v 1 0");
    return;
  } else {
    odriveSerial.println(cmd);
    return;
  }

  odriveSerial.printf("t 0 %.2f\n", currentTargetPos0);
  odriveSerial.printf("t 1 %.2f\n", currentTargetPos1);
}

void subscription_callback(const void * msgin) {
  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
  float left_vel = msg->linear.x - msg->angular.z;
  float right_vel = msg->linear.x + msg->angular.z;
  odriveSerial.printf("v 0 %.2f\n", left_vel);
  odriveSerial.printf("v 1 %.2f\n", right_vel);
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  if (timer != NULL) {
    char feedback[64];
    snprintf(feedback, sizeof(feedback), "Pos0: %.2f Pos1: %.2f", currentTargetPos0, currentTargetPos1);
    msg_pub.data.data = feedback;
    msg_pub.data.size = strlen(feedback);
    RCSOFTCHECK(rcl_publish(&publisher, &msg_pub, NULL));
  }
}

void setup() {
  Serial.begin(115200);
  odriveSerial.begin(115200, SERIAL_8N1, 16, 17);

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED) { delay(500); }

  ArduinoOTA.setHostname(OTA_HOSTNAME);
  ArduinoOTA.setPassword(OTA_PASS);
  ArduinoOTA.begin();

  set_microros_wifi_transports((char*)WIFI_SSID, (char*)WIFI_PASS, (char*)AGENT_IP, AGENT_PORT);
  allocator = rcl_get_default_allocator();

  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "base", "", &support));

  RCCHECK(rclc_subscription_init_default(&subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd_vel"));
  RCCHECK(rclc_publisher_init_default(&publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "odrive_status"));

  const unsigned int timer_timeout = 100;
  RCCHECK(rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(timer_timeout), timer_callback));

  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg_sub, &subscription_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));

  webServer.on("/", HTTP_GET, [](AsyncWebServerRequest *request){ request->send_P(200, "text/html", index_html); });
  webServer.on("/cmd", HTTP_GET, [](AsyncWebServerRequest *request){
    if (request->hasParam("v")) { processCommand(request->getParam("v")->value()); request->send(200, "text/plain", "OK"); }
  });

  webServer.begin();
  telnetServer.begin();
}

void loop() {
  ArduinoOTA.handle();
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));

  if (telnetServer.hasClient()) {
    if (!telnetClient || !telnetClient.connected()) {
      if (telnetClient) telnetClient.stop();
      telnetClient = telnetServer.available();
      inputBuffer = ""; 
    } else { telnetServer.available().stop(); }
  }

  if (telnetClient && telnetClient.connected() && telnetClient.available()) {
    while (telnetClient.available()) {
      char c = telnetClient.read();
      if (c == '\n' || c == '\r') { processCommand(inputBuffer); inputBuffer = ""; }
      else { inputBuffer += c; }
    }
  }
}
