// Project 2: USB or Wi-Fi micro-ROS interface for a classic dual-core ESP32 Dev Module.
// Core 0 owns GPIO and hardware step pulses. Core 1 owns micro-ROS.
// No robot arm control and no product sequence run here.
// The physical E-stop must cut actuator power independently of this sketch.
#include <Arduino.h>
#include <esp_timer.h>
#include <esp_system.h>
#include <esp_arduino_version.h>
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>
#include <std_msgs/msg/u_int32_multi_array.h>
#include <std_msgs/msg/u_int64_multi_array.h>
#include "io_config.h"
#include "transport_config.h"
#if PROJECT2_WIFI
#include <WiFi.h>
#endif

#if defined(CONFIG_FREERTOS_UNICORE) && CONFIG_FREERTOS_UNICORE
#error "Select a classic dual-core ESP32 Dev Module."
#endif

// Command: sequence, enable, step frequency Hz, direction, tower (0/1/2), alarm.
struct Command { uint32_t value[6]; };
// State field order is documented in README.md.
struct State { uint64_t value[14]; };
QueueHandle_t commands, states;
uint32_t bootId;
constexpr int PWM_CHANNEL = 0;
rclc_support_t support;
rcl_node_t node;
rcl_publisher_t publisher;
rcl_subscription_t subscriber;
rclc_executor_t executor;
std_msgs__msg__UInt32MultiArray commandMsg;
std_msgs__msg__UInt64MultiArray stateMsg;
uint32_t commandBuffer[6];
uint64_t stateBuffer[14];

// Debounce ordinary controls; the wired emergency power cut is independent.
struct Input {
  int pin;
  bool candidate = false;
  bool stable = false;
  uint32_t changed = 0;
  bool read(uint32_t now) {
    if (pin < 0) return false;
    bool raw = digitalRead(pin) == LOW;
    if (raw != candidate) { candidate = raw; changed = now; }
    if (now - changed >= 10) stable = candidate;
    return stable;
  }
};

void tower(bool red, bool yellow, bool green, bool alarm) {
  if (!IO_VERIFIED) return;
  digitalWrite(TOWER_RED_PIN, red == TOWER_ACTIVE_HIGH ? HIGH : LOW);
  digitalWrite(TOWER_YELLOW_PIN, yellow == TOWER_ACTIVE_HIGH ? HIGH : LOW);
  digitalWrite(TOWER_GREEN_PIN, green == TOWER_ACTIVE_HIGH ? HIGH : LOW);
  digitalWrite(ALARM_PIN, alarm == TOWER_ACTIVE_HIGH ? HIGH : LOW);
}

void drive(uint32_t hz, bool direction) {
  if (!IO_VERIFIED) return;
  // Stop pulses before changing direction. Driver timing is still to be verified.
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(STEP_PIN, 0);
#else
  ledcWrite(PWM_CHANNEL, 0);
#endif
  digitalWrite(ENABLE_PIN, ENABLE_ACTIVE_LOW ? HIGH : LOW);
  digitalWrite(DIR_PIN, direction ? HIGH : LOW);
  if (hz == 0) return;
  // The provisional 10 us setup delay runs on the hardware core only.
  delayMicroseconds(10);
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcChangeFrequency(STEP_PIN, hz, 8);
#else
  ledcSetup(PWM_CHANNEL, hz, 8);
#endif
  digitalWrite(ENABLE_PIN, ENABLE_ACTIVE_LOW ? LOW : HIGH);
  delayMicroseconds(10);
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(STEP_PIN, 128);
#else
  ledcWrite(PWM_CHANNEL, 128);
#endif
}

void hardwareTask(void *) {
  bool ready = false;
  Input green{GREEN_BUTTON_PIN}, red{RED_BUTTON_PIN};
  Input selectorA{SELECTOR_A_PIN}, selectorB{SELECTOR_B_PIN};
  Input proximity{PROXIMITY_PIN};
  // Inputs can be inspected while motor/tower outputs remain locked.
  for (int pin : {GREEN_BUTTON_PIN, RED_BUTTON_PIN, SELECTOR_A_PIN,
                  SELECTOR_B_PIN, PROXIMITY_PIN, ESTOP_HEALTHY_PIN})
    if (pin >= 0) pinMode(pin, INPUT);  // GPIO35 needs an external bias resistor.
  if (IO_VERIFIED) {
    digitalWrite(ENABLE_PIN, ENABLE_ACTIVE_LOW ? HIGH : LOW);
    pinMode(ENABLE_PIN, OUTPUT);
    digitalWrite(DIR_PIN, LOW);
    pinMode(DIR_PIN, OUTPUT);
    for (int pin : {TOWER_RED_PIN, TOWER_YELLOW_PIN, TOWER_GREEN_PIN, ALARM_PIN}) {
      digitalWrite(pin, TOWER_ACTIVE_HIGH ? LOW : HIGH);
      pinMode(pin, OUTPUT);
    }
#if ESP_ARDUINO_VERSION_MAJOR >= 3
    ready = ledcAttachChannel(STEP_PIN, 1000, 8, PWM_CHANNEL);
#else
    ready = ledcSetup(PWM_CHANNEL, 1000, 8) > 0;
    ledcAttachPin(STEP_PIN, PWM_CHANNEL);
#endif
    drive(0, false);
  }
  Command current = {};
  uint32_t lastCommand = 0, lastPublish = 0, applied = 0, actualHz = 0;
  bool actualDirection = false;
  // A stop invalidates motion permission. A disabled command must arrive
  // before any subsequent enable command can restart the conveyor.
  bool armed = false;
  for (;;) {
    uint32_t now = millis();
    bool greenOn = GREEN_BUTTON_PIN >= 0 && (green.read(now) == BUTTON_ACTIVE_LOW);
    bool redOn = RED_BUTTON_PIN >= 0 && (red.read(now) == BUTTON_ACTIVE_LOW);
    bool healthy = ESTOP_HEALTHY_PIN >= 0 && (digitalRead(ESTOP_HEALTHY_PIN) ==
        (ESTOP_HEALTHY_HIGH ? HIGH : LOW));
    bool occupied = PROXIMITY_PIN >= 0 && (proximity.read(now) == PROXIMITY_ACTIVE_LOW);
    uint32_t mode = SELECTOR_A_PIN >= 0 ? uint32_t(selectorA.read(now) == SELECTOR_ACTIVE_LOW) : 0;
    if (SELECTOR_B_PIN >= 0) mode += 2 * uint32_t(selectorB.read(now) == SELECTOR_ACTIVE_LOW);
    bool newCommand = xQueueReceive(commands, &current, 0) == pdTRUE;
    if (newCommand) {
      lastCommand = now;
      applied = current.value[0];
      if (!current.value[1] && healthy && !redOn) armed = true;
    }
    bool expired = applied == 0 || uint32_t(now - lastCommand) > WATCHDOG_MS;
    if (!healthy || redOn || expired) armed = false;
    bool run = ready && healthy && !redOn && !expired && armed &&
        current.value[1] && !occupied;
    uint32_t hz = run ? current.value[2] : 0;
    bool direction = current.value[3];
    if (ready && (hz != actualHz || direction != actualDirection)) {
      drive(hz, direction);
      actualHz = hz;
      actualDirection = direction;
    }
    uint32_t light = (!healthy || redOn || expired) ? 0 : current.value[4];
    bool alarm = ready && (!healthy || (!expired && current.value[5]));
    tower(light == 0, light == 1, light == 2, alarm);
    if (now - lastPublish >= 50) {
      State reading = {{bootId, uint64_t(esp_timer_get_time()), uint64_t(ready),
        uint64_t(greenOn), uint64_t(redOn), uint64_t(healthy), mode,
        uint64_t(occupied), applied, actualHz, light, uint64_t(alarm),
        uint64_t(expired), uint64_t(armed)}};
      xQueueOverwrite(states, &reading);
      lastPublish = now;
    }
    vTaskDelay(1);
  }
}

void receive(const void * message) {
  const auto * msg = static_cast<const std_msgs__msg__UInt32MultiArray *>(message);
  if (msg->data.size != 6 || msg->data.data[0] == 0 ||
      msg->data.data[1] > 1 || msg->data.data[2] > MAX_STEP_HZ ||
      msg->data.data[3] > 1 || msg->data.data[4] > 2 || msg->data.data[5] > 1) return;
  Command command;
  memcpy(command.value, msg->data.data, sizeof(command.value));
  xQueueOverwrite(commands, &command);
}

void restartLink() {
  // Allow the independent GPIO watchdog to stop pulses before rebooting.
  vTaskDelay(pdMS_TO_TICKS(WATCHDOG_MS + 100));
  ESP.restart();
}
#define CHECK(call) do { if ((call) != RCL_RET_OK) { restartLink(); return; } } while (0)

void rosTask(void *) {
#if PROJECT2_WIFI
  Serial.begin(115200);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) vTaskDelay(pdMS_TO_TICKS(250));
  Serial.print("ESP32 Wi-Fi IP: "); Serial.println(WiFi.localIP());
  set_microros_wifi_transports((char*)WIFI_SSID, (char*)WIFI_PASSWORD, (char*)AGENT_IP, AGENT_PORT);
#else
  set_microros_transports();  // Serial carries micro-ROS; do not print debug text here.
#endif
  while (rmw_uros_ping_agent(100, 1) != RMW_RET_OK) vTaskDelay(pdMS_TO_TICKS(200));
  rcl_allocator_t allocator = rcl_get_default_allocator();
  rcl_init_options_t options = rcl_get_zero_initialized_init_options();
  CHECK(rcl_init_options_init(&options, allocator));
  CHECK(rcl_init_options_set_domain_id(&options, ROS_DOMAIN));
  CHECK(rclc_support_init_with_options(&support, 0, nullptr, &options, &allocator));
  CHECK(rclc_node_init_default(&node, "project2_io", "", &support));
  CHECK(rclc_publisher_init_best_effort(&publisher, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt64MultiArray), "/project2/io/state"));
  CHECK(rclc_subscription_init_default(&subscriber, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt32MultiArray), "/project2/io/command"));
  commandMsg.data.data = commandBuffer;
  commandMsg.data.capacity = 6;
  stateMsg.data.data = stateBuffer;
  stateMsg.data.size = stateMsg.data.capacity = 14;
  executor = rclc_executor_get_zero_initialized_executor();
  CHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  CHECK(rclc_executor_add_subscription(&executor, &subscriber, &commandMsg, receive, ON_NEW_DATA));
  uint32_t lastPing = millis();
  for (;;) {
    rcl_ret_t status = rclc_executor_spin_some(&executor, RCL_MS_TO_NS(5));
    if (status != RCL_RET_OK && status != RCL_RET_TIMEOUT) { restartLink(); return; }
    State reading;
    if (xQueueReceive(states, &reading, 0) == pdTRUE) {
      memcpy(stateBuffer, reading.value, sizeof(stateBuffer));
      CHECK(rcl_publish(&publisher, &stateMsg, nullptr));
    }
    if (uint32_t(millis() - lastPing) > 500) {
      if (rmw_uros_ping_agent(50, 1) != RMW_RET_OK) { restartLink(); return; }
      lastPing = millis();
    }
    vTaskDelay(1);
  }
}

void setup() {
  bootId = esp_random();
  if (!bootId) bootId = 1;
  commands = xQueueCreate(1, sizeof(Command));
  states = xQueueCreate(1, sizeof(State));
  if (!commands || !states) { ESP.restart(); return; }
  if (xTaskCreatePinnedToCore(hardwareTask, "project2_gpio", 4096, nullptr, 2,
                            nullptr, 0) != pdPASS) { ESP.restart(); return; }
  if (xTaskCreatePinnedToCore(rosTask, "project2_ros", 12000, nullptr, 1,
                            nullptr, 1) != pdPASS) { ESP.restart(); return; }
}
void loop() { vTaskDelay(pdMS_TO_TICKS(1000)); }
