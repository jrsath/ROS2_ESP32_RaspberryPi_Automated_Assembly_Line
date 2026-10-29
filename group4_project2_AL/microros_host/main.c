// Desktop micro-ROS board. Same six command and fourteen state fields as ESP32.
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>
#include <std_msgs/msg/u_int32_multi_array.h>
#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/u_int64_multi_array.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <time.h>
#include <unistd.h>

#define CHECK(call) do { if ((call) != RCL_RET_OK) { fprintf(stderr, "micro-ROS setup failed at line %d\n", __LINE__); return 1; } } while (0)
static volatile sig_atomic_t running = 1;
static uint32_t command[6], panel[5];
static uint64_t last_command, last_panel;
static int armed = 0, proximity = 0;
static uint64_t last_sensor;
static void receive_sensor(const void *raw) {
  proximity = ((const std_msgs__msg__Bool *)raw)->data;
  struct timespec t; clock_gettime(CLOCK_MONOTONIC, &t);
  last_sensor = (uint64_t)t.tv_sec*1000000+t.tv_nsec/1000;
}
static uint64_t micros(void) {
  struct timespec t; clock_gettime(CLOCK_MONOTONIC, &t);
  return (uint64_t)t.tv_sec * 1000000 + t.tv_nsec / 1000;
}
static void stop(int sig) { (void)sig; running = 0; }
static void receive_command(const void *raw) {
  const std_msgs__msg__UInt32MultiArray *m = raw;
  if (m->data.size != 6 || !m->data.data[0] || m->data.data[1] > 1 ||
      m->data.data[2] > 20000 || m->data.data[3] > 1 ||
      m->data.data[4] > 2 || m->data.data[5] > 1) return;
  for (int i=0; i<6; ++i) command[i] = m->data.data[i];
  last_command = micros();
  // A disabled command re-arms the board after a stop.
  if (!command[1] && panel[2] && !panel[1]) armed = 1;
}
static void receive_panel(const void *raw) {
  const std_msgs__msg__UInt32MultiArray *m = raw;
  if (m->data.size != 5 || m->data.data[0] > 1 || m->data.data[1] > 1 ||
      m->data.data[2] > 1 || m->data.data[3] > 3 || m->data.data[4] > 1) return;
  for (int i=0; i<5; ++i) panel[i] = m->data.data[i];
  last_panel = micros();
}

int main(void) {
  signal(SIGINT, stop); signal(SIGTERM, stop);
  rcl_allocator_t alloc = rcl_get_default_allocator();
  rcl_init_options_t options = rcl_get_zero_initialized_init_options();
  CHECK(rcl_init_options_init(&options, alloc));
  const char *domain = getenv("ROS_DOMAIN_ID");
  CHECK(rcl_init_options_set_domain_id(&options, domain ? strtoul(domain, NULL, 10) : 42));
  rmw_init_options_t *rmw = rcl_init_options_get_rmw_init_options(&options);
  const char *host = getenv("PROJECT2_AGENT_IP");
  const char *port = getenv("PROJECT2_AGENT_PORT");
  CHECK(rmw_uros_options_set_udp_address(host ? host : "127.0.0.1", port ? port : "8889", rmw));
  rclc_support_t support;
  CHECK(rclc_support_init_with_options(&support, 0, NULL, &options, &alloc));
  rcl_node_t node;
  CHECK(rclc_node_init_default(&node, "project2_virtual_board", "", &support));
  rcl_publisher_t pub;
  rcl_subscription_t cmd_sub, panel_sub, sensor_sub;
  CHECK(rclc_publisher_init_best_effort(&pub, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt64MultiArray), "/project2/sim/io/state"));
  CHECK(rclc_subscription_init_default(&cmd_sub, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt32MultiArray), "/project2/sim/io/command"));
  CHECK(rclc_subscription_init_default(&panel_sub, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt32MultiArray), "/project2/sim/panel"));
  CHECK(rclc_subscription_init_best_effort(&sensor_sub, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), "/project2/sim/proximity"));
  std_msgs__msg__Bool sensor_msg = {0};
  uint32_t cmd_buf[6], panel_buf[5];
  uint64_t state_buf[14];
  std_msgs__msg__UInt32MultiArray cmd_msg = {0}, panel_msg = {0};
  std_msgs__msg__UInt64MultiArray state_msg = {0};
  cmd_msg.data.data=cmd_buf; cmd_msg.data.capacity=6;
  panel_msg.data.data=panel_buf; panel_msg.data.capacity=5;
  state_msg.data.data=state_buf; state_msg.data.capacity=state_msg.data.size=14;
  rclc_executor_t executor = rclc_executor_get_zero_initialized_executor();
  CHECK(rclc_executor_init(&executor, &support.context, 3, &alloc));
  CHECK(rclc_executor_add_subscription(&executor, &cmd_sub, &cmd_msg, receive_command, ON_NEW_DATA));
  CHECK(rclc_executor_add_subscription(&executor, &panel_sub, &panel_msg, receive_panel, ON_NEW_DATA));
  CHECK(rclc_executor_add_subscription(&executor, &sensor_sub, &sensor_msg, receive_sensor, ON_NEW_DATA));
  uint64_t start=micros(), published=0, pinged=start;
  uint32_t boot=(uint32_t)start | 1;
  puts("Virtual board: micro-ROS UDP 127.0.0.1:8889. Waiting for panel and coordinator.");
  fflush(stdout);
  while (running) {
    rcl_ret_t spin_result = rclc_executor_spin_some(&executor, RCL_MS_TO_NS(5));
    (void)spin_result;
    uint64_t now=micros();
    int healthy=last_panel && now-last_panel<1000000 && panel[2];
    int expired=!last_command || now-last_command>500000;
    if (!healthy || panel[1] || expired) armed=0;
    // A stale sensor is occupied: stop until its measurement returns.
    int occupied=panel[4] || (last_sensor && (proximity || now-last_sensor>750000));
    int run=healthy && !panel[1] && !expired && armed && command[1] && !occupied;
    uint64_t state[14]={boot,now-start,1,panel[0],panel[1],healthy,panel[3],occupied,
      command[0],run?command[2]:0,(!healthy||panel[1]||expired)?0:command[4],
      !healthy||(!expired&&command[5]),expired,armed};
    if (now-published>=50000) {
      for (int i=0;i<14;++i) state_buf[i]=state[i];
      rcl_ret_t result=rcl_publish(&pub,&state_msg,NULL); (void)result;
      published=now;
    }
    // Exit on agent loss; the launch file restarts a fresh, disarmed client.
    if (now-pinged>1000000) {
      if (rmw_uros_ping_agent(100,1)!=RMW_RET_OK) break;
      pinged=now;
    }
    usleep(1000);
  }
  return 0;
}

