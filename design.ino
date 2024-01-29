#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/char.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/int8.h>
#include <std_msgs/msg/int16.h>

rcl_subscription_t esp_subscriber;
// rcl_publisher_t ultrasonic_publisher;

std_msgs__msg__Char esp_msg;
std_msgs__msg__Int32 msg;
// std_msgs__msg__Int16 ultrasonic_msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
// rcl_timer_t timer1;


#define LED_PIN 13

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

// void timer_callback(rcl_timer_t * timer1, int64_t last_call_time){  
//   RCLC_UNUSED(last_call_time);
//   if (timer1 != NULL) {
//     // IMU_AHRS.update();
//     RCCHECK(rcl_publish(&ultrasonic_publisher, &ultrasonic_msg, NULL));
//     ultrasonic_msg.data = ultrasonico();
//   }
// }

