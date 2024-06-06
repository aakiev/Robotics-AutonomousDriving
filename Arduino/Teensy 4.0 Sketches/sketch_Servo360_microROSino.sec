#include <Servo.h>
//---------------------microROS---------------------------------------
#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32.h>

rcl_subscription_t subscriber;
std_msgs__msg__Int32 msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

#define LED_PIN 13

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}
//-------------------------End of microROS------------------------------

Servo servoRight;
Servo servoLeft;

//Motorconnections
constexpr int pinFeedbackLeft = 7;
constexpr int pinFeedbackRight = 8;
constexpr int pinControlLeft = 5;
constexpr int pinControlRight = 6;

int pos = 0;

void error_loop(){
  while(1){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

void motors_move(int right, int left){
      servoRight.attach(pinControlRight);
      servoLeft.attach(pinControlLeft);
      servoRight.write(right);
      servoLeft.write(left);
      delay(500);
      servoRight.detach();
      servoLeft.detach();
}

void subscription_callback(const void * msgin)
{  
  const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;
  if (msg->data == 0) motors_move(94, 94);
  else if (msg->data == 1) motors_move(89, 98);
  else if (msg->data == 2) motors_move(98, 89);
  else if (msg->data == 3) motors_move(98, 98);
  else if (msg->data == 4) motors_move(89, 89);  
}

void setup(){
  set_microros_transports();
  
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);  
  
  delay(2000);

  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support));

  // create subscriber CHANGE THE NAME OF THE MESSAGE TO YOUR OWN NAME!!!!!!!!!!
  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "micro_ros_algorri_subscriber"));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));

  pinMode(pinFeedbackRight, INPUT);
  pinMode(pinFeedbackLeft, INPUT);
  } 

void loop(){
  delay(100);
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}



