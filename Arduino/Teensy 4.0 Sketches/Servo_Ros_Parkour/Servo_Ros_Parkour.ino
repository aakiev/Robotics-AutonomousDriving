#include <Servo.h>
#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>

// Define pins
#define LED_PIN 13
constexpr int pinControlLeft = 11;
constexpr int pinControlRight = 10;

// Define states
enum states {
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
} state;

// Define macros for error checking
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}
#define EXECUTE_EVERY_N_MS(MS, X)  do { \
  static volatile int64_t init = -1; \
  if (init == -1) { init = uxr_millis();} \
  if (uxr_millis() - init > MS) { X; init = uxr_millis();} \
} while (0)

// Declare global variables
rcl_subscription_t subscriber;
std_msgs__msg__Int32 msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
Servo servoRight;
Servo servoLeft;

// Function prototypes
void error_loop();
void motors_move(int direction);
void subscription_callback(const void * msgin);
bool create_entities();
void destroy_entities();

// Error loop function
void error_loop(){
  while(1){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

// Function to move motors
void motors_move(int direction){
  int right_speed, left_speed;
  switch(direction) {
    case 0: // Stop
      right_speed = 1500;
      left_speed = 1500;
      Serial.println("Stoppe das Auto");
      break;
    case 1: // Turn right
      right_speed = 1541;
      left_speed = 1541;
      Serial.println("Linie wurde links erkannt, steuere nach rechts");
      break;
    case 2: // Turn left
      right_speed = 1415;
      left_speed = 1435;
      Serial.println("Linie wurde rechts erkannt, steuere nach links");
      break;
    case 3: // Go straight
      right_speed = 1300;
      left_speed = 1550;
      Serial.println("Keine Linien erkannt, fahre geradeaus");
      break;
    case 4: // Adjust forward or potential 90-degree turn
      right_speed = 1380;
      left_speed = 1435;
      break;
    case 5:
      right_speed = 1581;
      left_speed = 1541;
      break;
    default:
      right_speed = 1500;
      left_speed = 1500;
      Serial.println("Unbekannte Anweisung, stoppe das Auto");
      break;
  }
  
  servoRight.attach(pinControlRight);
  servoLeft.attach(pinControlLeft);
  servoRight.writeMicroseconds(right_speed);
  servoLeft.writeMicroseconds(left_speed);
  delay(100);
  servoRight.detach();
  servoLeft.detach();
}

// Callback function for subscription
void subscription_callback(const void * msgin) {
  const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;
  motors_move(msg->data);
}

// Function to create entities
bool create_entities() {
  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support));
  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "drive_inputs"
  ));
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));
  return true;
}

// Function to destroy entities
void destroy_entities() {
  rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
  (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);
  rcl_subscription_fini(&subscriber, &node);
  rcl_node_fini(&node);
  rclc_executor_fini(&executor);
  rclc_support_fini(&support);
}

// Setup function
void setup(){
  set_microros_transports();
  pinMode(LED_PIN, OUTPUT);
  Serial.begin(115200);
  digitalWrite(LED_PIN, HIGH);
  state = WAITING_AGENT;
}

// Main loop function
void loop(){
  switch (state) {
    case WAITING_AGENT:
      EXECUTE_EVERY_N_MS(500, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;);
      break;
    case AGENT_AVAILABLE:
      state = (true == create_entities()) ? AGENT_CONNECTED : WAITING_AGENT;
      if (state == WAITING_AGENT) {
        destroy_entities();
      }
      break;
    case AGENT_CONNECTED:
      EXECUTE_EVERY_N_MS(200, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
      if (state == AGENT_CONNECTED) {
        RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
      }
      break;
    case AGENT_DISCONNECTED:
      destroy_entities();
      state = WAITING_AGENT;
      break;
    default:
      break;
  }

  digitalWrite(LED_PIN, (state == AGENT_CONNECTED) ? HIGH : LOW);
}
