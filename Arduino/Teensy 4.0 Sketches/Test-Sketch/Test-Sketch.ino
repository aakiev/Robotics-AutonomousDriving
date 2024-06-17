#include <Servo.h>
//---------------------microROS---------------------------------------
#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>

#include <std_msgs/msg/int32.h>

#define LED_PIN 13
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){return false;}}
#define EXECUTE_EVERY_N_MS(MS, X)  do { \
  static volatile int64_t init = -1; \
  if (init == -1) { init = uxr_millis();} \
  if (uxr_millis() - init > MS) { X; init = uxr_millis();} \
} while (0)\

rclc_support_t support;
rcl_node_t node;
rcl_timer_t timer;
rclc_executor_t executor_sub;
rclc_executor_t executor_pub;
rcl_allocator_t allocator;
rcl_publisher_t publisher;
rcl_subscription_t subscriber;
std_msgs__msg__Int32 msg;
std_msgs__msg__Int32 timer_data;
bool micro_ros_init_successful;

enum states {
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
} state;

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
  (void) last_call_time;
  if (timer != NULL) {
    rcl_publish(&publisher, &timer_data, NULL);
    
  }
}

// Functions create_entities and destroy_entities can take several seconds.
// In order to reduce this rebuild the library with
// - RMW_UXRCE_ENTITY_CREATION_DESTROY_TIMEOUT=0
// - UCLIENT_MAX_SESSION_CONNECTION_ATTEMPTS=3

bool create_entities()
{
  allocator = rcl_get_default_allocator();
  rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
  rcl_init_options_init(&init_options, allocator);
  rcl_init_options_set_domain_id(&init_options, 7);

  // create init_options
  RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options ,&allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "danlias_teensy_node", "", &support));

  // create publisher
  RCCHECK(rclc_publisher_init_best_effort(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "danlias_echo"));

  //create subscriber
  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "danlias_commands"));

  // create timer,
  const unsigned int timer_timeout = 3000;
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));

  // create executor_sub
  executor_sub = rclc_executor_get_zero_initialized_executor();
  RCCHECK(rclc_executor_init(&executor_sub, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor_sub, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));
  return true;

  //create executor_pub
  // executor_pub = rclc_executor_get_zero_initialized_executor();
  // RCCHECK(rclc_executor_init(&executor_pub, &support.context, 1, &allocator));
  // RCCHECK(rclc_executor_add_timer(&executor_pub, &timer));

 
}

void destroy_entities()
{
  rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
  (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  rcl_publisher_fini(&publisher, &node);
  rcl_subscription_fini(&subscriber, &node);
  rcl_timer_fini(&timer);
  rclc_executor_fini(&executor_sub);
  rcl_node_fini(&node);
  rclc_support_fini(&support);
}

//-------------------------End of microROS------------------------------
Servo servoRight;
Servo servoLeft;

//Motorconnections
constexpr int pinFeedbackLeft = 7;
constexpr int pinFeedbackRight = 8;
constexpr int pinControlLeft = 11;
constexpr int pinControlRight = 10;


void motors_move(int right, int left){
      servoRight.writeMicroseconds(right);
      servoLeft.writeMicroseconds(left);
}

void subscription_callback(const void * msgin)
{  
  const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;
  digitalWrite(LED_PIN, (msg->data == 0) ? LOW : HIGH); 
  rcl_publish(&publisher, msg, NULL);
  if (msg->data == 0) motors_move(1500, 1500);
  else if (msg->data == 1) motors_move(1500-84, 1500+53); //-104 //+73
  else if (msg->data == 2) motors_move(1500+93, 1500-128);
  else if (msg->data == 3) motors_move(1500-60, 1500-60); //turn left //je um 5 erhöht
  else if (msg->data == 4) motors_move(1500+60, 1500+60); //turn right
  else if (msg->data == 5) motors_move(1500-70, 1500+53); //moving turn left //methode für langsam um die kurve muss noch
  else if (msg->data == 6) motors_move(1500-84, 1500+43); //moving turn left

}

void setup() {
  set_microros_transports();
  pinMode(LED_PIN, OUTPUT);

  state = WAITING_AGENT;

  timer_data.data = 420;
  servoRight.attach(pinControlRight);
  servoLeft.attach(pinControlLeft);
}

void loop() {
  switch (state) {
    case WAITING_AGENT:
      EXECUTE_EVERY_N_MS(500, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;);
      break;
    case AGENT_AVAILABLE:
      state = (true == create_entities()) ? AGENT_CONNECTED : WAITING_AGENT;
      if (state == WAITING_AGENT) {
        destroy_entities();
      };
      break;
    case AGENT_CONNECTED:
      EXECUTE_EVERY_N_MS(200, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
      if (state == AGENT_CONNECTED) {
        rclc_executor_spin_some(&executor_sub, RCL_MS_TO_NS(100)); //führt den executor_sub aus, in unserem Fall subscription callback
      }
      break;
    case AGENT_DISCONNECTED:
      destroy_entities();
      state = WAITING_AGENT;
      break;
    default:
      break;
  }


  //error feedback
  if (state != AGENT_CONNECTED){
    digitalWrite(LED_PIN, 1);
    delay(10);
    digitalWrite(LED_PIN, 0);
  }
}
