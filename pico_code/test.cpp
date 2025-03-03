#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "pico/stdio_usb.h"

#include "hardware/pwm.h"
#include "hardware/spi.h"
#include "hardware/gpio.h"

#include "pico_uart_transports.h" // Restored inclusion of pico_uart_transports.h

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/subscription.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/string.h>  // Include for debug messages
#include <rmw_microros/rmw_microros.h>

#include <micro_ros_utilities/type_utilities.h>
#include <micro_ros_utilities/string_utilities.h>


#include <control_msgs/action/follow_joint_trajectory.h>
#include <sensor_msgs/msg/joint_state.h>
#include <std_srvs/srv/trigger.h>

#include "rcl/logging.h"
#include "rcutils/logging_macros.h"

//  FreeRTOS
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"


//using namespace mbed;
//using namespace rtos;

#define MAX_GOALS 1
#define QUEUE_SIZE 5
#define TIMEOUT_MS 10

// Define SPI settings
#define SPI_PORT_0 spi0
#define SPI_MISO_PIN_0 16 // MISO
#define SPI_CS_PIN_0 17   // CSn
#define SPI_CLK_PIN_0 18  // SCK
#define SPI_MOSI_PIN_0 19 // MOSI

// Declare global variables
rcl_publisher_t joint_state_publisher;
rclc_action_server_t action_server_follow_joint_trajectory;

std_msgs__msg__Float32 msg_sub;
std_msgs__msg__Float32 msg;
std_msgs__msg__String sub_msg0, sub_msg1;
rcl_publisher_t debug_uROS, debug_controller;  // Debug publisher

std_msgs__msg__String debug_msg_uROS, debug_msg_controller;  // Debug message


rcl_timer_t timer;
rcl_node_t node;
rcl_allocator_t allocator;
rclc_support_t support;
rclc_executor_t executor;





rcl_client_t service_client;
std_srvs__srv__Trigger_Request service_request;
std_srvs__srv__Trigger_Response service_response;
int64_t service_request_seq_num;

control_msgs__action__FollowJointTrajectory_SendGoal_Request ros_goal_request_arm[MAX_GOALS];
sensor_msgs__msg__JointState joint_states_msg;

typedef enum { REQ_ARM, REQ_GRIPPER } req_type_t;
typedef struct {
  rclc_action_goal_handle_t *handle;
  req_type_t req_type; 
} goal_t;  

const int N_JOINTS = 2;

goal_t* goal;

QueueHandle_t queue;

BaseType_t controller_handler, uROS_handler;
// TaskHandle_t xHandle = NULL;

// Top Leg Joints
char joint_names[N_JOINTS][20] = {
    "top_joint",
    "top_leg_joint",
  };

unsigned int num_handles = 1/*Publishers*/ + 1/*Action Server*/+1/*Service*/;

const float MAX_ANGLE[2] = {300.0, 244.8};
const float MIN_ANGLE[2] = {132.0, 77.6};
const float MID_ANGLE[2] = {0.0, 0.0};
const float CONVERSION_SCALE = 3.0;
const float MARGIN[2] = {2.0, 2.0};

#define LED_R 12    // RED
#define LED_Y 13    // YELLOW
#define LED_G 14    // GREEN
#define LED_B 15    // BLUE


#define RAD_TO_DEG 57.2958



// Publish debug message
void publish_debug_uROS(const char *message) {
  
  debug_msg_uROS.data.data = (char *)message;
  debug_msg_uROS.data.size = strlen(message);
  rcl_ret_t ret = rcl_publish(&debug_uROS, &debug_msg_uROS, NULL);
  if (ret != RCL_RET_OK) {
      // Error handling for publishing debug messages
  }
}

// Publish debug message
void publish_debug_controller(const char *message) {
  
  debug_msg_controller.data.data = (char *)message;
  debug_msg_controller.data.size = strlen(message);
  rcl_ret_t ret = rcl_publish(&debug_controller, &debug_msg_controller, NULL);
  if (ret != RCL_RET_OK) {
      // Error handling for publishing debug messages
  }
}
  
void motor_control(bool motor_id, bool enable, bool direction){

    if(!motor_id){
        if (enable){
            if(direction == 0){     //DOWN
                gpio_put(LED_R, 1);
            }   
            else if(direction == 1){    //UP
                gpio_put(LED_Y, 1);
            }     
        }
        else{
            gpio_put(LED_R, 0);
            gpio_put(LED_Y, 0);
        }
    }
    else if (motor_id){
        if (enable){
            if(!direction){             //RIGHT
                gpio_put(LED_G, 1);

            }           
            else if(direction){         //LEFT
                gpio_put(LED_B, 1);
            }    
        }
        else{
            gpio_put(LED_G, 1);
            gpio_put(LED_B, 1);
        }
    }
    return;
}


// Read encoder angle via SPI
float read_encoder_angle(bool id) {
    uint16_t tx_buf[1] = {0xFFFF};  // Example TX buffer
    uint16_t rx_buf[1] = {0x00};  // Example RX buffer

    uint16_t dummy_tx = 0xFFFF; //Prevent warning of using NULL in         spi_read16_blocking(SPI_PORT_0, NULL, rx_buf, 1);

    // Write and read data from SPI

    if (!id){
        spi_write16_blocking(SPI_PORT_0,tx_buf,1);
        spi_read16_blocking(SPI_PORT_0, dummy_tx, rx_buf, 1);
    }else if (id){
        // spi_write16_blocking(SPI_PORT_1,tx_buf,1);
        // spi_read16_blocking(SPI_PORT_1, NULL, rx_buf, 1);

    }
    //spi_write_read_blocking(SPI_PORT_0, tx_buf, rx_buf, 2);

    // Process rx_buf to extract angle (example processing)
    uint16_t raw_angle = rx_buf[0] & 0X3FFF;
    return raw_angle * 360.0f / 16384.0f;  // Convert to degrees
}


void rotate_angle(bool id, double goal_angle){
    char debug_buffer[100];
    float current_angle; 
    int direction = 0;    
    
    
    while(1){
      current_angle = read_encoder_angle(id);
      
        // snprintf(debug_buffer, sizeof(debug_buffer), "Current_angle: %.2f \t Goal angle: %.2f", current_angle, goal_angle);
        // publish_debug_uROS(debug_buffer);
        
        if ( ((current_angle > MAX_ANGLE[id]) && (direction == 1))|| ((current_angle < MIN_ANGLE[id]) && (direction == 2) ) ){

            // snprintf(debug_buffer, sizeof(debug_buffer), "ERROR: CURRENT ANGLE OUT OF BOUNDS // MAX: %.2f // MIN: %.2f", MAX_ANGLE[id], MIN_ANGLE[id]);
            // publish_debug_uROS(debug_buffer);
            return;
        }
        if ( (goal_angle > MAX_ANGLE[id] ) || (goal_angle < MIN_ANGLE[id])){
            // snprintf(debug_buffer, sizeof(debug_buffer), "ERROR: ANGLE OUT OF BOUNDS // MAX: %.2f // MIN: %.2f", MAX_ANGLE[id], MIN_ANGLE[id]);
            // publish_debug_uROS(debug_buffer);
            return;
        }
         
        if( (goal_angle > current_angle+ MARGIN[id]) && (goal_angle < MAX_ANGLE[id]) ){
            direction = 1;
            motor_control(id,1,1);
        } else if( (goal_angle < current_angle-MARGIN[id]) && (goal_angle > MIN_ANGLE[id]) ){
            motor_control(id,1,0);
            direction = 2;
        }else{
            motor_control(id,0,0);
            return;
        }
    }
    return;
}



void set_initial_position()
{
  float top_joint_pos           = 0.0f; //RAD_TO_DEG * 1.48353f;
  float top_leg_joint_pos       = RAD_TO_DEG * 2.8f;
  //Braccio.moveTo(grippper_pos, wrist_roll_pos, wrist_pitch_pos, elbow_pos, shoulder_pos, base_pos);
  vTaskDelay(pdMS_TO_TICKS(100));  
}


// void error_loop(rcl_ret_t temp_rc) {
//   bool on = false;
//   for (;;) {
//     digitalWrite(LED_R, (on ? HIGH : LOW));
//     on = !on;
//     vTaskDelay(pdMS_TO_TICKS(250));  
//     publish_debug_uROS("Error: %d\n", temp_rc);
//   }
// }

void setup_outputs(){
  gpio_init(LED_R);
  gpio_set_dir(LED_R, GPIO_OUT);
  gpio_init(LED_Y);
  gpio_set_dir(LED_Y, GPIO_OUT);
  gpio_init(LED_G);
  gpio_set_dir(LED_G, GPIO_OUT);
  gpio_init(LED_B);
  gpio_set_dir(LED_B, GPIO_OUT);

  return;
}

void leds_off()
{
  gpio_put(LED_R, 0);
  gpio_put(LED_Y, 0);
  gpio_put(LED_G, 0);
  gpio_put(LED_B, 0);
  return;
}

void create_service_response_message_memory()
{
  service_response.message.data = (char *) malloc(100);
  service_response.message.size = 0;
  service_response.message.capacity = 100;
}

void service_client_callback(const void * s_response)
{
  char debug_buffer[100];
  
  std_srvs__srv__Trigger_Response * s_response_in = (std_srvs__srv__Trigger_Response *) s_response;
  // snprintf(debug_buffer, sizeof(debug_buffer), "service_client_callback: %d\n", s_response_in->success);
  // publish_debug_uROS(debug_buffer);
}


void execute_arm_trajectory(trajectory_msgs__msg__JointTrajectory *trajectory)
{
  //  MoveIt2 sends the joint_name in alphabetical order
  //  joint_names:
  //  - top_joint
  //  - top_leg_joint

  char debug_buffer[100];
  // snprintf(debug_buffer, sizeof(debug_buffer), "Begin trajectory execution\n");
  // publish_debug_controller(debug_buffer);
  //unsigned long int execution_start_ms = millis();

  for (int i = 0; i < trajectory->points.size; i++) {
    float top_joint       = RAD_TO_DEG * trajectory->points.data[i].positions.data[0];
    float top_leg_joint   = RAD_TO_DEG * trajectory->points.data[i].positions.data[1];

    //unsigned long int move_start_ms = millis();
    //Braccio.moveTo(grippper_pos, wrist_roll_pos, wrist_pitch_pos, elbow_pos, shoulder_pos, base_pos);
    vTaskDelay(pdMS_TO_TICKS(50));  
    //publish_debug_controller("moveTo took [%ld ms]\n", millis() - move_start_ms);
  }
  //publish_debug_controller("Trajectory executed in [%ld ms]\n", millis() - execution_start_ms);
}

void arm_control(rclc_action_goal_handle_t *goal_handle)
{

  char debug_buffer[100];

  rcl_action_goal_state_t goal_state;

  control_msgs__action__FollowJointTrajectory_SendGoal_Request * req =
    (control_msgs__action__FollowJointTrajectory_SendGoal_Request *) goal_handle->ros_goal_request;

  control_msgs__action__FollowJointTrajectory_SendGoal_Response response = {0};
  control_msgs__action__FollowJointTrajectory_FeedbackMessage feedback;

  //print_arm_trajectory(req);

  if (req->goal.trajectory.header.frame_id.size = 0) {
    goal_state = GOAL_STATE_ABORTED;
    gpio_put(LED_R, 1);
  } else {
    if (!goal_handle->goal_cancelled) {
      goal_state = GOAL_STATE_SUCCEEDED;
      gpio_put(LED_G, 1);
      execute_arm_trajectory(&req->goal.trajectory);
    } else {
      goal_state = GOAL_STATE_CANCELED;
      gpio_put(LED_R, 1);
    }
  }

  rcl_ret_t rc;
  do {
    rc = rclc_action_send_result(goal_handle, goal_state, &response);
    vTaskDelay(pdMS_TO_TICKS(500));  
    // snprintf(debug_buffer, sizeof(debug_buffer), "goal_handler_thread: rc =  %d\n", rc);
    // publish_debug_controller(debug_buffer);
  } while (rc != RCL_RET_OK);
  
  leds_off();
}


void controller_thread(void *pvParameters)
  {
      char debug_buffer[100];
  
      // Check if queue is valid
      if (queue == NULL) {
          // snprintf(debug_buffer, sizeof(debug_buffer), "Queue is NULL! Stopping task.");
          // publish_debug_controller(debug_buffer);
          vTaskDelete(NULL);  // Stop the task
      }
  
      snprintf(debug_buffer, sizeof(debug_buffer), "Controller started.");
      publish_debug_controller(debug_buffer);
  
      while (true) {
          goal_t *goal = NULL;
  
          if (xQueueReceive(queue, &goal, portMAX_DELAY) == pdPASS) {
              if (goal == NULL) {
                  // snprintf(debug_buffer, sizeof(debug_buffer), "Received NULL goal!");
                  // publish_debug_controller(debug_buffer);
                  continue;
              }
  
              if (goal->req_type == REQ_ARM) {
                  // snprintf(debug_buffer, sizeof(debug_buffer), "%s: arm goal_handle received.", __FUNCTION__);
                  // publish_debug_controller(debug_buffer);
                  arm_control(goal->handle);
              } else {
                  // snprintf(debug_buffer, sizeof(debug_buffer), "%s: req_type [%d] not handled.", __FUNCTION__, goal->req_type);
                  // publish_debug_controller(debug_buffer);
              }
  
              vPortFree(goal);
              goal = NULL;
          }
  
          //vTaskDelay(pdMS_TO_TICKS(50));
      }
  }




rcl_ret_t arm_goal_callback(rclc_action_goal_handle_t *goal_handle, void *context)
{
  (void) context;

  char debug_buffer[100];
  // snprintf(debug_buffer, sizeof(debug_buffer), "arm_goal_callback\n");

  // publish_debug_controller(debug_buffer);

  control_msgs__action__FollowJointTrajectory_SendGoal_Request * req =  (control_msgs__action__FollowJointTrajectory_SendGoal_Request *) goal_handle->ros_goal_request;

  // TODO: add FollowJointTrajectory request validation here

  goal = (goal_t*)pvPortMalloc(sizeof(goal_t));

  if (goal == NULL) {
    // snprintf(debug_buffer, sizeof(debug_buffer), "%s: mpool allocation failed.\n", __FUNCTION__);
    // publish_debug_controller(debug_buffer);
    return RCL_RET_ACTION_GOAL_REJECTED;
  }

  goal->handle = goal_handle;
  goal->req_type = REQ_ARM;
  xQueueSend(queue, &goal, portMAX_DELAY);

  return RCL_RET_ACTION_GOAL_ACCEPTED;
}

bool arm_cancel_callback(rclc_action_goal_handle_t * goal_handle, void * context)
{
  (void) context;
  (void) goal_handle;

  char debug_buffer[100];
  // snprintf(debug_buffer, sizeof(debug_buffer), "arm_cancel_callback");
  // publish_debug_controller(debug_buffer);

  return true;
}

void create_joint_states_message_memory()
{
  static micro_ros_utilities_memory_conf_t conf = {0};
  conf.max_string_capacity = 50;
  conf.max_ros2_type_sequence_capacity = N_JOINTS;
  conf.max_basic_type_sequence_capacity = N_JOINTS;
  
  bool success = micro_ros_utilities_create_message_memory(
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
    &joint_states_msg,
    conf
  );
  micro_ros_string_utilities_set(joint_states_msg.header.frame_id, "xbot_joint_states");
  
  for (int i = 0; i < N_JOINTS; i++) {
    micro_ros_string_utilities_set(joint_states_msg.name.data[i], joint_names[i]);
  }
  joint_states_msg.name.size = N_JOINTS;
}

void create_joint_trajectory_request_message_memory()
{
  static micro_ros_utilities_memory_conf_t conf = {0};
  conf.max_string_capacity = 50;
  conf.max_ros2_type_sequence_capacity =  1;
  conf.max_basic_type_sequence_capacity = N_JOINTS;
  
  
  char debug_buffer[100];
  
  
  
  micro_ros_utilities_memory_rule_t rules[] = {
    {"goal.trajectory.joint_names", N_JOINTS},
    {"goal.trajectory.points", 55},
    {"goal.trajectory.points.positions", N_JOINTS},
    {"goal.trajectory.points.velocities", N_JOINTS},
    {"goal.trajectory.points.accelerations", N_JOINTS},
  };
  conf.rules = rules;
  conf.n_rules = sizeof(rules) / sizeof(rules[0]);
  
  size_t dynamic_size = micro_ros_utilities_get_dynamic_size(
    ROSIDL_GET_MSG_TYPE_SUPPORT(control_msgs, action, FollowJointTrajectory_SendGoal_Request),
    conf
  );
  size_t message_total_size = dynamic_size + sizeof(control_msgs__action__FollowJointTrajectory_SendGoal_Request);
  
  // snprintf(debug_buffer, sizeof(debug_buffer), "Message Size = %ld\n", message_total_size);
  // publish_debug_controller(debug_buffer);
  
  for (int i = 0; i < MAX_GOALS; i++) {
    bool success = micro_ros_utilities_create_message_memory(
                     ROSIDL_GET_MSG_TYPE_SUPPORT(control_msgs, action, FollowJointTrajectory_SendGoal_Request),
                     &ros_goal_request_arm[i],
                     conf
                    );
    
    // snprintf(debug_buffer, sizeof(debug_buffer), "Published angle 1: %.2f", msg.data);
    // publish_debug_controller(debug_buffer);
  }
}

uint32_t measure_pulse(uint gpio, bool level, uint32_t timeout) {
  uint32_t start_time = time_us_32();
  uint32_t end_time;
  
  // Wait for the pulse to start
  while (gpio_get(gpio) != level) {
    if (time_us_32() - start_time > timeout) {
            return 0;
          }
        }
    
    // Start measuring the pulse width
    start_time = time_us_32();
    while (gpio_get(gpio) == level) {
      if (time_us_32() - start_time > timeout) {
            return 0;
        }
    }
    
    end_time = time_us_32();
    return end_time - start_time;
  }
  
void service_callback(const void * request_msg, void * response_msg){
    // Cast messages to expected types
    // my_interfaces__srv__SetColor_Request * req_in =  (my_interfaces__srv__SetColor_Request *) request_msg;
    // my_interfaces__srv__SetColor_Response * res_in = (my_interfaces__srv__SetColor_Response *) response_msg;
    
    // Handle request message and set the response message values
    
    
    //service_counter++;
    
    //res_in->count = service_counter;
  }
     
// Initialize SPI
void init_spi() {
    spi_init(SPI_PORT_0, 1000000);  // Initialize SPI with 1 MHz speed
    spi_set_format(SPI_PORT_0, 16, SPI_CPOL_0, SPI_CPHA_1, SPI_MSB_FIRST);
    gpio_set_function(SPI_CLK_PIN_0, GPIO_FUNC_SPI);  // Set SCK pin
    gpio_set_function(SPI_MOSI_PIN_0, GPIO_FUNC_SPI); // Set MOSI pin
    gpio_set_function(SPI_MISO_PIN_0, GPIO_FUNC_SPI); // Set MISO pin
    gpio_set_function(SPI_CS_PIN_0, GPIO_FUNC_SPI);   // Set CSn pin

}

void timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    // msg.data = read_encoder_angle(0);
    // rcl_ret_t ret = rcl_publish(&debug_publisher, &msg, NULL);
    // if (ret != RCL_RET_OK) {
    //     publish_debug_uROS("Error publishing angle data 0");
    // }
    // // Publish debug message for angle value
    // char debug_buffer[50];
    // snprintf(debug_buffer, sizeof(debug_buffer), "Published angle 0: %.2f", msg.data);
    // publish_debug_uROS(debug_buffer);

    // msg.data = read_encoder_angle(1);
    // ret = rcl_publish(&debug_publisher, &msg, NULL);
    // if (ret != RCL_RET_OK) {
    //     publish_debug_uROS("Error publishing angle data 1");
    // }
    // // Publish debug message for angle value 
    //  snprintf(debug_buffer, sizeof(debug_buffer), "Published angle 1: %.2f", msg.data);
    //  publish_debug_uROS(debug_buffer);

    // }
}

void init_uROS(){
  char debug_buffer[50];

  // Initialize Micro-ROS
  rmw_uros_set_custom_transport(
    true,
    NULL,
    pico_serial_transport_open,
    pico_serial_transport_close,
    pico_serial_transport_write,
    pico_serial_transport_read
  );
  

  allocator = rcl_get_default_allocator();

  // Wait for agent successful ping for 2 minutes.
  const int timeout_ms = 1000; 
  const uint8_t attempts = 120;

  rcl_ret_t ret = rmw_uros_ping_agent(timeout_ms, attempts);

  if (ret != RCL_RET_OK){
      // Unreachable agent, exiting program.
      //return ret;
  }
  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "pico_node", "", &support);

  // Create follow joint trajectory action service
  rclc_action_server_init_default(
      &action_server_follow_joint_trajectory,
      &node,
      &support,
      ROSIDL_GET_ACTION_TYPE_SUPPORT(control_msgs, FollowJointTrajectory),
      "/arm/follow_joint_trajectory"
    );

  rclc_publisher_init_default(
      &joint_state_publisher,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
      "leg_controller/controller_state"
      );
  rclc_publisher_init_default(
      &debug_uROS,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
      "debug_topic_uROS"
      );
  rclc_publisher_init_default(
      &debug_controller,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
      "debug_topic_controller"
      );    
  rclc_timer_init_default(
      &timer,
      &support,
      RCL_MS_TO_NS(50),
      timer_callback
      );
  // create service client
  rclc_client_init_default(
      &service_client,
      &node,
      ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, Trigger),
      "/trigger_motion_planning"
  );


  std_msgs__msg__Float32__init(&msg_sub);

  // Creating messages memory
  create_joint_states_message_memory();
  create_joint_trajectory_request_message_memory();
  create_service_response_message_memory();

  //std_msgs__msg__String__init(&sub_msg);

  rclc_executor_init(&executor, &support.context, num_handles, &allocator);
  
  rclc_executor_add_timer(&executor, &timer);

  rclc_executor_add_action_server(
      &executor,
      &action_server_follow_joint_trajectory,
      MAX_GOALS,
      ros_goal_request_arm,
      sizeof(control_msgs__action__FollowJointTrajectory_SendGoal_Request),
      arm_goal_callback,
      arm_cancel_callback,
      (void *) &action_server_follow_joint_trajectory
  );

  rclc_executor_add_client(
      &executor,
      &service_client,
      &service_response,
      &service_client_callback
  );
  rmw_uros_sync_session(TIMEOUT_MS);

}


void uROS_executor(void *pvParameters){

  char debug_buffer[100];

  snprintf(debug_buffer, sizeof(debug_buffer), "Started Executor");  publish_debug_uROS(debug_buffer);

  while (true)
  {
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(50));
    
    std_srvs__srv__Trigger_Request__init(&service_request);
    service_request.structure_needs_at_least_one_member = 0;
    rcl_ret_t ret = rcl_send_request(
      &service_client,
      &service_request,
      &service_request_seq_num
    );
    if (ret != RCL_RET_OK) {
      // Handle error
    }
  }
}

void Launch( void) {

  char debug_buffer[100];
  
  uROS_handler = xTaskCreate(uROS_executor, "uROS", 1024, NULL, 1, NULL);
  controller_handler = xTaskCreate(controller_thread, "Controller", 1024, NULL, 2, NULL);
  
  if (uROS_handler != pdPASS || controller_handler != pdPASS) {
    snprintf(debug_buffer, sizeof(debug_buffer), "Error creating queue");      publish_debug_uROS(debug_buffer);
  }  
  
  if (uROS_handler == pdPASS  && controller_handler == pdPASS) {
    // Set up the event queue
    
    queue = xQueueCreate(QUEUE_SIZE, sizeof(goal_t));
    
    
    if (queue == NULL) {
      snprintf(debug_buffer, sizeof(debug_buffer), "Error creating queue");
      publish_debug_uROS(debug_buffer);
    }
    
    gpio_put(LED_G, 1);
    snprintf(debug_buffer, sizeof(debug_buffer), "Start Scheduler");   publish_debug_uROS(debug_buffer);

    // Start the sceduler
    vTaskStartScheduler();



  }

}

int main()
{
  stdio_init_all(); // Initialize UART for debugging

  // Pause to allow the USB path to initialize
  sleep_ms(1000);
  
  setup_outputs();      //Initialize GPIO Pins
  init_spi();           // Initialize SPI
  gpio_put(LED_R, 1);
  init_uROS();
  gpio_put(LED_Y, 1);

  
  //set_initial_position(); //Initial joint positions


  Launch();             // Launch threads and freeRTOS task scheduler 
                        // After      vTaskStartScheduler();  the code never returns to main


  while (true)
  {   
    /*............*/
  }
  return 0;
}