#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <geometry_msgs/msg/twist.h>


#define RXD1 16
#define TXD1 17

// Dynamixel control table address
#define ADDR_PRO_TORQUE_ENABLE          64
#define ADDR_PRO_GOAL_POSITION          116
#define ADDR_PRO_LED                    65
#define ADDR_PRO_PRESENT_POSITION       132

// Protocol version
#define PROTOCOL_VERSION                2.0                 // See which protocol version is used in the Dynamixel

// Dynamixel Config
#define DXL_ID                          1                   // Dynamixel ID: 1
#define BAUDRATE                        57600
#define TORQUE_ENABLE                   1                   // Value for enabling the torque
#define TORQUE_DISABLE                  0                   // Value for disabling the torque
#define DXL_MINIMUM_POSITION_VALUE      100                 // Dynamixel will rotate between this value
#define DXL_MAXIMUM_POSITION_VALUE      4000                // and this value (note that the Dynamixel would not move when the position value is out of movable range. 
#define DXL_MOVING_STATUS_THRESHOLD     20
#define LED_ON                          1
#define LED_OFF                         0

// FIXME: Replace with your Wi-Fi credentials.
#define SSID "JaxHotspot"
#define SSID_PW "niclavision"

// Subscriber
static const char *k_twist = "cmd_vel";
static rcl_subscription_t subscriber_twist;
static geometry_msgs__msg__Twist twist_msg;


// Node and executor variables.
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

#define RCCHECK(fn)                                                    \
  {                                                                    \
    rcl_ret_t temp_rc = fn;                                            \
    if ((temp_rc != RCL_RET_OK)) {                                     \
      printf("Failed status on line %d: %d. Message: %s, Aborting.\n", \
             __LINE__, (int)temp_rc, rcl_get_error_string().str);      \
      error_loop(temp_rc);                                             \
    }                                                                  \
  }

#define RCSOFTCHECK(fn)                                               \
  {                                                                   \
    rcl_ret_t temp_rc = fn;                                           \
    if ((temp_rc != RCL_RET_OK)) {                                    \
      printf("Failed status on line %d: %d. Continuing.\n", __LINE__, \
             (int)temp_rc);                                           \
    }                                                                 \
  }

// Error handle loop
void error_loop(rcl_ret_t rc) {
  Serial.println("Error loop");
  Serial.println(rc);
  while (1) {
    delay(100);
  }
}

// Forward the Twist message over serial and output in debug serial
void twist_callback(const void *msg_in) {
  const geometry_msgs__msg__Twist *msg =
      (const geometry_msgs__msg__Twist *)msg_in;
  Serial1.print(msg->linear.x);
  Serial1.print(",");
  Serial1.print(msg->angular.z);
  Serial1.print(";");
  Serial1.print(msg->linear.y);
  Serial1.println();
  
  Serial.print(msg->linear.x);
  Serial.print(",");
  Serial.print(msg->angular.z);
  Serial.print(";");
  Serial.print(msg->linear.y);
  Serial.println();
}

void setup() {
    // Configure serial transport
    Serial.begin(115200);
    Serial.println("Started"); 

    // Setup WIFI
    char ssid[] = SSID;
    char ssid_pw[] = SSID_PW;
    // FIXME: Replace with your Wi-Fi credentials. (main ros2 node ip)
    IPAddress agent_ip(192, 168, 111, 124);
    const uint16_t k_agent_port = 8888;
    set_microros_wifi_transports(ssid, ssid_pw, agent_ip, k_agent_port);
    delay(500);
    Serial.println("Connected to WIFI");
    
    // Initialize the secondary serial port (UART1) for communication with OpenRB150
    Serial1.begin(9600, SERIAL_8N1, RXD1, TXD1);
    Serial.println("ESP32 is ready to send messages to OpenRB150.");

    // Echo the sent message to the serial monitor via Serial (USB)
    Serial.println("Message sent to OpenRB150: Hello, OpenRB150!");

    // Create init_options
    allocator = rcl_get_default_allocator();
    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    RCCHECK(rcl_init_options_init(&init_options, allocator));
    RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));

    // create node
    RCCHECK(rclc_node_init_default(&node, "micro_ros_platformio_node", "", &support));
    
    // Create twist subscriber.
    RCCHECK(rclc_subscription_init_best_effort(
        &subscriber_twist, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), k_twist));

    // create executor
    RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
    RCCHECK(rclc_executor_add_subscription(
      &executor, &subscriber_twist, &twist_msg, &twist_callback, ON_NEW_DATA));
}

void loop() {
    delay(50);
    RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(50)));
}
