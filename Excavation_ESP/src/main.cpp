#include <Arduino.h>
#include <Ethernet.h>
#include <SPI.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include "Publisher/genPublisher.h"                     // Include this library to use the publisher      
#include "Subscriber/genSubscriber.h"                   // Include this library to use the Subscriber
#include <MicroROS_Transport.h>                         // IMPORTANT: MAKE SURE TO INCLUDE FOR CONNECTION BETWEEN THE ESP AND THE MICRO ROS AGENT TO WORK

// Define W5500 Ethernet Chip Pins
// Use constexpr instead of #define, more useful for modern C++ at compile time
constexpr int W5500_CS = 14;                            // CS (Chip Select) PIN
constexpr int W5500_RST = 9;                            // Reset PIN
constexpr int W5500_INT = 10;                           // Interrupt PIN 
constexpr int W5500_MISO = 12;                          // MISO PIN
constexpr int W5500_MOSI = 11;                          // MOSI PIN
constexpr int W5500_SCK = 13;                           // Serial Clock PIN

// Network Configuration
byte esp_mac[] = { 0xDE, 0xAD, 0xAF, 0x91, 0x4E, 0x69 };    // Mac address of ESP32 (Make sure its unique for each ESP32)
IPAddress esp_ip(192, 168, 0, 14);                          // IP address of ESP32   (Make sure its unique for each ESP32)
IPAddress dns(192, 168, 0, 1);                              // DNS Server           (Modify if necessary)
IPAddress gateway(192, 168, 0, 1);                          // Default Gateway      (Modify if necessary)
IPAddress agent_ip(192, 168, 0, 80);                        // IP address of Micro ROS agent   (Modify if necessary)        
size_t agent_port = 8888;                                   // Micro ROS Agent Port Number     (Modify if necessary)

// Define Functions
void error_loop();                                                                            
void HandleConnectionState();
bool CreateEntities();
void DestroyEntities();
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}     // Checks for Errors in Micro ROS Setup
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}              // Checks for Errors in Micro ROS Setup

// Define shared ROS entities
rcl_allocator_t allocator;
rclc_support_t support;
rclc_executor_t executor;
rcl_node_t node;
rmw_context_t* rmw_context;

// Define Node Name
const char* node_name = "Excavation_ESP";

// Define MicroROS Subscriber and Publisher entities
genSubscriber Excavation_Fork;

// Define Callback functions for the Subscribers
void Excavation_Fork_Callback(const void* msgin);

// Connection status for the HandleConnection()
enum class ConnectionState {
  Initializing,
  WaitingForAgent,
  Connecting,
  Connected,
  Disconnected
};

ConnectionState connection_state = ConnectionState::Initializing;
// --- CAN ---
#include "driver/twai.h"
#include "SRT_Odrive.h"
#include "SRT_OpenCan.h"

// -------- CAN config --------
#define CAN_TX_GPIO GPIO_NUM_18
#define CAN_RX_GPIO GPIO_NUM_16
#define CAN_BAUDRATE_500K TWAI_TIMING_CONFIG_500KBITS()

//#define CAN_BAUDRATE_500K TWAI_TIMING_CONFIG_500KBITS()

// -------- Node ID arrays --------

// CANopen: 0..31, but 0 reserved for broadcast
//set lichuan motors to as many as needed and and there can_id addresses
constexpr uint8_t OPENCAN_NODE_IDS[]  = {8};
constexpr uint8_t OPENCAN_GEAR_RATIOS[] = {50};
constexpr uint8_t OPENCAN_MICROSTEP = 400;
//do not touch this function this tests how many of each motor there is of lichuan
constexpr size_t  NUM_OPENCAN_MOTORS  = sizeof(OPENCAN_NODE_IDS) / sizeof(OPENCAN_NODE_IDS[0]);

// -------- Shared CAN TX (don’t touch logic) --------
// if you want to read more on this go ahead https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/twai.html#_CPPv418twai_transmitPK18twai_message_t15TickType_t
int send_can_msg(uint16_t canid, uint8_t len, uint8_t* data, bool rtr){
    twai_message_t msg{};
    msg.identifier = canid;
    msg.extd = 0;// 11-bit ID normal frame length for can
    msg.rtr = rtr ? 1 : 0; // remote transmission request this is for requesting data
    msg.self = 0; // self reception request 
    msg.ss = 0; // single shot transmission

    //this makes sure that the length of data being sent is not more than 8 bytes if it is it trims it down to 8 bytes
    if (len > 8) len = 8; 
    msg.data_length_code = len;
    if (!rtr && len && data) memcpy(msg.data, data, len);
// pdMS_TO_TICKS converts milliseconds to RTOS ticks
    return (twai_transmit(&msg, pdMS_TO_TICKS(500)) == ESP_OK) ? 0 : -1;
}

// -------- TWAI init --------
bool init_twai(gpio_num_t tx, gpio_num_t rx) {
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(tx, rx, TWAI_MODE_NORMAL);
    g_config.rx_queue_len = 20;
    twai_timing_config_t t_config = CAN_BAUDRATE_500K;
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK) {
        Serial.println("TWAI driver install failed");
        return false;
    }
    if (twai_start() != ESP_OK) {
        Serial.println("TWAI start failed");
        return false;
    }
    Serial.println("TWAI started @ 500 kbit/s");
    return true;
}


SRT_CanOpenMtr  opencans[NUM_OPENCAN_MOTORS] = {
  SRT_CanOpenMtr(&send_can_msg, OPENCAN_NODE_IDS[0],OPENCAN_GEAR_RATIOS[0])
};

void init_opencan_motors() {
    for (size_t i = 0; i < NUM_OPENCAN_MOTORS; ++i) {
        opencans[i].enable_motor();
        delay(10);
    }
}

// -------- CAN RX: hand frames to all motors, they self-filter by node_id --------
void can_check_recv() {
    twai_message_t msg;
    while (twai_receive(&msg, pdMS_TO_TICKS(2)) == ESP_OK) {
        if (msg.rtr) continue;

        /*
        for (size_t i = 0; i < NUM_ODRIVE_MOTORS; ++i) {
            odrives[i].process_msg(msg.identifier, msg.data_length_code, msg.data);
        }
        */
        // CANopen motors
        for (size_t i = 0; i < NUM_OPENCAN_MOTORS; ++i) {
            opencans[i].process_msg(msg.identifier, msg.data_length_code, msg.data);
        }
    }
}



// Include the code for startinh up the ethernet chip and initialising the Micro ROS transport
void setup() {

  Serial.begin(115200);
  delay(1000);
  Serial.println("Starting Ethernet Connection... ");


  SPI.begin(W5500_SCK, W5500_MISO, W5500_MOSI, W5500_CS);                                   // Initialize SPI with custom pin configuration    
  Ethernet.init(W5500_CS);                                                                  // Select CS PIN and initialize Ethernet chip

  Serial.println("[INIT] Starting micro-ROS node...");
  set_microros_eth_transports(esp_mac, esp_ip, dns, gateway, agent_ip, agent_port);         // IMPORTANT: Start Micro ROS Transport Connection 

  delay(2000);

  connection_state = ConnectionState::WaitingForAgent;

  if (!init_twai(CAN_TX_GPIO, CAN_RX_GPIO)) {
    Serial.println("CAN init failed");
    while (1) delay(1000);
  }

  init_opencan_motors();

  Serial.println("All motors initilised");

};

void loop() {
  HandleConnectionState();
  delay(1000);
  can_check_recv();
}

// This function handles the connect between Micro ROS inside the ESP and the Micro ROS agent
void HandleConnectionState() {
  switch (connection_state) {
    case ConnectionState::WaitingForAgent:
      // Ping to the Micro ROS agent
      if (RMW_RET_OK == rmw_uros_ping_agent(200, 3)) {
        Serial.println("[ROS] Agent found, establishing connection...");
        connection_state = ConnectionState::Connecting;
      }
      break;

    case ConnectionState::Connecting:
      // Create all micro ROS entities
      if (CreateEntities()) {
        Serial.println("[ROS] Connected and ready!");
        connection_state = ConnectionState::Connected;
      } else {
        Serial.println("[ROS] Connection failed, retrying...");
        connection_state = ConnectionState::WaitingForAgent;
      }
      break;

    case ConnectionState::Connected:
    // If Micro ROS agent gets disconnected...
      if (RMW_RET_OK != rmw_uros_ping_agent(200, 3)) {
        Serial.println("[ROS] Agent disconnected!");
        connection_state = ConnectionState::Disconnected;
      } else {
        //Serial.println("heartbeat");                                                      // Use it for testing if the code is working


        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));                                  // Spins the executor (Important for Subscribers)
      }
      break;

    case ConnectionState::Disconnected:
      DestroyEntities();
      Serial.println("[ROS] Waiting for agent...");
      connection_state = ConnectionState::WaitingForAgent;
      break;

    default:
      break;
  }
}

// This function creates/initialises all micro ros entites (Publishers, Subscribers, Executors, Nodes, Support...)
bool CreateEntities() {

  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // Create node object
  RCCHECK(rclc_node_init_default(&node, node_name, "", &support));
  RCCHECK(rclc_executor_init(&executor, &support.context, 10, &allocator)); // number of subscribers the executor handles is hard coded atm
  
  // ADD ALL YOUR PUBLISHERS AND SUBSCRIBER INITIALISATION HERE
  Excavation_Fork.init(&node, "Excavation_Fork", &executor,Excavation_Fork_Callback, DOUBLE);
  return true;
}



// This function destroys all micro ros entites (Publishers, Subscribers, Executors, Nodes, Support...)
void DestroyEntities() {
    rmw_context = rcl_context_get_rmw_context(&support.context);
    (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);
    
    // ADD FUNCTION THAT DESTROYS PUBLISHER AND SUBSCRIBER HERE
    
    Excavation_Fork.destroy(&node);

    rclc_executor_fini(&executor);                              // Destroy Executors
    RCCHECK(rcl_node_fini(&node));                              // Destroy Node
    rclc_support_fini(&support);                                // Destroy Support
}

// Error handle loop
void error_loop() {
  Serial.println("An error has occured. Restarting...");
  delay(2000);
  ESP.restart();

};

// ========================================= CALLBACK FUNCTIONS ========================================= //
// Create your callback functions here.



// Example Callback funtion for Double (Float64) values
void Excavation_Fork_Callback(const void * msgin) {

  const std_msgs__msg__Float64 * msg_double = (const std_msgs__msg__Float64 *)msgin;          // IMPORTANT: DO NOT FORGET TO ADD THIS !!!

  // Enter code here for when the subscriber receives a message.
  //Serial.print("Double value: ");
  //Serial.println(msg_double->data);
  opencans[0].move_absolute(static_cast<int32_t>(msg_double->data)/1000*OPENCAN_MICROSTEP,4,300,300);

}
