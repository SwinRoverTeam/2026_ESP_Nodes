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
#define W5500_CS    14    // CS (Chip Select) PIN
#define W5500_RST   9     // Reset PIN
#define W5500_INT   10    // Interrupt PIN 
#define W5500_MISO  12    // MISO PIN
#define W5500_MOSI  11    // MOSI PIN
#define W5500_SCK   13    // Serial Clock PIN

// Network Configuration
byte esp_mac[] = { 0xDE, 0xAD, 0xAF, 0x91, 0x3E, 0x69 };    // Mac address of ESP32 (Make sure its unique for each ESP32)
IPAddress esp_ip(192, 168, 0, 12);                          // IP address of ESP32   (Make sure its unique for each ESP32)
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
const char * node_name = "PivotESP";


// Define MicroROS Subscriber and Publisher entities
// --- Subscribers ---

genSubscriber Pivot_Drive;
genSubscriber Pivot_Rotate;

genSubscriber Pivot_Stop;
genSubscriber Pivot_Home;
genSubscriber Pivot_Restart;
genSubscriber KillSwitch;

// --- Publishers ---

genPublisher Pivot_Diagnostics;

// Define variables for the publishers to send values with

// Define Callback functions for the Subscribers

void Pivot_Drive_Callback(const void * msgin);
void Pivot_Rotate_Callback(const void * msgin);

void Pivot_Stop_Callback(const void * msgin);
void Pivot_Home_Callback(const void * msgin);
void Pivot_Restart_Callback(const void * msgin);
void KillSwitch_Callback(const void * msgin);


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

#include <Arduino.h>
#include "driver/twai.h"
#include "SRT_Odrive.h"
#include "SRT_OpenCan.h"

// -------- CAN config --------
#define CAN_TX_GPIO GPIO_NUM_18
#define CAN_RX_GPIO GPIO_NUM_16
#define CAN_BAUDRATE_500K TWAI_TIMING_CONFIG_500KBITS()

//#define CAN_BAUDRATE_500K TWAI_TIMING_CONFIG_500KBITS()

// -------- Node ID arrays --------
// ODrive: 1+4n, between 1 and 29
//set odrive motors to as many as needed and and there can_id addresses
constexpr uint8_t ODRIVE_NODE_IDS[]   = {1, 5, 9, 13};
//do not touch this function this tests how many of each motor there m4 is of odrive
constexpr size_t  NUM_ODRIVE_MOTORS   = sizeof(ODRIVE_NODE_IDS) / sizeof(ODRIVE_NODE_IDS[0]);

// CANopen: 0..31, but 0 reserved for broadcast
//set lichuan motors to as many as needed and and there can_id addresses
constexpr uint8_t OPENCAN_NODE_IDS[]  = {2, 3, 4, 6};
//do not touch this function this tests how many of each motor there is of lichuan
constexpr size_t  NUM_OPENCAN_MOTORS  = sizeof(OPENCAN_NODE_IDS) / sizeof(OPENCAN_NODE_IDS[0]);;

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

// -------- Motor objects (arrays) --------
SRT_OdriveMtr   odrives[NUM_ODRIVE_MOTORS] = {
    SRT_OdriveMtr(&send_can_msg, ODRIVE_NODE_IDS[0],8), // First motor with can id 1
    SRT_OdriveMtr(&send_can_msg, ODRIVE_NODE_IDS[1],8), // Second motor with can id 5
    SRT_OdriveMtr(&send_can_msg, ODRIVE_NODE_IDS[2],8), // Third motor with can id 9
    SRT_OdriveMtr(&send_can_msg, ODRIVE_NODE_IDS[3],8), // Fourth motor with can id 13
};

SRT_CanOpenMtr  opencans[NUM_OPENCAN_MOTORS] = {
    SRT_CanOpenMtr(&send_can_msg, OPENCAN_NODE_IDS[0],14),
    SRT_CanOpenMtr(&send_can_msg, OPENCAN_NODE_IDS[1],14),
    SRT_CanOpenMtr(&send_can_msg, OPENCAN_NODE_IDS[2],14),
    SRT_CanOpenMtr(&send_can_msg, OPENCAN_NODE_IDS[3],14),
};
void init_odrive_motors() {
    for (size_t i = 0; i < NUM_ODRIVE_MOTORS; ++i) {
        odrives[i].begin();
        //Populate cases when more motors are added or we want seperate settings for each motor
        switch (i) {
            default:
                odrives[i].clear_errors();
                odrives[i].set_cont_mode(velocity_control, vel_ramp);
                odrives[i].set_axis_state(closed_loop_control);
                odrives[i].set_lim(10.0f, 20.0f);
                odrives[i].set_traj_vel_limit(5.0f);
                odrives[i].set_traj_accel_limits(5.0f, 5.0f);
                odrives[i].set_absolute_position(0.0f);
                break;
        }
    }
}

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

        // ODrive motors
        for (size_t i = 0; i < NUM_ODRIVE_MOTORS; ++i) {
            odrives[i].process_msg(msg.identifier, msg.data_length_code, msg.data);
        }
        // CANopen motors
        for (size_t i = 0; i < NUM_OPENCAN_MOTORS; ++i) {
            opencans[i].process_msg(msg.identifier, msg.data_length_code, msg.data);
        }
    }
}



// Include the code for startinh up the ethernet chip and initialising the Micro ROS transport
void setup() {

  Serial.begin(115200);
  delay(2000);
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

  init_odrive_motors();
  init_opencan_motors();

  Serial.println("All motors initilised");

};

void loop() {
  HandleConnectionState();
  can_check_recv();
  delay(5);
  /*
  opencans[0].move_absolute(500,20,5,10);
  opencans[1].move_absolute(500,20,5,10);
  opencans[2].move_absolute(500,20,5,10);
  opencans[3].move_absolute(500,20,5,10);
  while(true){

  }
  */
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
      // ADD HERE FOR PUBLISHING VALUES CONTINUOUSLY
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
  RCCHECK(rclc_executor_init(&executor, &support.context, 20, &allocator)); // number of subscribers the executor handles is hard coded atm
  
  // ADD ALL YOUR PUBLISHERS AND SUBSCRIBER INITIALISATION HERE
  
  // --- Subscribers

  KillSwitch.init(&node, "KillSwitch", &executor, KillSwitch_Callback, BOOL);
  Pivot_Stop.init(&node, "Pivot_Stop", &executor, Pivot_Stop_Callback, BOOL);
  Pivot_Home.init(&node, "Pivot_Home", &executor, Pivot_Home_Callback, BOOL);
  Pivot_Restart.init(&node, "Pivot_Restart", &executor, Pivot_Restart_Callback, BOOL);

  Pivot_Rotate.init(&node, "Pivot_Rotate", &executor, Pivot_Rotate_Callback, FLOAT64_ARRAY);       // Initialise Float64 Array Subscriber
  Pivot_Drive.init(&node, "Pivot_Drive", &executor, Pivot_Drive_Callback, FLOAT64_ARRAY);

  // --- Publishers
  Pivot_Diagnostics.init(&node,"Pivot_Diagnostics",STRING);


  return true;
}



// This function destroys all micro ros entites (Publishers, Subscribers, Executors, Nodes, Support...)
void DestroyEntities() {
    rmw_context = rcl_context_get_rmw_context(&support.context);
    (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);
    
    // ADD FUNCTION THAT DESTROYS PUBLISHER AND SUBSCRIBER HERE
    // --- Subscribers
    KillSwitch.destroy(&node);
    Pivot_Stop.destroy(&node);
    Pivot_Home.destroy(&node);
    Pivot_Restart.destroy(&node);

    Pivot_Rotate.destroy(&node);
    Pivot_Drive.destroy(&node);

    // --- Publishers
    Pivot_Diagnostics.destroy(&node);

    rclc_executor_fini(&executor);                              // Destroy Executors
    RCCHECK(rcl_node_fini(&node));                              // Destroy Node
    rclc_support_fini(&support);                                // Destroy Support
}

// Error handle loop
void error_loop() {
  Serial.println("An error has occured. Restarting...");
  Pivot_Diagnostics.publish("PivotESP has hit an error. Restarting ESP");

  delay(2000);
  DestroyEntities();
  ESP.restart();
};

// ========================================= CALLBACK FUNCTIONS ========================================= //
// Create your callback functions here.


// Example Callback funtion for Integer values

// Example Callback funtion for Double (Float64) values

void Pivot_Rotate_Callback(const void * msgin) {

    const std_msgs__msg__Float64MultiArray * DoubleArrmsg = (const std_msgs__msg__Float64MultiArray *)msgin;              // IMPORTANT: DO NOT FORGET TO ADD THIS !!!

    // Access the data array
    size_t size = DoubleArrmsg->data.size;
    
    const double * array_data = DoubleArrmsg->data.data;
      for(size_t i = 0; i < size; i++)
    {
      opencans[i].move_absolute(array_data[i],8,300,300);
    }
}

void Pivot_Drive_Callback(const void * msgin) {

    const std_msgs__msg__Float64MultiArray * DoubleArrmsg = (const std_msgs__msg__Float64MultiArray *)msgin;              // IMPORTANT: DO NOT FORGET TO ADD THIS !!!

    // Access the data array
    size_t size = DoubleArrmsg->data.size;
    
    const double * array_data = DoubleArrmsg->data.data;
    

    for(size_t i = 0; i < size; i++)
    {
      Serial.print("Pivot_Drive: ");Serial.print(i);Serial.print(" send:");Serial.println(array_data[i]);
      odrives[i].set_ip_vel(map(array_data[i],0,1,0,0.8), 0.5f);
    }

}
void Pivot_Stop_Callback(const void * msgin) {

  const std_msgs__msg__Bool * msg_bool = (const std_msgs__msg__Bool *)msgin;                  // IMPORTANT: DO NOT FORGET TO ADD THIS !!!
  
  //Serial.print("Boolean value: ");
  //Serial.println(msg_bool->data);
  
  if(msg_bool->data==true){
    //Stop all motor movement
    for (size_t i = 0; i < NUM_ODRIVE_MOTORS; ++i) {
      odrives[i].set_ip_vel(0.0f, 0.5f);
    }
    for (size_t i = 0; i < NUM_OPENCAN_MOTORS; ++i) {
      opencans[i].stop();
    }
  }
}
void Pivot_Home_Callback(const void * msgin) {

  const std_msgs__msg__Bool * msg_bool = (const std_msgs__msg__Bool *)msgin;                  // IMPORTANT: DO NOT FORGET TO ADD THIS !!!

  //Serial.print("Boolean value: ");
  //Serial.println(msg_bool->data);

  if(msg_bool->data==true){
    Pivot_Diagnostics.publish("PivotESP is homing");

    //Homing sequence
    Pivot_Diagnostics.publish("PivotESP does not have homing yet!");

    //Pivot_Diagnostics.publish("PivotESP is done homing");
  }
}

void Pivot_Restart_Callback(const void * msgin) {

  const std_msgs__msg__Bool * piv_restart = (const std_msgs__msg__Bool *)msgin;                  // IMPORTANT: DO NOT FORGET TO ADD THIS !!!

  //Serial.print("Boolean value: ");
  //Serial.println(msg_bool->data);

  if(piv_restart->data==true){
    Pivot_Diagnostics.publish("PivotESP is restarting");
    delay(2000);
    // DestroyEntities();
    // ESP.restart();
  }
}


void KillSwitch_Callback(const void * msgin) {

  const std_msgs__msg__Bool * msg_bool = (const std_msgs__msg__Bool *)msgin;                  // IMPORTANT: DO NOT FORGET TO ADD THIS !!!

  //Serial.print("Boolean value: ");
  //Serial.println(msg_bool->data);

  if(msg_bool->data==true){
    Pivot_Diagnostics.publish("PivotESP has received a KillSwitch! System requires a shutdown");
    //Kill it all!!!!!!

    for (size_t i = 0; i < NUM_ODRIVE_MOTORS; ++i) {
      odrives[i].stop(); //This is Estop. not just stop.
      odrives[i].set_ip_vel(0.0f, 0.5f);
    }
    for (size_t i = 0; i < NUM_OPENCAN_MOTORS; ++i) {
      opencans[i].Estop();
    }
    //Enter A never ending loop! System requires a shutdown
  }
}
