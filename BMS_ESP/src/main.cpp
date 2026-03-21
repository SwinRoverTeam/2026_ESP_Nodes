#include <Arduino.h>
#include <Ethernet.h>
#include <SPI.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include "Publisher/genPublisher.h"                     // Include this library to use the publisher      
#include "Subscriber/genSubscriber.h"                   // Include this library to use the Subscriber
#include <MicroROS_Transport.h>                         // IMPORTANT: MAKE SURE TO INCLUDE FOR CONNECTION BETWEEN THE ESP AND THE MICRO ROS AGENT TO WORK
#include "PreCharge/Pre_charge.h"
#include "BMS/Jikong_Handler.h"
#include <stdint.h>
#include <ArduinoJson.h>
// Define W5500 Ethernet Chip Pins
// Use constexpr instead of #define, more useful for modern C++ at compile time
constexpr int W5500_CS = 14;                            // CS (Chip Select) PIN
constexpr int W5500_RST = 9;                            // Reset PIN
constexpr int W5500_INT = 10;                           // Interrupt PIN 
constexpr int W5500_MISO = 12;                          // MISO PIN
constexpr int W5500_MOSI = 11;                          // MOSI PIN
constexpr int W5500_SCK = 13;                           // Serial Clock PIN
uint8_t S2_RX = 16;
uint8_t S2_TX = 18;
const unsigned long TIME_THRESHOLD = 5000;
unsigned long PREV_TIME;

uint8_t NUM_CELLS = 12; 
uint16_t BMS_COMMS_TIMEOUT_ms = 1000;
//PreCharge Pre_Charger(39, 15); enable for remote pre-charge status
JikongMessenger JKmessenger(&Serial2, BMS_COMMS_TIME OUT_ms, NUM_CELLS);

// Network Configuration
byte esp_mac[] = { 0xDE, 0xAD, 0xAF, 0x91, 0x5F, 0x49 };    // Mac address of ESP32 (Make sure its unique for each ESP32)
IPAddress esp_ip(192, 168, 0, 15);                          // IP address of ESP32   (Make sure its unique for each ESP32)
IPAddress dns(192, 168, 0, 1);                              // DNS Server           (Modify if necessary)
IPAddress gateway(192, 168, 0, 1);                          // Default Gateway      (Modify if necessary)
IPAddress agent_ip(192, 168, 0, 80);                        // IP address of Micro ROS agent   (Modify if necessary)        
size_t agent_port = 8888;                                   // Micro ROS Agent Port Number     (Modify if necessary)

// Define Functions
void error_loop();                                                                            
void HandleConnectionState();
bool CreateEntities();
void DestroyEntities();

void Kill_Switch_Call_Back(const void * msgin);
void Reset_Req_Call_Back(const void * msgin); 
void Set_BMS_Call_Back(const void * msgin);


// #define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}     // Checks for Errors in Micro ROS Setup
// #define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}              // Checks for Errors in Micro ROS Setup
// #define ARRAY_LEN(arr) { (sizeof(arr) / sizeof(arr[0])) }                                     // Calculate the Array Length (Needed for the Int32 Array Publisher)

// Define shared ROS entities
rcl_allocator_t allocator;
rclc_support_t support;
rclc_executor_t executor;
rcl_node_t node;
rmw_context_t* rmw_context;

// Define Node Name
const char* node_name = "BMS_Diagnostics";

// Define MicroROS Subscriber and Publisher entities
genPublisher BMS_Diagnostics;

genSubscriber Set_BMS;                                 
genSubscriber KillSwitch; 
genSubscriber BMS_Reset;                             



// Define variables for the publisher to send values
int INTarr[] = {1, 2, 3, 4, 5};
double DBarr[] = {1.0, 1.5, 2.0, 2.5, 3.0};


// Connection status for the HandleConnection()
enum class ConnectionState {
  Initializing,
  WaitingForAgent,
  Connecting,
  Connected,
  Disconnected
};

ConnectionState connection_state = ConnectionState::Initializing;

// Include the code for startinh up the ethernet chip and initialising the Micro ROS transport
void setup() {
  Serial.begin(115200); 
  while(!Serial) { delay(10); }
  Serial2.begin(115200, SERIAL_8N1, S2_RX, S2_TX);
  JKmessenger.begin(115200);

  delay(1000);
  Serial.println("Starting Ethernet Connection... ");


  SPI.begin(W5500_SCK, W5500_MISO, W5500_MOSI, W5500_CS);                                   // Initialize SPI with custom pin configuration    
  Ethernet.init(W5500_CS);                                                                  // Select CS PIN and initialize Ethernet chip

  Serial.println("[INIT] Starting micro-ROS node...");
  set_microros_eth_transports(esp_mac, esp_ip, dns, gateway, agent_ip, agent_port);         // IMPORTANT: Start Micro ROS Transport Connection 

  delay(2000);

  connection_state = ConnectionState::WaitingForAgent;
};

String formatString(){
  const JikongMessenger::Warning_Flags* warnings = JKmessenger.get_warning_flags();
  const JikongMessenger::Status_Flags* status = JKmessenger.get_status_flags(); 
  JsonDocument doc;
  JsonObject bms = doc["BMS Data"].to<JsonObject>();
  //bms["power_tube_temp_dC"] = JKmessenger.get_power_tube_temp_dC(); 
  bms["get_battery_temp_dC"] = JKmessenger.get_battery_temp_dC(); 
  bms["battery_box_temp_dC"] = JKmessenger.get_battery_box_temp_dC();
  bms["current_dA()"] = JKmessenger.get_current_dA(); 
  bms["remaining_capacity_pct"] = JKmessenger.get_remaining_capacity_pct(); 
  /*bms["temp_sensor_count_n"] = JKmessenger.get_temp_sensor_count_n();
  bms["cycle_count"] = JKmessenger.get_cycle_count(); 
  bms["total_cycle_capacity_Ah"] = JKmessenger.get_total_cycle_capacity_Ah(); 
  bms["string_count_n"] = JKmessenger.get_string_count_n();
  bms["pack_overvoltage_mV"] = JKmessenger.get_pack_overvoltage_mV(); 
  bms["pack_undervoltage_mV"] = JKmessenger.get_pack_undervoltage_mV(); 
  bms["cell_overvoltage_mV"] = JKmessenger.get_cell_overvoltage_mV();
  bms["cell_overvoltage_recovery_mV"] = JKmessenger.get_cell_overvoltage_recovery_mV();
  bms["cell_overvoltage_delay_s"] = JKmessenger.get_cell_overvoltage_delay_s(); 
  bms["cell_undervoltage_mV"] = JKmessenger.get_cell_undervoltage_mV(); 
  bms["cell_overvoltage_delay_s"] = JKmessenger.get_cell_overvoltage_delay_s(); 
  bms["monomer_undervoltage_release_mV"] = JKmessenger.get_monomer_undervoltage_release_mV(); 
  bms["cell_undervoltage_delay_s"] = JKmessenger.get_cell_undervoltage_mV();
  bms["cell_pressure_diff_protection_mV"] = JKmessenger.get_cell_pressure_diff_protection_mV();
  bms["discharge_overcurrent_A"] = JKmessenger.get_discharge_overcurrent_A();  
  bms["discharge_overcurrent_delay_s"] = JKmessenger.get_discharge_overcurrent_delay_s();
  bms["charge_overcurrent_A"] = JKmessenger.get_charge_overcurrent_A(); 
  bms["charge_overcurrent_delay_s"] = JKmessenger.get_charge_overcurrent_delay_s();
  bms["balance_start_voltage_mV"] = JKmessenger.get_balance_start_voltage_mV(); 
  bms["balance_diff_mV"] = JKmessenger.get_balance_diff_mV(); 
  bms["power_tube_temp_protection_value_dC"] = JKmessenger.get_power_tube_temp_protection_value_dC(); 
  bms["power_tube_temp_recovery_value_dC"] = JKmessenger.get_power_tube_temp_recovery_value_dC(); 
  bms["box_high_temp_dC"] = JKmessenger.get_box_high_temp_dC();
  bms["box_temp_recovery_dC"] = JKmessenger.get_box_temp_recovery_dC();
  bms["battery_temp_diff_dC"] = JKmessenger.get_battery_temp_diff_dC(); 
  bms["charge_high_temp_dC"] = JKmessenger.get_charge_high_temp_dC(); 
  bms["discharge_high_temp_dC"] = JKmessenger.get_discharge_high_temp_dC(); 
  bms["charge_low_temp_dC"] = JKmessenger.get_charge_low_temp_dC();
  bms["charge_low_temp_recovery_dC"] = JKmessenger.get_charge_low_temp_recovery_dC();
  bms["discharge_low_temp_dC"] = JKmessenger.get_discharge_low_temp_dC(); 
  bms["discharge_low_temp_recovery_dC"] = JKmessenger.get_discharge_low_temp_recovery_dC(); 
  bms["battery_string_setting"] = JKmessenger.get_battery_string_setting();
  bms["battery_capacity_setting"] = JKmessenger.get_battery_capacity_setting(); 
  bms["battery_capacity_Ah"] = JKmessenger.get_battery_capacity_Ah(); 
  bms["active_balance"] = status->balance_switch_state ? "ON" : "OFF"; */  
  bms["charge_mos"] = status->charging_MOS_status ? "ON" : "OFF"; 
  bms["discharge_mos"] = status->discharge_MOS_status ? "ON" : "OFF"; 
  if(warnings->low_capacity) bms["low_capacity"] = warnings->low_capacity ? "WARNING!" : "no alarm"; 
  if(warnings->MOS_tube_OT) bms["MOS_tube_OT"] = warnings->MOS_tube_OT ? "WARNING!" : "no alarm";
  if(warnings->charging_OV) bms["charging_OV"] = warnings->charging_OV ? "WARNING!" : "no alarm";
  if(warnings->discharge_UV) bms["discharge_UV"] = warnings->discharge_UV ? "WARNING!" : "no alarm";
  if(warnings->battery_OT) bms["battery_OT"] = warnings->battery_OT ? "WARNING!" : "no alarm";
  if(warnings->charging_OC) bms["charging_OC"] = warnings->charging_OC ? "WARNING!" : "no alarm";
  if(warnings->discharge_OC) bms["discharge_OC"] = warnings->discharge_OC ? "WARNING!" : "no alarm";
  if(warnings->Cell_pressure_differential) bms["Cell_pressure_differential"] = warnings->Cell_pressure_differential ? "WARNING!" : "no alarm";
  if(warnings->BB_OT) bms["Battery_Box_Over_Temp"] = warnings->BB_OT ? "WARNING!" : "no alarm";
  if(warnings->battery_low_temp) bms["battery_low_temp"] = warnings->battery_low_temp ? "WARNING!" : "no alarm";
  if(warnings->monomer_OV) bms["monomer_Over_Voltage"] = warnings->monomer_OV ? "WARNING!" : "no alarm";
  if(warnings->monomer_UV) bms["monomer_Under_Voltage"] = warnings->monomer_UV ? "WARNING!" : "no alarm";  
  if(warnings->protection_309A) bms["protection_309A"] = warnings->protection_309A ? "WARNING!" : "no alarm";
  bms["battery_conn_status"] = status->battery_conn_status ? "ON" : "OFF"; 
  bms["total_voltage_mV"] = JKmessenger.get_total_voltage_mV();

  String output;
  serializeJson(doc, output);
  return output;
}

void loop() {
  HandleConnectionState();
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
        if(millis() - PREV_TIME >= TIME_THRESHOLD){
          JKmessenger.request_data();                                                   
          
        // ADD HERE FOR PUBLISHING VALUES CONTINUOUSLY
          String BMS_data = formatString();
          BMS_Diagnostics.publish(BMS_data.c_str());
          rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)); 
          PREV_TIME = millis(); 
        }                                 
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
  RCCHECK(rclc_executor_init(&executor, &support.context, 10, &allocator)); 
  
  // initilize publisher and subscribers
  BMS_Diagnostics.init(&node, "BMS_Diagnostics", STRING);                              
                         
  KillSwitch.init(&node, "KillSwitch", &executor, Kill_Switch_Call_Back, BOOL);                                               
  Set_BMS.init(&node, "Set_BMS", &executor,  Set_BMS_Call_Back, INT32_ARRAY);
  BMS_Reset.init(&node, "BMS_Reset", &executor, Reset_Req_Call_Back, BOOL);           
   
  return true;
}

// This function destroys all micro ros entites (Publishers, Subscribers, Executors, Nodes, Support...)
void DestroyEntities() {
    rmw_context = rcl_context_get_rmw_context(&support.context);
    (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);
    
    // ADD FUNCTION THAT DESTROYS PUBLISHER AND SUBSCRIBER HERE
    BMS_Diagnostics.destroy(&node);                                                                   
    Set_BMS.destroy(&node);                                                                  
    KillSwitch.destroy(&node);  
    BMS_Reset.destroy(&node);                                   


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

// {0xAB, 0, 0XAC, 0}  disable charge and discharge 
// {0xAB, 1, 0XAC, 0}  enable charge, disable discharge
// {0xAB, 0, 0XAC, 1}  disable charge enable discharge 
// {0xAB, 1, 0XAC, 1}  enable charge and discharge
void Set_BMS_Call_Back(const void * msgin) {

    const std_msgs__msg__Int32MultiArray* command_frame = (const std_msgs__msg__Int32MultiArray *)msgin;              // IMPORTANT: DO NOT FORGET TO ADD THIS !!!

    // Access the data array
    size_t size = command_frame->data.size;
    const int32_t* command_data = command_frame->data.data;

    if(command_data[0] != 0xAB && command_data[2] != 0xAC) return;

    switch((uint8_t)command_data[1] << 8 | (uint8_t)command_data[3]){
      case 0:
        Serial.print("disabling charge/discharge");
        BMS_Diagnostics.publish("disabling charge/discharge");
        JKmessenger.setMOS_state(false, false); 
        break;
      case 1:
        Serial.print("enabling discharge disabling charge");
        BMS_Diagnostics.publish("enabling discharge disabling charge");
        JKmessenger.setMOS_state(true, false); 
        break; 
      case 256:
        Serial.print("enabling charge disabling discharge");
        BMS_Diagnostics.publish("enabling charge disabling discharge");
        JKmessenger.setMOS_state(false, true); 
        break;
      case 257:
        Serial.print("enabling charge/discharge");
        BMS_Diagnostics.publish("enabling charge/discharge");
        JKmessenger.setMOS_state(true, true); 
        break; 
    }
}

void Kill_Switch_Call_Back(const void * msgin) {

  const std_msgs__msg__Bool * Estop = (const std_msgs__msg__Bool *)msgin;                  // IMPORTANT: DO NOT FORGET TO ADD THIS !!!
  BMS_Diagnostics.publish("Emergency Stop Required - Disconnecting Battery");
  if(Estop->data == true){JKmessenger.setMOS_state(false, false);};                        

}

void Reset_Req_Call_Back(const void * msgin) {

  const std_msgs__msg__Bool * BMS_restart = (const std_msgs__msg__Bool *)msgin;                  // IMPORTANT: DO NOT FORGET TO ADD THIS !!!
  if(BMS_restart->data == true){JKmessenger.setMOS_state(false, false); 
    BMS_Diagnostics.publish("Reset Request Acknowleged - Resetting");
    DestroyEntities();
    ESP.restart();
   }
}
