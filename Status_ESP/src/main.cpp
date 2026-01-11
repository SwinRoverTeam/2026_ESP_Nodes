#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <Ethernet.h>
#include <SPI.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include "Publisher/genPublisher.h"                     // Include this library to use the publisher      
#include "Subscriber/genSubscriber.h"                   // Include this library to use the Subscriber
#include <MicroROS_Transport.h>                         // IMPORTANT: MAKE SURE TO INCLUDE FOR CONNECTION BETWEEN THE ESP AND THE MICRO ROS AGENT TO WORK

#ifdef __AVR__
  #include <avr/power.h>
#endif

#define PIN 3           // Pin connected to NeoPixel

// Define W5500 Ethernet Chip Pins
#define W5500_CS    14    // CS (Chip Select) PIN
#define W5500_RST   9     // Reset PIN
#define W5500_INT   10    // Interrupt PIN 
#define W5500_MISO  12    // MISO PIN
#define W5500_MOSI  11    // MOSI PIN
#define W5500_SCK   13    // Serial Clock PIN

// Network Configuration
byte esp_mac[] = { 0xDE, 0xAD, 0xAF, 0x91, 0x3E, 0x42 };    // Mac address of ESP32 (Make sure its unique for each ESP32)
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
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}     // Checks for Errors in Micro ROS Setup
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}              // Checks for Errors in Micro ROS Setup
#define ARRAY_LEN(arr) { (sizeof(arr) / sizeof(arr[0])) }                                     // Calculate the Array Length (Needed for the Int32 Array Publisher)

// Define shared ROS entities
rcl_allocator_t allocator;
rclc_support_t support;
rclc_executor_t executor;
rcl_node_t node;
rmw_context_t* rmw_context;

// Define Node Name
const char * node_name = "nodeExample";

// Define MicroROS Subscriber and Publisher entities
genSubscriber MechFailure;                                  // Boolean Subscriber
genSubscriber HomeSuccess;                                  // Boolean Subscriber
genSubscriber AutoMode;                                     // Boolean Subscriber
genSubscriber ManMode;                                      // Boolean Subscriber
genSubscriber EStopBtn;                                     // Boolean Subscriber


// Define Callback functions for the Subscribers
void MechFailCallback(const void * msgin);
void HomeSuccessCallback(const void * msgin);
void AutoModeCallback(const void * msgin);
void ManModeCallback(const void * msgin);
void EStopCallback(const void * msgin);



// Connection status for the HandleConnection()
enum class ConnectionState {
  Initializing,
  WaitingForAgent,
  Connecting,
  Connected,
  Disconnected
};

ConnectionState connection_state = ConnectionState::Initializing;

// ================================================ LED SETUP ================================================ //

void colorWipe(uint32_t c);
void LEDStatus();



Adafruit_NeoPixel strip = Adafruit_NeoPixel(60, PIN, NEO_GRB + NEO_KHZ800);

enum class LEDStates {
  INITIALISING,
  HOMING,                               // Rover is homing                            (LED Colour = Magenta)
  IDLE,                                 // Rover is idle                              (LED Colour = White)
  AUTO_IDLE,                            // Rover is Automatic Mode (Homing)           (LED Colour = Cyan)
  AUTO_START,                           // Rover is in motion (Automatic Mode)        (LED Colour = Green)
  MANUAL,                               // Rover is in Manual Mode                    (LED Colour = Blue)
  STATE_ERR,                            // Conflicting LED States                     (LED Colour = Magenta)
  MECH_LOCK,                            // Mechanical Failure or Locked               (LED Colour = Yellow)
  LED_ERR,                              // LED Indicator Error (Connection issue...)  (LED Colour = Magenta)
  ROVER_ERR                             // Error in the Rover Network                 (LED Colour = Red)

};

struct StateControl {
    bool MechFail;
    bool HomeSuccess;
    bool ManModeOn;
    bool AutoModeOn;
    bool EStop;
} control;

LEDStates LED_status;

// Include the code for startinh up the ethernet chip and initialising the Micro ROS transport
void setup() {

  #if defined (__AVR_ATtiny85__)
    if (F_CPU == 16000000) clock_prescale_set(clock_div_1);
  #endif

  strip.begin();
  strip.setBrightness(50);
  strip.show();

  colorWipe(strip.Color(255, 0, 255));        // Magenta

  Serial.begin(115200);
  delay(1000);
  Serial.println("Starting Ethernet Connection... ");


  SPI.begin(W5500_SCK, W5500_MISO, W5500_MOSI, W5500_CS);                                   // Initialize SPI with custom pin configuration    
  Ethernet.init(W5500_CS);                                                                  // Select CS PIN and initialize Ethernet chip

  Serial.println("[INIT] Starting micro-ROS node...");
  set_microros_eth_transports(esp_mac, esp_ip, dns, gateway, agent_ip, agent_port);         // IMPORTANT: Start Micro ROS Transport Connection 

  delay(2000);

  connection_state = ConnectionState::WaitingForAgent;
  LED_status = LEDStates::HOMING;

};

void loop() {
  HandleConnectionState();
  LEDStatus();
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));

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
        // Serial.println("heartbeat");                                                      // Use it for testing if the code is working
        
        // ADD HERE FOR PUBLISHING VALUES CONTINUOUSL

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
  
  // ADD ALL YOUR PUBLISHERS AND SUBSCRIBER INITIALISATION HERE                                                       // Initialise String Publisher

  MechFailure.init(&node, "MechFailure", &executor, MechFailCallback, BOOL);                           // Initialise Boolean Subscriber
  HomeSuccess.init(&node, "HomeSuccess", &executor, HomeSuccessCallback, BOOL);                           // Initialise Boolean Subscriber
  AutoMode.init(&node, "AutoMode", &executor, AutoModeCallback, BOOL);                           // Initialise Boolean Subscriber
  ManMode.init(&node, "ManMode", &executor, ManModeCallback, BOOL);
  EStopBtn.init(&node, "EStopBtn", &executor, EStopCallback, BOOL);

  return true;
}



// This function destroys all micro ros entites (Publishers, Subscribers, Executors, Nodes, Support...)
void DestroyEntities() {
    rmw_context = rcl_context_get_rmw_context(&support.context);
    (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);
    
    // ADD FUNCTION THAT DESTROYS PUBLISHER AND SUBSCRIBER HERE

    MechFailure.destroy(&node);                           // Initialise Boolean Subscriber
    HomeSuccess.destroy(&node);                           // Initialise Boolean Subscriber
    AutoMode.destroy(&node);                           // Initialise Boolean Subscriber
    ManMode.destroy(&node);
    EStopBtn.destroy(&node);

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

// Example Callback funtion for Boolean values
void MechFailCallback(const void * msgin) {

  const std_msgs__msg__Bool * msg_bool = (const std_msgs__msg__Bool *)msgin;                  // IMPORTANT: DO NOT FORGET TO ADD THIS !!!

  // Enter code here for when the subscriber receives a message.
  control.MechFail = msg_bool->data;

  Serial.print("Mechanical Failure: ");
  Serial.println(control.MechFail);
}

void HomeSuccessCallback(const void * msgin) {

  const std_msgs__msg__Bool * msg_bool = (const std_msgs__msg__Bool *)msgin;                  // IMPORTANT: DO NOT FORGET TO ADD THIS !!!

  // Enter code here for when the subscriber receives a message.
  control.HomeSuccess = msg_bool->data;

  Serial.print("Homing Status: ");
  Serial.println(control.HomeSuccess);
}

void AutoModeCallback(const void * msgin) {

  const std_msgs__msg__Bool * msg_bool = (const std_msgs__msg__Bool *)msgin;                  // IMPORTANT: DO NOT FORGET TO ADD THIS !!!

  // Enter code here for when the subscriber receives a message.
  control.AutoModeOn = msg_bool->data;

  Serial.print("Auto Mode Status: ");
  Serial.println(control.AutoModeOn);
}

void ManModeCallback(const void * msgin) {

  const std_msgs__msg__Bool * msg_bool = (const std_msgs__msg__Bool *)msgin;                  // IMPORTANT: DO NOT FORGET TO ADD THIS !!!

  // Enter code here for when the subscriber receives a message.
  control.ManModeOn = msg_bool->data;

  Serial.print("Manual Mode Status: ");
  Serial.println(control.ManModeOn);
}

void EStopCallback(const void * msgin) {

  const std_msgs__msg__Bool * msg_bool = (const std_msgs__msg__Bool *)msgin;                  // IMPORTANT: DO NOT FORGET TO ADD THIS !!!

  // Enter code here for when the subscriber receives a message.
  control.EStop = msg_bool->data;

  Serial.print("EStop Status: ");
  Serial.println(control.EStop);
}



// ========================================= ESP32 RGB LED CODE ========================================= //
void colorWipe(uint32_t c) {
  for(uint16_t i = 0; i < strip.numPixels(); i++) {
    strip.setPixelColor(i, c);
    strip.show();
  }
}

void LEDStatus() {
  // State-aware actions
  switch(LED_status) {
    
    case LEDStates::INITIALISING:
      Serial.println("Current State: Initialising");

    break;

    case LEDStates::HOMING:
      Serial.println("Current State: HOMING");
      colorWipe(strip.Color(255, 0, 255));        // Magenta

      if (connection_state != ConnectionState::Connected) {
        LED_status = LEDStates::LED_ERR;
      }

      if (control.EStop) {
        LED_status = LEDStates::ROVER_ERR;
      }

      if (control.MechFail) {
        LED_status = LEDStates::MECH_LOCK;
      }

      if (control.HomeSuccess) {
        LED_status = LEDStates::IDLE;
      }
      
    break;

    case LEDStates::IDLE:
      Serial.println("Current State: IDLE");
      colorWipe(strip.Color(255, 255, 255));      // White

      if (connection_state != ConnectionState::Connected) {
        LED_status = LEDStates::LED_ERR;
      }

      if (control.EStop) {
        LED_status = LEDStates::ROVER_ERR;
      }

      if (control.AutoModeOn && !control.ManModeOn) {
        LED_status = LEDStates::AUTO_IDLE;
      }

      if (control.ManModeOn && !control.AutoModeOn) {
        LED_status = LEDStates::MANUAL;
      }

      if (control.AutoModeOn && control.ManModeOn) {
        LED_status = LEDStates::STATE_ERR;
      }

    break;

    case LEDStates::AUTO_IDLE:
      Serial.println("Current State: AUTO_IDLE");
      colorWipe(strip.Color(0, 255, 255));        // Cyan

      delay(5000);

      LED_status = LEDStates::AUTO_START;

    break;

    case LEDStates::AUTO_START:
      Serial.println("Current State: AUTO_START");
      colorWipe(strip.Color(0, 255, 0));          // Green

      if (connection_state != ConnectionState::Connected) {
        LED_status = LEDStates::LED_ERR;
      }

      if (control.EStop) {
        LED_status = LEDStates::ROVER_ERR;
      }

      if (control.MechFail) {
        LED_status = LEDStates::MECH_LOCK;
      }

      if (control.ManModeOn) {
        LED_status = LEDStates::STATE_ERR;
      }

      if (!control.AutoModeOn) {
        LED_status = LEDStates::IDLE;
      }

    break;

    case LEDStates::MANUAL:
      Serial.println("Current State: MANUAL");
      colorWipe(strip.Color(0, 0, 255));          // Blue

      if (connection_state != ConnectionState::Connected) {
        LED_status = LEDStates::LED_ERR;
      }

      if (control.EStop) {
        LED_status = LEDStates::ROVER_ERR;
      }

      if (control.MechFail) {
        LED_status = LEDStates::MECH_LOCK;
      }

      if (control.AutoModeOn) {
        LED_status = LEDStates::STATE_ERR;
      }

      if (!control.ManModeOn) {
        LED_status = LEDStates::IDLE;
      }

    break;

    case LEDStates::STATE_ERR:
      Serial.println("Current State: STATE_ERR");
      colorWipe(strip.Color(255, 0, 255));        // Magenta

      if (connection_state != ConnectionState::Connected) {
        LED_status = LEDStates::LED_ERR;
      }

      if (control.EStop) {
        LED_status = LEDStates::ROVER_ERR;
      }

      if (control.MechFail) {
        LED_status = LEDStates::MECH_LOCK;
      }
      
    break;

    case LEDStates::MECH_LOCK:
      Serial.println("Current State: MECH_LOCK");
      colorWipe(strip.Color(255, 255, 0));        // Yellow

      if (connection_state != ConnectionState::Connected) {
        LED_status = LEDStates::LED_ERR;
      }

      if (control.EStop) {
        LED_status = LEDStates::ROVER_ERR;
      }
      
    break;

    case LEDStates::LED_ERR:
      Serial.println("Current State: LED_ERR");
      colorWipe(strip.Color(255, 0, 255));        // Magenta

      if (connection_state == ConnectionState::Connected) {
        LED_status = LEDStates::HOMING;
      }

      if (connection_state == ConnectionState::Connected && control.HomeSuccess ) {
        LED_status = LEDStates::IDLE;
      }

      if (connection_state == ConnectionState::Connected && control.AutoModeOn) {
        LED_status = LEDStates::AUTO_START;
      }

      if (connection_state == ConnectionState::Connected && control.ManModeOn) {
        LED_status = LEDStates::MANUAL;
      }

      if (connection_state == ConnectionState::Connected && control.AutoModeOn && control.ManModeOn){
        LED_status = LEDStates::STATE_ERR;
      }

      if (connection_state == ConnectionState::Connected && control.MechFail) {
        LED_status = LEDStates::MECH_LOCK;
      }

      if (connection_state == ConnectionState::Connected && control.EStop) {
        LED_status = LEDStates::ROVER_ERR;
      }
      
    break;

    case LEDStates::ROVER_ERR:
      Serial.println("Current State: ROVER_ERR");
      colorWipe(strip.Color(255, 0, 0));          // Red

      if (connection_state != ConnectionState::Connected) {
        LED_status = LEDStates::LED_ERR;
      }
      
    break;
  }

}
