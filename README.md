# MicroROS Template
This repository contains a sample code on creating multiple publishers and subscribers under multiple datatypes.

## Current Libraries Used
The following libraries is used for this project:
- External Libraries:
  - [Ethernet](https://docs.arduino.cc/libraries/ethernet/#Ethernet%20Class)
  - [Micro ROS for PlatformIO](https://github.com/micro-ROS/micro_ros_platformio)
- Custom Libraries (make sure its present under the lib folder.)
  - CustomTransport      (handles the communication between the MicroROS agent and the ESP32)
  - MicroROSFunctions    (allows you to create multiple publishers and subscribers)

## Current available functions
- Publisher
  - .init() = initialises the Publisher
  - .publish() = Publish a value under a topic
  - .destroy() = destroy publisher
- Subscriber
  - .init() = initialises the Subscriber
  - .destroy() =  destroy Subscriber
- Other
  - RCCHECK(fn) = checks for errors in the MicroROS functions
  - RCSOFTCHECK(fn) = soft checks for errors in the MicroROS functions
  - ARRAY_LEN(arr) = calculates the array length.

## Current Message types availables
The following messages types can used for both publishers and subscribers functions:
- Boolean
- Int32 (Signed Integers)
- Double
- Int32 Arrays
- String (Publishers only)

## Additional Notes
- Callback functions are needed for subscribers and must be created in the main.cpp. It then must be added when initialising the subscriber.
- If publishing an Int32 Array, Array length must be known (you use the ARRAY_LEN function)
- If multiple ESPs are used, make sure the IP address and the MAC address of the ESP is unique.
