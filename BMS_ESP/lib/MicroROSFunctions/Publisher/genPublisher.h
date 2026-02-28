#pragma once

#include <Arduino.h>
#include <variant>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/float64.h>
#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/string.h>
#include <std_msgs/msg/int32_multi_array.h>
#include <std_msgs/msg/float64_multi_array.h>

#include "datatype.h"


class genPublisher {
public:

    genPublisher();
    void init(rcl_node_t * node, const char * topic, DataType datatype);
    void publish(int data);
    void publish(bool data);
    void publish(double data);
    void publish(const char* text);
    void publish(int32_t arr[], int INTArrLen);
    void publish(double arr[], int DBArrLen);


    void destroy(rcl_node_t * node);

private:
    rcl_publisher_t publisher;
    const char * topic_name;
    DataType data_type;
    const void* data;

};