#ifndef GEN_SUBSCRIBER_H
#define GEN_SUBSCRIBER_H

#include <Arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/float64.h>
#include <std_msgs/msg/int32_multi_array.h>
#include <std_msgs/msg/float64_multi_array.h>
#include "datatype.h"

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();} }
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){} }

class genSubscriber{
public:
    genSubscriber();
    void init(rcl_node_t * node, const char * topic, rclc_executor_t * executor, rclc_subscription_callback_t callback, DataType datatype);
    void destroy(rcl_node_t * node);
    
private:
    rcl_subscription_t subscriber;
    const char * topic_name;
    DataType datatype;
    DataMsg msg;
    rclc_subscription_callback_t callback;

};

#endif // INT_SUBSCRIBER_H
