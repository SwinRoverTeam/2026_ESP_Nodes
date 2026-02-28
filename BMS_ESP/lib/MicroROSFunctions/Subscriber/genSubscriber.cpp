#include "genSubscriber.h"
#include "datatype.h"
#include <std_msgs/msg/int32_multi_array.h>
#include <std_msgs/msg/float64_multi_array.h>

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){} else{} }
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){} }

genSubscriber::genSubscriber(){}

void genSubscriber::init(rcl_node_t * node, const char * topic, rclc_executor_t * executor, rclc_subscription_callback_t callback, DataType datatype){
    this->topic_name = topic;
    this->datatype = datatype;

    switch (datatype) {
        case INT:
            rclc_subscription_init_default(
                &subscriber,
                node,
                ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
                topic_name
            );

            rclc_executor_add_subscription(executor, &subscriber, &msg.intmsg, callback, ON_NEW_DATA);

        break;

        case DOUBLE:
            rclc_subscription_init_default(
                &subscriber,
                node,
                ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64),
                topic_name
            );

            rclc_executor_add_subscription(executor, &subscriber, &msg.doublemsg, callback, ON_NEW_DATA);

        break;

        case BOOL:
            rclc_subscription_init_default(
                &subscriber,
                node,
                ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
                topic_name
            );

            rclc_executor_add_subscription(executor, &subscriber, &msg.boolmsg, callback, ON_NEW_DATA);

        break;

        case INT32_ARRAY:
            

            rclc_subscription_init_default(
                &subscriber,
                node,
                ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
                topic_name
            );

            rclc_executor_add_subscription(executor, &subscriber, &msg.intArrmsg, callback, ON_NEW_DATA);

              msg.intArrmsg.data.data = (int32_t *) malloc(50 * sizeof(int32_t));
              msg.intArrmsg.data.capacity = 50;
              msg.intArrmsg.data.size = 0;

        break;

        case FLOAT64_ARRAY:
            
            rclc_subscription_init_default(
                &subscriber,
                node,
                ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64MultiArray),
                topic_name
            );

            rclc_executor_add_subscription(executor, &subscriber, &msg.doubleArrmsg, callback, ON_NEW_DATA);

              msg.doubleArrmsg.data.data = (double *) malloc(50 * sizeof(double));
              msg.doubleArrmsg.data.capacity = 50;
              msg.doubleArrmsg.data.size = 0;

        break;


        default:
            Serial.println("ERROR: Subscriber initialisation has failed!");
        break;
    }

    
}

void genSubscriber::destroy(rcl_node_t * node) {
    RCCHECK(rcl_subscription_fini(&subscriber, node));
}

