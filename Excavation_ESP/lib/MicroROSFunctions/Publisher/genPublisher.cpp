#include "genPublisher.h"
#include "datatype.h"



genPublisher::genPublisher(){}

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){} else{} }
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){} }

void genPublisher::init(rcl_node_t * node, const char * topic, DataType datatype){
    this->topic_name = topic;
    this->data_type = datatype;

    switch (datatype) {
        case DataType::BOOL:
            
            rclc_publisher_init_default(
                &publisher,
                node,
                ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
                topic_name

            );
        break;

        case DataType::INT:
            
            rclc_publisher_init_default(
                &publisher,
                node,
                ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
                topic_name
                
            );
            
        break;

        case DataType::DOUBLE:

            rclc_publisher_init_default(
                &publisher,
                node,
                ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64),
                topic_name
               
            );

        break;

        case DataType::STRING:
            rclc_publisher_init_default(
                &publisher,
                node,
                ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
                topic_name
               
            );
        break;

        case DataType::INT32_ARRAY:
            rclc_publisher_init_default(
                &publisher,
                node,
                ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
                topic_name
               
            );
        break;

        case DataType::FLOAT64_ARRAY:
            rclc_publisher_init_default(
                &publisher,
                node,
                ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64MultiArray),
                topic_name
               
            );
        break;

        default:
            while (1)
            {
                delay(2000);
                Serial.println("ERROR: Code doesn't work.");

            }
        break;
    }
};


void genPublisher::publish(int data) {
    if(data_type == INT){
        std_msgs__msg__Int32 msg_int;
        msg_int.data = data;  // Assign to the message
        RCCHECK(rcl_publish(&publisher, &msg_int, NULL));

    } else {
        //some error handling
    }

}

void genPublisher::publish(bool data) {
    if(data_type == BOOL) {
        std_msgs__msg__Bool msg_bool;
        msg_bool.data = data;  // Assign to the message
        RCCHECK(rcl_publish(&publisher, &msg_bool, NULL));
    } else {
        //some error handling
    }

}

void genPublisher::publish(double data) {
    if(data_type == DOUBLE) {
        std_msgs__msg__Float64 msg_doub;
        msg_doub.data = data;  // Assign to the message
        RCCHECK(rcl_publish(&publisher, &msg_doub, NULL));
    } else {
        //some error handling
    }
}

void genPublisher::publish(const char* text) {
    if(data_type == STRING) {
        std_msgs__msg__String msg_str;

        size_t len = strlen(text);
        

        msg_str.data.capacity = len + 1;
        msg_str.data.size = len;
        msg_str.data.data = (char*) malloc(msg_str.data.capacity * sizeof(char));

        memcpy(msg_str.data.data, text, msg_str.data.capacity); // includes '\0'

        RCCHECK(rcl_publish(&publisher, &msg_str, NULL));

    } else {
        //some error handling
    }
}


void genPublisher::publish(int32_t arr[], int INTArrLen) {
    if (data_type == INT32_ARRAY) {
        std_msgs__msg__Int32MultiArray IntArr;

        IntArr.data.capacity = INTArrLen + 1;
        IntArr.data.size = INTArrLen;
        IntArr.data.data = (int32_t *) malloc(INTArrLen * sizeof(int32_t));

        IntArr.layout.dim.capacity = 0;
        IntArr.layout.dim.size = 0;
        IntArr.layout.dim.data = NULL;
        IntArr.layout.data_offset = 0;

        memcpy(IntArr.data.data, arr, INTArrLen * sizeof(int32_t));
        
        RCCHECK(rcl_publish(&publisher, &IntArr, NULL));
        // free(IntArr.data.data);                                          // Might remove it; 

    } else {
        //some error handling
    }
}

void genPublisher::publish(double arr[], int DBArrLen) {
    if (data_type == FLOAT64_ARRAY) {
        std_msgs__msg__Float64MultiArray DoubleArr;

        DoubleArr.data.capacity = DBArrLen + 1;
        DoubleArr.data.size = DBArrLen;
        DoubleArr.data.data = (double *) malloc(DBArrLen * sizeof(double));

        DoubleArr.layout.dim.capacity = 0;
        DoubleArr.layout.dim.size = 0;
        DoubleArr.layout.dim.data = NULL;
        DoubleArr.layout.data_offset = 0;

        memcpy(DoubleArr.data.data, arr, DBArrLen * sizeof(double));
        
        RCCHECK(rcl_publish(&publisher, &DoubleArr, NULL));
        // free(msgArr.data.data);                                          // Might remove it

    } else {
        //some error handling
    }
}

void genPublisher::destroy(rcl_node_t * node) {

    // might add free() here instead...

    RCCHECK(rcl_publisher_fini(&publisher, node));
}