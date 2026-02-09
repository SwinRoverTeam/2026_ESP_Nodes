#pragma once

enum DataType {
    INT,
    DOUBLE,
    BOOL,
    STRING,
    INT32_ARRAY,
    FLOAT64_ARRAY
};

union DataMsg {
    std_msgs__msg__Int32 intmsg;
    std_msgs__msg__Float64 doublemsg;
    std_msgs__msg__Bool boolmsg;
    std_msgs__msg__Int32MultiArray intArrmsg;
    std_msgs__msg__Float64MultiArray doubleArrmsg;

};

struct AnyValue {
    DataType type;
    DataMsg msg;
};