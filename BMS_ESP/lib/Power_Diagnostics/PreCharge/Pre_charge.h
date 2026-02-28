#ifndef Pre_charge_H
#define Pre_charge_H
#include <Arduino.h>

class PreCharge{

public:
    //constructor
    PreCharge(int Channel_A, int Channel_B);
    bool contactor_status();

private: 
    void _initilize();
    int _Ch_A;
    int _Ch_B;
};

#endif