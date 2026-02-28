#include "Pre_charge.h"

PreCharge::PreCharge(int Channel_A, int Channel_B) : _Ch_A(Channel_A), _Ch_B(Channel_B){_initilize();}

bool PreCharge::contactor_status(){
  return digitalRead(_Ch_A) && digitalRead(_Ch_B) ? true : false; 
}

void PreCharge::_initilize(){  
  pinMode(_Ch_A, INPUT_PULLDOWN);
  pinMode(_Ch_B, INPUT_PULLDOWN);
}




