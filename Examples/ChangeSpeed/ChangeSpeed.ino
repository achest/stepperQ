
// Copyright (C) 2015  Alexander Chestnov

#include <stepperQ.h>

int c = 0;

int dir_pin = 7;
int step_pin = 6 ;

void setup() {
  
  stepperq.init(dir_pin,step_pin);
  stepperq.setMaxSpeed(200);
  stepperq.setAcceleration(9900);
  stepperq.moveTo(8000);
    
  stepperq.start();
   
}
void loop () {

  if ( stepperq.distanceToGo() <4000 ) {
      stepperq.move(8000);
  }
  
  return ;  
  
  if (c%2 == 0)
  {
      stepperq.setMaxSpeed(200);
      
  }
  else {
  
    stepperq.setMaxSpeed(300);
  }
  
  c++;
  delay(1000);

}
