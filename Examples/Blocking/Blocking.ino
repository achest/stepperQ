
// Copyright (C) 2015  Alexander Chestnov

#include <stepperQ.h>

int c = 0;



int dir_pin = 7;
int step_pin = 6 ;

void setup() {
  
  stepperq.init(dir_pin,step_pin);
  stepperq.setMaxSpeed(200);
  stepperq.setAcceleration(9900);
   
}
void loop () {

  if ( stepperq.distanceToGo() == 0  ) {
  
   if (c%2 == 0)
    {
        stepperq.moveTo(400);
        stepperq.start();  
    }
    else {
        stepperq.moveTo(0);
        stepperq.start();  
    }
    
  c++;
  }   
  
  delay(100);

}
