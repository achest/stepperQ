
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
  
   switch (c%4) {
   
	case 0:
        stepperq.moveTo(200);
        stepperq.start();  
        break;
	case 1: 
        stepperq.moveTo(400);
        stepperq.start();  
        break;
    case 2:
        stepperq.moveTo(200);
        stepperq.start();  
        break;
    case 3:
        stepperq.moveTo(0);
        stepperq.start();  
        break;
        
    }
  c++;
  }   
	//do Something other...
  
  delay(100);

}
