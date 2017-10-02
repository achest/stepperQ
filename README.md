# stepperQ
Arduino SteperMotor Driver. Acceleration and intterupt


ESP 8266 Support Added.


Testet with:
WemosD1 Mini
Arduino Nano
Arduino Uno.

Drivers:
-TB5660 Driver  ( Not gut very Idea)
-DRV8825  https://www.pololu.com/product/2133  
-L298E   https://coeleveld.com/arduino-stepper-l298n/  ( Not reccomendet!!!) 
-uln2003a  https://coeleveld.com/arduino-stepper-uln2003a/


Example Code:

#include <stepperQ.h>

int dir_pin = D3;
int step_pin = D4 ;

void setup() {
  
  stepperq.init(dir_pin,step_pin);
  stepperq.setMaxSpeed(800);
  stepperq.setAcceleration(4800);
  stepperq.moveTo(80000);
    
  stepperq.start();
  delay(40000);
}

void loop() {
    //your code hier.
   delay(2000);

}
For contueres run:


void loop () {

  if ( stepperq.distanceToGo() <4000 ) {
      stepperq.move(8000);
  }
	//your code hier.

}
