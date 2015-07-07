// ProportionalControl.pde
// -*- mode: C++ -*-
//
// Make a single stepper follow the analog value read from a pot or whatever
// The stepper will move at a constant speed to each newly set posiiton, 
// depending on the value of the pot.
//
// Copyright (C) 2015  Alexander Chestnov

#include <stepperQ.h>



// Define a stepper and the pins it will use

int dir_pin = 7;
int step_pin = 6 ;


// This defines the analog input pin for reading the control voltage
// Tested with a 10k linear pot between 5v and GND
#define ANALOG_IN A0

void setup()
{  
  stepperq.init(dir_pin,step_pin);
  stepperq.setAcceleration(9900);
  stepperq.setMaxSpeed(1000);
  stepperq.moveTo(8000);
   
   stepperq.start();
}

void loop()
{
  if ( stepperq.distanceToGo() <4000 ) {
   stepperq.move(8000);

  }
  
  // Read new Speed 
  int analog_in = analogRead(ANALOG_IN);
  stepperq.setMaxSpeed(analog_in);
  delay(50); 
}
