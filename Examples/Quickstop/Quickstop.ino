// Quickstop.pde
// -*- mode: C++ -*-
//
// Check stop handling.
// Calls stop() while the stepper is travelling at full speed, causing
// the stepper to stop as quickly as possible, within the constraints of the
// current acceleration.
//
// Copyright (C) 2015  Alexander Chestnov

#include <stepperQ.h>


int dir_pin = 7;
int step_pin = 6 ;

void setup()
{  
  stepperq.setMaxSpeed(150);
  stepperq.setAcceleration(900);
  stepperq.moveTo(4000);
  stepperq.start();  
}

void loop()
{
  delay(1000);
  stepperq.stop();  

}
