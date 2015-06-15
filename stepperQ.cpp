//stepperQ
// Copyright (C) 2015 Alexander.Chestnov 

#include "stepperQ.h"


StepperQ::StepperQ(uint8_t dirpin, uint8_t steppin)
{
    _currentPos = 0;
    _targetPos = 0;
    _speed = 0.0;
    _maxSpeed = 1.0;
    _acceleration = 1.0;
    _sqrt_twoa = 1.0;
    _stepInterval = 0;
    _minPulseWidth = 1;
    _enablePin = 0xff;
    _lastStepTime = 0;
    _dirpin = dirpin;
    _steppin = steppin;
   

    // NEW
    _n = 0;
    _c0 = 0.0;
    _cn = 0.0;
    _cmin = 1.0;
    _direction = DIRECTION_CCW;

    enableOutputs();
}


void StepperQ::moveTo(long absolute)
{
    if (_targetPos != absolute)
    {
	_targetPos = absolute;

	computeNewSpeed();
	// compute new n?
    }
}

void StepperQ::move(long relative)
{
    moveTo(_currentPos + relative);
}

long StepperQ::distanceToGo()
{
    return _targetPos - _currentPos;
}

long StepperQ::targetPosition()
{
    return _targetPos;
}

long StepperQ::currentPosition()
{
    return _currentPos;
}

// Useful during initialisations or after initial positioning
// Sets speed to 0
void StepperQ::setCurrentPosition(long position)
{
    _targetPos = _currentPos = position;
    _n = 0;
    _stepInterval = 0;
}

void StepperQ::callback(){

    stepUp();

    stepDown();

}

void StepperQ:: stepUp() {

// digitalWrite(_dirpin, _direction ==1 ? 1:0);	
    digitalWrite(_steppin, HIGH);	
}
void StepperQ:: stepDown() {

    digitalWrite(_steppin, LOW);	

}

void StepperQ::stop()
{
   // move(abs(_n));  
   _direction ==1 ? abs(_n):-abs(_n);
  
}

