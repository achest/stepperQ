//stepperQ
// Copyright (C) 2015 Alexander.Chestnov 

#include "stepperQ.h"


AccelStepper1::AccelStepper1(uint8_t dirpin, uint8_t steppin)
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


void AccelStepper1::moveTo(long absolute)
{
    if (_targetPos != absolute)
    {
	_targetPos = absolute;

	computeNewSpeed();
	// compute new n?
    }
}

void AccelStepper1::move(long relative)
{
    moveTo(_currentPos + relative);
}

long AccelStepper1::distanceToGo()
{
    return _targetPos - _currentPos;
}

long AccelStepper1::targetPosition()
{
    return _targetPos;
}

long AccelStepper1::currentPosition()
{
    return _currentPos;
}

// Useful during initialisations or after initial positioning
// Sets speed to 0
void AccelStepper1::setCurrentPosition(long position)
{
    _targetPos = _currentPos = position;
    _n = 0;
    _stepInterval = 0;
}
