// AccelStepper1.cpp
//
// Copyright (C) 2009-2013 Mike McCauley
// $Id: AccelStepper1.cpp,v 1.17 2013/08/02 01:53:21 mikem Exp mikem $

#include "AccelStepper1.h"


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

// Run the motor to implement speed and acceleration in order to proceed to the target position
// You must call this at least once per step, preferably in your main loop
// If the motor is in the desired position, the cost is very small
// returns true if we are still running to position
boolean AccelStepper1::run()
{
    if (runSpeed())
	computeNewSpeed();
    return true;
}


// Implements steps according to the current step interval
// You must call this at least once per step
// returns true if a step occurred
boolean AccelStepper1::runSpeed()
{
    // Dont do anything unless we actually have a step interval
    if (!_stepInterval)
	return false;

    unsigned long time = micros();
    // Gymnastics to detect wrapping of either the nextStepTime and/or the current time
    unsigned long nextStepTime = _lastStepTime + _stepInterval;
    if (   ((nextStepTime >= _lastStepTime) && ((time >= nextStepTime) || (time < _lastStepTime)))
	|| ((nextStepTime < _lastStepTime) && ((time >= nextStepTime) && (time < _lastStepTime))))

    {
	_currentPos += DIRECTION_CW;
	step();

	_lastStepTime = time;
	return true;
    }
    else
    {
	return false;
    }
}



void AccelStepper1::computeNewSpeed()
{
    long distanceTo = distanceToGo(); // +ve is clockwise from curent location

   // long stepsToStop = _stepsToStop;
   // (long)((_speed * _speed) / (2.0 * _acceleration)); // Equation 16

    if (distanceTo == 0 && _n <= 1)
    {
	// We are at the target and its time to stop
	_stepInterval = 0;
	_speed = 0.0;
	_n = 0;
	return;
    }
    
 	

    if (distanceTo > 0)
    {
	// We are anticlockwise from the target
	// Need to go clockwise from here, maybe decelerate now
	if (_n > 0)
	{
	    // Currently accelerating, need to decel now? Or maybe going the wrong way?
	    if ((_n >= distanceTo) || _direction == DIRECTION_CCW)
		_n = -_n ; // Start deceleration
	}
	else if (_n < 0)
	{
	    // Currently decelerating, need to accel again?
	    if ((-_n < distanceTo) && _direction == DIRECTION_CW)
		_n = -_n; // Start accceleration
	}
    }
    else if (distanceTo < 0)
    {
	// We are clockwise from the target
	// Need to go anticlockwise from here, maybe decelerate
	if (_n > 0)
	{
	    // Currently accelerating, need to decel now? Or maybe going the wrong way?
	    if ((_n >= -distanceTo) || _direction == DIRECTION_CW)
		_n = -_n; // Start deceleration
	}
	else if (_n < 0)
	{
	    // Currently decelerating, need to accel again?
	    if ((-_n < -distanceTo) && _direction == DIRECTION_CCW)
		_n = -_n; // Start accceleration
	}
    }

    // Need to accelerate or decelerate
    if (_n == 0)
    {
	//Serial.print("\n first step"); 
	// First step from stopped
	_cn = _c0;
	_direction = (distanceTo > 0) ? DIRECTION_CW : DIRECTION_CCW;
	changeDirection() ;
        _stepInterval = _cn;
	_n++;
    }
    else  if (_n > 0 && _stepInterval > _cmin ) {


//	Serial.print("  2222_n=");     
//	Serial.print(_n);
	// Subsequent step. Works for accel (n is +_ve) and decel (n is -ve).
	_cn = _cn - ((2.0 * _cn) / ((4.0 * _n) + 1)); // Equation 13
//	_cn = max(_cn, _cmin); 
	//}
        _stepInterval = _cn;
	_n++;
    } else if (_n <= 0) {

	_cn = _cn - ((2.0 * _cn) / ((4.0 * _n) + 1)); // Equation 13
        _stepInterval = _cn;
	_n++;
	}
    

///    if (_n < _stepsToStop ) { Serial.print("  33333333_n="); 
//			    Serial.print(_n);
//	      _n++;
//		Serial.print("  44444444_n="); 
//    Serial.print(_n);
	
//      }

//    _speed = 1000000.0 / _cn;
//    _stepsToStop = (long)((_speed * _speed) / (2.0 * _acceleration)); // Equation 16

//    if (_direction == DIRECTION_CCW)
//	_speed = -_speed;

//     Serial.print("\n _stepsToStop="); 
//    Serial.print(_stepsToStop);	
//    Serial.print("\n  _n="); 
//    Serial.print(_n);
//    Serial.print(" _cn="); 
//    Serial.print(_cn);
//    Serial.print(" distanceTo="); 
//    Serial.print(distanceTo);
}




void AccelStepper1::setMaxSpeed(float speed)
{
    if (_maxSpeed != speed)
    {
	_maxSpeed = speed;
	_cmin = 1000000.0 / speed;

  	 _speed = 1000000.0 / _cn;
	 _stepsToStop = (long)((speed * speed) / (2.0 * _acceleration));



	// Recompute _n from current speed and adjust speed if accelerating or cruising
	if (_n > 0)
	{
	    _n = (long)((_speed * _speed) / (2.0 * _acceleration)); // Equation 16
           
	    computeNewSpeed();
	}
    }
}

void AccelStepper1::setAcceleration(float acceleration)
{
    if (acceleration == 0.0)
	return;
    if (_acceleration != acceleration)
    {
	// Recompute _n per Equation 17
	_n = _n * (_acceleration / acceleration);
        _stepsToStop = (long)((_maxSpeed * _maxSpeed) / (2.0 * _acceleration));
	// New c0 per Equation 7
	_c0 = sqrt(2.0 / acceleration) * 1000000.0;
	_acceleration = acceleration;
	
	computeNewSpeed();
    }
}

void AccelStepper1::setSpeed(float speed)
{
    if (speed == _speed)
        return;
    speed = constrain(speed, -_maxSpeed, _maxSpeed);
    if (speed == 0.0)
	_stepInterval = 0;
    else
    {
	_stepInterval = fabs(1000000.0 / speed);
	_direction = (speed > 0.0) ? DIRECTION_CW : DIRECTION_CCW;
	changeDirection();
    }
    _speed = speed;
}

float AccelStepper1::speed()
{
    return _speed;
}



// You might want to override this to implement eg serial output
// bit 0 of the mask corresponds to _pin[0]
// bit 1 of the mask corresponds to _pin[1]
// ....
void AccelStepper1::setOutputPins(uint8_t mask)
{
   
}

void AccelStepper1::changeDirection()
{
   digitalWrite(_dirpin, _direction ==1 ? 1:0);
	
}

// 1 pin step function (ie for stepper drivers)
// This is passed the current step number (0 to 7)
// Subclasses can override
void AccelStepper1::step()
{
   // digitalWrite(_dirpin, _direction ==1 ? 1:0);	
    digitalWrite(_steppin, HIGH);	

    // Caution 200ns setup time 
    // Delay the minimum allowed pulse width
    delayMicroseconds(_minPulseWidth);
    digitalWrite(_steppin, LOW);	


}


    
// Prevents power consumption on the outputs
void    AccelStepper1::disableOutputs()
{   
    if (! _interface) return;

    setOutputPins(0); // Handles inversion automatically
    if (_enablePin != 0xff)
        digitalWrite(_enablePin, LOW ^ _enableInverted);
}

void    AccelStepper1::enableOutputs()
{
    pinMode(_dirpin, OUTPUT);
    pinMode(_steppin, OUTPUT);
}

void AccelStepper1::setMinPulseWidth(unsigned int minWidth)
{
    _minPulseWidth = minWidth;
}

void AccelStepper1::setEnablePin(uint8_t enablePin)
{
    _enablePin = enablePin;

    // This happens after construction, so init pin now.
    if (_enablePin != 0xff)
    {
        pinMode(_enablePin, OUTPUT);
        digitalWrite(_enablePin, HIGH ^ _enableInverted);
    }
}

void AccelStepper1::setPinsInverted(bool directionInvert, bool stepInvert, bool enableInvert)
{
    _pinInverted[0] = stepInvert;
    _pinInverted[1] = directionInvert;
    _enableInverted = enableInvert;
}

void AccelStepper1::setPinsInverted(bool pin1Invert, bool pin2Invert, bool pin3Invert, bool pin4Invert, bool enableInvert)
{    
    _pinInverted[0] = pin1Invert;
    _pinInverted[1] = pin2Invert;
    _pinInverted[2] = pin3Invert;
    _pinInverted[3] = pin4Invert;
    _enableInverted = enableInvert;
}

// Blocks until the target position is reached and stopped
void AccelStepper1::runToPosition()
{
    while (_speed != 0 || distanceToGo() != 0)
	run();
}

boolean AccelStepper1::runSpeedToPosition()
{
    if (_targetPos == _currentPos)
	return false;
    if (_targetPos >_currentPos)
	_direction = DIRECTION_CW;
    else
	_direction = DIRECTION_CCW;
    changeDirection();	
    return runSpeed();
}

// Blocks until the new target position is reached
void AccelStepper1::runToNewPosition(long position)
{
    moveTo(position);
    runToPosition();
}

void AccelStepper1::stop()
{
    move(abs(_n));  
    return;
  if (_speed != 0.0)
    {    
	long stepsToStop = (long)((_speed * _speed) / (2.0 * _acceleration)) + 1; // Equation 16 (+integer rounding)
	if (_speed > 0)
	    move(stepsToStop);
	else
	    move(-stepsToStop);
    }
}
long AccelStepper1::stepsToStop()
{
    return abs(_n);

}
