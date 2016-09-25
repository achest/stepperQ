//stepperQ
// Copyright (C) 2015 Alexander.Chestnov 

#include "stepperQ.h"


StepperQ stepperq;              // preinstatiate

ISR(TIMER1_OVF_vect)          // interrupt service routine that wraps a user defined function supplied by attachInterrupt
{
  stepperq.isrCallback();
}



void StepperQ::init(uint8_t dirpin, uint8_t steppin)
{
    _currentPos = 0;
    _targetPos = 0;
    _maxSpeed = 1.0;
    _acceleration = 1.0;
    _sqrt_twoa = 1.0;
   
   
    _pin[1] = dirpin;
    _pin[0] = steppin;
   

    // NEW
    _n = 0;
    _c0 = 0.0;
    _cn = 0.0;
    _cmin = 1.0;
    _direction = DIRECTION_CCW;

    pinMode(_enablePin, OUTPUT);

    pinMode(_pin[1], OUTPUT);
    pinMode(_pin[0], OUTPUT);
    _debug = false;
}
void StepperQ::init(uint8_t pin1, uint8_t pin2,uint8_t pin3, uint8_t pin4,uint8_t interface)
{


    _interface = interface;

    pinMode(_pin[2], OUTPUT);
    pinMode(_pin[3], OUTPUT);
     _pin[2] = pin3;
    _pin[3] =  pin4;
    init (pin1,pin2);
    //Serial.print("\n init4"); 
    //Serial.print(_interface); 

}



void StepperQ::moveTo(long absolute)
{
    if (_targetPos != absolute)
    {
	_targetPos = absolute;
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
     
    _currentPos = position;
    
}



void StepperQ::stop()
{
   if (_debug) 
         Serial.print("\n StepperQ:stop"); 
    move(abs(_n));  
   //_direction ==1 ? abs(_n):-abs(_n);
  
}

void StepperQ::start(){
	
  if (_n==0 && distanceToGo() != 0) { 
 
	  _cn = _c0;
	_direction = (distanceToGo() > 0) ? DIRECTION_CW : DIRECTION_CCW;
	 changeDirection() ;
	 step(HIGH);
         setPeriod(_cn);
	_n++;
	 initTimer(_cn);
	if (_debug) 
         Serial.print("\n first step"); 
       delayMicroseconds(_cmin); 
      step(LOW);
  }

}

void StepperQ::setMaxSpeed(float speed)
{
	_maxSpeed = speed;
	_cmin = 1000000.0 / speed;
        _stepsToStop = (long)((speed * speed) / (2.0 * _acceleration));

}
float StepperQ::maxSpeed() {
   return _maxSpeed;
}
float StepperQ::speed() {
   return 1000000.0/_cn;
}

void StepperQ::setAcceleration(float acceleration)
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
    }
}

float StepperQ::getAcceleration() {
	
	return _acceleration;
	
	}
long StepperQ::stepsToStop () {
	return abs(_n);
}
long StepperQ::maxstepsToStop() {

     return _stepsToStop ;
}

void StepperQ::isrCallback(){

   step(HIGH);
   _currentPos += _direction;
   calculateSpeed();
	
    step(LOW);

}	
void StepperQ::calculateSpeed() {

    long distanceTo = distanceToGo(); // +ve is clockwise from curent location

if (_debug) {
    Serial.print("\n n=");
    Serial.print(_n);
   Serial.print(" _cn=");
    Serial.print(_cn);
}
    float cnalt= _cn;
    if (distanceTo == 0 && _n <= 1)
    {
	// We are at the target and its time to stop
	_n = 0;
        stopTimer();
     if (_debug) {
     Serial.print(" Stopped");
 }
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

     if (_n == 0)
    {
	//Serial.print("\n first step"); 
	// First step from stopped
	_cn = _c0;
	_direction = (distanceTo > 0) ? DIRECTION_CW : DIRECTION_CCW;
	changeDirection() ;
         setPeriod(_cn);
	_n++;
    }
    else  if (_n > 0 && _cn > _cmin ) {


//	Serial.print("  2222_n=");     
//	Serial.print(_n);
	// Subsequent step. Works for accel (n is +_ve) and decel (n is -ve).
	_cn = _cn - ((2.0 * _cn) / ((4.0 * _n) + 1)); // Equation 13
        _cn = max(_cn, _cmin);
	
	_n++;
    }  else if (_n > 0 && _cn < _cmin) {  // speed was changed. Need no decel
           
	 	_cn = _cn + ((2.0 * _cn) / ((4.0 * _n) + 1)); // Equation 13
		_n--;	    
     }

     else if (_n <= 0) {

	_cn = _cn - ((2.0 * _cn) / ((4.0 * _n) + 1)); // Equation 13
	_n++;
	}

  // if (abs(cnalt - _cn )>10) {
  //        setPeriod(_cn);
  //  }
             setPeriod(_cn);


}


void StepperQ::changeDirection()
{
	if (_reverse)  {
		digitalWrite(_pin[1], _direction ==1 ? 0:1);
	} else {
		//Normal
		digitalWrite(_pin[1], _direction ==1 ? 1:0);
		}
	
}

int StepperQ::getDirection()
{
    return _direction;
}
void StepperQ::setDirOrder(boolean reverse ) {

	_reverse =reverse ;
}

void StepperQ::setPeriod(long microseconds)
{
  long cycles = (F_CPU * microseconds) / 2000000;                                // the counter runs backwards after TOP, interrupt is at BOTTOM so divide microseconds by 2
  if(cycles < RESOLUTION)              clockSelectBits = _BV(CS10);              // no prescale, full xtal
  else if((cycles >>= 3) < RESOLUTION) clockSelectBits = _BV(CS11);              // prescale by /8
  else if((cycles >>= 3) < RESOLUTION) clockSelectBits = _BV(CS11) | _BV(CS10);  // prescale by /64
  else if((cycles >>= 2) < RESOLUTION) clockSelectBits = _BV(CS12);              // prescale by /256
  else if((cycles >>= 2) < RESOLUTION) clockSelectBits = _BV(CS12) | _BV(CS10);  // prescale by /1024
  else        cycles = RESOLUTION - 1, clockSelectBits = _BV(CS12) | _BV(CS10);  // request was out of bounds, set as maximum
  ICR1 = cycles;                                                     // ICR1 is TOP in p & f correct pwm mode
  TCCR1B &= ~(_BV(CS10) | _BV(CS11) | _BV(CS12));
  TCCR1B |= clockSelectBits;                                                     // reset clock select register
}

void StepperQ::initTimer(long microseconds)
{
  TCCR1A = 0;                 // clear control register A 
  TCCR1B = _BV(WGM13);        // set mode as phase and frequency correct pwm, stop the timer
  if(microseconds > 0) setPeriod(microseconds);
  TIMSK1 = _BV(TOIE1);                                     // sets the timer overflow interrupt enable bit
  sei();                                                   // ensures that interrupts are globally enabled
   TCCR1B |= clockSelectBits;
}
void StepperQ::stopTimer()
{
  TCCR1B &= ~(_BV(CS10) | _BV(CS11) | _BV(CS12));          // clears all clock selects bits
}


void StepperQ::debug( boolean debug) {

  _debug= debug;

}
void StepperQ::setEnablePin(uint8_t enablePin) {

    _enablePin = enablePin;
}


// Subclasses can override
void StepperQ::step(uint8_t first)
{
   //Serial.print("step:"); 
   //Serial.print(_interface); 

   if (first == HIGH) {
    switch (_interface)
    {
   
	case DRIVER:
	    step1(first);
	    break;
    
	case FULL2WIRE:
	    step2(_currentPos);
	    break;
    
	case FULL3WIRE:
	    step3(_currentPos);
	    break;  

	case FULL4WIRE:
	    step4(_currentPos);
	    break;  

	case HALF3WIRE:
	    step6(_currentPos);
	    break;  
		
	case HALF4WIRE:
	    step8(_currentPos);
	    break;  
    }
  } else {
       switch (_interface)
    {
   
	case DRIVER:
	    step1(first);
	    break;
    }  //switch

    } // else 
}

// You might want to override this to implement eg serial output
// bit 0 of the mask corresponds to _pin[0]
// bit 1 of the mask corresponds to _pin[1]
// ....
void StepperQ::setOutputPins(uint8_t mask)
{
    uint8_t numpins = 2;
    if (_interface == FULL4WIRE || _interface == HALF4WIRE)
	numpins = 4;
    uint8_t i;
    for (i = 0; i < numpins; i++)
	digitalWrite(_pin[i], (mask & (1 << i)) ? (HIGH) : (LOW ));
}



// 1 pin step function (ie for stepper drivers)
// This is passed the current step number (0 to 7)
// Subclasses can override
void StepperQ::step1(uint8_t  up)
{
    digitalWrite(_pin[0],up);
}


// 2 pin step function
// This is passed the current step number (0 to 7)
// Subclasses can override
void StepperQ::step2(long step)
{
    switch (step & 0x3)
    {
	case 0: /* 01 */
	    setOutputPins(0b10);
	    break;

	case 1: /* 11 */
	    setOutputPins(0b11);
	    break;

	case 2: /* 10 */
	    setOutputPins(0b01);
	    break;

	case 3: /* 00 */
	    setOutputPins(0b00);
	    break;
    }
}
// 3 pin step function
// This is passed the current step number (0 to 7)
// Subclasses can override
void StepperQ::step3(long step)
{
    switch (step % 3)
    {
	case 0:    // 100
	    setOutputPins(0b100);
	    break;

	case 1:    // 001
	    setOutputPins(0b001);
	    break;

	case 2:    //010
	    setOutputPins(0b010);
	    break;
	    
    }
}

// 4 pin step function for half stepper
// This is passed the current step number (0 to 7)
// Subclasses can override
void StepperQ::step4(long step)
{

   // Serial.print("step4"); 
   // Serial.print(step); 

    switch (step & 0x3)
    {
	case 0:    // 1010
	    setOutputPins(0b0101);
	    break;

	case 1:    // 0110
	    setOutputPins(0b0110);
	    break;

	case 2:    //0101
	    setOutputPins(0b1010);
	    break;

	case 3:    //1001
	    setOutputPins(0b1001);
	    break;
    }
}

// 3 pin half step function
// This is passed the current step number (0 to 7)
// Subclasses can override
void StepperQ::step6(long step)
{
    switch (step % 6)
    {
	case 0:    // 100
	    setOutputPins(0b100);
            break;
	    
        case 1:    // 101
	    setOutputPins(0b101);
            break;
	    
	case 2:    // 001
	    setOutputPins(0b001);
            break;
	    
        case 3:    // 011
	    setOutputPins(0b011);
            break;
	    
	case 4:    // 010
	    setOutputPins(0b010);
            break;
	    
	case 5:    // 011
	    setOutputPins(0b110);
            break;
	    
    }
}

// 4 pin half step function
// This is passed the current step number (0 to 7)
// Subclasses can override
void StepperQ::step8(long step)
{
    switch (step & 0x7)
    {
	case 0:    // 1000
	    setOutputPins(0b0001);
            break;
	    
        case 1:    // 1010
	    setOutputPins(0b0101);
            break;
	    
	case 2:    // 0010
	    setOutputPins(0b0100);
            break;
	    
        case 3:    // 0110
	    setOutputPins(0b0110);
            break;
	    
	case 4:    // 0100
	    setOutputPins(0b0010);
            break;
	    
        case 5:    //0101
	    setOutputPins(0b1010);
            break;
	    
	case 6:    // 0001
	    setOutputPins(0b1000);
            break;
	    
        case 7:    //1001
	    setOutputPins(0b1001);
            break;
    }
}


