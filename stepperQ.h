#ifndef StepperQ_h
#define StepperQ_h

#include <stdlib.h>
#if ARDUINO >= 100
#include <Arduino.h>
#else
#include <WProgram.h>
#include <wiring.h>
#endif

#include <avr/io.h>
#include <avr/interrupt.h>
#define RESOLUTION 65536    // Timer1 is 16 bit
// These defs cause trouble on some versions of Arduino
#undef round


class StepperQ
{
public:
    void init(uint8_t dirpin = 2, uint8_t steppin = 3);
    /// Set the target position. The run() function will try to move the motor
    /// from the current position to the target position set by the most
    /// recent call to this function. Caution: moveTo() also recalculates the speed for the next step. 
    /// If you are trying to use constant speed movements, you should call setSpeed() after calling moveTo().
    /// \param[in] absolute The desired absolute position. Negative is
    /// anticlockwise from the 0 position.
    void    moveTo(long absolute); 

    /// Set the target position relative to the current position
    /// \param[in] relative The desired position relative to the current position. Negative is
    /// anticlockwise from the current position.
    void    move(long relative);
   /// Sets the maximum permitted speed. the run() function will accelerate
    /// up to the speed set by this function.
    /// \param[in] speed The desired maximum speed in steps per second. Must
    /// be > 0. Caution: Speeds that exceed the maximum speed supported by the processor may
    /// Result in non-linear accelerations and decelerations.
    void    setMaxSpeed(float speed);

/// Sets the acceleration and deceleration parameter.
    /// \param[in] acceleration The desired acceleration in steps per second
    /// per second. Must be > 0.0. This is an expensive call since it requires a square 
    /// root to be calculated. Dont call more ofthen than needed
    void    setAcceleration(float acceleration);

    /// The distance from the current position to the target position.
    /// \return the distance from the current position to the target position
    /// in steps. Positive is clockwise from the current position.
    long    distanceToGo();

    /// The most recently set target position.
    /// \return the target position
    /// in steps. Positive is clockwise from the 0 position.
    long    targetPosition();

    /// The currently motor position.
    /// \return the current motor position
    /// in steps. Positive is clockwise from the 0 position.
    long    currentPosition();  

    /// Resets the current position of the motor, so that wherever the motor
    /// happens to be right now is considered to be the new 0 position. Useful
    /// for setting a zero position on a stepper after an initial hardware
    /// positioning move.
    /// Has the side effect of setting the current motor speed to 0.
    /// \param[in] position The position in steps of wherever the motor
    /// happens to be right now.
    void    setCurrentPosition(long position);  
/// Sets a new target position that causes the stepper
    /// to stop as quickly as possible, using to the current speed and acceleration parameters.
    void stop();

    void start();
    long stepsToStop();

    void isrCallback();	
    void debug( boolean debug);
protected:

    /// \brief Direction indicator
    /// Symbolic names for the direction the motor is turning
    typedef enum
    {
	DIRECTION_CCW = -1,  ///< Clockwise
        DIRECTION_CW  = 1   ///< Counter-Clockwise
    } Direction;
/// Called to execute a step. Only called when a new step is
    /// required. Subclasses may override to implement new stepping
    /// interfaces. The default calls step1(), step2(), step4() or step8() depending on the
    /// number of pins defined for the stepper.
    /// \param[in] step The current step phase number (0 to 7)
    virtual void   stepUp();
    virtual void   stepDown();
     virtual void changeDirection();

private:
 
   
    void setPeriod(long microseconds);
    void initTimer(long microseconds);
    void stopTimer();
    void calculateSpeed() ;
	/// Arduino pin number assignments for the 2 or 4 pins required to interface to the
    /// stepper motor or driver
    uint8_t        _dirpin;
    uint8_t        _steppin;


 /// Enable pin for stepper driver, or 0xFF if unused.
    uint8_t        _enablePin;


    /// The current absolution position in steps.
    long           _currentPos;    // Steps

    /// The target position in steps. The StepperQ library will move the
    /// motor from the _currentPos to the _targetPos, taking into account the
    /// max speed, acceleration and deceleration
    long           _targetPos;     // Steps

     /// The maximum permitted speed in steps per second. Must be > 0.
     float          _maxSpeed;

    /// The acceleration to use to accelerate or decelerate the motor in steps
    /// per second per second. Must be > 0
    float          _acceleration;
    float          _sqrt_twoa; // Precomputed sqrt(2*_acceleration)

   /// The step counter for speed calculations
    long _n;

    /// Initial step size in microseconds
     //long  _c0;
    float _c0;

    /// Last step size in microseconds
    float _cn ;
	
   // long _cn;

    /// Min step size in microseconds based on maxSpeed
    float _cmin; // at max speed
    
    //long  _cmin; // at max speed

    /// Current direction motor is spinning in
    boolean _direction; // 1 == CW

    long _stepsToStop ;

		//timer Vars
     unsigned char clockSelectBits;
    boolean _debug;
};

extern StepperQ stepperq;

#endif 
