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
     /// \brief Symbolic names for number of pins.
    /// Use this in the pins argument the AccelStepper1 constructor to 
    /// provide a symbolic name for the number of pins
    /// to use.
    typedef enum
    {
	DRIVER    = 1, ///< Stepper Driver, 2 driver pins required
	FULL2WIRE = 2, ///< 2 wire stepper, 2 motor pins required
	FULL3WIRE = 3, ///< 3 wire stepper, such as HDD spindle, 3 motor pins required
        FULL4WIRE = 4, ///< 4 wire full stepper, 4 motor pins required
	HALF3WIRE = 6, ///< 3 wire half stepper, such as HDD spindle, 3 motor pins required
	HALF4WIRE = 8  ///< 4 wire half stepper, 4 motor pins required
    } MotorInterfaceType;

public:
    void init(uint8_t dirpin = 2, uint8_t steppin = 3);
    void init( uint8_t pin1 , uint8_t pin2 , uint8_t pin3,  uint8_t pin4,uint8_t interface = StepperQ::FULL4WIRE );
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

    // returns MaxSpeed
    float maxSpeed();
    //return currentSpeed
    float speed();

 
/// Sets the acceleration and deceleration parameter.
    /// \param[in] acceleration The desired acceleration in steps per second
    /// per second. Must be > 0.0. This is an expensive call since it requires a square 
    /// root to be calculated. Dont call more ofthen than needed
    void    setAcceleration(float acceleration);

/// gets  the acceleration and deceleration parameter.
    /// acceleration The desired acceleration in steps per second
    float     getAcceleration();

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

    /// Sets the enable pin number for stepper drivers.
    /// 0xFF indicates unused (default).
    /// Otherwise, if a pin is set, the pin will be turned on when 
    /// enableOutputs() is called and switched off when disableOutputs() 
    /// is called.
    /// \param[in] enablePin Arduino digital pin number for motor enable
    /// \sa setPinsInverted
    void    setEnablePin(uint8_t enablePin = 0xff);

	void setDirOrder(boolean reverse );
/// Sets a new target position that causes the stepper
    /// to stop as quickly as possible, using to the current speed and acceleration parameters.
    void stop();

    void start();
    long stepsToStop();
    long maxstepsToStop();

    void isrCallback();	
    void debug( boolean debug);
    virtual int getDirection();
protected:

    /// \brief Direction indicator
    /// Symbolic names for the direction the motor is turning
    typedef enum
    {
	DIRECTION_CCW = -1,  ///< Clockwise
        DIRECTION_CW  = 1   ///< Counter-Clockwise
    } Direction;
 /// Low level function to set the motor output pins
    /// bit 0 of the mask corresponds to _pin[0]
    /// bit 1 of the mask corresponds to _pin[1]
    /// You can override this to impment, for example serial chip output insted of using the
    /// output pins directly
    virtual void   setOutputPins(uint8_t mask);
    /// Called to execute a step. Only called when a new step is
    /// required. Subclasses may override to implement new stepping
    /// interfaces. The default calls step1(), step2(), step4() or step8() depending on the
    /// number of pins defined for the stepper.
    /// \param[in] step The current step phase number (0 to 7)
    virtual void   step(uint8_t first);
   virtual void changeDirection();

   

   /// Called to execute a step on a stepper driver (ie where pins == 1). Only called when a new step is
    /// required. Subclasses may override to implement new stepping
    /// interfaces. The default sets or clears the outputs of Step pin1 to step, 
    /// and sets the output of _pin2 to the desired direction. The Step pin (_pin1) is pulsed for 1 microsecond
    /// which is the minimum STEP pulse width for the 3967 driver.
    /// \param[in] step The current step phase number (0 to 7)
    virtual void   step1(uint8_t step);

    /// Called to execute a step on a 2 pin motor. Only called when a new step is
    /// required. Subclasses may override to implement new stepping
    /// interfaces. The default sets or clears the outputs of pin1 and pin2
    /// \param[in] step The current step phase number (0 to 7)
    virtual void   step2(long step);

    /// Called to execute a step on a 3 pin motor, such as HDD spindle. Only called when a new step is
    /// required. Subclasses may override to implement new stepping
    /// interfaces. The default sets or clears the outputs of pin1, pin2,
    /// pin3
    /// \param[in] step The current step phase number (0 to 7)
    virtual void   step3(long step);

    /// Called to execute a step on a 4 pin motor. Only called when a new step is
    /// required. Subclasses may override to implement new stepping
    /// interfaces. The default sets or clears the outputs of pin1, pin2,
    /// pin3, pin4.
    /// \param[in] step The current step phase number (0 to 7)
    virtual void   step4(long step);

    /// Called to execute a step on a 3 pin motor, such as HDD spindle. Only called when a new step is
    /// required. Subclasses may override to implement new stepping
    /// interfaces. The default sets or clears the outputs of pin1, pin2,
    /// pin3
    /// \param[in] step The current step phase number (0 to 7)
    virtual void   step6(long step);

    /// Called to execute a step on a 4 pin half-steper motor. Only called when a new step is
    /// required. Subclasses may override to implement new stepping
    /// interfaces. The default sets or clears the outputs of pin1, pin2,
    /// pin3, pin4.
    /// \param[in] step The current step phase number (0 to 7)
    virtual void   step8(long step);


private:
 
   
    void setPeriod(long microseconds);
    void initTimer(long microseconds);
    void stopTimer();
    void calculateSpeed() ;
    /// Number of pins on the stepper motor. Permits 2 or 4. 2 pins is a
    /// bipolar, and 4 pins is a unipolar.
    uint8_t        _interface= 1;          //  1, 2, 4, 8, See MotorInterfaceType

    /// Arduino pin number assignments for the 2 or 4 pins required to interface to the
    /// stepper motor or driver
    uint8_t        _pin[4];

  //  uint8_t        _dirpin;
  //  uint8_t        _steppin;
	boolean _reverse;

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
    int  _direction; // 1 == CW
    /// Calculatet Steps to Stop. If the Max speed reaches.
    long _stepsToStop ;

    //timer Vars
    unsigned char clockSelectBits;
    /// set True für Debug output
    boolean _debug;
};

extern StepperQ stepperq;

#endif 
