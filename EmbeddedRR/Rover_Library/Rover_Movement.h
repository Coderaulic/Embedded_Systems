//---------------------------- Rover_Movement --------------------------
// Filename:      	Rover_Movement.h
// Project Team:  	EmbeddedRR
// Group Members: 	Robert Griswold and Ryu Muthui
// Date:          	26 Nov 2016
// Description:   	Class for motor controls.
//------------------------------ Includes ------------------------------
#ifndef _Rover_Movement_h_
#define _Rover_Movement_h_

#include <Adafruit_MotorShield.h>
#include <Arduino.h>

//------------------------------ Definitions ---------------------------
#define SPEED_MAX 255
#define SPEED_STOP 0
#define SPEED_NORMAL 50		// target speed for forward/reverse methods
#define TURN_RATE 35		// turn rate for rotation methods
#define DELAY_VAL 50
// #define ALPHA 8 // The smoothing factor for the cubic function

//------------------------------ Class Functions -----------------------
//----------------------------------------------------------------------
// move_setupMotors() -- Initializes the motors
//----------------------------------------------------------------------
void move_setupMotors();

//----------------------------------------------------------------------
// move_smoothTurnRate() -- smooths the turn rate based on a cubic function
//----------------------------------------------------------------------
//void move_smoothTurnRate(int *current, int target, int *smoothCounter);

//----------------------------------------------------------------------
// move_setTarget() -- set the motor speeds for both at same time
//----------------------------------------------------------------------
void move_setTarget(int left, int right);

//----------------------------------------------------------------------
// move_setTargetLeft() -- set the motor speeds for left motor
//----------------------------------------------------------------------
void move_setTargetLeft(int left);

//----------------------------------------------------------------------
// move_setTargetRight() -- set the motor speeds for right motor
//----------------------------------------------------------------------
void move_setTargetRight(int right);

//----------------------------------------------------------------------
// move_updateMotors() -- update the motor speeds per iteration
//----------------------------------------------------------------------
void move_updateMotors();

//----------------------------------------------------------------------
// move_moveForward() -- slow start move forward. Manual does not rely 
// on move_updateMotors.
//----------------------------------------------------------------------
void move_moveForward(bool manual);

//----------------------------------------------------------------------
// move_moveReverse() - slow start move backwards. Manual does not rely 
// on move_updateMotors.
//----------------------------------------------------------------------
void move_moveReverse(bool manual);

//----------------------------------------------------------------------
// move_fullStop() -- sets the motor speed to 0
//----------------------------------------------------------------------
void move_fullStop();

//----------------------------------------------------------------------
// move_rotateLeft90() -- rotates the rover approx (90 degrees) to left
//----------------------------------------------------------------------
void move_rotateLeft90();

//----------------------------------------------------------------------
// move_rotateRight90() -- rotates the rover approx (90 degrees) to right
//----------------------------------------------------------------------
void move_rotateRight90();

//----------------------------------------------------------------------
// move_getTargetLeft() -- Getter for the target left value.
//----------------------------------------------------------------------
int move_getTargetLeft();

//----------------------------------------------------------------------
// move_getTargetRight() -- Getter for the target right value.
//----------------------------------------------------------------------
int move_getTargetRight();

//----------------------------------------------------------------------
// move_getCurrentLeft() -- Getter for the current left value.
//----------------------------------------------------------------------
int move_getCurrentLeft();

//----------------------------------------------------------------------
// move_getCurrentRight() -- Getter for the current right value.
//----------------------------------------------------------------------
int move_getCurrentRight();

#endif
