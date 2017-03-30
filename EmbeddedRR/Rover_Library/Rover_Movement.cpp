//---------------------------- Rover_Movement --------------------------
// Filename:      	Rover_Movement.cpp
// Project Team:  	EmbeddedRR
// Group Members: 	Robert Griswold and Ryu Muthui
// Date:          	26 Nov 2016
// Description:   	Class for motor controls.
//----------------------------------------------------------------------
#include "Rover_Movement.h"

//---------------------------- Initialization --------------------------
// Motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
// Select which 'port' M1, M2, M3 or M4. In this case, M3 and M4
Adafruit_DCMotor *rightMotor = AFMS.getMotor(3);
Adafruit_DCMotor *leftMotor = AFMS.getMotor(4);

int m_currentRight, m_currentLeft;
int m_targetRight, m_targetLeft;
// int m_smoothCounterLeft, m_smoothCounterRight;

//------------------------------ Class Functions -----------------------
//----------------------------------------------------------------------
// move_setupMotors() -- Initializes the motors
//----------------------------------------------------------------------
void move_setupMotors() {
	AFMS.begin();
	rightMotor->setSpeed(SPEED_STOP);
	leftMotor->setSpeed(SPEED_STOP);
	rightMotor->run(RELEASE);
	leftMotor->run(RELEASE);
	m_currentRight = 0;
	m_currentLeft = 0;
	m_targetRight = 0;
	m_targetLeft = 0;
	// m_smoothCounterLeft = 0;
	// m_smoothCounterRight = 0;
}

//----------------------------------------------------------------------
// move_smoothTurnRate() -- smooths the turn rate based on a cubic function
//----------------------------------------------------------------------
/*void move_smoothTurnRate(int *current, int target, int *smoothCounter) { // avg 328 microseconds with pow
	//unsigned long start = micros(); // timer from device power on

	// determine if smoothCounter increments or decrements
	if ((target - (*current)) < 0)
		(*smoothCounter)--;
	else
		(*smoothCounter)++;
	//Serial.print("current: " + String(*current) + " target:" + String(target) + " smoothCounter:" + String(*smoothCounter));

	double delta = 0.0;
	if ((*smoothCounter) > 0) {
		delta = ceil(fastPow((((double)(*smoothCounter))/ALPHA),3));
		*current = min((*current) + delta, target);
	}
	else {
		delta = floor(fastPow((((double)(*smoothCounter))/ALPHA),3));
		*current = max((*current) + delta, target);
	}
	//Serial.println(" delta: " + String(delta) + " newCurrent:" + String(*current));
	//Serial.println(fastPow((((double)(*smoothCounter))/ALPHA),3));
	//Serial.println("calculation time:" + String(micros() - start));
}*/

//----------------------------------------------------------------------
// move_setTarget() -- set the motor speeds for both at same time
//----------------------------------------------------------------------
void move_setTarget(int left, int right) {
	m_targetRight = right;
	m_targetLeft = left;
}

//----------------------------------------------------------------------
// move_setTargetLeft() -- set the motor speeds for left motor
//----------------------------------------------------------------------
void move_setTargetLeft(int left) {
	m_targetLeft = left;
}

//----------------------------------------------------------------------
// move_setTargetRight() -- set the motor speeds for right motor
//----------------------------------------------------------------------
void move_setTargetRight(int right) {
	m_targetRight = right;
}

//----------------------------------------------------------------------
// move_updateMotors() -- update the motor speeds per iteration
//----------------------------------------------------------------------
void move_updateMotors() {
	if (m_targetLeft != m_currentLeft) {
		if (m_targetLeft > m_currentLeft) { // 4 microseconds
			m_currentLeft++;
		}
		else {
			m_currentLeft--;
		}
		//move_smoothTurnRate(&m_currentLeft, m_targetLeft, &m_smoothCounterLeft);
		leftMotor->setSpeed(abs(m_currentLeft));

		// set dir of left motor
		if (m_currentLeft == 1) {
			leftMotor->run(FORWARD);
		}
		else if (m_currentLeft == -1) {
			leftMotor->run(BACKWARD);
		}
		else if (m_currentLeft == 0) {
			leftMotor->run(RELEASE);
		}
	}
	/*else {
		m_smoothCounterLeft = 0;
	}*/

	if (m_targetRight != m_currentRight) {
		if (m_targetRight > m_currentRight) {
			m_currentRight++;
		}
		else {
			m_currentRight--;
		}
		//move_smoothTurnRate(&m_currentRight, m_targetRight, &m_smoothCounterRight);
		rightMotor->setSpeed(abs(m_currentRight));

		// set dir of right motor
		if (m_currentRight == 1) {
			rightMotor->run(FORWARD);
		}
		else if (m_currentRight == -1) {
			rightMotor->run(BACKWARD);
		}
		else if (m_currentRight == 0) {
			rightMotor->run(RELEASE);
		}
	}
	/*else {
		m_smoothCounterRight = 0;
	}*/
	
	//Serial.println("TargetR: " + String(targetRight) + " CurrentR: " + String(m_currentRight) +
				// " TargetL: " + String(targetLeft) + " CurrentL: " + String(m_currentLeft));
	//delay(1000);
}

//----------------------------------------------------------------------
// move_moveForward() -- slow start move forward. Manual does not rely 
// on move_updateMotors.
//----------------------------------------------------------------------
void move_moveForward(bool manual) {
	rightMotor->run(FORWARD);
	leftMotor->run(FORWARD);
	
	m_targetRight = SPEED_NORMAL;
	m_targetLeft = SPEED_NORMAL;
	m_currentRight = 20;
	m_currentLeft = 20;
	
	if (manual) {
		// accelerate (forward)
		for (uint8_t i = 20; i < SPEED_NORMAL; i++) { 
			leftMotor->setSpeed(i); 
			rightMotor->setSpeed(i);
			delay(DELAY_VAL);
		}
		
		m_currentRight = SPEED_NORMAL;
		m_currentLeft = SPEED_NORMAL;
	}
}

//----------------------------------------------------------------------
// move_moveReverse() - slow start move backwards. Manual does not rely 
// on move_updateMotors.
//----------------------------------------------------------------------
void move_moveReverse(bool manual) {
	rightMotor->run(BACKWARD);
	leftMotor->run(BACKWARD);
	
	m_targetRight = - SPEED_NORMAL;
	m_targetLeft = - SPEED_NORMAL;
	m_currentRight = -20;
	m_currentLeft = -20;
	
	if (manual) {
		// accelerate (backwards)
		for (uint8_t i = 20; i < SPEED_NORMAL; i++) { 
			leftMotor->setSpeed(i);
			rightMotor->setSpeed(i);  
			delay(DELAY_VAL);
		}
		
		m_currentRight = - SPEED_NORMAL;
		m_currentLeft = - SPEED_NORMAL;
	}
}

//----------------------------------------------------------------------
// move_fullStop() -- sets the motor speed to 0
//----------------------------------------------------------------------
void move_fullStop() {
	rightMotor->setSpeed(SPEED_STOP);
	leftMotor->setSpeed(SPEED_STOP);
	rightMotor->run(RELEASE);
	leftMotor->run(RELEASE);
	
	m_currentRight = 0;
	m_currentLeft = 0;
	m_targetRight = 0;
	m_targetLeft = 0;
}

//----------------------------------------------------------------------
// move_rotateLeft90() -- rotates the rover approx (90 degrees) to left
//----------------------------------------------------------------------
void move_rotateLeft90() {
	rightMotor->run(FORWARD);
	leftMotor->run(BACKWARD);

	uint8_t right = TURN_RATE;
	uint8_t left = TURN_RATE;
	for (uint8_t i = 1; i <= TURN_RATE; i++) {
		rightMotor->setSpeed(right--); 
		leftMotor->setSpeed(left--);
		delay(DELAY_VAL);
	}
	
	rightMotor->setSpeed(SPEED_STOP);
	leftMotor->setSpeed(SPEED_STOP); 
	rightMotor->run(RELEASE);
	leftMotor->run(RELEASE);
}

//----------------------------------------------------------------------
// move_rotateRight90() -- rotates the rover approx (90 degrees) to right
//----------------------------------------------------------------------
void move_rotateRight90() {
	rightMotor->run(BACKWARD);
	leftMotor->run(FORWARD);

	uint8_t right = TURN_RATE;
	uint8_t left = TURN_RATE;
	for (uint8_t i = 1; i <= TURN_RATE; i++) {
		rightMotor->setSpeed(right--);
		leftMotor->setSpeed(left--); 
		delay(DELAY_VAL);
	}
	
	rightMotor->setSpeed(SPEED_STOP);
	leftMotor->setSpeed(SPEED_STOP); 
	rightMotor->run(RELEASE);
	leftMotor->run(RELEASE);
}

//----------------------------------------------------------------------
// move_getTargetLeft() -- Getter for the target left value.
//----------------------------------------------------------------------
int move_getTargetLeft() {
	return m_targetLeft;
}

//----------------------------------------------------------------------
// move_getTargetRight() -- Getter for the target right value.
//----------------------------------------------------------------------
int move_getTargetRight() {
	return m_targetRight;
}

//----------------------------------------------------------------------
// move_getCurrentLeft() -- Getter for the current left value.
//----------------------------------------------------------------------
int move_getCurrentLeft() {
	return m_currentLeft;
}

//----------------------------------------------------------------------
// move_getCurrentRight() -- Getter for the current right value.
//----------------------------------------------------------------------
int move_getCurrentRight() {
	return m_currentRight;
}
