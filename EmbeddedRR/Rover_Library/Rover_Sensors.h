//----------------------------- Rover_Sensors --------------------------
// Filename:      	Rover_Sensors.h
// Project Team:  	EmbeddedRR
// Group Members: 	Robert Griswold and Ryu Muthui
// Date:          	02 Dec 2016
// Description:   	Class for Sensor controls.
//                	Code for the QRE1113 Digital board.
//                	Outputs via the serial terminal.
//                	Lower numbers mean more reflected.
//                	3000 or more means nothing was reflected.
//					Code for the LSM303DLHC board.	
//------------------------------ Includes ------------------------------
#ifndef _Rover_Sensors_h_
#define _Rover_Sensors_h_

#include <Arduino.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>

//----------------------------- Configuration ---------------------------
// IR Sensors
#define IR_RR 50  // right-most IR
#define IR_R 51   // middle-right IR
#define IR_L 52   // middle-left IR
#define IR_LL 53  // left-most IR

// Debug
#define SENSOR_PRINT

//-----------------------------------------------------------------------
// sensor_setup() -- Initializaion of I2C Sensor
//-----------------------------------------------------------------------
void sensor_setup();

//-----------------------------------------------------------------------
// sensor_readSensorAt(int pin) -- read IR data from the pin
//-----------------------------------------------------------------------
int sensor_readSensorAt(int QRE1113_Pin);

//-----------------------------------------------------------------------
// sensor_readSensors() -- read and display all sensor values
//-----------------------------------------------------------------------
void sensor_readSensors();

//-----------------------------------------------------------------------
// sensor_getSensorVals() -- get all current IR sensor values in an array
//-----------------------------------------------------------------------
void sensor_getSensorVals(int *arrSensorVals);

//-----------------------------------------------------------------------
// sensor_getAccelData() -- gets the accelerometer data in an array
//-----------------------------------------------------------------------
void sensor_getAccelData(float &x, float &y, float &z);

//-----------------------------------------------------------------------
// sensor_getMagData() -- gets the magnetometer data in an array
//-----------------------------------------------------------------------
void sensor_getMagData(float &x, float &y, float &z);

#endif
