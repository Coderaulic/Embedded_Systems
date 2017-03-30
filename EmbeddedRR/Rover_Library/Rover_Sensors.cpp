//----------------------------- Rover_Sensors --------------------------
// Filename:      	Rover_Sensors.cpp
// Project Team:  	EmbeddedRR
// Group Members: 	Robert Griswold and Ryu Muthui
// Date:          	02 Dec 2016
// Description:   	Class for Sensor controls.
//                	Code for the QRE1113 Digital board.
//                	Outputs via the serial terminal.
//                	Lower numbers mean more reflected.
//                	3000 or more means nothing was reflected.
//					Code for the LSM303DLHC board.
//----------------------------------------------------------------------
#include "Rover_Sensors.h"

// I2C Configuration
/* Assign a unique ID to this sensor at the same time */
Adafruit_LSM303_Mag_Unified s_mag = Adafruit_LSM303_Mag_Unified(12345);
Adafruit_LSM303_Accel_Unified s_accel = Adafruit_LSM303_Accel_Unified(54321);

//-----------------------------------------------------------------------
// sensor_setup() -- Initializaion of I2C Sensor
//-----------------------------------------------------------------------
void sensor_setup() {
	/* Enable auto-gain */
	s_mag.enableAutoRange(true);

	if(!s_accel.begin()) {
	/* There was a problem detecting the ADXL345 ... check your connections */
		#ifdef SENSOR_PRINT
			Serial.println("Accel: No LSM303 detected on this device");
		#endif
	}
	if(!s_mag.begin()) {
	/* There was a problem detecting the ADXL345 ... check your connections */
		#ifdef SENSOR_PRINT
			Serial.println("Mag: No LSM303 detected on this device");
		#endif
	}
}

//-----------------------------------------------------------------------
// sensor_readSensorAt(int pin) -- read IR data from the pin
//-----------------------------------------------------------------------
int sensor_readSensorAt(int QRE1113_Pin) {
	long time = micros();
	pinMode(QRE1113_Pin, OUTPUT);
	digitalWrite(QRE1113_Pin, HIGH);  
	delayMicroseconds(10);
	pinMode(QRE1113_Pin, INPUT);

	// time how long the input is HIGH, 
	// but quit after 3ms as nothing happens after that
	while ((digitalRead(QRE1113_Pin) == HIGH) && ((micros() - time) < 3000)); 
	int diff = micros() - time;
	return diff;
}

//-----------------------------------------------------------------------
// sensor_readSensors() -- read and display all sensor values
//-----------------------------------------------------------------------
void sensor_readSensors() {
	// Read all four pins
	int QRE_Value = sensor_readSensorAt(IR_LL);   // left left
	Serial.print("LL " + (String)QRE_Value + " ");

	QRE_Value = sensor_readSensorAt(IR_L);   // left
	Serial.print("L: " + (String)QRE_Value + " ");

	QRE_Value = sensor_readSensorAt(IR_R);   // right
	Serial.print("R: " + (String)QRE_Value + " ");

	QRE_Value = sensor_readSensorAt(IR_RR);  // right right
	Serial.println("RR " + (String)QRE_Value);
  
	/* Get a new acceleration sensor event */
    sensors_event_t event;
    s_accel.getEvent(&event);
	char aclX[10], aclY[10], aclZ[10];
	dtostrf(event.acceleration.x, 9, 2, aclX);
	dtostrf(event.acceleration.y, 9, 2, aclY);
	dtostrf(event.acceleration.z, 9, 2, aclZ);
	Serial.print("X: "); Serial.print(aclX); Serial.print("  ");
	Serial.print("Y: "); Serial.print(aclY); Serial.print("  ");
	Serial.print("Z: "); Serial.print(aclZ); Serial.print("  "); Serial.println("m/s^2");
	
	/* Get a new magnetic sensor event */
    s_mag.getEvent(&event);
	char magX[10], magY[10], magZ[10];
	dtostrf(event.magnetic.x, 9, 2, magX);
	dtostrf(event.magnetic.y, 9, 2, magY);
	dtostrf(event.magnetic.z, 9, 2, magZ);
	Serial.print("X: "); Serial.print(magX); Serial.print("  ");
	Serial.print("Y: "); Serial.print(magY); Serial.print("  ");
	Serial.print("Z: "); Serial.print(magZ); Serial.print("  "); Serial.println("uT");
}

//-----------------------------------------------------------------------
// sensor_getSensorVals() -- get all current IR sensor values in an array
//-----------------------------------------------------------------------
void sensor_getSensorVals(int *arrSensorVals) {
	arrSensorVals[0] = sensor_readSensorAt(IR_LL);
	arrSensorVals[1] = sensor_readSensorAt(IR_L);
	arrSensorVals[2] = sensor_readSensorAt(IR_R);
	arrSensorVals[3] = sensor_readSensorAt(IR_RR);
}

//-----------------------------------------------------------------------
// sensor_getAccelData() -- gets the raw accelerometer data in an array
//-----------------------------------------------------------------------
void sensor_getAccelData(float &x, float &y, float &z) {
	/* Get a new acceleration sensor event */
    sensors_event_t event;
    s_accel.getEvent(&event);
	
	x = event.acceleration.x;
	y = event.acceleration.y;
	z = event.acceleration.z;
}

//-----------------------------------------------------------------------
// sensor_getMagData() -- gets the raw magnetometer data in an array
//-----------------------------------------------------------------------
void sensor_getMagData(float &x, float &y, float &z) {
	/* Get a new magnetic sensor event */
    sensors_event_t event;
    s_mag.getEvent(&event);
	
	x = event.magnetic.x;
	y = event.magnetic.y;
	z = event.magnetic.z;
}
