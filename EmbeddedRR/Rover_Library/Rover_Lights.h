//--------------------------- Rover_Lights -----------------------------
// Filename:      	Rover_Lights.h
// Project Team:  	EmbeddedRR
// Group Members: 	Robert Griswold and Ryu Muthui
// Date:          	28 Nov 2016
// Description:   	Class for LED light controls.
//------------------------------ Includes ------------------------------
#ifndef _Rover_Lights_h_
#define _Rover_Lights_h_

#include <Adafruit_NeoPixel.h>
#include <Arduino.h>

//---------------------------- Definitions -----------------------------
#define MAX_VAL 150 // MAX brightness value

//------------------------------ Class Functions -----------------------
//----------------------------------------------------------------------
// light_setupLights() -- Initializes the lights
//----------------------------------------------------------------------
void light_setupLights();

//----------------------------------------------------------------------
// light_clearLights() -- Turns all the LEDs off, reseting them.
//----------------------------------------------------------------------
void light_clearLights();

//----------------------------------------------------------------------
// light_lightRed() -- Turns both the LEDs to RED.
//----------------------------------------------------------------------
void light_lightRed();

//----------------------------------------------------------------------
// light_lightPurple() -- Turns both the LEDs to PURPLE.
//----------------------------------------------------------------------
void light_lightPurple();

//----------------------------------------------------------------------
// light_lightGreen() -- Turns both the LEDs to GREEN.
//----------------------------------------------------------------------
void light_lightGreen();

//----------------------------------------------------------------------
// light_lightBlue() -- Turns both the LEDs to BLUE.
//----------------------------------------------------------------------
void light_lightBlue();

//----------------------------------------------------------------------
// light_lightYellow() -- Turns both the LEDs to YELLOW.
//----------------------------------------------------------------------
void light_lightYellow();

//----------------------------------------------------------------------
// light_lightWhite() -- Turns both the LEDs to WHITE.
//----------------------------------------------------------------------
void light_lightWhite();

//----------------------------------------------------------------------
// light_turnLeft() -- Turns bottom LED to GREEN, top to YELLOW.
//----------------------------------------------------------------------
void light_turnLeft();

//----------------------------------------------------------------------
// light_turnRight() -- Turns bottom LED to YELLOW, top to GREEN.
//----------------------------------------------------------------------
void light_turnRight();

//----------------------------------------------------------------------
// light_PoliceMode() -- Flicker police flash 10 times
//----------------------------------------------------------------------
void light_PoliceMode();

#endif
