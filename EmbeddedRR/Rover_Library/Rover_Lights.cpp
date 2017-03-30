//--------------------------- Rover_Lights -----------------------------
// Filename:      	Rover_Lights.cpp
// Project Team:  	EmbeddedRR
// Group Members: 	Robert Griswold and Ryu Muthui
// Date:          	28 Nov 2016
// Description:   	Class for LED light controls.
//----------------------------------------------------------------------
#include "Rover_Lights.h"

//---------------------------- Initialization --------------------------
Adafruit_NeoPixel l_strip = Adafruit_NeoPixel(2, A0, NEO_RGB + NEO_KHZ800);

//------------------------------ Class Functions -----------------------
//----------------------------------------------------------------------
// light_setupLights() -- Initializes the lights
//----------------------------------------------------------------------
void light_setupLights() {
	l_strip.begin();
	light_clearLights();
}

//----------------------------------------------------------------------
// light_clearLights() -- Turns all the LEDs off, reseting them.
//----------------------------------------------------------------------
void light_clearLights() {
	l_strip.setPixelColor(0, 0, 0, 0); // bottom
	l_strip.setPixelColor(1, 0, 0, 0); // top
	l_strip.show();
}

//----------------------------------------------------------------------
// light_lightRed() -- Turns both the LEDs to RED.
//----------------------------------------------------------------------
void light_lightRed() {
	l_strip.setPixelColor(0, MAX_VAL, 0, 0);
	l_strip.setPixelColor(1, MAX_VAL, 0, 0);
	l_strip.show();
}

//----------------------------------------------------------------------
// light_lightPurple() -- Turns both the LEDs to PURPLE.
//----------------------------------------------------------------------
void light_lightPurple() {
	l_strip.setPixelColor(0, MAX_VAL, 0, MAX_VAL);
	l_strip.setPixelColor(1, MAX_VAL, 0, MAX_VAL);
	l_strip.show();
}

//----------------------------------------------------------------------
// light_lightGreen() -- Turns both the LEDs to GREEN.
//----------------------------------------------------------------------
void light_lightGreen() {
	l_strip.setPixelColor(0, 0, MAX_VAL, 0);
	l_strip.setPixelColor(1, 0, MAX_VAL, 0);
	l_strip.show();
}

//----------------------------------------------------------------------
// light_lightBlue() -- Turns both the LEDs to BLUE.
//----------------------------------------------------------------------
void light_lightBlue() {
	l_strip.setPixelColor(0, 0, 0, MAX_VAL);
	l_strip.setPixelColor(1, 0, 0, MAX_VAL);
	l_strip.show();
}

//----------------------------------------------------------------------
// light_lightYellow() -- Turns both the LEDs to YELLOW.
//----------------------------------------------------------------------
void light_lightYellow() {
	l_strip.setPixelColor(0, MAX_VAL, MAX_VAL, 0);
	l_strip.setPixelColor(1, MAX_VAL, MAX_VAL, 0);
	l_strip.show();
}

//----------------------------------------------------------------------
// light_lightWhite() -- Turns both the LEDs to WHITE.
//----------------------------------------------------------------------
void light_lightWhite() {
	l_strip.setPixelColor(0, MAX_VAL, MAX_VAL, MAX_VAL);
	l_strip.setPixelColor(1, MAX_VAL, MAX_VAL, MAX_VAL);
	l_strip.show();
}

//----------------------------------------------------------------------
// light_turnLeft() -- Turns bottom LED to GREEN, top to YELLOW.
//----------------------------------------------------------------------
void light_turnLeft() {
	l_strip.setPixelColor(0, 0, MAX_VAL, 0);
	l_strip.setPixelColor(1, MAX_VAL, MAX_VAL, 0);
	l_strip.show();
}

//----------------------------------------------------------------------
// light_turnRight() -- Turns bottom LED to YELLOW, top to GREEN.
//----------------------------------------------------------------------
void light_turnRight() {
	l_strip.setPixelColor(0, MAX_VAL, MAX_VAL, 0);
	l_strip.setPixelColor(1, 0, MAX_VAL, 0);
	l_strip.show();
}

//----------------------------------------------------------------------
// light_PoliceMode() -- Flicker police flash 10 times
//----------------------------------------------------------------------
void light_PoliceMode() {
	for (uint8_t i = 0; i < 10; i++) {
		l_strip.setPixelColor(0, 0, 0, MAX_VAL);// blue
		l_strip.setPixelColor(1, MAX_VAL, 0, 0);// red              
		l_strip.show();
		delay(50);
		l_strip.setPixelColor(0, MAX_VAL, MAX_VAL, MAX_VAL);// white
		l_strip.setPixelColor(1, MAX_VAL, MAX_VAL, MAX_VAL);// white
		l_strip.show();
		delay(50);
		l_strip.setPixelColor(0, MAX_VAL, 0, 0);// red
		l_strip.setPixelColor(1, 0, 0, MAX_VAL);// blue
		l_strip.show();
		delay(50);
	}
	light_clearLights();
}
