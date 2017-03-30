/*
	libxbee - a C/C++ library to aid the use of Digi's XBee wireless modules
	          running in API mode.

	Copyright (C) 2009 onwards  Attie Grande (attie@attie.co.uk)

	libxbee is free software: you can redistribute it and/or modify it
	under the terms of the GNU Lesser General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.

	libxbee is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	See the
	GNU Lesser General Public License for more details.

	You should have received a copy of the GNU Lesser General Public License
	along with libxbee. If not, see <http://www.gnu.org/licenses/>.
*/

//------------------------------- main.c ------------------------------
// Filename:      	main.c
// Project Team:  	EmbeddedRR
// Group Members: 	Robert Griswold and Ryu Muthui
// Date:          	2 Dec 2016
// Description:   	Console terminal for two xbee arduino rovers. See 
//					help section for more information.
//------------------------------ Includes  ----------------------------

// Includes
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <xbee.h>

// Configuration
#define R1_ADDR_0 0x00
#define R1_ADDR_1 0x13
#define R1_ADDR_2 0xA2
#define R1_ADDR_3 0x00
#define R1_ADDR_4 0x40
#define R1_ADDR_5 0xF9
#define R1_ADDR_6 0xCE
#define R1_ADDR_7 0xDE

#define R2_ADDR_0 0x00
#define R2_ADDR_1 0x13
#define R2_ADDR_2 0xA2
#define R2_ADDR_3 0x00
#define R2_ADDR_4 0x41
#define R2_ADDR_5 0x03
#define R2_ADDR_6 0xDA
#define R2_ADDR_7 0x0F

// #define COM_USE_ROVER_ACKS // Whether to use xbee acks or roverpacket acks
// Note that no additional data should be packed with a roverpacket ack

// #define DEBUG_DATA
// #define DEBUG_ADDR
// #define DEBUG_ENCODE

#define MAG_STR 45

/* ACK error codes:
 *  01: An expected MAC acknowledgement never occured
 *  02: CCA failure
 *  03: Packet was purged without being transmitted
 *  04: Physical error on the interface with the WiFi transceiver
 *  18: No Buffers
 *  21: Expected network acknowledgement never occured
 *  22: Not joined to network
 *  23: Self-addressed
 *  24: Address not found
 *  25: Route not found
 *  26: Broadcast relay was not heard
 *  2B: Invalid Binding Table Index
 *  2C: Invalid Endpoint
 *  31: A software erroe occured
 *  32: Resource Error
 *  74: Data payload too large
 *  76: Client socket creation attempt failed
 *  BB: Key not authorized
 */

/* Packet error codes:
 *  1: CHECKSUM_FAILURE
 *  2: PACKET_EXCEEDS_BYTE_ARRAY_LENGTH
 *  3: UNEXPECTED_START_BYTE
 */

/* RoverPacket commands:
 *	0000 0x0 Emergency Stop
 *	0001 0x1 Slow Stop
 *	0010 0x2 Forward
 *	0011 0x3 Backward
 *	0100 0x4 Turn left 90
 *	0101 0x5 Turn right 90
 *	0110 0x6 Start Search/Follow
 *	0111 0x7 IR Sensor Data/Request
 *	1000 0x8 Mag Sensor Data
 *	1001 0x9 
 *	1010 0xA Navigation Data/ACK
 *	1011 0xB Packets Encoded/Decoded
 *	1100 0xC Msgs from Master/Slave
 *	1101 0xD Msgs to Slave/Acks from Slave
 *	1110 0xE Msgs to Master/Acks from Master
 *	1111 0xF Failed Encodes/Packets Queued
 */

// Rover Packet (7 bytes):
// 32-bit time || 10-bit data(left) || 10-bit data(right) || 4-bit command
struct RoverPacket {
	unsigned char byte0; // time 0-7
	unsigned char byte1; // time 8-15
	unsigned char byte2; // time 16-23
	unsigned char byte3; // time 24-31
	unsigned char byte4; // data(l) 32-39
	unsigned char byte5; // data(l) 40-41 || data(r) 42-47
	unsigned char byte6; // data(r) 48-51 || command 52-55
};

int pendingAck = 0; // bool to indicate whether a command is still pending an ack

//----------------------------------------------------------------------
// help ----------- Displays a brief explanation of the rover controller
//					as well as a list of supported key commands.
// Preconditions:   None.
// Postconditions:  Message displayed to user using printf.
//----------------------------------------------------------------------
void help(void) {
	printf("------------- Rover Controller Help -------------\n");
	printf("Input is parsed one character at a time (case    \n");
	printf("insensitive) with a one second delay between     \n");
	printf("messages so that commands can be queued. Note    \n");
	printf("that this feature prevents the ability to        \n");
	printf("emergency stop until the buffer is cleared.      \n");
	printf("------------------- Controls --------------------\n");
	printf("\t\t< Rover 1 >\n");
	printf("\t[Q]\t[W]\t[E]\t[R]\n");
	printf("\tStop\tForward\tE-Stop\tSensors\n\n");
	printf("\t[A]\t[S]\t[D]\t[F]\n");
	printf("\tLeft\tBack\tRight\tSearch\n\n");
	printf("\t\t< Rover 2 >\n");
	printf("\t[U]\t[I]\t[O]\n");
	printf("\tStop\tForward\tE-Stop\n\n");
	printf("[H]\t[J]\t[K]\t[L]\n");
	printf("Follow\tLeft\tBack\tRight\n\n");
	printf("\t\t< General >\n");
	printf("\t[_]\t[X]\t[?]\n");
	printf("\tSleep\tExit\tHelp\n");
	printf("-------------------------------------------------\n");
}

//----------------------------------------------------------------------
// determineDirection Determines approximate direction assuming a mag
//					sensor is mounted to the side of the device
//					providing roll (x) and heading (z).
//					Only determines N, E, S, W (no combination).
// Preconditions:   None.
// Postconditions:  Message displayed to user using printf.
//----------------------------------------------------------------------
void determineDirection(short x, short z) {
	// 0 East <-Z-> West 45
	// 0 South <-X-> North 45
	z -= MAG_STR / 2;
	x -= MAG_STR / 2;
	if (abs(z) > abs(x)) { // East/West stronger
		if (z >= 0) // West
			printf("Approximate Direction:\tWest\n");
		else // East
			printf("Approximate Direction:\tEast\n");
	}
	else { // North/South stronger
		if (x >= 0) // North
			printf("Approximate Direction:\tNorth\n");
		else // South
			printf("Approximate Direction:\tSouth\n");
	}
}

//----------------------------------------------------------------------
// encodePacket --- Encodes data as a roverPacket and sends it over the
//					provided xbee connection.
// Preconditions:   Connection is configured. lData and rData are only 
//					10 bits each, and cmd is only 4 bits. Additional
//					bits will be ignored.
// Postconditions:  Message is transmitted over the xbee connection and 
//					pendingAck is updated if using regular xbee acks.
//----------------------------------------------------------------------
void encodePacket(unsigned char cmd, short lData, short rData, struct xbee_con *con) {
	// initialize the packet
	struct RoverPacket thePacket;
	// thePacket.byte0 = 0;
	// thePacket.byte1 = 0;
	// thePacket.byte2 = 0;
	// thePacket.byte3 = 0;
	thePacket.byte4 = 0;
	thePacket.byte5 = 0;
	thePacket.byte6 = 0;
	
	// timestamp 32-bit: 0-7, 8-15, 16-23, 24-31
	thePacket.byte3 = 0xFF;
	thePacket.byte2 = 0xFF;
	thePacket.byte1 = 0xFF;
	thePacket.byte0 = 0xFF;
	
	// data left 10-bit: 32-39, 40-41
	if (lData != 0) {
		thePacket.byte5 = lData & 0x03;
		thePacket.byte5 <<= 6;
		thePacket.byte4 = (lData >> 2) & 0xFF;
	}
	
	// data right 10-bit: 42-47, 48-51
	if (rData != 0) {
		thePacket.byte6 = rData & 0x0F;
		thePacket.byte6 <<= 4;
		thePacket.byte5 += (rData >> 4) & 0x3F;
	}
	
	// command 4-bit: 52-55
	thePacket.byte6 += cmd & 0x0F;
	
	#ifdef DEBUG_ENCODE
		printf("Encoded cmd: 0x%X lData: %i rData: %i", cmd, lData, rData);
		printf(" -> [%02X%02X%02X%02X%02X%02X%02X]\n", thePacket.byte0,
				thePacket.byte1, thePacket.byte2, thePacket.byte3,
				thePacket.byte4, thePacket.byte5, thePacket.byte6);
	#endif
	
	// send the packet
	xbee_err ret;
	unsigned char retVal;
	if ((ret = xbee_conTx(con, &retVal, "%c%c%c%c%c%c%c", thePacket.byte0,
				thePacket.byte1, thePacket.byte2, thePacket.byte3,
				thePacket.byte4, thePacket.byte5, thePacket.byte6)) != XBEE_ENONE) {
        if (ret == XBEE_ETX) {
			fprintf(stderr, "A transmission error occured. (0x%02X)\n", retVal);
        } else {
			fprintf(stderr, "An error occured. %s\n", xbee_errorToStr(ret));
        }
	} else {
		#ifndef COM_USE_ROVER_ACKS
			// Regular ack received
			pendingAck = 0;
		#endif
	}
}

//----------------------------------------------------------------------
// roverCallback -- Receives xbee messages and decodes the messages as 
//					rover packets. Also sends back roverpacket acks if
//					enabled.
// Preconditions:   Connection is configured.
// Postconditions:  Message is received, data decoded as a rover packet
//					and displayed to user, pendingAck is updated if 
//					using rover packet acks, and a rocker packet ack
//					is returned to sender if enabled.
//----------------------------------------------------------------------
// TODO: use void data not xbee_pkt
void roverCallback(struct xbee *xbee, struct xbee_con *con, struct xbee_pkt **pkt, void **data) {
	#ifdef DEBUG_ADDR
		if ((*pkt)->address.addr64_enabled) {
			printf("64-bit address: 0x%02X%02X%02X%02X 0x%02X%02X%02X%02X\n",
						  (*pkt)->address.addr64[0], (*pkt)->address.addr64[1],
						  (*pkt)->address.addr64[2], (*pkt)->address.addr64[3],
						  (*pkt)->address.addr64[4], (*pkt)->address.addr64[5],
						  (*pkt)->address.addr64[6], (*pkt)->address.addr64[7]);
		}
	#endif
	
	#ifdef DEBUG_DATA
		printf("rx: [");
		for (int i = 0; i < (*pkt)->dataLen; i++) {
			if(i % 7 == 0 && i != 0)
				printf("\n");
			printf(" %02X ", (*pkt)->data[i]);
		}
		printf("]\n\n");
	#endif
		
	if ((*pkt)->dataLen > 0 && (*pkt)->dataLen % 7 == 0) { // require a non-empty payload divisible by 7 bytes
		unsigned long timestamp;
		unsigned char cmd;
		short lData, rData;
		
		// decode rover packet one at a time
		for (int i = 0; i < (*pkt)->dataLen; i = i + 7) {
			// Check if timestamp is 0 and ignore if so
			if ((*pkt)->data[i + 3] == 0 && (*pkt)->data[i + 2] == 0 &&
					(*pkt)->data[i + 1] == 0 && (*pkt)->data[i] == 0) {
				break;
			}
			
			// timestamp 0-7, 8-15, 16-23, 24-31
			timestamp = (*pkt)->data[i];
			timestamp <<= 8;
			timestamp += (*pkt)->data[i + 1];
			timestamp <<= 8;
			timestamp += (*pkt)->data[i + 2];
			timestamp <<= 8;
			timestamp += (*pkt)->data[i + 3];
			
			// data left 32-39, 40-41
			lData = (*pkt)->data[i + 4];
			lData <<= 8; // set the sign bit
			lData >>= 6; // repeat the sign bit
			lData += (*pkt)->data[i + 5] >> 6; // grab last 2 bits
			
			// data right 42-47, 48-51
			rData = (*pkt)->data[i + 5];
			rData <<= 10; // set the sign bit and discard first two bits
			rData >>= 6; // repeat the sign bit
			rData += (*pkt)->data[i + 6] >> 4; // grab last 4 bits
			
			// command 52-55
			cmd = (*pkt)->data[i + 6] & 0x0F;
			
			// Display the sender if this isnt an ack
			if (i == 0 && cmd != 0xA) {
				printf("--------------------");
				if ((*pkt)->address.addr64[6] == R1_ADDR_6 && (*pkt)->address.addr64[7] == R1_ADDR_7) {
					printf(" Rover 1 ");
				} else if ((*pkt)->address.addr64[6] == R2_ADDR_6 && (*pkt)->address.addr64[7] == R2_ADDR_7) {
					printf(" Rover 2 ");
				} else {
					printf("---------");
				}
				printf("--------------------\n");
			}
			
			switch (cmd) {
				case 0x7: // IR Sensor Data
					printf("leftDiff: %i\t\trightDiff: %i\n", lData, rData);
					break;
					
				case 0x8: // Mag Sensor Data
					printf("magX: %i \t\tmagZ: %i\n", lData, rData);
					determineDirection(lData, rData);
					break;
					
				case 0xA: // Rover ACK
					if (timestamp == 0xFFFFFFFF) {
						// printf("Rover ACK received.\n");
						pendingAck = 0;
					}
					break;
					
				case 0xB: // Packets Encoded/Decoded
					printf("encodedPackets: %i\tdecodedPackets: %i\n", lData, rData);
					break;
					
				case 0xC: // Msgs from Master/Slave
					printf("msgsFromMaster: %i\tmsgsFromSlave: %i\n", lData, rData);
					break;
					
				case 0xD: // Msgs to Slave/Acks from Slave
					printf("msgsToSlave: %i\t\tacksFromSlave: %i\n", lData, rData);
					break;
					
				case 0xE: // Master Msgs To/From
					printf("msgsToMaster: %i \tacksFromMaster: %i\n", lData, rData);
					break;
					
				case 0xF: // Failed Encodes/Packets Queued
					printf("failedEncodes: %i \tqueuedPackets: %i\n", lData, rData);
					break;
					
			}
		}
		
		#ifdef COM_USE_ROVER_ACKS
			// send rover ack
			if (cmd != 0xA) {
				#ifdef DEBUG_ENCODE
					printf("Encoded cmd: 0x%X lData: %i rData: %i", 0xA, 0, 0);
					printf(" -> [%02X%02X%02X%02X%02X%02X%02X]\n",
							0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x0A);
				#endif
				
				xbee_err ret;
				unsigned char retVal;
				if ((ret = xbee_conTx(con, &retVal, "%c%c%c%c%c%c%c",
							0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x0A)) != XBEE_ENONE) {
					if (ret == XBEE_ETX) {
							fprintf(stderr, "ACK: A transmission error occured. (0x%02X)\n", retVal);
					} else {
							fprintf(stderr, "ACK: An error occured. %s\n", xbee_errorToStr(ret));
					}
				}
			}
		#endif
		
		if (cmd != 0xA) {
			printf("-------------------------------------------------\n");
		}
	}
}

//----------------------------------------------------------------------
// parseCommand --- Parses a character input to determine the desired
//					rover packet to send.
// Preconditions:   Connections are configured.
// Postconditions:  An appropriate packet is sent using encodePacket on 
//					the desired connection, and pendingAck is set true.
//----------------------------------------------------------------------
// returns whether or not a character was sucessfully parsed
int parseCommand(char input, struct xbee_con *r1Connection, struct xbee_con *r2Connection) {
	int retVal = 1;
	pendingAck = 1;
	
	switch(input) {
		case 'e': // Emergency Stop - Rover 1
		case 'E':
			printf("Sending Emergency Stop (0x%X) command to Rover 1...\n", 0x0);
			encodePacket(0x0, 0x0, 0x0, r1Connection);
			break;

		case 'q': // Slow Stop - Rover 1
		case 'Q':
			printf("Sending Slow Stop (0x%X) command to Rover 1...\n", 0x1);
			encodePacket(0x1, 0x0, 0x0, r1Connection);
			break;
		  
		case 'w': // Forward - Rover 1
		case 'W':
			printf("Sending Forward (0x%X) command to Rover 1...\n", 0x2);
			encodePacket(0x2, 0x0, 0x0, r1Connection);
			break;

		case 's': // Backward - Rover 1
		case 'S':
			printf("Sending Backward (0x%X) command to Rover 1...\n", 0x3);
			encodePacket(0x3, 0x0, 0x0, r1Connection);
			break;

		case 'a': // Turn Left 90 - Rover 1
		case 'A':
			printf("Sending Turn Left 90 (0x%X) command to Rover 1...\n", 0x4);
			encodePacket(0x4, 0x0, 0x0, r1Connection);
			break;

		case 'd': // Turn Right 90 - Rover 1
		case 'D':
			printf("Sending Turn Right 90 (0x%X) command to Rover 1...\n", 0x5);
			encodePacket(0x5, 0x0, 0x0, r1Connection);
			break;

		case 'f': // Start Search - Rover 1
		case 'F':
			printf("Sending Start Search (0x%X) command to Rover 1...\n", 0x6);
			encodePacket(0x6, 0x0, 0x0, r1Connection);
			break;

		case 'r': // Sensor Request - Rover 1
		case 'R':
			printf("Sending Sensor Request (0x%X) command to Rover 1...\n", 0x7);
			encodePacket(0x7, 0x0, 0x0, r1Connection);
			break;
			
		case 'o': // Emergency Stop - Rover 2
		case 'O':
			printf("Sending Emergency Stop (0x%X) command to Rover 2...\n", 0x0);
			encodePacket(0x0, 0x0, 0x0, r2Connection);
			break;

		case 'u': // Slow Stop - Rover 2
		case 'U':
			printf("Sending Slow Stop (0x%X) command to Rover 2...\n", 0x1);
			encodePacket(0x1, 0x0, 0x0, r2Connection);
			break;
		  
		case 'i': // Forward - Rover 2
		case 'I':
			printf("Sending Forward (0x%X) command to Rover 2...\n", 0x2);
			encodePacket(0x2, 0x0, 0x0, r2Connection);
			break;

		case 'k': // Backward - Rover 2
		case 'K':
			printf("Sending Backward (0x%X) command to Rover 2...\n", 0x3);
			encodePacket(0x3, 0x0, 0x0, r2Connection);
			break;

		case 'j': // Turn Left 90 - Rover 2
		case 'J':
			printf("Sending Turn Left 90 (0x%X) command to Rover 2...\n", 0x4);
			encodePacket(0x4, 0x0, 0x0, r2Connection);
			break;

		case 'l': // Turn Right 90 - Rover 2
		case 'L':
			printf("Sending Turn Right 90 (0x%X) command to Rover 2...\n", 0x5);
			encodePacket(0x5, 0x0, 0x0, r2Connection);
			break;

		case 'h': // Start Follow - Rover 2
		case 'H':
			printf("Sending Start Search (0x%X) command to Rover 2...\n", 0x6);
			encodePacket(0x6, 0x0, 0x0, r2Connection);
			break;

		/*case 'p': // Sensor Request - Rover 2
		case 'P':
			printf("Sending Sensor Request (0x%X) command to Rover 2...\n", input);
			encodePacket(0x7, 0x0, 0x0, r2Connection);
			break;*/
			
		case '?': // Help
		case '/':
			help();
			pendingAck = 0;
			break;
			
		case ' ': // NOP
		case '_':
			printf("Sleep...\n");
			pendingAck = 0;
			break;
			
		case 'x': // Exit
		case 'X':
			printf("Exiting...\n");
			pendingAck = 0;
			retVal = -1;
			break;
			
		default:
			retVal = 0; // quickly move on to the next character
			break;			
	}
	
	return retVal;
}

//----------------------------------------------------------------------
// main ----------- Performs initialization of xbee and handles main 
//					logic of the rover controller to take input from user.
// Preconditions:   None.
// Postconditions:  None.
//----------------------------------------------------------------------
int main(void) {
	void *d;
	struct xbee *xbee;
	struct xbee_con *r1Connection;
	struct xbee_con *r2Connection;
	struct xbee_conAddress r1Address;
	struct xbee_conAddress r2Address;
	xbee_err ret;
	char lastCmd = '!';
	
	// setup local xbee connection
	if ((ret = xbee_setup(&xbee, "xbee1", "/dev/ttyUSB0", 9600)) != XBEE_ENONE) {
		printf("ret: %d (%s)\n", ret, xbee_errorToStr(ret));
		return ret;
	}
	
	// create the memory address objects
	memset(&r1Address, 0, sizeof(r1Address));
	r1Address.addr64_enabled = 1;
	r1Address.addr64[0] = R1_ADDR_0;
	r1Address.addr64[1] = R1_ADDR_1;
	r1Address.addr64[2] = R1_ADDR_2;
	r1Address.addr64[3] = R1_ADDR_3;
	r1Address.addr64[4] = R1_ADDR_4;
	r1Address.addr64[5] = R1_ADDR_5;
	r1Address.addr64[6] = R1_ADDR_6;
	r1Address.addr64[7] = R1_ADDR_7;
	
	memset(&r2Address, 0, sizeof(r2Address));
	r2Address.addr64_enabled = 1;
	r2Address.addr64[0] = R2_ADDR_0;
	r2Address.addr64[1] = R2_ADDR_1;
	r2Address.addr64[2] = R2_ADDR_2;
	r2Address.addr64[3] = R2_ADDR_3;
	r2Address.addr64[4] = R2_ADDR_4;
	r2Address.addr64[5] = R2_ADDR_5;
	r2Address.addr64[6] = R2_ADDR_6;
	r2Address.addr64[7] = R2_ADDR_7;
	
	// open a connection with rover 1 and attach to the callback
	if ((ret = xbee_conNew(xbee, &r1Connection, "64-bit Data", &r1Address)) != XBEE_ENONE) {
		xbee_log(xbee, -1, "xbee_conNew() for rover 1 returned: %d (%s)", ret, xbee_errorToStr(ret));
		return ret;
	}

	if ((ret = xbee_conDataSet(r1Connection, xbee, NULL)) != XBEE_ENONE) {
		xbee_log(xbee, -1, "xbee_conDataSet() for rover 1 returned: %d", ret);
		return ret;
	}

	if ((ret = xbee_conCallbackSet(r1Connection, roverCallback, NULL)) != XBEE_ENONE) {
		xbee_log(xbee, -1, "xbee_conCallbackSet() for rover 1 returned: %d", ret);
		return ret;
	}
	
	// open a connection with rover 2 and attach to the callback
	if ((ret = xbee_conNew(xbee, &r2Connection, "64-bit Data", &r2Address)) != XBEE_ENONE) {
		xbee_log(xbee, -1, "xbee_conNew() for rover 2 returned: %d (%s)", ret, xbee_errorToStr(ret));
		return ret;
	}

	if ((ret = xbee_conDataSet(r2Connection, xbee, NULL)) != XBEE_ENONE) {
		xbee_log(xbee, -1, "xbee_conDataSet() for rover 2 returned: %d", ret);
		return ret;
	}

	if ((ret = xbee_conCallbackSet(r2Connection, roverCallback, NULL)) != XBEE_ENONE) {
		xbee_log(xbee, -1, "xbee_conCallbackSet() for rover 2 returned: %d", ret);
		return ret;
	}
	
	#ifdef COM_USE_ROVER_ACKS
		// disable normal acks
		struct xbee_conSettings settings;
		if (xbee_conSettings(r1Connection, NULL, &settings) != XBEE_ENONE) return -1;
		settings.disableAck = 1;
		if (xbee_conSettings(r1Connection, &settings, NULL) != XBEE_ENONE) return -1;
		if (xbee_conSettings(r2Connection, NULL, &settings) != XBEE_ENONE) return -1;
		settings.disableAck = 1;
		if (xbee_conSettings(r2Connection, &settings, NULL) != XBEE_ENONE) return -1;
	#endif
	
	help(); // display help
	
	// continually check callback status and read input from user
	for (;;) {
		void *p;

		if ((ret = xbee_conCallbackGet(r1Connection, (xbee_t_conCallback*)&p)) != XBEE_ENONE) {
			xbee_log(xbee, -1, "xbee_conCallbackGet() for rover 1 returned: %d", ret);
			return ret;
		}
		
		if ((ret = xbee_conCallbackGet(r2Connection, (xbee_t_conCallback*)&p)) != XBEE_ENONE) {
			xbee_log(xbee, -1, "xbee_conCallbackGet() for rover 2 returned: %d", ret);
			return ret;
		}

		if (p == NULL) break;
		
		// Check if ack was received for last message and retry once if not
		if (pendingAck) {
			pendingAck = 0;
			printf("RETRY: ");
			parseCommand(lastCmd, r1Connection, r2Connection);
		}	
		
		// Read input from user
		int goodParse = 0;
		while (!goodParse) {
			scanf("%c", &lastCmd);
			goodParse = parseCommand(lastCmd, r1Connection, r2Connection);
			if (goodParse == -1) { // exit request
				xbee_shutdown(xbee);
				exit(0);
			}
		}

		sleep(1);
	}

	if ((ret = xbee_conEnd(r1Connection)) != XBEE_ENONE) {
		xbee_log(xbee, -1, "xbee_conEnd() for rover 1 returned: %d", ret);
		return ret;
	}
	
	if ((ret = xbee_conEnd(r2Connection)) != XBEE_ENONE) {
		xbee_log(xbee, -1, "xbee_conEnd() for rover 2 returned: %d", ret);
		return ret;
	}

	xbee_shutdown(xbee);

	return 0;
}
