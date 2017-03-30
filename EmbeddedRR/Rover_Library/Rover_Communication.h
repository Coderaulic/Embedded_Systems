//----------------------- Rover_Communication --------------------------
// Filename:      	Rover_Communication.h
// Project Team:  	EmbeddedRR
// Group Members: 	Robert Griswold and Ryu Muthui
// Date:          	2 Dec 2016
// Description:   	XBee communication code for two way communication
//					between devices in API 2 mode.
//------------------------------ Includes ------------------------------
#ifndef _Rover_Communication_h_
#define _Rover_Communication_h_

#include <XBee.h>
#include <QueueArray.h>
#include <Arduino.h>

//---------------------------- Definitions -----------------------------
// Configuration
#define MAX_SIZE 84 	// Number of bytes max in a payload (xbee supports 100) must be a multiple of MIN_SIZE
#define MASTER_SIZE 35 	// Number of bytes max in a master payload (xbee supports 100) must be a multiple of MIN_SIZE
#define MIN_SIZE 7 		// Number of bytes min for a roverPacket

// #define COM_USE_ROVER_ACKS // Whether to use xbee acks or roverpacket acks
// Note that no additional data should be packed with a roverpacket ack

// #define COM_DEBUG_ENCODE
// #define COM_DEBUG_UNWRAP
// #define COM_DEBUG_XBEE
// #define COM_DEBUG_STATS
// #define COM_DEBUG_QUEUE

// Status
#define ACK_SUCCESS 0
#define ACK_FAILURE -1
#define RCV_SIXTEEN 1
#define RCV_SIXTYFOUR 2
#define RCV_ERROR -1
#define RCV_UNTRUSTED -2
#define ENCODE_ERROR -1

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
	unsigned char byte0 = 0; // time 0-7
	unsigned char byte1 = 0; // time 8-15
	unsigned char byte2 = 0; // time 16-23
	unsigned char byte3 = 0; // time 24-31
	unsigned char byte4 = 0; // data(l) 32-39
	unsigned char byte5 = 0; // data(l) 40-41 || data(r) 42-47
	unsigned char byte6 = 0; // data(r) 48-51 || command 52-55
};

//------------------------------ Class Functions ------------------------
//----------------------------------------------------------------------
// com_setupComs -- Initializes the xbee communication with a master and 
// 					slave address for future xbee communication.
//----------------------------------------------------------------------
void com_setupComs(uint32_t msb_master, uint32_t lsb_master, uint32_t msb_slave, uint32_t lsb_slave);

//----------------------------------------------------------------------
// com_getAck -----	Waits for a TX_STATUS_RESPONSE packet from xbee and 
// 					returns an integer reporting the status of the ACK 
//					message. The TxStatusResponse is stored in txStatus.
// Preconditions:   xbee object is configured.
// Postconditions:  Returns ACK_SUCCESS (0) if the recieved packet 
//					contains a success ACK response,
//                  Returns ACK_FAILURE (-1) the packet has an error 
//					code or timed out,
//                  Returns >0 corresponding to a TX_STATUS_RESPONSE error.
//----------------------------------------------------------------------
int com_getAck(int timeout);
int com_getAck(); // Overloaded com_getAck with a default 500ms timout.

//----------------------------------------------------------------------
// com_getRoverAck64 Waits for a RX_64_RESPONSE packet from xbee and 
// 					returns an integer reporting the status of the ACK 
//					message. The ack is stored in rx64 just like a
//					regular message using com_receiveData.
// Preconditions:   xbee object is configured.
// Postconditions:  Returns ACK_SUCCESS (0) if the recieved packet 
//					is a rover packet 0xA with a FFFFFFFF timestamp.
//                  Returns ACK_FAILURE (-1) if it timed out or some 
//					other unexpected packet was received (dropping that
//					packet).
//----------------------------------------------------------------------
int com_getRoverAck64(int timeout);
int com_getRoverAck64(); // Overloaded com_getRoverAck64 with a default 500ms timout.

//----------------------------------------------------------------------
// com_receiveData  Reads xbee for a RX_16_RESPONSE or RX_64_RESPONSE 
// 					packet, storing the data in rx16 or rx64, and returns
//					an integer value indicating the type of response 
//					recieved or error.
// Preconditions:   xbee object is configured.
// Postconditions:  Returns RCV_SIXTEEN (1) if the recieved packet was a 
//					RX_16_RESPONSE,
//                  Returns RCV_SIXTYFOUR (2) if the received packet was 
//					a RX_64_RESPONSE,
//                  Returns RCV_ERROR (-1) if an error occured.
//----------------------------------------------------------------------
int com_receiveData(int timeout, bool ack);
int com_receiveData(); // Overloaded com_receiveData with a default 0s timeout.

//----------------------------------------------------------------------
// com_sendMaster64 Sends the currently loaded payload to the master and
//					then checks for an ack if checkAck is true.	
// Preconditions:   xbee object is configured.
// Postconditions:  payloadMaster is wiped and payloadEndMaster index is 
//					reset to 0 if we arent checking for an ack or the 
//					ack was successful. Int code returned is ack status code.
//					See com_getAck for more information. ACK_FAILURE (-1) 
//					is always returned if checkAck is false.
//----------------------------------------------------------------------
int com_sendMaster64(bool checkAck);

//----------------------------------------------------------------------
// com_sendSlave64  Sends the currently loaded payload to the slave and
//					then checks for an ack if checkAck is true.	
// Preconditions:   xbee object is configured.
// Postconditions:  payloadSlave is wiped and payloadEndSlave index is 
//					reset to 0 if we arent checking for an ack or the 
//					ack was successful. Int code returned is ack status code.
//					See com_getAck for more information. ACK_FAILURE (-1) 
//					is always returned if checkAck is false.
//----------------------------------------------------------------------
int com_sendSlave64(bool checkAck);

//----------------------------------------------------------------------
// com_unwrapAndQueue64 Parses the data in the last rx64 xbee packet as 
//					7 byte rover packets that are enqueued.
// Preconditions:   Data has been receieved already and enough memory
//					is available.
// Postconditions:  7 byte rover packets are enqueued. If a high priority
//					packet is received, true is returned. Worst rssi 
// 					value is also updated.
//----------------------------------------------------------------------
bool com_unwrapAndQueue64();

//----------------------------------------------------------------------
// com_decodeNext - Dequeues the next roverPacket and decodes it.
// Preconditions:   All parameters point to valid memory.
// Postconditions:  Returns true if a packet was decoded and sets the 
// 					passed in paramters to the decoded values.
//----------------------------------------------------------------------
bool com_decodeNext(unsigned long* timestamp, unsigned char* cmd, int* lData, int* rData);

//----------------------------------------------------------------------
// com_encodeSlavePacket Encodes data as a roverPacket and loads it in 
//					to the payload for the slave. An integer indicating 
//					how many more packets can be loaded into the payload
//					will be returned. Any subsequent calls to this 
//					method while the payload is full will simply 
//					return ENCODE_ERROR (-1) and the data will not be 
//					saved.
// Preconditions:   xbee object is configured. lData and rData are only 
//					10 bits each, and cmd is only 4 bits. Additional
//					bits will be ignored.
// Postconditions:  Returns an integer indicating how many more packets
//					can be loaded into the payload before it is full.
//----------------------------------------------------------------------
int com_encodeSlavePacket(unsigned char cmd, int lData, int rData);

//----------------------------------------------------------------------
// com_encodeMasterPacket Encodes data as a roverPacket and loads it in 
//					to the payload for the master. An integer indicating 
//					how many more packets can be loaded into the payload
//					will be returned. Any subsequent calls to this 
//					method while the payload is full will simply 
//					return ENCODE_ERROR (-1) and the data will not be 
//					saved.
// Preconditions:   xbee object is configured. lData and rData are only 
//					10 bits each, and cmd is only 4 bits. Additional
//					bits will be ignored.
// Postconditions:  Returns an integer indicating how many more packets
//					can be loaded into the payload before it is full.
//----------------------------------------------------------------------
int com_encodeMasterPacket(unsigned char cmd, int lData, int rData);

//----------------------------------------------------------------------
// com_emptyPayload Empties the payload for master or the slave.
// Preconditions:   None.
// Postconditions:  Payload is emptied and payloadEnd index is 0.
//----------------------------------------------------------------------
void com_emptyPayload(bool slave);

//----------------------------------------------------------------------
// com_getLastRssi  Getter for lastRssi. Higher magnitude is worse.
// Preconditions:   None.
// Postconditions:  Returns an uint8_t.
//----------------------------------------------------------------------
uint8_t com_getLastRssi();

//----------------------------------------------------------------------
// com_getMsgsToMaster  Getter for msgsToMaster.
// Preconditions:   None.
// Postconditions:  Returns an unsigned int.
//----------------------------------------------------------------------
unsigned int com_getMsgsToMaster();

//----------------------------------------------------------------------
// com_getMsgsToSlave  Getter for msgsToSlave.
// Preconditions:   None.
// Postconditions:  Returns an unsigned int.
//----------------------------------------------------------------------
unsigned int com_getMsgsToSlave();

//----------------------------------------------------------------------
// com_getMsgsToMaster  Getter for msgsFromMaster.
// Preconditions:   None.
// Postconditions:  Returns an unsigned int.
//----------------------------------------------------------------------
unsigned int com_getMsgsFromMaster();

//----------------------------------------------------------------------
// com_getMsgsToSlave  Getter for msgsFromSlave.
// Preconditions:   None.
// Postconditions:  Returns an unsigned int.
//----------------------------------------------------------------------
unsigned int com_getMsgsFromSlave();

//----------------------------------------------------------------------
// com_getAcksFromMaster  Getter for acksFromMaster.
// Preconditions:   None.
// Postconditions:  Returns an unsigned int.
//----------------------------------------------------------------------
unsigned int com_getAcksFromMaster();

//----------------------------------------------------------------------
// com_getAcksFromSlave  Getter for acksFromSlave.
// Preconditions:   None.
// Postconditions:  Returns an unsigned int.
//----------------------------------------------------------------------
unsigned int com_getAcksFromSlave();

//----------------------------------------------------------------------
// com_getEncodedPackets  Getter for encodedPackets.
// Preconditions:   None.
// Postconditions:  Returns an unsigned int.
//----------------------------------------------------------------------
unsigned int com_getEncodedPackets();

//----------------------------------------------------------------------
// com_getDecodedPackets  Getter for decodedPackets.
// Preconditions:   None.
// Postconditions:  Returns an unsigned int.
//----------------------------------------------------------------------
unsigned int com_getDecodedPackets();

//----------------------------------------------------------------------
// com_getFailedEncodes  Getter for failedEncodes.
// Preconditions:   None.
// Postconditions:  Returns an unsigned int.
//----------------------------------------------------------------------
unsigned int com_getFailedEncodes();

//----------------------------------------------------------------------
// com_getQueuedPackets  Getter for total queuedPackets.
// Preconditions:   None.
// Postconditions:  Returns an unsigned int.
//----------------------------------------------------------------------
unsigned int com_getQueuedPackets();

//----------------------------------------------------------------------
// com_getQueuedPackets  Getter for currently queuedPackets.
// Preconditions:   None.
// Postconditions:  Returns an int.
//----------------------------------------------------------------------
int com_getCurrentlyQueuedPackets();

//----------------------------------------------------------------------
// com_sendStatistics64 Sends the current communnication statistics to 
// 					master using roverPackets with an optional ack.
// Preconditions:   xbee object is configured
// Postconditions:  payloadMaster is wiped and payloadEndMaster index is 
//					reset to 0 per com_sendMaster64. Int code returned is 
//					sum of ack status code(s). See com_getAck for more 
//					information. ACK_FAILURE (-1) is always returned if 
//					checkAck is false.
//----------------------------------------------------------------------
int com_sendStatistics64(bool checkAck);

//----------------------------------------------------------------------
// com_resetStatistics Resets the debugging network statistics back to 0.
// Preconditions:   None.
// Postconditions:  msgsToMaster, msgsToSlave, msgsFromMaster, 
//					msgsFromSlave, acksFromMaster, acksFromSlave, 
// 					encodedPackets, decodedPackets, queuedPackets, and 
//					failedEncodes are all reset to 0.
//----------------------------------------------------------------------
void com_resetStatistics();

//----------------------------------------------------------------------
// com_emptyQueue Empties the packetQueue
// Preconditions:   None.
// Postconditions:  packetQueue is emptied.
//----------------------------------------------------------------------
void com_emptyQueue();

//----------------------------------------------------------------------
// com_getMasterSlotsLeft Getter for the space remaining in the master
//					payload.
// Preconditions:   None.
// Postconditions:  Returns an integer indicating how many more packets
//					can be loaded into the payload before it is full.
//----------------------------------------------------------------------
int com_getMasterSlotsLeft();

//----------------------------------------------------------------------
// com_getSlaveSlotsLeft Getter for the space remaining in the slave
//					payload.
// Preconditions:   None.
// Postconditions:  Returns an integer indicating how many more packets
//					can be loaded into the payload before it is full.
//----------------------------------------------------------------------
int com_getSlaveSlotsLeft();

//----------------------------------------------------------------------
// com_geMaxtMasterSlots Getter for the maximum space in the master
//					payload.
// Preconditions:   None.
// Postconditions:  Returns an integer indicating the maximum number 
//					of packets that can be loaded into the payload.
//----------------------------------------------------------------------
int com_geMaxtMasterSlots();

//----------------------------------------------------------------------
// com_getMaxSlaveSlots Getter for the maximum space in the slave
//					payload.
// Preconditions:   None.
// Postconditions:  Returns an integer indicating the maximum number 
//					of packets that can be loaded into the payload.
//----------------------------------------------------------------------
int com_getMaxSlaveSlots();

#endif