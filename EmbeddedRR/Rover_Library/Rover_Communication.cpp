//----------------------- Rover_Communication --------------------------
// Filename:      	Rover_Communication.cpp
// Project Team:  	EmbeddedRR
// Group Members: 	Robert Griswold and Ryu Muthui
// Date:          	2 Dec 2016
// Description:   	XBee communication code for two way communication
//					between devices in API 2 mode.
//----------------------------------------------------------------------
#include "Rover_Communication.h"

//---------------------------- Initialization --------------------------
XBee xbee = XBee();

// XBeeResponse response = XBeeResponse();
Rx16Response c_rx16 = Rx16Response(); 	// reusable response16 object
Rx64Response c_rx64 = Rx64Response(); 	// reusable response64 object

uint8_t c_payloadMaster[MASTER_SIZE]; 	// allocate the tx payload for master
uint8_t c_payloadEndMaster = 0; 		// index for the next available space in the payload
uint8_t c_payloadSlave[MAX_SIZE]; 		// allocate the tx payload for master
uint8_t c_payloadEndSlave = 0; 			// index for the next available space in the payload

XBeeAddress64 c_addr64Master;  			// reusable address object to master
XBeeAddress64 c_addr64Slave;  			// reusable address object to slave
Tx64Request c_tx64Master; 				// reusable tx object to master
Tx64Request c_tx64Slave; 				// reusable tx object to slave

#ifdef COM_USE_ROVER_ACKS
	uint8_t c_payloadAck[MIN_SIZE]; 	// allocates the tx payload for rover acks
	Tx64Request c_tx64MasterAck; 		// reusable tx object to master for rover acks
	Tx64Request c_tx64SlaveAck; 		// reusable tx object to slave for rover acks
#endif

QueueArray<RoverPacket> c_packetQueue;

uint8_t c_lastRssi = 0; 				// magnitude of last rssi - higher is worse
unsigned int c_failedEncodes = 0;		// count of failed attempts to encode a packet because buffer was full
unsigned int c_msgsToMaster = 0;		// count of msgs sent to master
unsigned int c_msgsFromMaster = 0;		// count of msgs received from master
unsigned int c_acksFromMaster = 0;		// count of acks from master
unsigned int c_msgsToSlave = 0;			// count of msgs sent to slave
unsigned int c_msgsFromSlave = 0;		// count of msgs received from slave
unsigned int c_acksFromSlave = 0;		// count of acks from slave
unsigned int c_encodedPackets = 0;		// count of roverPackets encoded
unsigned int c_decodedPackets = 0;		// count of roverPackets decoded
unsigned int c_queuedPackets = 0;		// count of roverPackets unwrapped and queued

//------------------------------ Class Functions -----------------------
//----------------------------------------------------------------------
// com_setupComs -- Initializes the xbee communication with a master and 
// 					slave address for future xbee communication.
//----------------------------------------------------------------------
void com_setupComs(uint32_t msb_master, uint32_t lsb_master, uint32_t msb_slave, uint32_t lsb_slave) {
	xbee.setSerial(Serial); // Use hardware serial
	
	c_addr64Master = XBeeAddress64(msb_master, lsb_master);
	c_addr64Slave = XBeeAddress64(msb_slave, lsb_slave);
	
	#ifdef COM_USE_ROVER_ACKS
		c_tx64Master = Tx64Request(c_addr64Master, 0x01, c_payloadMaster, sizeof(c_payloadMaster), 0x0);
		c_tx64Slave = Tx64Request(c_addr64Slave, 0x01, c_payloadSlave, sizeof(c_payloadSlave), 0x0);
		c_tx64MasterAck = Tx64Request(c_addr64Master, 0x01, c_payloadAck, sizeof(c_payloadAck), 0x0);
		c_tx64SlaveAck = Tx64Request(c_addr64Slave, 0x01, c_payloadAck, sizeof(c_payloadAck), 0x0);
		c_payloadAck[0] = 0xFF;
		c_payloadAck[1] = 0xFF;
		c_payloadAck[2] = 0xFF;
		c_payloadAck[3] = 0xFF;
		c_payloadAck[4] = 0x00;
		c_payloadAck[5] = 0x00;
		c_payloadAck[6] = 0x0A;
	#endif
	#ifndef COM_USE_ROVER_ACKS
		c_tx64Master = Tx64Request(c_addr64Master, c_payloadMaster, sizeof(c_payloadMaster));
		c_tx64Slave = Tx64Request(c_addr64Slave, c_payloadSlave, sizeof(c_payloadSlave));
	#endif
	
	com_resetStatistics();
	
	#ifdef COM_DEBUG_QUEUE
		c_packetQueue.setPrinter(Serial);
	#endif
}

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
int com_getAck(int timeout) {
	int retVal = ACK_FAILURE;
  
	// after sending a tx request, we expect a status response
	// wait up to 5 seconds for the status response
	if (xbee.readPacket(timeout)) {
		// got a response!

		// should be a znet tx status
		if (xbee.getResponse().getApiId() == TX_STATUS_RESPONSE) {
			TxStatusResponse txStatus = TxStatusResponse();
			xbee.getResponse().getTxStatusResponse(txStatus);

			// get the delivery status, the fifth byte
			if (txStatus.getStatus() == SUCCESS) {
				retVal = ACK_SUCCESS;
			}
			else {
				// the remote XBee did not receive our packet.
				retVal = txStatus.getStatus();
			}
		}
    
	}
	else if (xbee.getResponse().isError()) {
		#ifdef COM_DEBUG_XBEE
			Serial.print("Error reading ACK packet. Error code: ");
			Serial.println(xbee.getResponse().getErrorCode());
			delay(100);
		#endif
	}
	else {
		// local XBee did not provide a timely TX Status Response.
		// Radio is not configured properly or connected.
	}
	
	return retVal;
}

// Overloaded getAck with a default 500ms timout.
int com_getAck() {
	return com_getAck(500);
}

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
int com_getRoverAck64(int timeout) {
	int retVal = com_receiveData(timeout, false);
	if (retVal != RCV_SIXTYFOUR)
		return ACK_FAILURE;
	
	// Parse the xbee packet
	int packetSize = c_rx64.getDataLength();
	c_lastRssi = c_rx64.getRssi();
    uint8_t* data = c_rx64.getData();
	
	#ifdef COM_DEBUG_UNWRAP
		uint8_t opt = c_rx64.getOption();
		uint32_t senderMsb = c_rx64.getRemoteAddress64().getMsb();
		uint32_t senderLsb = c_rx64.getRemoteAddress64().getLsb();
		Serial.println();
		Serial.print("From: ");
		Serial.print((long)senderMsb, HEX);
		Serial.print((long)senderLsb, HEX);
		Serial.print(" RSSI: ");
		Serial.print(c_lastRssi, HEX);
		Serial.print(" opt: ");
		Serial.print(opt, HEX);
		Serial.print(" size: ");
		Serial.print(packetSize);
		Serial.print(" data:");
		for (int i = 0; i < packetSize; i++)  { // MAX_SIZE
			if(i%7==0)
				Serial.println();
			Serial.print(data[i], HEX);
			Serial.print(" ");
		}
		Serial.println();
	#endif
	
	// Check packet size
	if (packetSize < MIN_SIZE)
		return ACK_FAILURE;
	
	// Create a packet
	RoverPacket thePacket;
	thePacket.byte0 = data[0]; // time 0-7
	thePacket.byte1 = data[1]; // time 8-15
	thePacket.byte2 = data[2]; // time 16-23
	thePacket.byte3 = data[3]; // time 24-31
	thePacket.byte4 = data[4]; // data(l) 32-39
	thePacket.byte5 = data[5]; // data(l) 40-41 || data(r) 42-47
	thePacket.byte6 = data[6]; // data(r) 48-51 || command 52-55
	
	// Check if timestamp is 0xFFFFFFFF
	if (thePacket.byte3 != 0xFF && thePacket.byte2 != 0xFF && thePacket.byte1 != 0xFF && thePacket.byte0 != 0xFF)
		return ACK_FAILURE;
	
	#ifdef COM_DEBUG_ENCODE
		Serial.println();
		Serial.println("com_getRoverAck64: thePacket Data:");
		Serial.print(thePacket.byte0, HEX);
		Serial.print(" ");
		Serial.print(thePacket.byte1, HEX);
		Serial.print(" ");
		Serial.print(thePacket.byte2, HEX);
		Serial.print(" ");
		Serial.print(thePacket.byte3, HEX);
		Serial.print(" | ");
		Serial.print(thePacket.byte4, HEX);
		Serial.print(" ");
		Serial.print(thePacket.byte5, HEX);
		Serial.print(" ");
		Serial.println(thePacket.byte6, HEX);
	#endif
	
	// Check if cmd is 0xA
	if ((thePacket.byte6 & 0x0F) != 0xA)
		return ACK_FAILURE;
	
	return ACK_SUCCESS;
}
	
// Overloaded com_getRoverAck64 with a default 500ms timout.
int com_getRoverAck64() {
	return com_getRoverAck64(500);
}

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
int com_receiveData(int timeout, bool ack) {
	int retVal = RCV_ERROR;
	
	if (timeout == 0)
		xbee.readPacket();
	else
		xbee.readPacket(timeout);
	
	if (xbee.getResponse().isAvailable()) { // got something
		if (xbee.getResponse().getApiId() == RX_16_RESPONSE || xbee.getResponse().getApiId() == RX_64_RESPONSE) {
			// got a rx packet

			if (xbee.getResponse().getApiId() == RX_16_RESPONSE) {
				xbee.getResponse().getRx16Response(c_rx16);
				retVal = RCV_SIXTEEN;
				
				// determine from who DEBUG
				// uint16_t sender16 = c_rx16.getRemoteAddress16();
			}
			else {
				xbee.getResponse().getRx64Response(c_rx64);
				retVal = RCV_SIXTYFOUR;
				
				// determine from who DEBUG
				uint32_t senderMsb = c_rx64.getRemoteAddress64().getMsb();
				uint32_t senderLsb = c_rx64.getRemoteAddress64().getLsb();
				if (senderLsb == c_addr64Master.getLsb() && senderMsb == c_addr64Master.getMsb()) {
					c_msgsFromMaster++; // Got the message from master
					#ifdef COM_USE_ROVER_ACKS
						if (ack) {
							xbee.send(c_tx64MasterAck);
							#ifdef COM_DEBUG_ENCODE
								Serial.println("\nRX: Sent RoverPacket ACK to master");
								delay(100);
							#endif
						}
						
					#endif
				}
				else if (senderLsb == c_addr64Slave.getLsb() && senderMsb == c_addr64Slave.getMsb()) {
					c_msgsFromSlave++; // Got the message from slave
					#ifdef COM_USE_ROVER_ACKS
						if (ack) {
							xbee.send(c_tx64SlaveAck);
							#ifdef COM_DEBUG_ENCODE
								Serial.println("\nRX: Sent RoverPacket ACK to slave");
								delay(100);
							#endif
						}
					#endif
				}
				else {
					#ifdef COM_DEBUG_XBEE
						Serial.print("RX: Untrusted source: ");
						Serial.print(senderMsb);
						Serial.println(senderLsb);
						delay(100);
					#endif
					retVal = RCV_UNTRUSTED; // Untrusted source
				}
			}
		}
		else {
			// not something we were expecting
			#ifdef COM_DEBUG_XBEE
				Serial.print("RX: Unexpected response. API ID: ");   
				Serial.println(xbee.getResponse().getApiId());
				delay(100);
			#endif
		}
	}
	else if (xbee.getResponse().isError()) {
		#ifdef COM_DEBUG_XBEE
			Serial.print("RX: Error reading packet. Error code: ");  
			Serial.println(xbee.getResponse().getErrorCode());
			delay(100);
		#endif
	}
	
	return retVal;
}

// Overloaded receiveData with a default 0s timeout.
int com_receiveData() {
	return com_receiveData(0, true);
}

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
int com_sendMaster64(bool checkAck) {
	c_msgsToMaster++; // Debug
	int retVal;
	
	if (checkAck) { 
		// Send the normal payload
		xbee.send(c_tx64Master);
		
		// Check for the ack
		#ifdef COM_USE_ROVER_ACKS
			retVal = com_getRoverAck64(); // 500ms timeout?
		#endif
		#ifndef COM_USE_ROVER_ACKS
			retVal = com_getAck(); // 500ms timeout?
		#endif
		
		if (retVal == ACK_SUCCESS)
			c_acksFromMaster++; // Debug
	}
	else {
		#ifdef COM_USE_ROVER_ACKS
			xbee.send(c_tx64Master); // recipient doesn't know we aren't checking for an ack
		#endif
		#ifndef COM_USE_ROVER_ACKS
			Tx64Request txNoACK = Tx64Request(c_addr64Master, 0x01, c_payloadMaster, sizeof(c_payloadMaster), 0x0);
			xbee.send(txNoACK);
		#endif
		
		retVal = ACK_FAILURE;
	}
	
	// Empty the payload if we arent checking for an ack or the ack was successful
	if (checkAck == false || retVal == ACK_SUCCESS)
		com_emptyPayload(false);
	
	return retVal;
}

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
int com_sendSlave64(bool checkAck) {
	c_msgsToSlave++; // Debug
	int retVal;
	
	if (checkAck) { 
		// Send the normal payload
		xbee.send(c_tx64Slave);
		
		// Check for the ack
		#ifdef COM_USE_ROVER_ACKS
			retVal = com_getRoverAck64(); // 500ms timeout?
		#endif
		#ifndef COM_USE_ROVER_ACKS
			retVal = com_getAck(); // 500ms timeout?
		#endif
		
		if (retVal == ACK_SUCCESS)
			c_acksFromSlave++; // Debug
	}
	else {
		#ifdef COM_USE_ROVER_ACKS
			xbee.send(c_tx64Slave); // recipient doesn't know we aren't checking for an ack
		#endif
		#ifndef COM_USE_ROVER_ACKS
			Tx64Request txNoACK = Tx64Request(c_addr64Slave, 0x01, c_payloadSlave, sizeof(c_payloadSlave), 0x0);
			xbee.send(txNoACK);
		#endif
		
		retVal = ACK_FAILURE;
	}
	
	// Empty the payload if we arent checking for an ack or the ack was successful
	if (checkAck == false || retVal == ACK_SUCCESS)
		com_emptyPayload(true);
	
	return retVal;
}

//----------------------------------------------------------------------
// com_unwrapAndQueue64 Parses the data in the last rx64 xbee packet as 
//					7 byte rover packets that are enqueued.
// Preconditions:   Data has been receieved already and enough memory
//					is available.
// Postconditions:  7 byte rover packets are enqueued. If a high priority
//					packet is received, true is returned. RSSI value is
// 					also updated at this step.
//----------------------------------------------------------------------
bool com_unwrapAndQueue64() {
	bool retVal = false; // bool to return if a high priority packet was received
	
	// Parse the xbee packet
	int packetSize = c_rx64.getDataLength();
	c_lastRssi = c_rx64.getRssi();
    uint8_t* data = c_rx64.getData();
    
	#ifdef COM_DEBUG_UNWRAP
		uint8_t opt = c_rx64.getOption();
		uint32_t senderMsb = c_rx64.getRemoteAddress64().getMsb();
		uint32_t senderLsb = c_rx64.getRemoteAddress64().getLsb();
		Serial.println();
		Serial.print("From: ");
		Serial.print((long)senderMsb, HEX);
		Serial.print((long)senderLsb, HEX);
		Serial.print(" RSSI: ");
		Serial.print(c_lastRssi, HEX);
		Serial.print(" opt: ");
		Serial.print(opt, HEX);
		Serial.print(" size: ");
		Serial.print(packetSize);
		Serial.print(" data:");
		for (int i = 0; i < packetSize; i++)  { // MAX_SIZE
		if(i%7==0)
				Serial.println();
			Serial.print(data[i], HEX);
			Serial.print(" ");
		}
		Serial.println();
		delay(100);
	#endif
	
    for (uint8_t i = 0; i < packetSize;)  {
		// Create a packet
		RoverPacket thePacket;
		thePacket.byte0 = data[i++]; // time 0-7
		thePacket.byte1 = data[i++]; // time 8-15
		thePacket.byte2 = data[i++]; // time 16-23
		thePacket.byte3 = data[i++]; // time 24-31
		thePacket.byte4 = data[i++]; // data(l) 32-39
		thePacket.byte5 = data[i++]; // data(l) 40-41 || data(r) 42-47
		thePacket.byte6 = data[i++]; // data(r) 48-51 || command 52-55
		
		// Check if timestamp is 0 and ignore if so
		if (thePacket.byte3 == 0 && thePacket.byte2 == 0 && thePacket.byte1 == 0 && thePacket.byte0 == 0)
			break;
		
		#ifdef COM_USE_ROVER_ACKS
			// Check if it is an unexpected ack and ignore if so (should been caught by com_getRoverAck64)
			if ((thePacket.byte6 & 0x0F) == 0x0A && thePacket.byte3 == 0xFF && thePacket.byte2 == 0xFF && thePacket.byte1 == 0xFF && thePacket.byte0 == 0xFF)
				break;
		#endif
		
		// Queue the packet
		c_packetQueue.enqueue(thePacket);
		c_queuedPackets++; // Debug
		
		#ifdef COM_DEBUG_QUEUE
			Serial.println();
			Serial.print("Size of queue is now ");
			Serial.print(c_packetQueue.count());
			Serial.println(" after an enqueue");
			Serial.println();
			delay(100);
		#endif
		
		// Check if it was a high priority packet
		int test = thePacket.byte6 << 4;
		switch (test) {
			case 0: // estop
				retVal = true;
				break;
		}
    }
	
	return retVal;
}

//----------------------------------------------------------------------
// com_decodeNext - Dequeues the next roverPacket and decodes it.
// Preconditions:   All parameters point to valid memory.
// Postconditions:  Returns true if a packet was decoded and sets the 
// 					passed in paramters to the decoded values.
//----------------------------------------------------------------------
bool com_decodeNext(unsigned long* timestamp, unsigned char* cmd, int* lData, int* rData) {
	// note that signed shorts may be more ideal - we want 16 bit data
	
	if (c_packetQueue.isEmpty())
		return false;
	
	// Dequeue the packet
	RoverPacket thePacket = c_packetQueue.dequeue();
	c_decodedPackets++; // Debug
	
	#ifdef COM_DEBUG_QUEUE
		Serial.println();
		Serial.print("Size of queue is now ");
		Serial.print(c_packetQueue.count());
		Serial.println(" after a dequeue");
		Serial.println();
		delay(100);
	#endif
	
	#ifdef COM_DEBUG_ENCODE
		Serial.println();
		Serial.println("com_decodeNext: thePacket Data:");
		Serial.print(thePacket.byte0, HEX);
		Serial.print(" ");
		Serial.print(thePacket.byte1, HEX);
		Serial.print(" ");
		Serial.print(thePacket.byte2, HEX);
		Serial.print(" ");
		Serial.print(thePacket.byte3, HEX);
		Serial.print(" | ");
		Serial.print(thePacket.byte4, HEX);
		Serial.print(" ");
		Serial.print(thePacket.byte5, HEX);
		Serial.print(" ");
		Serial.println(thePacket.byte6, HEX);
		delay(100);
	#endif
	
	// timestamp 0-7, 8-15, 16-23, 24-31
	(*timestamp) = thePacket.byte0;
	(*timestamp) <<= 8;
	(*timestamp) += thePacket.byte1;
	(*timestamp) <<= 8;
	(*timestamp) += thePacket.byte2;
	(*timestamp) <<= 8;
	(*timestamp) += thePacket.byte3;
	
	// data left 32-39, 40-41
	(*lData) = thePacket.byte4;
	(*lData) <<= 8; // set the sign bit
	(*lData) >>= 6; // repeat the sign bit
	(*lData) += thePacket.byte5 >> 6; // grab last 2 bits
	
	// data right 42-47, 48-51
	(*rData) = thePacket.byte5;
	(*rData) <<= 10; // set the sign bit and discard first two bits
	(*rData) >>= 6; // repeat the sign bit
	(*rData) += thePacket.byte6 >> 4; // grab last 4 bits
	
	// command 52-55
	(*cmd) = thePacket.byte6 & 0x0F;
	
	return true;
}

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
int com_encodeSlavePacket(unsigned char cmd, int lData, int rData) {
	// note that signed shorts may be more ideal - we want 16 bit data
	if (c_payloadEndSlave > MAX_SIZE) {
		c_failedEncodes++; // Debug
		return ENCODE_ERROR;
	}
	
	c_encodedPackets++; // Debug
	RoverPacket thePacket;
	
	// timestamp 32-bit: 0-7, 8-15, 16-23, 24-31
	unsigned long timestamp = millis();
	thePacket.byte3 = timestamp & 0x000000FF;
	timestamp >>= 8;
	thePacket.byte2 = timestamp & 0x0000FF;
	timestamp >>= 8;
	thePacket.byte1 = timestamp & 0x00FF;
	thePacket.byte0 = timestamp >> 8;
	// c_payloadSlave[c_payloadEndSlave++] = timestamp >> 24;
	// c_payloadSlave[c_payloadEndSlave++] = (timestamp >> 16) & 0xFF;
	// c_payloadSlave[c_payloadEndSlave++] = (timestamp >> 8) & 0xFF;
	// c_payloadSlave[c_payloadEndSlave++] = timestamp & 0xFF;
	
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
	
	// store the packet in payload
	c_payloadSlave[c_payloadEndSlave++] = thePacket.byte0;
	c_payloadSlave[c_payloadEndSlave++] = thePacket.byte1;
	c_payloadSlave[c_payloadEndSlave++] = thePacket.byte2;
	c_payloadSlave[c_payloadEndSlave++] = thePacket.byte3;
	c_payloadSlave[c_payloadEndSlave++] = thePacket.byte4;
	c_payloadSlave[c_payloadEndSlave++] = thePacket.byte5;
	c_payloadSlave[c_payloadEndSlave++] = thePacket.byte6;
	
	#ifdef COM_DEBUG_ENCODE
		Serial.println();
		Serial.println("Encoded cmd:" + String(cmd) + " l: " + String(lData) + " r: " + String(rData));
		Serial.print(thePacket.byte0, HEX);
		Serial.print(" ");
		Serial.print(thePacket.byte1, HEX);
		Serial.print(" ");
		Serial.print(thePacket.byte2, HEX);
		Serial.print(" ");
		Serial.print(thePacket.byte3, HEX);
		Serial.print(" | ");
		Serial.print(thePacket.byte4, HEX);
		Serial.print(" ");
		Serial.print(thePacket.byte5, HEX);
		Serial.print(" ");
		Serial.println(thePacket.byte6, HEX);
		Serial.println("(MAX_SIZE - c_payloadEndSlave + 1) / MIN_SIZE = " + String((MAX_SIZE - c_payloadEndSlave + 1) / MIN_SIZE));
		delay(100);
	#endif
	
	return com_getSlaveSlotsLeft();
}

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
int com_encodeMasterPacket(unsigned char cmd, int lData, int rData) {
	// note that signed shorts may be more ideal - we want 16 bit data
	if (c_payloadEndMaster > MASTER_SIZE) {
		c_failedEncodes++; // Debug
		return ENCODE_ERROR;
	}
	
	c_encodedPackets++; // Debug
	RoverPacket thePacket;
	
	// timestamp 32-bit: 0-7, 8-15, 16-23, 24-31
	unsigned long timestamp = millis();
	thePacket.byte3 = timestamp & 0x000000FF;
	timestamp >>= 8;
	thePacket.byte2 = timestamp & 0x0000FF;
	timestamp >>= 8;
	thePacket.byte1 = timestamp & 0x00FF;
	thePacket.byte0 = timestamp >> 8;
	// c_payloadMaster[c_payloadEndMaster++] = timestamp >> 24;
	// c_payloadMaster[c_payloadEndMaster++] = (timestamp >> 16) & 0xFF;
	// c_payloadMaster[c_payloadEndMaster++] = (timestamp >> 8) & 0xFF;
	// c_payloadMaster[c_payloadEndMaster++] = timestamp & 0xFF;
	
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
	
	// store the packet in payload
	c_payloadMaster[c_payloadEndMaster++] = thePacket.byte0;
	c_payloadMaster[c_payloadEndMaster++] = thePacket.byte1;
	c_payloadMaster[c_payloadEndMaster++] = thePacket.byte2;
	c_payloadMaster[c_payloadEndMaster++] = thePacket.byte3;
	c_payloadMaster[c_payloadEndMaster++] = thePacket.byte4;
	c_payloadMaster[c_payloadEndMaster++] = thePacket.byte5;
	c_payloadMaster[c_payloadEndMaster++] = thePacket.byte6;
	
	#ifdef COM_DEBUG_ENCODE
		Serial.println();
		Serial.println("Encoded cmd:" + String(cmd) + " l: " + String(lData) + " r: " + String(rData));
		Serial.print(thePacket.byte0, HEX);
		Serial.print(" ");
		Serial.print(thePacket.byte1, HEX);
		Serial.print(" ");
		Serial.print(thePacket.byte2, HEX);
		Serial.print(" ");
		Serial.print(thePacket.byte3, HEX);
		Serial.print(" | ");
		Serial.print(thePacket.byte4, HEX);
		Serial.print(" ");
		Serial.print(thePacket.byte5, HEX);
		Serial.print(" ");
		Serial.println(thePacket.byte6, HEX);
		Serial.println("MASTER_SIZE - c_payloadEndMaster + 1) / MIN_SIZE = " + String((MASTER_SIZE - c_payloadEndMaster + 1) / MIN_SIZE));
		delay(100);
	#endif
	
	return com_getMasterSlotsLeft();
}

//----------------------------------------------------------------------
// com_emptyPayload Empties the payload for master or the slave.
// Preconditions:   None.
// Postconditions:  Payload is emptied and payloadEnd index is 0.
//----------------------------------------------------------------------
void com_emptyPayload(bool slave) {
	if (slave) {
		// Empty the slave payload
		for (; c_payloadEndSlave > 0; c_payloadEndSlave--)
			c_payloadSlave[c_payloadEndSlave - 1] = 0;
	}
	else {
		// Empty the master payload
		for (; c_payloadEndMaster > 0; c_payloadEndMaster--)
			c_payloadMaster[c_payloadEndMaster - 1] = 0;
	}
}

//----------------------------------------------------------------------
// com_getLastRssi  Getter for lastRssi. Higher magnitude is worse.
// Preconditions:   None.
// Postconditions:  Returns an uint8_t.
//----------------------------------------------------------------------
uint8_t com_getLastRssi() {
	return c_lastRssi;
}

//----------------------------------------------------------------------
// com_getMsgsToMaster  Getter for msgsToMaster.
// Preconditions:   None.
// Postconditions:  Returns an unsigned int.
//----------------------------------------------------------------------
unsigned int com_getMsgsToMaster() {
	return c_msgsToMaster;
}

//----------------------------------------------------------------------
// com_getMsgsToSlave  Getter for msgsToSlave.
// Preconditions:   None.
// Postconditions:  Returns an unsigned int.
//----------------------------------------------------------------------
unsigned int com_getMsgsToSlave() {
	return c_msgsToSlave;
}

//----------------------------------------------------------------------
// com_getMsgsToMaster  Getter for msgsFromMaster.
// Preconditions:   None.
// Postconditions:  Returns an unsigned int.
//----------------------------------------------------------------------
unsigned int com_getMsgsFromMaster() {
	return c_msgsFromMaster;
}

//----------------------------------------------------------------------
// com_getMsgsToSlave  Getter for msgsFromSlave.
// Preconditions:   None.
// Postconditions:  Returns an unsigned int.
//----------------------------------------------------------------------
unsigned int com_getMsgsFromSlave() {
	return c_msgsFromSlave;
}

//----------------------------------------------------------------------
// com_getAcksFromMaster  Getter for acksFromMaster.
// Preconditions:   None.
// Postconditions:  Returns an unsigned int.
//----------------------------------------------------------------------
unsigned int com_getAcksFromMaster() {
	return c_acksFromMaster;
}

//----------------------------------------------------------------------
// com_getAcksFromSlave  Getter for acksFromSlave.
// Preconditions:   None.
// Postconditions:  Returns an unsigned int.
//----------------------------------------------------------------------
unsigned int com_getAcksFromSlave() {
	return c_acksFromSlave;
}

//----------------------------------------------------------------------
// com_getEncodedPackets  Getter for encodedPackets.
// Preconditions:   None.
// Postconditions:  Returns an unsigned int.
//----------------------------------------------------------------------
unsigned int com_getEncodedPackets() {
	return c_encodedPackets;
}

//----------------------------------------------------------------------
// com_getDecodedPackets  Getter for decodedPackets.
// Preconditions:   None.
// Postconditions:  Returns an unsigned int.
//----------------------------------------------------------------------
unsigned int com_getDecodedPackets() {
	return c_decodedPackets;
}

//----------------------------------------------------------------------
// com_getFailedEncodes  Getter for failedEncodes.
// Preconditions:   None.
// Postconditions:  Returns an unsigned int.
//----------------------------------------------------------------------
unsigned int com_getFailedEncodes() {
	return c_failedEncodes;
}

//----------------------------------------------------------------------
// com_getQueuedPackets  Getter for queuedPackets.
// Preconditions:   None.
// Postconditions:  Returns an unsigned int.
//----------------------------------------------------------------------
unsigned int com_getQueuedPackets() {
	return c_queuedPackets;
}

//----------------------------------------------------------------------
// com_getQueuedPackets  Getter for currently queuedPackets.
// Preconditions:   None.
// Postconditions:  Returns an int.
//----------------------------------------------------------------------
int com_getCurrentlyQueuedPackets() {
	return c_packetQueue.count();
}

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
int com_sendStatistics64(bool checkAck) {
	int retVal = 0;
	int temp;
	
	#ifdef COM_DEBUG_STATS
		Serial.println("Stats BEFORE");
		Serial.println("lastRssi: -" + String(c_lastRssi) + "db");
		Serial.println("c_failedEncodes: " + String(c_failedEncodes) + " c_queuedPackets: " + String(c_queuedPackets));
		Serial.println("c_msgsToMaster: " + String(c_msgsToMaster) + " c_msgsFromMaster: " + String(c_msgsFromMaster));
		Serial.println("c_msgsToSlave: " + String(c_msgsToSlave) + " c_msgsFromSlave: " + String(c_msgsFromSlave));
		Serial.println("c_acksFromMaster: " + String(c_acksFromMaster) + " c_acksFromSlave: " + String(c_acksFromSlave));
		Serial.println("c_encodedPackets: " + String(c_encodedPackets) + " c_decodedPackets: " + String(c_decodedPackets));
		delay(100);
	#endif
	
	// Failed Encodes/Packets Queued
	temp = com_encodeMasterPacket(0xF, c_failedEncodes, c_queuedPackets);
	if (temp <= 0)
		retVal += com_sendMaster64(checkAck);
	
	// Packets Encoded/Decoded
	temp = com_encodeMasterPacket(0xB, c_encodedPackets, c_decodedPackets);
	if (temp <= 0)
		retVal += com_sendMaster64(checkAck);
	
	// Msgs to Master/Acks from Master
	temp = com_encodeMasterPacket(0xE, c_msgsToMaster, c_acksFromMaster);
	if (temp <= 0)
		retVal += com_sendMaster64(checkAck);
	
	// Msgs to Slave/Acks from Slave
	temp = com_encodeMasterPacket(0xD, c_msgsToSlave, c_acksFromSlave);
	if (temp <= 0)
		retVal += com_sendMaster64(checkAck);
	
	// Msgs from Master/Slave
	temp = com_encodeMasterPacket(0xC, c_msgsFromMaster, c_msgsFromSlave);
	retVal += com_sendMaster64(checkAck);	
	
	#ifdef COM_DEBUG_STATS
		Serial.println();
		Serial.println("AFTER lastRssi: -" + String(c_lastRssi) + "db c_queuedPackets: " + String(c_queuedPackets));
		Serial.println("c_msgsToMaster: " + String(c_msgsToMaster) + " c_msgsFromMaster: " + String(c_msgsFromMaster));
		Serial.println("c_msgsToSlave: " + String(c_msgsToSlave) + " c_msgsFromSlave: " + String(c_msgsFromSlave));
		Serial.println("c_acksFromMaster: " + String(c_acksFromMaster) + " c_acksFromSlave: " + String(c_acksFromSlave));
		Serial.println("c_encodedPackets: " + String(c_encodedPackets) + " c_decodedPackets: " + String(c_decodedPackets));
		delay(100);
	#endif
	
	return retVal;
}

//----------------------------------------------------------------------
// com_resetStatistics Resets the debugging network statistics back to 0.
// Preconditions:   None.
// Postconditions:  msgsToMaster, msgsToSlave, msgsFromMaster, 
//					msgsFromSlave, acksFromMaster, acksFromSlave, 
// 					encodedPackets, decodedPackets, queuedPackets, and 
//					failedEncodes are all reset to 0.
//----------------------------------------------------------------------
void com_resetStatistics() {
	c_msgsToMaster = 0;			// count of msgs sent to master
	c_msgsFromMaster = 0;		// count of msgs received from master
	c_acksFromMaster = 0;		// count of acks from master
	c_msgsToSlave = 0;			// count of msgs sent to slave
	c_msgsFromSlave = 0;		// count of msgs received from slave
	c_acksFromSlave = 0;		// count of acks from slave
	c_encodedPackets = 0;		// count of roverPackets encoded
	c_decodedPackets = 0;		// count of roverPackets decoded
	c_queuedPackets = 0;		// count of roverPackets unwrapped and queued
	c_failedEncodes = 0;		// count of failed attempts to encode a packet because buffer was full
}

//----------------------------------------------------------------------
// com_emptyQueue Empties the packetQueue
// Preconditions:   None.
// Postconditions:  packetQueue is emptied.
//----------------------------------------------------------------------
void com_emptyQueue() {
	while (!c_packetQueue.isEmpty())
		c_packetQueue.dequeue();
	
	#ifdef COM_DEBUG_QUEUE
		Serial.println();
		Serial.print("Size of queue is now ");
		Serial.print(c_packetQueue.count());
		Serial.println(" after an empty");
		Serial.println();
		delay(100);
	#endif
}

//----------------------------------------------------------------------
// com_getMasterSlotsLeft Getter for the space remaining in the master
//					payload.
// Preconditions:   None.
// Postconditions:  Returns an integer indicating how many more packets
//					can be loaded into the payload before it is full.
//----------------------------------------------------------------------
int com_getMasterSlotsLeft() {
	return (MASTER_SIZE - c_payloadEndMaster + 1) / MIN_SIZE;
}

//----------------------------------------------------------------------
// com_getSlaveSlotsLeft Getter for the space remaining in the slave
//					payload.
// Preconditions:   None.
// Postconditions:  Returns an integer indicating how many more packets
//					can be loaded into the payload before it is full.
//----------------------------------------------------------------------
int com_getSlaveSlotsLeft() {
	return (MAX_SIZE - c_payloadEndSlave + 1) / MIN_SIZE;
}

//----------------------------------------------------------------------
// com_geMaxtMasterSlots Getter for the maximum space in the master
//					payload.
// Preconditions:   None.
// Postconditions:  Returns an integer indicating the maximum number 
//					of packets that can be loaded into the payload.
//----------------------------------------------------------------------
int com_geMaxtMasterSlots() {
	return MASTER_SIZE / MIN_SIZE;
}

//----------------------------------------------------------------------
// com_getMaxSlaveSlots Getter for the maximum space in the slave
//					payload.
// Preconditions:   None.
// Postconditions:  Returns an integer indicating the maximum number 
//					of packets that can be loaded into the payload.
//----------------------------------------------------------------------
int com_getMaxSlaveSlots() {
	return MAX_SIZE / MIN_SIZE;
}
