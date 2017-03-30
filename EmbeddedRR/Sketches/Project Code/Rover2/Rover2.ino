//----------------------------- Rover2.ino ----------------------------
// Filename:      Rover2.ino
// Project Team:  EmbeddedRR
// Group Members: Robert Griswold and Ryu Muthui
// Date:          2 Dec 2016
// Description:   Combination of lights, movement, and communication
//                code for Rover 2.
//------------------------------ Includes  ----------------------------
#include <Rover_Communication.h>
#include <Rover_Lights.h>
#include <Rover_Movement.h>
#include <Rover_Sensors.h>

//-------------------------- Configuration  ---------------------------
// communication
#define MSTR_ADDR_SH 0x0013A200
#define MSTR_ADDR_SL 0x40F9CEDC
#define R1_ADDR_SH 0x0013A200
#define R1_ADDR_SL 0x40F9CEDE
// #define MSTR_ADDR 0x4321        // CEDC 16 bit addr
// #define R1_ADDR 0x1234          // CEDE 16 bit addr

// debug
// #define RVR_DEBUG

//------------------------------ Globals  -----------------------------
#define STATE_STOP 0
#define STATE_STRAIGHT 1
#define STATE_LEFT 2
#define STATE_RIGHT 3
// No STATE_LOST 4
#define STATE_READY 5
#define STATE_MANUAL 6

int currentState = STATE_STOP;
long masterOffset = 0;

// Navigation Packet (8 bytes):
struct NavigationPacket {
  unsigned long timestamp = 0;
  int leftPower = 0; // wastes 6 bits
  int rightPower = 0; // wastes 6 bits
};

QueueArray<NavigationPacket> navigationQueue;

//------------------------------ Setup  -------------------------------
void setup() {
  // Set up Serial library at 9600 bps
  Serial.begin(9600);
  
  #ifdef RVR_DEBUG
    navigationQueue.setPrinter(Serial);
  #endif
  
  move_setupMotors();
  light_setupLights();
  com_setupComs(MSTR_ADDR_SH, MSTR_ADDR_SL, R1_ADDR_SH, R1_ADDR_SL);
  light_lightRed();
}

//----------------------------------------------------------------------
//------------------------------ Main Loop -----------------------------
//----------------------------------------------------------------------
void loop() {
  updateState();
  move_updateMotors();
}

//----------------------------------------------------------------------
//----------------------------- State Functions ------------------------
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// updateState() -- Updates the state based on xbee communication.
//----------------------------------------------------------------------
void updateState() {
  int rcv = 0;
  unsigned long timestamp = 0;
  unsigned char cmd = 0;
  int lData = 0;
  int rData = 0;
  NavigationPacket thePacket;

  #ifdef RVR_DEBUG
    if (!navigationQueue.isEmpty()) {
      thePacket = navigationQueue.peek();
      unsigned long now = millis();
      Serial.print("Packet time: " + String(thePacket.timestamp) + " millis: " +
          String(now) + " offset: " + String(masterOffset));
      now = now + masterOffset - thePacket.timestamp;
      Serial.println(" millis() + masterOffset - thePacket.timestamp = " + String((long)now));
    }
  #endif

  switch(currentState) {
    //------------------------------------------------------------------
    // SEARCH
    //------------------------------------------------------------------
    case STATE_READY: // received navigation data, waiting to start
      // receive any new xbee data
      rcv = com_receiveData();
      if (rcv == RCV_SIXTYFOUR) {
        // got data, unwrap and queue it
        light_lightBlue();
        bool eStop = com_unwrapAndQueue64();
        if (eStop) {
          emergencyStop(true); // sends stats
          return;
        }
        
        // and since we are waiting, decode it all
        while (com_decodeNext(&timestamp, &cmd, &lData, &rData) == true) { // keep decoding

          #ifdef RVR_DEBUG
            Serial.println();
            Serial.print("cmd:"); // debug
            Serial.print(cmd, HEX);
            Serial.print(" lData:");
            Serial.print(lData, HEX);
            Serial.print(" rData:");
            Serial.print(rData, HEX);
            Serial.print(" time:");
            Serial.print(timestamp, HEX);
            Serial.print(" navQueue:");
            Serial.println(navigationQueue.count());
          #endif
          
          switch (cmd) {
            /*case 0x0: // Emergency Stop
              emergencyStop(true); // sends stats
              break;*/
  
            case 0xA: // Navigation Data
              thePacket.timestamp = timestamp;
              thePacket.leftPower = lData;
              thePacket.rightPower = rData;
              navigationQueue.enqueue(thePacket);
              light_lightYellow();
              break;
              
            case 0x6: // Start Follow
              thePacket = navigationQueue.dequeue();
              masterOffset = thePacket.timestamp - millis(); // syncs the clocks
              executeNav(thePacket.leftPower, thePacket.rightPower);
              break;
          }

          // clear temp data for next dequeue
          timestamp = 0;
          cmd = 0;
          lData = 0;
          rData = 0;
        }
      }
      break;
      
    //------------------------------------------------------------------
    // STRAIGHT
    //------------------------------------------------------------------
    case STATE_STRAIGHT: // moving forward
      // check the navigation queue
      if (!navigationQueue.isEmpty()) {
        thePacket = navigationQueue.peek();
        long comparison = millis() + masterOffset - thePacket.timestamp;
        if (comparison >= 0) { // is the next target now (or in the past)?
          navigationQueue.dequeue(); // remove it from queue
          executeNav(thePacket.leftPower, thePacket.rightPower);
        }
      }
      else { // no navigation data to work with
        emergencyStop(true); // sends stats
      }

      // receive any new xbee data
      rcv = com_receiveData();
      if (rcv == RCV_SIXTYFOUR) {
        // got data, unwrap and queue it
        light_lightBlue();
        bool eStop = com_unwrapAndQueue64();
        if (eStop) {
          emergencyStop(true); // sends stats
          return;
        }
      }
        
      // decode the next command
      if (com_decodeNext(&timestamp, &cmd, &lData, &rData) == true) {

        #ifdef RVR_DEBUG
          Serial.println();
          Serial.print("cmd:"); // debug
          Serial.print(cmd, HEX);
          Serial.print(" lData:");
          Serial.print(lData, HEX);
          Serial.print(" rData:");
          Serial.print(rData, HEX);
          Serial.print(" time:");
          Serial.print(timestamp, HEX);
          Serial.print(" navQueue:");
          Serial.println(navigationQueue.count());
        #endif
        
        switch (cmd) {
          /*case 0x0: // Emergency Stop
            emergencyStop(true); // sends stats
            break;*/
            
          case 0xA: // Navigation Data
            thePacket.timestamp = timestamp;
            thePacket.leftPower = lData;
            thePacket.rightPower = rData;
            navigationQueue.enqueue(thePacket);
            light_lightGreen();
            break;
        }
      }
      break;
      
    //------------------------------------------------------------------
    // LEFT
    //------------------------------------------------------------------
    case STATE_LEFT: // turning left
      // check the navigation queue
      if (!navigationQueue.isEmpty()) {
        thePacket = navigationQueue.peek();
        long comparison = millis() + masterOffset - thePacket.timestamp;
        if (comparison >= 0) { // is the next target now (or in the past)?
          navigationQueue.dequeue(); // remove it from queue
          executeNav(thePacket.leftPower, thePacket.rightPower);
        }
      }
      else { // no navigation data to work with
        emergencyStop(true); // sends stats
      }

      // receive any new xbee data
      rcv = com_receiveData();
      if (rcv == RCV_SIXTYFOUR) {
        // got data, unwrap and queue it
        light_lightBlue();
        bool eStop = com_unwrapAndQueue64();
        if (eStop) {
          emergencyStop(true); // sends stats
          return;
        }
      }
        
      // decode the next command
      if (com_decodeNext(&timestamp, &cmd, &lData, &rData) == true) {

        #ifdef RVR_DEBUG
          Serial.println();
          Serial.print("cmd:"); // debug
          Serial.print(cmd, HEX);
          Serial.print(" lData:");
          Serial.print(lData, HEX);
          Serial.print(" rData:");
          Serial.print(rData, HEX);
          Serial.print(" time:");
          Serial.print(timestamp, HEX);
          Serial.print(" navQueue:");
          Serial.println(navigationQueue.count());
        #endif
        
        switch (cmd) {
          /*case 0x0: // Emergency Stop
            emergencyStop(true); // sends stats
            break;*/

          case 0xA: // Navigation Data
            thePacket.timestamp = timestamp;
            thePacket.leftPower = lData;
            thePacket.rightPower = rData;
            navigationQueue.enqueue(thePacket);
            light_turnLeft();
            break;
        }
      }
      break;
      
    //------------------------------------------------------------------
    // RIGHT
    //------------------------------------------------------------------
    case STATE_RIGHT: // turning right
      // check the navigation queue
      if (!navigationQueue.isEmpty()) {
        thePacket = navigationQueue.peek();
        long comparison = millis() + masterOffset - thePacket.timestamp;
        if (comparison >= 0) { // is the next target now (or in the past)?
          navigationQueue.dequeue(); // remove it from queue
          executeNav(thePacket.leftPower, thePacket.rightPower);
        }
      }
      else { // no navigation data to work with
        emergencyStop(true); // sends stats
      }

      // receive any new xbee data
      rcv = com_receiveData();
      if (rcv == RCV_SIXTYFOUR) {
        // got data, unwrap and queue it
        light_lightBlue();
        bool eStop = com_unwrapAndQueue64();
        if (eStop) {
          emergencyStop(true); // sends stats
          return;
        }
      }
        
      // decode the next command
      if (com_decodeNext(&timestamp, &cmd, &lData, &rData) == true) {

        #ifdef RVR_DEBUG
          Serial.println();
          Serial.print("cmd:"); // debug
          Serial.print(cmd, HEX);
          Serial.print(" lData:");
          Serial.print(lData, HEX);
          Serial.print(" rData:");
          Serial.print(rData, HEX);
          Serial.print(" time:");
          Serial.print(timestamp, HEX);
          Serial.print(" navQueue:");
          Serial.println(navigationQueue.count());
        #endif
        
        switch (cmd) {
          /*case 0x0: // Emergency Stop
            emergencyStop(true); // sends stats
            break;*/

          case 0xA: // Navigation Data
            thePacket.timestamp = timestamp;
            thePacket.leftPower = lData;
            thePacket.rightPower = rData;
            navigationQueue.enqueue(thePacket);
            light_turnRight();
            break;
        }
      }
      break;

    //------------------------------------------------------------------
    // STOP
    //------------------------------------------------------------------
    case STATE_STOP: // waiting for new commands
      // receive any new xbee data
      rcv = com_receiveData();
      if (rcv == RCV_SIXTYFOUR) {
        // got data, unwrap and queue it
        light_lightBlue();
        bool eStop = com_unwrapAndQueue64();
        if (eStop) {
          emergencyStop(false); // no stats
          return;
        }
        
        // and since we are waiting, decode it all
        while (com_decodeNext(&timestamp, &cmd, &lData, &rData) == true) { // keep decoding

          #ifdef RVR_DEBUG
            Serial.println();
            Serial.print("cmd:"); // debug
            Serial.print(cmd, HEX);
            Serial.print(" lData:");
            Serial.print(lData, HEX);
            Serial.print(" rData:");
            Serial.print(rData, HEX);
            Serial.print(" time:");
            Serial.print(timestamp, HEX);
            Serial.print(" navQueue:");
            Serial.println(navigationQueue.count());
          #endif
          
          switch (cmd) {
            /*case 0x0: // Emergency Stop
              emergencyStop(false); // no stats
              break;*/
  
            case 0xA: // Navigation Data
              enterReadyState();
              thePacket.timestamp = timestamp;
              thePacket.leftPower = lData;
              thePacket.rightPower = rData;
              navigationQueue.enqueue(thePacket);
              break;
              
            case 0x2: // Forward
              enterManualState();
              move_moveForward(false); // target adjustment of speed (non-blocking)
              break;
  
            case 0x3: // Backward
              enterManualState();
              move_moveReverse(false); // target adjustment of speed (non-blocking)
              break;
  
            case 0x4: // Turn left 90
              enterManualState();
              move_rotateLeft90();
              break;
  
            case 0x5: // Turn right 90
              enterManualState();
              move_rotateRight90();
              break;
          }

          // clear temp data for next dequeue
          timestamp = 0;
          cmd = 0;
          lData = 0;
          rData = 0;
        }
      }
      break;

    //------------------------------------------------------------------
    // MANUAL
    //------------------------------------------------------------------
    case STATE_MANUAL: // waiting for new commands
      // receive any new xbee data
      rcv = com_receiveData();
      if (rcv == RCV_SIXTYFOUR) {
        // got data, unwrap and queue it
        light_lightBlue();
        bool eStop = com_unwrapAndQueue64();
        if (eStop) {
          emergencyStop(false); // no stats
          return;
        }
        
        // and since we are waiting, decode it all
        while (com_decodeNext(&timestamp, &cmd, &lData, &rData) == true) { // keep decoding

          #ifdef RVR_DEBUG
            Serial.println();
            Serial.print("cmd:"); // debug
            Serial.print(cmd, HEX);
            Serial.print(" lData:");
            Serial.print(lData, HEX);
            Serial.print(" rData:");
            Serial.print(rData, HEX);
            Serial.print(" time:");
            Serial.print(timestamp, HEX);
            Serial.print(" navQueue:");
            Serial.println(navigationQueue.count());
          #endif
          
          switch (cmd) {
            /*case 0x0: // Emergency Stop
              emergencyStop(false); // no stats
              break;*/
  
            case 0x1: // Slow Stop
              enterStopState(false); // no stats
              break;
              
            case 0x2: // Forward
              move_moveForward(false); // target adjustment of speed (non-blocking)
              break;
  
            case 0x3: // Backward
              move_moveReverse(false); // target adjustment of speed (non-blocking)
              break;
  
            case 0x4: // Turn left 90
              move_rotateLeft90();
              break;
  
            case 0x5: // Turn right 90
              move_rotateRight90();
              break;
  
            case 0xA: // Navigation Data
              enterReadyState();
              thePacket.timestamp = timestamp;
              thePacket.leftPower = lData;
              thePacket.rightPower = rData;
              navigationQueue.enqueue(thePacket);
              break;
          }
          
          // clear temp data for next dequeue
          timestamp = 0;
          cmd = 0;
          lData = 0;
          rData = 0;
        }

        if (currentState == STATE_MANUAL)
          light_lightWhite();
      }
      break;
  }
}

//----------------------------------------------------------------------
// executeNav() -- Determines the direction based on motor speeds, sets
// the motor speeds, and enters the appropriate state.
//----------------------------------------------------------------------
void executeNav(int leftPower, int rightPower) {
  move_setTarget(leftPower, rightPower);
  if (leftPower == rightPower) {
    if (leftPower == 0)
      enterStopState(true); // sends stats
    else
      enterStraightState();
  }
  else if (leftPower > rightPower) {
    enterRightState();
  }
  else {
    enterLeftState();
  }

  #ifdef RVR_DEBUG
    Serial.println("State after execNav: " + String(currentState));
    Serial.println("lPower: " + String(move_getTargetLeft()) + " rPower: " +
        String(move_getTargetRight()));
  #endif
}

//----------------------------------------------------------------------
// enterManualState() -- Enters STATE_MANUAL
//----------------------------------------------------------------------
void enterManualState() {
  currentState = STATE_MANUAL;
  light_lightWhite();
}

//----------------------------------------------------------------------
// enterReadyState() -- Enters STATE_READY
//----------------------------------------------------------------------
void enterReadyState() {
  currentState = STATE_READY;
  light_lightYellow();
}

//----------------------------------------------------------------------
// enterStraightState() -- Enters STATE_STRAIGHT
//----------------------------------------------------------------------
void enterStraightState() {
  currentState = STATE_STRAIGHT;
  light_lightGreen();
}

//----------------------------------------------------------------------
// enterLeftState() -- Enters STATE_LEFT
//----------------------------------------------------------------------
void enterLeftState() {
  currentState = STATE_LEFT;
  light_turnLeft();
}

//----------------------------------------------------------------------
// enterRightState() -- Enters STATE_RIGHT
//----------------------------------------------------------------------
void enterRightState() {
  currentState = STATE_RIGHT;
  light_turnRight();
}

//----------------------------------------------------------------------
// enterStopState() -- Enters STATE_STOP, clears payloads, and optionally
// sends statistics.
//----------------------------------------------------------------------
void enterStopState(bool stats) {
  // change states
  currentState = STATE_STOP;
  light_lightRed();
  move_setTarget(0, 0);

  // clear payloads
  com_emptyPayload(true); // slave payload
  com_emptyPayload(false); // master payload

  // empty the packetQueue
  com_emptyQueue();

  // empty navigation queue
  while (!navigationQueue.isEmpty())
    navigationQueue.dequeue();

  // send stats to master
  if (stats)
    com_sendStatistics64(true); // stats with ack(s) to master - no retry
}

//----------------------------------------------------------------------
// emergencyStop() -- Stops immediately, clears payloads, sends estop 
// command to other rover if not already stopped, enters STATE_STOP, 
// and optionally sends statistics.
//----------------------------------------------------------------------
void emergencyStop(bool stats) {
  // stop
  move_fullStop();
  light_lightPurple();

  // clear payloads 
  com_emptyPayload(true); // slave payload
  com_emptyPayload(false); // master payload

  // empty navigation queue
  while (!navigationQueue.isEmpty())
    navigationQueue.dequeue();
  
  // estop other rover if we are not already stopped
  if (currentState != STATE_STOP) {
    com_encodeSlavePacket(0, 0, 0);
    com_sendSlave64(true); // send payload to slave and request ack - no rety
  }

  // empty the packetQueue
  com_emptyQueue();

  // finalize state change and send statistics to master
  currentState = STATE_STOP;
  if (stats) {
    com_sendStatistics64(true); // stats with ack(s) to master - no rety
    com_resetStatistics();
  }
  delay(100);
  light_lightRed();
}
