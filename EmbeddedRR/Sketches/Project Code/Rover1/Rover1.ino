//----------------------------- Rover1.ino ----------------------------
// Filename:      Rover1.ino
// Project Team:  EmbeddedRR
// Group Members: Robert Griswold and Ryu Muthui
// Date:          2 Dec 2016
// Description:   Combination of sensor, lights, communication, and 
//                movement code for Rover 1.
//------------------------------ Includes  ----------------------------
#include <Rover_Communication.h>
#include <Rover_Lights.h>
#include <Rover_Movement.h>
#include <Rover_Sensors.h>

//-------------------------- Configuration  ---------------------------
// traversal
#define TURN_POWER -30          // Turning power of motor
#define STRAIGHT_POWER 30       // How fast to move
#define THRESHOLD 100           // The threshold difference between sensors
#define GIVE_UP_LIMIT 2000      // Time in miliseconds before we give up
#define STRAIGHT_TIME_MAX 5000  // Time in miliseconds to transmit a continous straight

// communication
#define MSTR_ADDR_SH 0x0013A200
#define MSTR_ADDR_SL 0x40F9CEDC
#define R2_ADDR_SH 0x0013A200
#define R2_ADDR_SL 0x4103DA0F
// #define MSTR_ADDR 0x4321        // CEDC 16 bit addr
// #define R2_ADDR 0x1243          // DA0F 16 bit addr

// debug
// #define RVR_DEBUG_SENSORS
// #define RVR_DEBUG_CMDS

//----------------------------- Globals  ------------------------------
#define STATE_STOP 0
#define STATE_STRAIGHT 1
#define STATE_LEFT 2
#define STATE_RIGHT 3
#define STATE_LOST 4
#define STATE_SEARCH 5
#define STATE_MANUAL 6

int currentState = STATE_STOP;
bool lastDirectionRight = true; // used for lost state direction
unsigned long giveUpStart = 0;
unsigned long straightTimeStart = 0;
int curSensorVals[4] = {0, 0, 0, 0};
int outlierThreshold = 0;
int halfSlavePayload;
int lastAck = ACK_SUCCESS;
bool retransmitToggle = false; // retransmit every other loop

//------------------------------ Setup  -------------------------------
void setup() {
  // Set up Serial library at 9600 bps
  Serial.begin(9600);
  
  halfSlavePayload = com_getMaxSlaveSlots() / 2;
  
  move_setupMotors();
  light_setupLights();
  com_setupComs(MSTR_ADDR_SH, MSTR_ADDR_SL, R2_ADDR_SH, R2_ADDR_SL);
  sensor_setup();
  
  light_lightRed();
  
  // delay(3000);
  // enterSearchState();
}

//----------------------------------------------------------------------
//------------------------------ Main Loop -----------------------------
//----------------------------------------------------------------------
void loop() {
  sensor_getSensorVals(curSensorVals);
  updateState();
  move_updateMotors();
}

//----------------------------------------------------------------------
//----------------------------- State Functions ------------------------
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// updateState() -- Updates the state based on sensor data in curSensorVals
//                  and xbee communication.
//----------------------------------------------------------------------
void updateState( ) {
  // sensor data (outer - inner)
  int leftDiff = curSensorVals[0] - curSensorVals[1];
  int rightDiff = curSensorVals[3] - curSensorVals[2];

  int rcv = 0;
  
  #ifdef RVR_DEBUG_SENSORS
    Serial.print(String(curSensorVals[0]) + ", " + String(curSensorVals[1]) + ", " +
        String(curSensorVals[2]) + ", " + String(curSensorVals[3]));
    Serial.print(" -> LDiff: " + String(leftDiff) + " RDiff: " + String(rightDiff));
    Serial.println( " (THRESHOLD: " + String(THRESHOLD) + " outlierThreshold: " + String(outlierThreshold) + " )");
    delay(500);
  #endif

  if (curSensorVals[0] > 3000 && curSensorVals[1] > 3000 && curSensorVals[2] > 3000 &&
      curSensorVals[3] > 3000 && currentState != STATE_STOP) { // picked up
    emergencyStop(true); // sends stats
    return;
  }

  // retransmit payload if last ack failed, there hasn't been a dropped packet, and payload is atleast half full
  if (lastAck != ACK_SUCCESS && com_getFailedEncodes() == 0 && com_getSlaveSlotsLeft() <= halfSlavePayload) { 
    retransmitToggle = !retransmitToggle;
    if (retransmitToggle) // retransmit every other
      lastAck = com_sendSlave64(true); // send payload to slave and request ack
  }
  
  switch(currentState) {
    //------------------------------------------------------------------
    // SEARCH
    //------------------------------------------------------------------
    case STATE_SEARCH: // from slow start, find the line
      if (leftDiff > THRESHOLD) { // if left diff is greater, steer left
        outlierThreshold = abs(leftDiff) + THRESHOLD;
        enterLeftState();
      }
      else if (rightDiff > THRESHOLD) { // if right diff is greater, steer right
        outlierThreshold = abs(rightDiff) + THRESHOLD;
        enterRightState();
      }
      else if (abs(leftDiff) > THRESHOLD || abs(rightDiff) > THRESHOLD) { // continue straight
        if (abs(leftDiff) > abs(rightDiff))
          outlierThreshold = abs(leftDiff) + THRESHOLD;
        else
          outlierThreshold = abs(rightDiff) + THRESHOLD;
        enterStraightState();
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
        
        // decode it all?
        unsigned long timestamp = 0;
        unsigned char cmd = 0;
        int lData = 0;
        int rData = 0;
        while (com_decodeNext(&timestamp, &cmd, &lData, &rData) == true) { // keep decoding

          #ifdef RVR_DEBUG_CMDS
            Serial.println();
            Serial.print("cmd:"); // debug
            Serial.print(cmd, HEX);
            Serial.print(" lData:");
            Serial.print(lData, HEX);
            Serial.print(" rData:");
            Serial.print(rData, HEX);
            Serial.print(" time:");
            Serial.println(timestamp, HEX);
          #endif
          
          switch (cmd) {
            /*case 0x0: // Emergency Stop
              emergencyStop(true); // sends stats
              break;*/
              
            case 0x1: // Slow Stop
              enterStopState(true); // sends stats
              break;

            case 0x7: // Sensor request
              com_encodeMasterPacket(0x7, leftDiff, rightDiff);
              float x, y, z;
              sensor_getMagData(x, y, z);
              com_encodeMasterPacket(0x8, (int)x, int(z));
              com_sendMaster64(true); // send payload to master and request ack - no retry
              light_lightYellow();
              break;
          }

          // clear temp data for next dequeue
          timestamp = 0;
          cmd = 0;
          lData = 0;
          rData = 0;
        }
      }
      
      // Update rover 2 with a continous straight after STRAIGHT_TIME_MAX time
      if (millis() > straightTimeStart + STRAIGHT_TIME_MAX) {
        straightTimeStart = millis();
        com_encodeSlavePacket(0xA, move_getTargetLeft(), move_getTargetRight());
        lastAck = com_sendSlave64(true); // send payload to slave and request ack
      }
      
      break;
      
    //------------------------------------------------------------------
    // STRAIGHT
    //------------------------------------------------------------------
    case STATE_STRAIGHT: // moving forward
      if (leftDiff > THRESHOLD && leftDiff < outlierThreshold) { // if left diff is greater, steer left
        enterLeftState();
      }
      else if (rightDiff > THRESHOLD && rightDiff < outlierThreshold) { // if right diff is greater, steer right
        enterRightState();
      }
      else if (abs(leftDiff) < THRESHOLD && abs(rightDiff) < THRESHOLD) { // if line is lost, consider giving up
        enterLostState();
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
        
        // decode it all?
        unsigned long timestamp = 0;
        unsigned char cmd = 0;
        int lData = 0;
        int rData = 0;
        while (com_decodeNext(&timestamp, &cmd, &lData, &rData) == true) { // keep decoding

          #ifdef RVR_DEBUG_CMDS
            Serial.println();
            Serial.print("cmd:"); // debug
            Serial.print(cmd, HEX);
            Serial.print(" lData:");
            Serial.print(lData, HEX);
            Serial.print(" rData:");
            Serial.print(rData, HEX);
            Serial.print(" time:");
            Serial.println(timestamp, HEX);
          #endif
          
          switch (cmd) {
            /*case 0x0: // Emergency Stop
              emergencyStop(true); // sends stats
              break;*/
              
            case 0x1: // Slow Stop
              enterStopState(true); // sends stats
              break;

            case 0x7: // Sensor request
              com_encodeMasterPacket(0x7, leftDiff, rightDiff);
              float x, y, z;
              sensor_getMagData(x, y, z);
              com_encodeMasterPacket(0x8, (int)x, int(z));
              com_sendMaster64(true); // send payload to master and request ack - no retry
              light_lightGreen();
              break;
          }

          // clear temp data for next dequeue
          timestamp = 0;
          cmd = 0;
          lData = 0;
          rData = 0;
        }
      }

      // Update rover 2 with a continous straight after STRAIGHT_TIME_MAX time
      if (millis() > straightTimeStart + STRAIGHT_TIME_MAX) {
        straightTimeStart = millis();
        com_encodeSlavePacket(0xA, move_getTargetLeft(), move_getTargetRight());
        lastAck = com_sendSlave64(true); // send payload to slave and request ack
      }
      
      break;
      
    //------------------------------------------------------------------
    // LEFT
    //------------------------------------------------------------------
    case STATE_LEFT: // turning left
      if (leftDiff < THRESHOLD) { // if left diff is less, continue forward
         enterStraightState();
      }
      /*else if (rightDiff > THRESHOLD && rightDiff < outlierThreshold) { // if right diff is greater, steer right
         enterRightState();
         // note that this could overload slaveSlots
      }*/
      else if (abs(leftDiff) < THRESHOLD && abs(rightDiff) < THRESHOLD) { // if line is lost, consider giving up
        enterLostState();
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
        
        // decode it all?
        unsigned long timestamp = 0;
        unsigned char cmd = 0;
        int lData = 0;
        int rData = 0;
        while (com_decodeNext(&timestamp, &cmd, &lData, &rData) == true) { // keep decoding

          #ifdef RVR_DEBUG_CMDS
            Serial.println();
            Serial.print("cmd:"); // debug
            Serial.print(cmd, HEX);
            Serial.print(" lData:");
            Serial.print(lData, HEX);
            Serial.print(" rData:");
            Serial.print(rData, HEX);
            Serial.print(" time:");
            Serial.println(timestamp, HEX);
          #endif
          
          switch (cmd) {
            /*case 0x0: // Emergency Stop
              emergencyStop(true); // sends stats
              break;*/
              
            case 0x1: // Slow Stop
              enterStopState(true); // sends stats
              break;

            case 0x7: // Sensor request
              com_encodeMasterPacket(0x7, leftDiff, rightDiff);
              float x, y, z;
              sensor_getMagData(x, y, z);
              com_encodeMasterPacket(0x8, (int)x, int(z));
              com_sendMaster64(true); // send payload to master and request ack - no retry
              light_turnLeft();
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
    // RIGHT
    //------------------------------------------------------------------
    case STATE_RIGHT: // turning right
      if (rightDiff < THRESHOLD) { // if right diff is less, continue forward
        enterStraightState();
      }
      /*else if (leftDiff > THRESHOLD && leftDiff < outlierThreshold) { // if left diff is greater, steer left
        enterLeftState();
        // note that this could overload slaveSlots
      }*/
      else if (abs(leftDiff) < THRESHOLD && abs(rightDiff) < THRESHOLD) { // if line is lost, consider giving up
        enterLostState();
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
        
        // decode it all?
        unsigned long timestamp = 0;
        unsigned char cmd = 0;
        int lData = 0;
        int rData = 0;
        while (com_decodeNext(&timestamp, &cmd, &lData, &rData) == true) { // keep decoding

          #ifdef RVR_DEBUG_CMDS
            Serial.println();
            Serial.print("cmd:"); // debug
            Serial.print(cmd, HEX);
            Serial.print(" lData:");
            Serial.print(lData, HEX);
            Serial.print(" rData:");
            Serial.print(rData, HEX);
            Serial.print(" time:");
            Serial.println(timestamp, HEX);
          #endif
          
          switch (cmd) {
            /*case 0x0: // Emergency Stop
              emergencyStop(true); // sends stats
              break;*/
              
            case 0x1: // Slow Stop
              enterStopState(true); // sends stats
              break;

            case 0x7: // Sensor request
              com_encodeMasterPacket(0x7, leftDiff, rightDiff);
              float x, y, z;
              sensor_getMagData(x, y, z);
              com_encodeMasterPacket(0x8, (int)x, int(z));
              com_sendMaster64(true); // send payload to master and request ack - no retry
              light_turnRight();
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
    // LOST
    //------------------------------------------------------------------
    case STATE_LOST: // Lost the line
      if (leftDiff > THRESHOLD && leftDiff < outlierThreshold) { // if left diff is greater, steer left
        enterLeftState();
      }
      else if (rightDiff > THRESHOLD && rightDiff < outlierThreshold) { // if right diff is greater, steer right
        enterRightState();
      }
      else if (abs(leftDiff) < THRESHOLD && abs(rightDiff) < THRESHOLD) { // if line is still lost, consider giving up
        if (millis() - giveUpStart > GIVE_UP_LIMIT) {
          enterStopState(true); // sends stats
        }
      }
      else { // On the line
        enterStraightState();
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
        
        // decode it all?
        unsigned long timestamp = 0;
        unsigned char cmd = 0;
        int lData = 0;
        int rData = 0;
        while (com_decodeNext(&timestamp, &cmd, &lData, &rData) == true) { // keep decoding

          #ifdef RVR_DEBUG_CMDS
            Serial.println();
            Serial.print("cmd:"); // debug
            Serial.print(cmd, HEX);
            Serial.print(" lData:");
            Serial.print(lData, HEX);
            Serial.print(" rData:");
            Serial.print(rData, HEX);
            Serial.print(" time:");
            Serial.println(timestamp, HEX);
          #endif
          
          switch (cmd) {
            /*case 0x0: // Emergency Stop
              emergencyStop(true); // sends stats
              break;*/
              
            case 0x1: // Slow Stop
              enterStopState(true); // sends stats
              break;

            case 0x7: // Sensor request
              com_encodeMasterPacket(0x7, leftDiff, rightDiff);
              float x, y, z;
              sensor_getMagData(x, y, z);
              com_encodeMasterPacket(0x8, (int)x, int(z));
              com_sendMaster64(true); // send payload to master and request ack - no retry
              light_lightYellow();
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
        unsigned long timestamp = 0;
        unsigned char cmd = 0;
        int lData = 0;
        int rData = 0;
        while (com_decodeNext(&timestamp, &cmd, &lData, &rData) == true) { // keep decoding

          #ifdef RVR_DEBUG_CMDS
            Serial.println();
            Serial.print("cmd:"); // debug
            Serial.print(cmd, HEX);
            Serial.print(" lData:");
            Serial.print(lData, HEX);
            Serial.print(" rData:");
            Serial.print(rData, HEX);
            Serial.print(" time:");
            Serial.println(timestamp, HEX);
          #endif
          
          switch (cmd) {
            /*case 0x0: // Emergency Stop
              emergencyStop(false); // no stats
              break;*/
              
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
  
            case 0x6: // Start Search
              enterSearchState();
              break;

            case 0x7: // Sensor request
              com_encodeMasterPacket(0x7, leftDiff, rightDiff);
              float x, y, z;
              sensor_getMagData(x, y, z);
              com_encodeMasterPacket(0x8, (int)x, int(z));
              com_sendMaster64(true); // send payload to master and request ack - no retry
              light_lightRed();
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
        unsigned long timestamp = 0;
        unsigned char cmd = 0;
        int lData = 0;
        int rData = 0;
        while (com_decodeNext(&timestamp, &cmd, &lData, &rData) == true) { // keep decoding

          #ifdef RVR_DEBUG_CMDS
            Serial.println();
            Serial.print("cmd:"); // debug
            Serial.print(cmd, HEX);
            Serial.print(" lData:");
            Serial.print(lData, HEX);
            Serial.print(" rData:");
            Serial.print(rData, HEX);
            Serial.print(" time:");
            Serial.println(timestamp, HEX);
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
  
            case 0x6: // Start Search
              enterSearchState();
              break;

            case 0x7: // Sensor request
              com_encodeMasterPacket(0x7, leftDiff, rightDiff);
              float x, y, z;
              sensor_getMagData(x, y, z);
              com_encodeMasterPacket(0x8, (int)x, int(z));
              com_sendMaster64(true); // send payload to master and request ack - no retry
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
// enterManualState() -- Enters STATE_MANUAL
//----------------------------------------------------------------------
void enterManualState() {
  currentState = STATE_MANUAL;
  light_lightWhite();
}

//----------------------------------------------------------------------
// enterSearchState() -- Enters STATE_SEARCH
//----------------------------------------------------------------------
void enterSearchState() {
  currentState = STATE_SEARCH;
  light_lightYellow();
  move_setTarget(STRAIGHT_POWER, STRAIGHT_POWER);
  giveUpStart = 0;
  straightTimeStart = millis();
  
  int slots = com_encodeSlavePacket(0xA, move_getTargetLeft(), move_getTargetRight());
  if (slots <= halfSlavePayload) // payload atleast half full
    lastAck = com_sendSlave64(true); // send payload to slave and request ack
}

//----------------------------------------------------------------------
// enterStraightState() -- Enters STATE_STRAIGHT
//----------------------------------------------------------------------
void enterStraightState() {
  currentState = STATE_STRAIGHT;
  light_lightGreen();
  move_setTarget(STRAIGHT_POWER, STRAIGHT_POWER);
  giveUpStart = 0;
  straightTimeStart = millis();

  if (com_getSlaveSlotsLeft() <= halfSlavePayload) // payload atleast half full
    lastAck = com_sendSlave64(true); // send payload to slave and request ack

  int slots = com_encodeSlavePacket(0xA, move_getTargetLeft(), move_getTargetRight());
  if (slots <= halfSlavePayload) // payload atleast half full
    lastAck = com_sendSlave64(true); // send payload to slave and request ack
}

//----------------------------------------------------------------------
// enterLostState() -- Enters STATE_LOST
//----------------------------------------------------------------------
void enterLostState() {
  currentState = STATE_LOST;
  light_lightYellow();
  giveUpStart = millis(); // timestamp

  if (lastDirectionRight) // try the last direction
    move_setTarget(STRAIGHT_POWER, TURN_POWER); // turn right
  else
    move_setTarget(TURN_POWER, STRAIGHT_POWER); // turn left

  if (com_getSlaveSlotsLeft() <= halfSlavePayload) // payload atleast half full
    lastAck = com_sendSlave64(true); // send payload to slave and request ack

  int slots = com_encodeSlavePacket(0xA, move_getTargetLeft(), move_getTargetRight());
  if (slots <= halfSlavePayload) // payload atleast half full
    lastAck = com_sendSlave64(true); // send payload to slave and request ack
}

//----------------------------------------------------------------------
// enterLeftState() -- Enters STATE_LEFT
//----------------------------------------------------------------------
void enterLeftState() {
  currentState = STATE_LEFT;
  light_turnLeft();
  move_setTarget(TURN_POWER, STRAIGHT_POWER);
  giveUpStart = 0;
  lastDirectionRight = false;

  com_encodeSlavePacket(0xA, move_getTargetLeft(), move_getTargetRight());
  // wait until we are going straight before sending
}

//----------------------------------------------------------------------
// enterRightState() -- Enters STATE_RIGHT
//----------------------------------------------------------------------
void enterRightState() {
  currentState = STATE_RIGHT;
  light_turnRight();
  move_setTarget(STRAIGHT_POWER, TURN_POWER);
  giveUpStart = 0;
  lastDirectionRight = true;

  com_encodeSlavePacket(0xA, move_getTargetLeft(), move_getTargetRight());
  // wait until we are going straight before sending
}

//----------------------------------------------------------------------
// enterStopState() -- Enters STATE_STOP, clears payloads, and optionally
// sends statistics.
//----------------------------------------------------------------------
void enterStopState(bool stats) {
  // change states
  light_lightRed();
  move_setTarget(0, 0);
  giveUpStart = 0;
  lastDirectionRight = true;

  if (currentState != STATE_MANUAL) {
    if (com_getSlaveSlotsLeft() <= halfSlavePayload) // payload atleast half full
      lastAck = com_sendSlave64(true); // send payload to slave and request ack
  
    com_encodeSlavePacket(0xA, 0, 0);
    lastAck = com_sendSlave64(true); // send payload to slave and request ack
    if (lastAck != ACK_SUCCESS) // retry once
      lastAck = com_sendSlave64(true); // send payload to slave and request ack
  }

  currentState = STATE_STOP;

  // clear payloads
  com_emptyPayload(true); // slave payload
  com_emptyPayload(false); // master payload
  lastAck = ACK_SUCCESS;

  // empty the packetQueue
  com_emptyQueue();

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
  lastAck = ACK_SUCCESS;
  
  // estop other rover if we are not already stopped
  if (currentState != STATE_STOP) {
    com_encodeSlavePacket(0, 0, 0);
    com_sendSlave64(true); // send payload to slave and request ack - no retry
  }

  // empty the packetQueue
  com_emptyQueue();

  // finalize state change and send statistics to master
  currentState = STATE_STOP;
  giveUpStart = 0;
  lastDirectionRight = true;
  if (stats) {
    com_sendStatistics64(true); // stats with ack(s) to master - no retry
    com_resetStatistics();
  }
  delay(100);
  light_lightRed();
}
