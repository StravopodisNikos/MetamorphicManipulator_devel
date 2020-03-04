#include <SPI.h>
#include <nRF24L01.h>
#include <RF24_config.h>
#include <RF24.h>
#include <stdio.h>

#include "PseudoRFcommMetamorphicManipulator.h"

#define CSN_Pin   8
#define CE_Pin    7
#define MOSI_Pin  11
#define SCK_Pin   13
#define MISO_Pin  12

// Definitions for PseudoJoints Communication
#define PSEUDO_NUMBER1 	1
#define STATE_LOCKED 	  1
#define STATE_UNLOCKED 	11
#define CMD_LOCK		    2
#define CMD_UNLOCK	    22
#define CMD_ACTIVATE	  3	// Calls pseudojoint to move to goal position
#define CMD_HOME		    4

// Declaration of default strings based on Communication
//const char STATE_LOCKED_STRING[] = "LOCKED";
//const char STATE_UNLOCKED_STRING[] = "UNLOCKED";
//const char MOVING[]= "MOVING";

/*
 * Create radio object for PseudoJoint         
 */
RF24 PSEUDO_SLAVE(CE_Pin, CSN_Pin); // CE, CSN

PseudoRFcommMetamorphicManipulator PSEUDOJOINT_COMM1(PSEUDO_SLAVE, PSEUDO_NUMBER1, CSN_Pin, CE_Pin, MISO_Pin, MOSI_Pin);

bool return_write_attempt;
bool return_read_attempt;
bool result = false;
bool return_function_state = false;
/*
 * Give name to addresses between master and pseudos  
 */
////char addresses[][6] = {"1P2CR","2P2CR"};     // for multiple ports: Pipes 1-5 should share the same address, except the first byte. Only the first byte in the array should be unique
/*      
 * Set the RADIO NUMBER TO IDENTIFY THE TRANCEIVER         
 */
uint8_t radioPseudo1Number = PSEUDO_NUMBER1;

struct outcomingDataStruct{
  unsigned long   _micros;
  int             pseudoNumber;
  int             pseudoState;      
  float           pseudoPosition;
}outcomingCommunicationData;

//typedef struct outcomingDataStruct outcomingCommunicationData;

struct incomingDataStruct{
  unsigned long   _micros;
  int             pseudoCommand;    
}incomingCommunicationData;

enum Mode slaveMode;
char address1;

void setup() {
  /*
   * In setup() Pseudo gives name/state to master that listens
   */
  Serial.begin(115200);  
  
  PSEUDO_SLAVE.begin();
  PSEUDO_SLAVE.setPALevel(RF24_PA_MAX);
  
/*
  // Open pipe to write to specified address
  PSEUDO_SLAVE.openWritingPipe(addresses[0]);
  //PSEUDO_SLAVE.openReadingPipe(radioPseudo1Number,addresses[0]);
  
  PSEUDO_SLAVE.setPALevel(RF24_PA_MAX);
  //PSEUDO_SLAVE.startListening();
  */

  // For the first time the SLAVE starts as listener
  
}


void loop() {
  // In loop start SLAVE MUST ALWAYS CHECK FOR mode
  PSEUDOJOINT_COMM1.setRxSlave(PSEUDO_SLAVE, radioPseudo1Number, pseudoAddresses);
      
/*  
  if(slaveMode == Tx)
  {
      // Call function to send Pseudo state
      address1 = pseudoAddresses[0];
      
      return_function_state = PSEUDOJOINT_COMM1.writePseudoStatePacket(PSEUDO_SLAVE, radioPseudo1Number, address1, STATE_LOCKED);
      if(return_function_state)
      {
        Serial.println("SUCCESS");
      }
    
      delay(500);
  }

  if(slaveMode == Rx)    
  {   
      address1 = pseudoAddresses[0];
      
      return_function_state = PSEUDOJOINT_COMM1.readCommandPseudoPacket(PSEUDO_SLAVE, radioPseudo1Number, address1);
      if(return_function_state)
      {
        Serial.println("SUCCESS");
      }
    
      delay(500);
      
  }  
*/
}
