#include <SPI.h>
#include <RF24.h>
#include <nRF24L01.h>
#include <RF24_config.h>
#include <stdio.h>
#include <string.h>

#include "PseudoRFcommMetamorphicManipulator.h"

#define CSN_Pin   8
#define CE_Pin    7
#define MOSI_Pin  11
#define SCK_Pin   13
#define MISO_Pin  12

// Definitions for PseudoJoints Communication
//#define PSEUDO_NUMBER1 	1
//#define STATE_LOCKED 	  1
//#define STATE_UNLOCKED 	11
//#define CMD_LOCK		    2
//#define CMD_UNLOCK	    22
//#define CMD_ACTIVATE	  3	  // Calls pseudojoint to move to goal position
//#define CMD_HOME		    4

// Declaration of default strings based on Communication
//const char STATE_LOCKED_STRING[] = "LOCKED";
//const char STATE_UNLOCKED_STRING[] = "UNLOCKED";
//const char MOVING[]= "MOVING";
/*
 * Create radio object for Master         
 */
RF24 OPENCR_MASTER(CE_Pin, CSN_Pin);                  // GIVE PINS: CE, CSN
PseudoRFcommMetamorphicManipulator MASTER_COMM1(OPENCR_MASTER, PSEUDO_NUMBER1, CSN_Pin, CE_Pin, MISO_Pin, MOSI_Pin);

/*
 * Give name to addresses between master and pseudos  
 */
//const uint8_t address[6] = "1P2CR";                      // PSEUDO1 -> OPENCR
////char addresses[][6] = {"1P2CR","2P2CR"};     // for multiple ports: Pipes 1-5 should share the same address, except the first byte. Only the first byte in the array should be unique
// openReadingPipe: Pipe 0 is also used by the writing pipe. So if you open pipe 0 for reading, and then startListening(), it will overwrite the writing pipe. Ergo, do an openWritingPipe() again before write().

/*      
 * Set the RADIO NUMBER TO IDENTIFY THE TRANCEIVER         
 */
uint8_t radioPseudo1Number = 1;
uint8_t radioPseudo2Number = 2;

bool return_write_attempt;
bool return_read_attempt;
bool result = false;
bool return_function_state = false;
/*
 * Data Handling
 * Struct members are the basic inormation exchanged 
 * between master and pseudos
 */
struct incomingDataStruct{
  unsigned long   _micros;
  int             pseudoNumber;
  int             pseudoState;      
  float           pseudoPosition;
}incomingCommunicationData;

struct outcomingDataStruct{
  unsigned long   _micros;
  int             pseudoCommand;   
}outcomingCommunicationData;

enum Mode masterMode;

const byte numChars = 32;
char receivedChars[numChars];

boolean newData = false;

void setup() {
  Serial.begin(115200);
  
  OPENCR_MASTER.begin(); 
  OPENCR_MASTER.setPALevel(RF24_PA_MAX);  // set to min for low power, in close distance
/*
  // For MASTER: AFTER POWER ON: MASTER LISTENS FOR PSEUDOS
  //OPENCR_MASTER.openWritingPipe(addresses[0]); //  set the destination of where to write to(Pseudo1)
  OPENCR_MASTER.openReadingPipe(radioPseudo1Number,addresses[0]); // set the destination of where to read from(Pseudo1)

  //OPENCR_MASTER.startListening();         // Starts listening for incoming messages from opened pipes
*/
  
}

void loop() {

  Serial.println("Set MASTER MODE: FORMAT <MODE>:");
  
  while (!Serial.available());
  recvWithStartEndMarkers();
  showNewData();

  if( strcmp(receivedChars,"TX") == 0 )
  {
    // FOTMAT THE WRITING/READING PIPES
    do{
    return_function_state = MASTER_COMM1.setTxMaster(OPENCR_MASTER, radioPseudo1Number, pseudoAddresses );
    if(return_function_state){
      Serial.println("MASTER in Tx mode: SUCCESS");
      masterMode = Tx;
    }
    else
    {
      Serial.println("MASTER in Tx mode: FAILED");
    }
    delay(500);
    Serial.print("return_function_state.setTxMaster="); Serial.println(return_function_state);
    }while(!return_function_state); // Try to write Modes until SUCCESS   
  }
  else if ( strcmp(receivedChars,"RX") == 0 )
  {
    // FOTMAT THE WRITING/READING PIPES
    // MASTER_COMM1.setRxMaster(OPENCR_MASTER, radioPseudo1Number, pseudoAddresses ); //not ready
    Serial.println("MASTER in Rx mode");
    masterMode = Rx;
  }
  
  Serial.print("masterMode="); Serial.println(masterMode);      
  if(masterMode == Rx)
  {
    Serial.println("Give ORDER: FORMAT <ORDER>:");
        
    while (!Serial.available());
    recvWithStartEndMarkers();
    showNewData();

    if( strcmp(receivedChars,"RS") == 0 )
    {
        // Here MASTER->Rx && SLAVE ->Tx
        Serial.println("Reads Pseudo State");
        do{
        char address1 = pseudoAddresses[0];
        return_function_state = MASTER_COMM1.readPseudoStatePacket(OPENCR_MASTER, radioPseudo1Number, address1);
        if(return_function_state)
        {
          Serial.println("SUCCESS");
        }
        delay(1000);
        }while(return_function_state);
    }
    else
    {
      Serial.println("Serial input mismatch");
    }

  }
    
  if(masterMode == Tx)
  {
    Serial.println("Give ORDER: FORMAT <ORDER>:");
    delay(10000);
        
    while (!Serial.available());
    recvWithStartEndMarkers();
    showNewData();        

    if( strcmp(receivedChars,"WC") == 0 )
    {
        Serial.println("Writes Pseudo Command");
        do{
          char address1 = pseudoAddresses[0];
          return_function_state = MASTER_COMM1.writeCommandPseudoPacket(OPENCR_MASTER, radioPseudo1Number, address1, CMD_SGP);
        delay(1000);
        }while(!return_function_state);
    }
    else
    {
      Serial.println("Serial input mismatch");
    }
        
  }
  
  if( (masterMode != Tx ) && (masterMode != Rx ) )
  {
    Serial.println("MASTER MODE MISMATCH!");
  }

} // LOOP

// Thanks to Robin2 and his post: https://forum.arduino.cc/index.php?topic=288234.0
void recvWithStartEndMarkers() {
    static boolean recvInProgress = false;
    static byte ndx = 0;
    char startMarker = '<';
    char endMarker = '>';
    char rc;
 
 // if (Serial.available() > 0) {
    while (Serial.available() > 0 && newData == false) {
        rc = Serial.read();

        if (recvInProgress == true) {
            if (rc != endMarker) {
                receivedChars[ndx] = rc;
                ndx++;
                if (ndx >= numChars) {
                    ndx = numChars - 1;
                }
            }
            else {
                receivedChars[ndx] = '\0'; // terminate the string
                recvInProgress = false;
                ndx = 0;
                newData = true;
            }
        }

        else if (rc == startMarker) {
            recvInProgress = true;
        }
    }
}

void showNewData() {
    if (newData == true) {
        Serial.print("ENTERED:");
        Serial.println(receivedChars);
        newData = false;
    }
}
