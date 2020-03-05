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

/*
 * Create radio object for Master         
 */
RF24 OPENCR_MASTER(CE_Pin, CSN_Pin);                  // GIVE PINS: CE, CSN

PseudoRFcommMetamorphicManipulator MASTER_COMM1(OPENCR_MASTER, PSEUDO_NUMBER1, CSN_Pin, CE_Pin, MISO_Pin, MOSI_Pin);


/*      
 * Set the RADIO NUMBER TO IDENTIFY THE TRANCEIVER AND SET ADDRESSES        
 */
uint8_t radioPseudo1Number = PSEUDO_NUMBER1;
uint8_t radioPseudo2Number = PSEUDO_NUMBER2;

bool return_write_attempt;
bool return_read_attempt;
bool result = false;
bool continue_exec = false;
bool return_function_state = false;

bool setup_success;
bool meta_success = true;
bool act_success = true;

const char * meta_exec = "METAMORPHOSIS";
const char * act_exec  = "ACTION";
int nl_char_shifting   = 10;
enum Mode masterMode;

String user_input;

void setup() {
  Serial.begin(115200);
  
  OPENCR_MASTER.begin(); 
  OPENCR_MASTER.setPALevel(RF24_PA_MAX);  // set to min for low power, in close distance

  // 1. At startup MASTER - > Rx
  
    do{
    return_function_state = MASTER_COMM1.setRxMaster(OPENCR_MASTER, radioPseudo1Number, pseudoAddresses );
    if(return_function_state){
      Serial.println("[   MASTER  ] WRITE Rx Mode to MASTER: SUCCESS");
      masterMode = Rx;
    }
    else
    {
      Serial.println("[   MASTER  ] WRITE Rx Mode to MASTER: FAILED");
    }
    delay(500);
    //Serial.print("return_function_state.setRxMaster="); Serial.println(return_function_state);
    }while(!return_function_state); // Try to write Modes until SUCCESS 
  
  // 2. Reads STATE - > Returns f1: setup_success
    return_function_state = MASTER_COMM1.readPseudoStatePacket(OPENCR_MASTER, radioPseudo1Number, pseudoAddresses);
    if(return_function_state)
    {
      Serial.println("MASTER_COMM1.readPseudoStatePacket SUCCESS");
      setup_success = true;
    }
    else
    {
      Serial.println("MASTER_COMM1.readPseudoStatePacket FAILED");
    }
} // END SETUP

void loop() {
    Serial.println("Set ROBOT OPERATION MODE: FORMAT <MODE>:");
    
    while (!Serial.available());
    user_input = Serial.readString();
    
    Serial.print("[ USER INPUT ]"); Serial.print(user_input); // ADDS NL CHARACTER
    
  /*
   *  METAMORPHOSIS if: 
   *  1. user inputs it
   *  2. + setup was successful: all pseudos returned STATE=LOCKED
   *  3. + either ACTION OR METAMORPHOSIS were SUCCESS
   */
     
    if( ( strcmp(user_input.c_str(),meta_exec)-nl_char_shifting == 0 ) && (setup_success) && ( (meta_success) || (act_success) ) )
    {
      //meta_success = false; // change flag
      
      Serial.println("Begin METAMORPHOSIS");
  
      // 1. Set MASTER to Tx
      do{
          return_function_state = MASTER_COMM1.setTxMaster(OPENCR_MASTER, radioPseudo1Number, pseudoAddresses );
          if(return_function_state){
            Serial.println("[   MASTER  ] WRITE Tx Mode to MASTER: SUCCESS");
            masterMode = Rx;
          }
          else
          {
            Serial.println("[   MASTER  ] WRITE Tx Mode to MASTER: FAILED");
          }
          delay(500);
          //Serial.print("return_function_state.setTxMaster="); Serial.println(return_function_state);
      }while(!return_function_state); // Try to write Modes until SUCCESS
      
      // 2. Write COMMAND: UNLOCK to PSEUDO1
  
      return_function_state = MASTER_COMM1.writeCommandPseudoPacket(OPENCR_MASTER, radioPseudo1Number, pseudoAddresses, CMD_UNLOCK);
      if(return_function_state){
        Serial.print("[   MASTER  ] WRITE COMMAND to [ PSEUDO:"); Serial.print(radioPseudo1Number); Serial.println(" ]: SUCCESS");
      }
      else
      {
        Serial.print("[   MASTER  ] WRITE COMMAND to [ PSEUDO:"); Serial.print(radioPseudo1Number); Serial.println(" ]: FAILED");
      }         
        
    }

    /*
   *  ACTION if: 
   *  1. user inputs it
   *  2. + setup was successful: all pseudos returned STATE=LOCKED
   *  3. + either ACTION OR METAMORPHOSIS were SUCCESS
   */
  if( ( strcmp(user_input.c_str(),act_exec)-nl_char_shifting == 0 ) && (setup_success) && ( (meta_success) || (act_success) ) )
  {
    //act_success = false; // change flag
    Serial.println("Mphka act");
      
  }

/*
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
      Serial.println("[ MASTER ] WRITE Tx Mode to MASTER: SUCCESS");
      masterMode = Tx;
    }
    else
    {
      Serial.println("[ MASTER ] WRITE Tx Mode to MASTER: FAILED");
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
        //char address1 = pseudoAddresses[0];
        return_function_state = MASTER_COMM1.readPseudoStatePacket(OPENCR_MASTER, radioPseudo1Number, pseudoAddresses);
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
        
    while (!Serial.available());
    recvWithStartEndMarkers();
    showNewData();        

    if( strcmp(receivedChars,"WC") == 0 )
    {
        Serial.println("Writes Pseudo Command");
        do{
          //char address1 = pseudoAddresses[0];
          return_function_state = MASTER_COMM1.writeCommandPseudoPacket(OPENCR_MASTER, radioPseudo1Number, pseudoAddresses, CMD_SGP);
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
*/
} // LOOP
