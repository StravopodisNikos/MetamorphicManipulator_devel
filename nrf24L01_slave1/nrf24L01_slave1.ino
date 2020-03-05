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


/*
 * Create radio object for PseudoJoint         
 */
RF24 PSEUDO_SLAVE(CE_Pin, CSN_Pin); // CE, CSN

PseudoRFcommMetamorphicManipulator PSEUDOJOINT_COMM1(PSEUDO_SLAVE, PSEUDO_NUMBER1, CSN_Pin, CE_Pin, MISO_Pin, MOSI_Pin);

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
bool return_function_state1 = false;
bool return_function_state2 = false;

enum Mode slaveMode;

void setup() {
  /*
   * In setup() Pseudo gives name/state to master that listens
   */
  Serial.begin(115200);  
  
  PSEUDO_SLAVE.begin();
  PSEUDO_SLAVE.setPALevel(RF24_PA_MAX);
  
  // 1. At startup SLAVE - > Tx

    do{
      return_function_state = PSEUDOJOINT_COMM1.setTxSlave(PSEUDO_SLAVE, radioPseudo1Number, pseudoAddresses );
      if(return_function_state){
        Serial.print("[ PSEUDO: "); Serial.print(radioPseudo1Number);  Serial.println(" ] WRITE Tx Mode to SLAVE: SUCCESS");
        slaveMode = Tx;
      }
      else
      {
        Serial.print("[ PSEUDO: "); Serial.print(radioPseudo1Number);  Serial.println(" ] WRITE Tx Mode to SLAVE: FAILED");
      }
      delay(100);
      //Serial.print("return_function_state.setTxSlave="); Serial.println(return_function_state);
    }while(!return_function_state); // Try to write Modes until SUCCESS 

    
  // 2. Writes STATE to MASTER
    return_function_state = PSEUDOJOINT_COMM1.writePseudoStatePacket(PSEUDO_SLAVE, radioPseudo1Number, pseudoAddresses, STATE_LOCKED);
    if(return_function_state)
    {
      Serial.println("PSEUDOJOINT_COMM1.writePseudoStatePacket SUCCESS");
    }
    else
    {
      Serial.println("PSEUDOJOINT_COMM1.writePseudoStatePacket FAILED");
    }
    
} // END SETUP


void loop() {

  // SLAVE ONLY CHECKS IN WHAT MODE HE IS SET BY MASTER

      
  // In loop start SLAVE MUST ALWAYS CHECK FOR mode
  do{
      // 1. Checks if SLAVE = RX
      return_function_state1 = PSEUDOJOINT_COMM1.setRxSlave(PSEUDO_SLAVE, radioPseudo1Number, pseudoAddresses);
      if(return_function_state1){
          Serial.print("[ PSEUDO: "); Serial.print(radioPseudo1Number);  Serial.println(" ] WRITE Rx Mode to SLAVE: SUCCESS");
          slaveMode = Rx;
      }
      else
      {
          Serial.print("[ PSEUDO: "); Serial.print(radioPseudo1Number);  Serial.println(" ] WRITE Rx Mode to SLAVE: FAILED");
      }
      delay(500);  

      //  if (2.) is quoted it works fine
      // 2. Checks if SLAVE = TX
      return_function_state2 = PSEUDOJOINT_COMM1.setTxSlave(PSEUDO_SLAVE, radioPseudo1Number, pseudoAddresses);
      if(return_function_state2){
          Serial.print("[ PSEUDO: "); Serial.print(radioPseudo1Number);  Serial.println(" ] WRITE Tx Mode to SLAVE: SUCCESS");
          slaveMode = Tx;
      }
      else
      {
          Serial.print("[ PSEUDO: "); Serial.print(radioPseudo1Number);  Serial.println(" ] WRITE Tx Mode to SLAVE: FAILED");
      }
      delay(500);

      // WORKING ON TERMINATING CONDITION!!!!!
   }while( ( (!return_function_state1) && return_function_state2 ) || ( (return_function_state1 && !return_function_state2) ) ); // Try to write Mode until SUCCESS

  
  if(slaveMode == Tx)
  {
      // Call function to send Pseudo state
      return_function_state = PSEUDOJOINT_COMM1.writePseudoStatePacket(PSEUDO_SLAVE, radioPseudo1Number, pseudoAddresses, STATE_LOCKED);
      if(return_function_state)
      {
        Serial.println("PSEUDOJOINT_COMM1.writePseudoStatePacket SUCCESS");
      }
      delay(500);
  }

  if(slaveMode == Rx)    
  {   
      Serial.println("Haha");      
      return_function_state = PSEUDOJOINT_COMM1.readCommandPseudoPacket(PSEUDO_SLAVE, radioPseudo1Number, pseudoAddresses);
      if(return_function_state)
      {
        Serial.println("PSEUDOJOINT_COMM1.readCommandPseudoPacket SUCCESS");
      }
      delay(500);
      
  }  
//*/
}
