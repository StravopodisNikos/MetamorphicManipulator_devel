/*
 *  Local Git Directory: ~/Arduino/MetaLunokhod101_Development/SPI/using_ISR_simple_master
 *  Hard linked using  : ln ~/Arduino/SPI/using_ISR_simple/using_ISR_simple_master/using_ISR_simple_master.ino ./using_ISR_simple_master.ino
 */
#include <SPI.h>
#include "PseudoSPIcommMetamorphicManipulator.h"

#define motor_execution_time 10

/*
 * CONFIGURE all the system-dependent-arrays
 */
int masterID = 0;
int pseudoIDs[] = {PSEUDO1_ID, PSEUDO2_ID};
int ssPins[]  = {SSpinPseudo1, SSpinPseudo2};
byte CURRENT_STATE[sizeof(pseudoIDs)];

/*
 * CONSTRUCT CLASS OBJECT FOR SPI COMMUNICATION BETWEEN DEVICES
 */
PseudoSPIcommMetamorphicManipulator MASTER_SPI(Tx, masterID, statusLED_Pin,  MOSI_NANO, MISO_NANO, SCK_NANO, TXled_Pin, RXled_Pin, ssPins);

/*
 * FLAGS USED TO CONTROL LOOPS
 */
bool metaMode;
bool metaExecution;

/*
 * EXTERN LIBRARY DEFINED VARIABLES
 */
bool return_function_state;
byte state_receive_from_slave;

void setup (void)
{
  Serial.begin (SERIAL_BAUDRATE);
/*
  pinMode(SCK_NANO, OUTPUT);
  pinMode(MOSI_NANO, OUTPUT);
  pinMode(MISO_NANO, INPUT);
  for (size_t i = 0; i < sizeof(ssPins); i++)
  {
    pinMode(ssPins[i], OUTPUT);
    pinMode(ssPins[i], HIGH);
  }
*/

 /*
  * Start SPI Com Protocol
  */
  SPI.begin ();
  SPI.setClockDivider(SPI_CLOCK_DIV128);      // Slow down the master a bit

 /*
  * Ping Pseudos-Each pseudo pinged: Blinks(2,500) green Led(ConnectedLED)
  */
  for (int pseudo_cnt = 0; pseudo_cnt < sizeof(ssPins); pseudo_cnt++) 
  {
    return_function_state = MASTER_SPI.connectPseudoMaster(pseudoIDs[pseudo_cnt], ssPins);
    if (return_function_state)
    {
        MASTER_SPI.statusLEDblink(2, 500);
        Serial.print("[   MASTER:  ]"); Serial.print(" CONNECTED TO: [   PSEUDO: "); Serial.print(pseudoIDs[pseudo_cnt]); Serial.println("  ]   SUCCESS");
    }
    else
    {
        MASTER_SPI.statusLEDblink(4, 250);
        Serial.print("[   MASTER:  ]"); Serial.print(" CONNECTED TO: [   PSEUDO: "); Serial.print(pseudoIDs[pseudo_cnt]); Serial.println("  ]   FAILED");
    }
    
  } // END IF PING
  
  digitalWrite(TXled_Pin,LOW); digitalWrite(RXled_Pin,LOW);

  /*
   * Set flags to start Metamorphosis: Only for now!
   */
   metaMode = true;
   metaExecution = true;
}  // end of setup



void loop (void)
{

  /*
   * I. 
   */
  // User input using external interrupts => Set <ROBOT OPERATION MODE>
  // buttonMetamorphosis
  // buttonAction

  /*
   * II. <METAMORPHOSIS>
   */
   
  /*
   * II.1 READ INITIAL STATE OF ALL PSEUDOS CONNECTED 
   */
  for (int pseudo_cnt = 0; pseudo_cnt < sizeof(ssPins); pseudo_cnt++) 
  {
    // Confirm that last argument is ok! 
    return_function_state = MASTER_SPI.readInitialStateMaster(pseudoIDs[pseudo_cnt], ssPins, &CURRENT_STATE[pseudo_cnt]);
    if (return_function_state)
    {
        MASTER_SPI.statusLEDblink(2, 500);
        Serial.print("[   MASTER:  ]"); Serial.print(" TALKED TO: [   PSEUDO: "); Serial.print(pseudoIDs[pseudo_cnt]); Serial.println("  ]   STATUS:  [  LOCKED  ]  SUCCESS");
    }
    else
    {
        MASTER_SPI.statusLEDblink(4, 250);
        Serial.print("[   MASTER:  ]"); Serial.print(" TALKED TO: [   PSEUDO: "); Serial.print(pseudoIDs[pseudo_cnt]); Serial.println("  ]   STATUS:  [  LOCKED  ]  FAILED");
    }
  } // END FOR READ INITIAL STATE

  // NOW METAMORPHOSIS CONTINUES PSEUDO PER PSEUDO FOR STEPS 2-5! (NOT STEP BY STEP)

  for (int pseudo_cnt = 0; pseudo_cnt < sizeof(ssPins); pseudo_cnt++) 
  {
  /*
   * II.2 IF CURRENT_STATE = LOCKED => SET GOAL POSITION
   */
      do{
          return_function_state = MASTER_SPI.setGoalPositionMaster(pseudoIDs[pseudo_cnt],ssPins, byte GP, &CURRENT_STATE[pseudo_cnt] )
      }while(!return_function_state);
  /*
   * II.3 IF CURRENT_STATE = READY => UNLOCK  
   */

  /*
   * II.4 IF CURRENT_STATE = UNLOCKED => MOVE  
   */

  /*
   * II.5 IF CURRENT_STATE = IN_POSITION => LOCK  
   */   
   
  }

  /*
   * II.6 IF ALL_LOCKED MASTER ASKS USER WHAT TO DO AND COMMANDS SLAVE  
   */  
  // ACCORDING TO SLAVE ANSWER WE CHANGE THE FLAGS TO TERMINATE/REPEAT LOOP II(<METAMORPHOSIS>)

  
  
  // enable Slave Select
  digitalWrite(SSpinPseudo1, LOW);    


  do{
    state_receive_from_slave = transferAndWait ((byte)CMD_MOVE);

    //CURRENT_STATE = (int) state_receive_from_slave;

    delay(100);
    
  }while(! (CURRENT_STATE == IN_POSITION));
  Serial.print ("New Motor State:"); Serial.println (state_receive_from_slave, DEC);
  
  // disable Slave Select
  digitalWrite(SSpinPseudo1, HIGH);
  
  

  if((CURRENT_STATE == IN_POSITION))
  {
  Serial.println("NEW COMMAND");
  // enable Slave Select
  digitalWrite(SSpinPseudo1, LOW);    

  do{
    
    state_receive_from_slave = transferAndWait ((byte)CMD_LOCK);

    //CURRENT_STATE = (int) state_receive_from_slave;

    delay(100);
    
  }while(! (CURRENT_STATE == STATE_LOCKED) );
  
  Serial.print ("New Motor State:"); Serial.println (state_receive_from_slave, DEC);

  
  // disable Slave Select
  transferAndWait (0);
  digitalWrite(SSpinPseudo1, HIGH);
  }
  
  delay (2000);  
}  

byte transferAndWait (byte what)
{
  byte a = SPI.transfer (what);
  delayMicroseconds (10);
  return a;
} 
