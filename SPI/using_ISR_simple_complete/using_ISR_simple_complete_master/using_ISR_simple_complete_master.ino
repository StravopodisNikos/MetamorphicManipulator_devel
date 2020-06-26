/*
 *  Local Git Directory: ~/Arduino/MetaLunokhod101_Development/SPI/using_ISR_simple_complete/ using_ISR_simple_complete_master
 *  Hard linked using  : ln ~/Arduino/SPI/using_ISR_simple_complete/using_ISR_simple_complete_master/using_ISR_simple_complete_master.ino ./using_ISR_simple_complete_master.ino
 */
// Used Libraries
#include <stdlib.h>
#include <stdio.h>
//#include <SPI.h>
#include "PseudoSPIcommMetamorphicManipulator.h"

/*
 * CONFIGURE all the system-dependent-arrays
 */
int masterID          = 0;
/*
const int TOTAL_PSEUDOS_CONNECTED = 2;
int pseudoIDs[]       = {PSEUDO1_ID, PSEUDO2_ID};
int ssPins[]          = {SSpinPseudo1, SSpinPseudo2};
byte desiredAnatomy[] = {2,4};                  // initialize ci for desired anatomy(these are the GP values given in setGoalPositionMaster function)
*/
const int TOTAL_PSEUDOS_CONNECTED = 1;
int pseudoIDs[]       = {PSEUDO1_ID};
int ssPins[]          = {SSpinPseudo1};
byte desiredAnatomy[] = {6,7,8};                  // initialize ci for desired anatomy(these are the GP values given in setGoalPositionMaster function)
byte anatomy_counter  = 0;                         // just to check repeat mode

byte CURRENT_STATE[sizeof(pseudoIDs)];        // empty states initialization array
bool META_MODES[sizeof(pseudoIDs)];           // empty pseudo mode initialization array
bool META_EXECS[sizeof(pseudoIDs)];           // empty pseudo mode-exec initialization array

/*
 * CONSTRUCT CLASS OBJECT FOR SPI COMMUNICATION BETWEEN DEVICES
 */
PseudoSPIcommMetamorphicManipulator MASTER_SPI(Tx, masterID, statusLED_Pin,  MOSI_NANO, MISO_NANO, SCK_NANO, TXled_Pin, RXled_Pin, ssPins);

/*
 * FLAGS USED TO CONTROL LOOPS
 */
bool END_METAMORPHOSIS;                       // flag defined by META_MODES
bool END_ACTION;
bool metaExecution;                           // flag defined by META_EXECS

/*
 * EXTERN LIBRARY DEFINED VARIABLES
 */
bool return_function_state;
byte state_receive_from_slave;

/*
 * USER SERIAL INPUT VARIABLES
 */
String user_input_string;
int user_input_int;
const char * meta_exec = "MET";
const char * act_exec  = "ACT";
const char * meta_cont = "R";
const char * meta_exit = "E";
int nl_char_shifting   = 10;

const byte numChars = 32;
char receivedChars[numChars];   // an array to store the received data

boolean newData = false;

int dataNumber = 0;             // new for this version

void setup (void)
{
  Serial.begin (SERIAL_BAUDRATE);

// MASTER PINMODE 
  pinMode(SCK_NANO, OUTPUT);
  pinMode(MOSI_NANO, OUTPUT);
  pinMode(MISO_NANO, INPUT);
  for (size_t i = 0; i < sizeof(ssPins); i++)
  {
    pinMode(ssPins[i], OUTPUT);
    pinMode(ssPins[i], HIGH);
  }


 /*
  * Start SPI Com Protocol
  */
  SPI.begin ();
  SPI.setClockDivider(SPI_CLOCK_DIV32);      // Slow down the master a bit

 /*
  * Ping Pseudos-Each pseudo pinged: Blinks(2,500) green Led(ConnectedLED)
  */
  for (int pseudo_cnt = 0; pseudo_cnt < TOTAL_PSEUDOS_CONNECTED; pseudo_cnt++) 
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
    delay(1);
  } // END IF PING
  
  digitalWrite(TXled_Pin,LOW); digitalWrite(RXled_Pin,LOW);

  /*
   * Set flags to start Metamorphosis: Only for now! => MUST IMPLEMENTED WITH EXTERNAL INTERRUPTS
   */
   END_METAMORPHOSIS = false;
   END_ACTION = false;
   metaExecution = true;
}  // end of setup



void loop (void)
{

  /*
   * I. 
   */
  Serial.println("Set ROBOT OPERATION MODE: FORMAT <MODE>:");
  
  while (Serial.available() == 0) {};
  user_input_string = Serial.readString();
  
  Serial.print("[ USER INPUT ]"); Serial.print("   ->   "); Serial.print(user_input_string);
    
  /*
   * II. <METAMORPHOSIS>
   */
   if( ( strcmp(user_input_string.c_str(),meta_exec)-nl_char_shifting == 0 ) )
   {
   
   while( (!END_METAMORPHOSIS) ) 
   {
    Serial.println("Begin METAMORPHOSIS...");
    /*
     * II.1 READ INITIAL STATE OF ALL PSEUDOS CONNECTED 
     */
    for (int pseudo_cnt = 0; pseudo_cnt < TOTAL_PSEUDOS_CONNECTED; pseudo_cnt++) 
    {
      
      return_function_state = MASTER_SPI.readCurrentStateMaster(pseudoIDs[pseudo_cnt], ssPins, &CURRENT_STATE[pseudo_cnt]);
      if (return_function_state)
      {
          MASTER_SPI.statusLEDblink(2, 500);
          Serial.print("[   MASTER:  ]"); Serial.print(" TALKED TO: [   PSEUDO: "); Serial.print(pseudoIDs[pseudo_cnt]); Serial.println("  ]   STATUS:  [  READY FOR METAMORPHOSIS  ]  SUCCESS");
      }
      else
      {
          MASTER_SPI.statusLEDblink(4, 250);
          Serial.print("[   MASTER:  ]"); Serial.print(" TALKED TO: [   PSEUDO: "); Serial.print(pseudoIDs[pseudo_cnt]); Serial.println("  ]   STATUS:  [  READY FOR METAMORPHOSIS  ]  FAILED");
      }
 
      // Doesn't execute Metamorphosis if neither of these 3 states is returned
      if( (CURRENT_STATE[pseudo_cnt] != STATE_LOCKED) && (CURRENT_STATE[pseudo_cnt] != META_REPEAT) && (CURRENT_STATE[pseudo_cnt] != META_FINISHED) ){
        metaExecution = false;
      }
      
    } // END FOR READ INITIAL STATE
    
    // NOW IF ALL LOCKED METAMORPHOSIS CONTINUES PSEUDO PER PSEUDO FOR STEPS 2-5! (NOT STEP BY STEP)
    if(metaExecution){
      
      for (int pseudo_cnt = 0; pseudo_cnt < TOTAL_PSEUDOS_CONNECTED; pseudo_cnt++) 
      {
        /*
         * II.2 IF CURRENT_STATE = LOCKED => SET GOAL POSITION
         */
        return_function_state = MASTER_SPI.setGoalPositionMaster(pseudoIDs[pseudo_cnt], ssPins, &desiredAnatomy[anatomy_counter], &CURRENT_STATE[pseudo_cnt] );
        if (return_function_state)
        {
            //MASTER_SPI.statusLEDblink(6, 500);
            Serial.print("[   MASTER:  ]"); Serial.print(" TALKED TO: [   PSEUDO: "); Serial.print(pseudoIDs[pseudo_cnt]); Serial.println("  ]   STATUS:  [      READY     ]  SUCCESS");
        }
        else
        {
            //MASTER_SPI.statusLEDblink(8, 250);
            Serial.print("[   MASTER:  ]"); Serial.print(" TALKED TO: [   PSEUDO: "); Serial.print(pseudoIDs[pseudo_cnt]); Serial.println("  ]   STATUS:  [      READY     ]  FAILED");
        }
        /*
         * II.3 IF CURRENT_STATE = READY => UNLOCK  
         */
        return_function_state = MASTER_SPI.unlockPseudoMaster(pseudoIDs[pseudo_cnt], ssPins, &CURRENT_STATE[pseudo_cnt]);
        if (return_function_state)
        {
            //MASTER_SPI.statusLEDblink(3, 500);
            Serial.print("[   MASTER:  ]"); Serial.print(" TALKED TO: [   PSEUDO: "); Serial.print(pseudoIDs[pseudo_cnt]); Serial.println("  ]   STATUS:  [    UNLOCKED    ]  SUCCESS");
        }
        else
        {
            //MASTER_SPI.statusLEDblink(5, 250);
            Serial.print("[   MASTER:  ]"); Serial.print(" TALKED TO: [   PSEUDO: "); Serial.print(pseudoIDs[pseudo_cnt]); Serial.println("  ]   STATUS:  [    UNLOCKED    ]  FAILED");
        }        
        /*
         * II.4 IF CURRENT_STATE = UNLOCKED => MOVE  
         */
        return_function_state = MASTER_SPI.movePseudoMaster(pseudoIDs[pseudo_cnt], ssPins, &CURRENT_STATE[pseudo_cnt]);
        if (return_function_state)
        {
            //MASTER_SPI.statusLEDblink(7, 500);
            Serial.print("[   MASTER:  ]"); Serial.print(" TALKED TO: [   PSEUDO: "); Serial.print(pseudoIDs[pseudo_cnt]); Serial.println("  ]   STATUS:  [  IN_POSITION   ]  SUCCESS");
        }
        else
        {
            //MASTER_SPI.statusLEDblink(9, 250);
            Serial.print("[   MASTER:  ]"); Serial.print(" TALKED TO: [   PSEUDO: "); Serial.print(pseudoIDs[pseudo_cnt]); Serial.println("  ]   STATUS:  [  IN_POSITION   ]  FAILED");
        }         
        /*
         * II.5 IF CURRENT_STATE = IN_POSITION => LOCK  
         */   
        return_function_state = MASTER_SPI.lockPseudoMaster(pseudoIDs[pseudo_cnt], ssPins, &CURRENT_STATE[pseudo_cnt]);
        if (return_function_state)
        {
            //MASTER_SPI.statusLEDblink(2, 500);
            Serial.print("[   MASTER:  ]"); Serial.print(" TALKED TO: [   PSEUDO: "); Serial.print(pseudoIDs[pseudo_cnt]); Serial.println("  ]   STATUS:  [     LOCKED     ]  SUCCESS");
        } 
        else
        {
            //MASTER_SPI.statusLEDblink(4, 250);
            Serial.print("[   MASTER:  ]"); Serial.print(" TALKED TO: [   PSEUDO: "); Serial.print(pseudoIDs[pseudo_cnt]); Serial.println("  ]   STATUS:  [     LOCKED     ]  FAILED");
        } 
      }
  
      /*
       * II.6 IF ALL_LOCKED MASTER ASKS USER WHAT TO DO AND COMMANDS SLAVE  
       */  
      // ACCORDING TO SLAVE ANSWER WE CHANGE THE FLAGS TO TERMINATE/REPEAT LOOP II(<METAMORPHOSIS>)
      Serial.println("To REPEAT <METAMORPHOSIS> press R (CMD CODE: 81) :");
      Serial.println("To EXIT   <METAMORPHOSIS> press E (CMD CODE: 80) :");
      
      while (Serial.available() == 0) {};
      user_input_string = Serial.readString();
      
      Serial.print("[ USER INPUT ]"); Serial.print("   ->   "); Serial.println(user_input_string);
      
      if( ( strcmp(user_input_string.c_str(),meta_cont)-nl_char_shifting == 0 ) )
      {
        Serial.println("Repeats METAMORPHOSIS...");
        user_input_int = CMD_CONT_META_EXEC;
      }
      else if( ( strcmp(user_input_string.c_str(),meta_exit)-nl_char_shifting == 0 ) )
      {
        Serial.println("Exits METAMORPHOSIS...");
        user_input_int = CMD_EXIT_META_EXEC;
      }
      else
      {
        Serial.println("[ WRONG INPUT ]");
      }
      
      for (int pseudo_cnt = 0; pseudo_cnt < TOTAL_PSEUDOS_CONNECTED; pseudo_cnt++) 
      {
        return_function_state = MASTER_SPI.continueMetaExecutionMaster( pseudoIDs[pseudo_cnt], ssPins,(byte)  user_input_int, &END_METAMORPHOSIS, &CURRENT_STATE[pseudo_cnt] );
        if (return_function_state)
        {
            // metamorphosis success
         
            // Master commands Slave to save current gloabal variables to its EEPROM
            if (END_METAMORPHOSIS)
            {
              Serial.print("[   MASTER:  ]"); Serial.print(" TALKED TO: [   PSEUDO: "); Serial.print(pseudoIDs[pseudo_cnt]); Serial.println("  ]   STATUS:  [    SAVED EEPROM     ]  SUCCESS");
              Serial.print("[   MASTER:  ]"); Serial.print(" TALKED TO: [   PSEUDO: "); Serial.print(pseudoIDs[pseudo_cnt]); Serial.println("  ]   STATUS:  [ EXITS METAMORPHOSIS ]  SUCCESS");      
            }
            else
            {
              Serial.print("[   MASTER:  ]"); Serial.print(" TALKED TO: [   PSEUDO: "); Serial.print(pseudoIDs[pseudo_cnt]); Serial.println("  ]   STATUS:  [ REPEATS METAMORPHOSIS ]  SUCCESS");
              // CHANGE ANATOMY Ci, just for demo
              if (anatomy_counter == 2)
              {
                anatomy_counter = 0;
              }
              else
              {
                anatomy_counter++;
              }
              Serial.print("Next ci: "); Serial.println(anatomy_counter);                   
            }
            
            // Torques off motor(must check that motor current state is locked!)

            // Powers off NANO
        }
        else
        {
          // metamorphosis error: 1. not appropriate state received (META_FINISHED or META_REPEAT) 2. not steppers locked
              Serial.print("[   MASTER:  ]"); Serial.print(" TALKED TO: [   PSEUDO: "); Serial.print(pseudoIDs[pseudo_cnt]); Serial.println("  ]   STATUS:  [      METAMORPHOSIS     ]  FAILED");        
        }
  
      }
    
    }// END IF META EXECUTION
    else // START IF META ERROR
    {
        // NOT READY
        Serial.println("META ERROR NOT READY YET...");        
    } // END IF META ERROR

   }  //  END while !END_META (controled by user input via continueMetaExecutionMaster)
   }  // END IF META MODE (controled by user input) 
  
  /*
   * III. <ACTION> -> Permisssible state from Metamorphosis Execution: META_FINISHED 
   */
  if( ( strcmp(user_input_string.c_str(),act_exec) - nl_char_shifting == 0 ) )
  {
  while( (END_METAMORPHOSIS) && (!END_ACTION) )
  {
    Serial.println("Begin ACTION...");

    END_ACTION = true;
  } //  END while !END_ACT (controled by user input but not ready yet)

  } // END IF ACT MODE (controled by user input)

  
  END_METAMORPHOSIS = false;
  END_ACTION 		= false;
  
  delay(500);
} // END LOOP  
