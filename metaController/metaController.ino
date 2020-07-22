/*
 *  Local Git Directory: ~/Arduino/MetaLunokhod101_Development/SPI/using_ISR_simple_complete/ using_ISR_simple_complete_master
 *  Hard linked using  : ln ~/Arduino/SPI/using_ISR_simple_complete/using_ISR_simple_complete_master/using_ISR_simple_complete_master.ino ./using_ISR_simple_complete_master.ino
 *  
 *  Update on Sun. 28.6.20: This file was converted to metaController from using_ISR_simple_complete_master
 */
// Used Libraries
#include <stdlib.h>
#include <stdio.h>
//#include <SPI.h>
#include "PseudoSPIcommMetamorphicManipulator.h"
#include <definitions.h>                            // Includes definitions of control table variables addresses/data lengths/ communication/ protocols
#include <motorIDs.h>                               // Includes motor IDs as set using Dynamixel Wizard

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
byte desiredAnatomy[nPseudoJoints];
byte anatomy_counter  = 0;                         // just to check repeat mode

volatile byte CURRENT_STATE_MASTER[sizeof(pseudoIDs)];        // empty states initialization array
volatile byte CURRENT_ANATOMY[sizeof(pseudoIDs)];
volatile byte CURRENT_Ci_IDENTITY;
volatile byte current_ci_returned;
bool META_MODES[sizeof(pseudoIDs)];           // empty pseudo mode initialization array
bool META_EXECS[sizeof(pseudoIDs)];           // empty pseudo mode-exec initialization array

/*
 * CONSTRUCT CLASS OBJECT FOR SPI COMMUNICATION BETWEEN DEVICES
 */
PseudoSPIcommMetamorphicManipulator MASTER_SPI(Tx, masterID, statusLED_Pin,  MOSI_NANO, MISO_NANO, SCK_NANO, TXled_Pin, RXled_Pin, ssPins);

/*
 * FLAGS USED TO CONTROL LOOPS
 */
bool END_METAMORPHOSIS;                      
bool END_ACTION;
bool END_HOME;
bool metaExecution;                           
bool homeExecution;                           

/*
 * EXTERN LIBRARY DEFINED VARIABLES
 */
bool return_function_state;
byte state_receive_from_slave;
unsigned long time_now_micros;
/*
 * USER SERIAL INPUT VARIABLES
 */
String user_input_string;
int user_input_int;
const char * meta_exec = "MET";
const char * home_exec = "HOME";
const char * act_exec  = "ACT";
const char * meta_cont = "R";
const char * meta_exit = "E";
const char * YES = "Y";
const char * NO = "N";
int nl_char_shifting   = 10;
const byte numChars = 32;
char receivedChars[numChars];   // an array to store the received data

boolean newData = false;

int dataNumber = 0;             // new for this version

void setup (void)
{
  Serial.begin (SERIAL_BAUDRATE);
  
  Serial.println("STARTING MASTER...");
  
// MASTER PINMODE 
  pinMode(SCK_MASTER, OUTPUT);
  pinMode(MOSI_MASTER, OUTPUT);
  pinMode(MISO_MASTER, INPUT);
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
        //MASTER_SPI.statusLEDblink(2, 500);
        Serial.print("[   MASTER:  ]"); Serial.print(" CONNECTED TO: [   PSEUDO: "); Serial.print(pseudoIDs[pseudo_cnt]); Serial.println("  ]   SUCCESS");
    }
    else
    {
        //MASTER_SPI.statusLEDblink(4, 250);
        Serial.print("[   MASTER:  ]"); Serial.print(" CONNECTED TO: [   PSEUDO: "); Serial.print(pseudoIDs[pseudo_cnt]); Serial.println("  ]   FAILED");
    }
  } 
  
  //digitalWrite(TXled_Pin,LOW); digitalWrite(RXled_Pin,LOW);

  /*
   *  Read current Anatomy
   */
    // Read current anatomy -> saved in byte array: CURRENT_ANATOMY
    Serial.println(F("[   MASTER:  ]  EXTRACTING CURRENT ANATOMY..."));

    for (int pseudo_cnt = 0; pseudo_cnt < TOTAL_PSEUDOS_CONNECTED; pseudo_cnt++) 
    {
        return_function_state = MASTER_SPI.readCurrentAnatomyMaster(pseudoIDs[pseudo_cnt], ssPins, &CURRENT_ANATOMY[pseudo_cnt], &CURRENT_Ci_IDENTITY);
        if (return_function_state)
        {
            Serial.print("[   MASTER:  ]"); Serial.print(" TALKED TO: [   PSEUDO: "); Serial.print(pseudoIDs[pseudo_cnt]); Serial.println("  ]   STATUS:  [  READ CURRENT Ci  ]  SUCCESS");
            Serial.print("[   MASTER:  ]"); Serial.print(" READ FROM: [   PSEUDO: "); Serial.print(pseudoIDs[pseudo_cnt]); Serial.print("  ]       CP:  ["); Serial.print(CURRENT_ANATOMY[pseudo_cnt]); Serial.println("  ]");
        }
        else
        {
            Serial.print("[   MASTER:  ]"); Serial.print(" TALKED TO: [   PSEUDO: "); Serial.print(pseudoIDs[pseudo_cnt]); Serial.println("  ]   STATUS:  [  READ CURRENT Ci  ]  FAILED");
        }
    }

    /*
     *  Read initial state of all pseudos connected
     */
     for (int pseudo_cnt = 0; pseudo_cnt < TOTAL_PSEUDOS_CONNECTED; pseudo_cnt++) 
     {
          return_function_state = MASTER_SPI.readCurrentStateMaster(pseudoIDs[pseudo_cnt], ssPins, &CURRENT_STATE_MASTER[pseudo_cnt]);
          if (return_function_state)
          {
              //MASTER_SPI.statusLEDblink(2, 500);
              Serial.print("[   MASTER:  ]"); Serial.print(" TALKED TO: [   PSEUDO: "); Serial.print(pseudoIDs[pseudo_cnt]); Serial.println("  ]   STATUS:  [  READY FOR METAMORPHOSIS  ]  SUCCESS");
          }
          else
          {
              //MASTER_SPI.statusLEDblink(4, 250);
              Serial.print("[   MASTER:  ]"); Serial.print(" TALKED TO: [   PSEUDO: "); Serial.print(pseudoIDs[pseudo_cnt]); Serial.println("  ]   STATUS:  [  READY FOR METAMORPHOSIS  ]  FAILED");
          }
      }
      /*
       *  Set flags to start Metamorphosis
       */
       END_METAMORPHOSIS = false;
       END_ACTION = false;
       END_HOME = false;
       
       metaExecution = true;
}  // end of setup



void loop (void)
{

  /*
   * I. User must specifu operating MODE to start - > ONLY <MET> TO ADVANCE!
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
    Serial.println("BEGIN METAMORPHOSIS...");

  /*
   *  II.1 Set goal anatomy
   */
    Serial.println("[   MASTER:  ]  SETTING NEW ANATOMY");
  
    // III.a.2.1 Read currentAnatomy and Construct desiredAnatomy
   /* for (int pseudo_cnt = 0; pseudo_cnt < TOTAL_PSEUDOS_CONNECTED; pseudo_cnt++) 
    {
        return_function_state = MASTER_SPI.readCurrentAnatomyMaster(pseudoIDs[pseudo_cnt], ssPins, CURRENT_ANATOMY);
        if (return_function_state)
        {
            Serial.print("[   MASTER:  ]"); Serial.print(" TALKED TO: [   PSEUDO: "); Serial.print(pseudoIDs[pseudo_cnt]); Serial.println("  ]   STATUS:  [  READ CURRENT Ci  ]  SUCCESS");
            Serial.print("[   MASTER:  ]"); Serial.print(" READ FROM: [   PSEUDO: "); Serial.print(pseudoIDs[pseudo_cnt]); Serial.print("  ]       CP:  ["); Serial.print(CURRENT_ANATOMY[pseudo_cnt]); Serial.println("  ]");
        }
        else
        {
            Serial.print("[   MASTER:  ]"); Serial.print(" TALKED TO: [   PSEUDO: "); Serial.print(pseudoIDs[pseudo_cnt]); Serial.println("  ]   STATUS:  [  READ CURRENT Ci  ]  FAILED");
        }
    }*/
    
    for (int pseudo_cnt = 0; pseudo_cnt < TOTAL_PSEUDOS_CONNECTED; pseudo_cnt++)
    {
        Serial.print("SET GOAL Ci FOR PSEUDO: [    "); Serial.print(pseudo_cnt+1); Serial.println("    ] :");
        while (Serial.available() == 0) {};
        desiredAnatomy[pseudo_cnt] = Serial.parseInt();
        Serial.print("[ USER INPUT ]"); Serial.print("   ->   "); Serial.println(desiredAnatomy[pseudo_cnt]);
     }    

    /*
     * II.2 READ INITIAL STATE OF ALL PSEUDOS CONNECTED 
     */
    for (int pseudo_cnt = 0; pseudo_cnt < TOTAL_PSEUDOS_CONNECTED; pseudo_cnt++) 
    {
      //Serial.println(CURRENT_STATE_MASTER[pseudo_cnt]);
      return_function_state = MASTER_SPI.readCurrentStateMaster(pseudoIDs[pseudo_cnt], ssPins, &CURRENT_STATE_MASTER[pseudo_cnt]);
      if (return_function_state)
      {
          //MASTER_SPI.statusLEDblink(2, 500);
          Serial.print("[   MASTER:  ]"); Serial.print(" TALKED TO: [   PSEUDO: "); Serial.print(pseudoIDs[pseudo_cnt]); Serial.println("  ]   STATUS:  [  READY FOR METAMORPHOSIS  ]  SUCCESS");
          metaExecution = true;      
      }
      else
      {
          //MASTER_SPI.statusLEDblink(4, 250);
          Serial.print("[   MASTER:  ]"); Serial.print(" TALKED TO: [   PSEUDO: "); Serial.print(pseudoIDs[pseudo_cnt]); Serial.println("  ]   STATUS:  [  READY FOR METAMORPHOSIS  ]  FAILED");
          metaExecution = false;
      }
      
    } // END FOR READ INITIAL STATE
    
    // NOW IF ALL META_FINISHED METAMORPHOSIS CONTINUES PSEUDO PER PSEUDO FOR STEPS 3-6! (NOT STEP BY STEP)
    if(metaExecution){
      for (int pseudo_cnt = 0; pseudo_cnt < TOTAL_PSEUDOS_CONNECTED; pseudo_cnt++) 
      {
        /*
         * II.3 IF CURRENT_STATE_MASTER = LOCKED => SET GOAL POSITION
         */
        Serial.print("[INFO]:   [   MASTER:  ]"); Serial.print(" TALKED TO: [   PSEUDO: "); Serial.print(pseudoIDs[pseudo_cnt]); Serial.print("  ]   SETS NEW GP:  [   "); Serial.print(desiredAnatomy[pseudo_cnt]); Serial.println("      ]");
        Serial.print("[INFO]:   [   MASTER:  ]"); Serial.print(" TALKED TO: [   PSEUDO: "); Serial.print(pseudoIDs[pseudo_cnt]); Serial.print("  ]   CURRENT  Ci:  [   "); Serial.print(CURRENT_ANATOMY[pseudo_cnt]); Serial.println("      ]");

        return_function_state = MASTER_SPI.setGoalPositionMaster(pseudoIDs[pseudo_cnt], ssPins, &desiredAnatomy[pseudo_cnt], &CURRENT_STATE_MASTER[pseudo_cnt] );
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
         * II.4 IF CURRENT_STATE_MASTER = READY => UNLOCK  
         */
        return_function_state = MASTER_SPI.unlockPseudoMaster(pseudoIDs[pseudo_cnt], ssPins, &CURRENT_STATE_MASTER[pseudo_cnt]);
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
         * II.5 IF CURRENT_STATE_MASTER = UNLOCKED => MOVE  
         */
        return_function_state = MASTER_SPI.movePseudoMaster(pseudoIDs[pseudo_cnt], ssPins, &CURRENT_STATE_MASTER[pseudo_cnt]);
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
         * II.6 IF CURRENT_STATE_MASTER = IN_POSITION => LOCK  
         */   
        return_function_state = MASTER_SPI.lockPseudoMaster(pseudoIDs[pseudo_cnt], ssPins, &CURRENT_STATE_MASTER[pseudo_cnt]);
        if (return_function_state)
        {
            //MASTER_SPI.statusLEDblink(2, 500);
            Serial.print("[   MASTER:  ]"); Serial.print(" TALKED TO: [   PSEUDO: "); Serial.print(pseudoIDs[pseudo_cnt]); Serial.println("  ]   STATUS:  [     LOCKED     ]  SUCCESS");

            // IF LOCKED SUCCESS CHANGE NEW POSITION
            CURRENT_ANATOMY[pseudo_cnt] = desiredAnatomy[pseudo_cnt];
        } 
        else
        {
            //MASTER_SPI.statusLEDblink(4, 250);
            Serial.print("[   MASTER:  ]"); Serial.print(" TALKED TO: [   PSEUDO: "); Serial.print(pseudoIDs[pseudo_cnt]); Serial.println("  ]   STATUS:  [     LOCKED     ]  FAILED");
        }

        Serial.print("State before save EEPROM:"); Serial.println(CURRENT_STATE_MASTER[pseudo_cnt]);

        return_function_state = MASTER_SPI.saveEEPROMsettingsMaster(pseudoIDs[pseudo_cnt], ssPins, &CURRENT_STATE_MASTER[pseudo_cnt]); 
        if (return_function_state)
        {
           Serial.print("[   MASTER:  ]"); Serial.print(" TALKED TO: [   PSEUDO: "); Serial.print(pseudoIDs[pseudo_cnt]); Serial.println("  ]   STATUS:  [   SAVED EEPROM   ]  SUCCESS");
        } 
        else
        {
            Serial.print("[   MASTER:  ]"); Serial.print(" TALKED TO: [   PSEUDO: "); Serial.print(pseudoIDs[pseudo_cnt]); Serial.println("  ]   STATUS:  [   SAVED EEPROM   ]  FAILED");
        }
    
      }
    
    metaExecution = false;
    }// END IF META EXECUTION
    else // START IF META ERROR
    {
      
        // NOT READY
        Serial.println("META ERROR NOT READY YET...");
                
    } // END IF META ERROR

      END_METAMORPHOSIS = true;
      END_ACTION        = false;
      END_HOME          = false;
   }  //  END while !END_META
   
   }  // END IF META MODE (controled by user input) 
  
  /*
   * III. <ACTION> -> Permisssible state from Metamorphosis Execution: META_FINISHED 
   */
  if( ( strcmp(user_input_string.c_str(),act_exec) - nl_char_shifting == 0 ) )
  {
  while( (END_METAMORPHOSIS) && (!END_ACTION) )
  {
    Serial.println("Begin ACTION...");
    delay(1000);
    
    END_METAMORPHOSIS = false;
    END_ACTION        = true;
    END_HOME          = false;
  } //  END while !END_ACT (controled by user input but not ready yet)

  } // END IF ACT MODE (controled by user input)

  /*
   * IV. <HOME> -> HOMES ALL PSEUDOS CONNECTED
   */
  if( ( strcmp(user_input_string.c_str(),home_exec)-nl_char_shifting == 0 ) )
  {
        Serial.println("BEGIN HOMING...");
        
        for (int pseudo_cnt = 0; pseudo_cnt < TOTAL_PSEUDOS_CONNECTED; pseudo_cnt++) 
        {
        return_function_state = MASTER_SPI.readCurrentStateMaster(pseudoIDs[pseudo_cnt], ssPins, &CURRENT_STATE_MASTER[pseudo_cnt]);
        if (return_function_state)
        {
            //MASTER_SPI.statusLEDblink(2, 500);
            Serial.print("[   MASTER:  ]"); Serial.print(" TALKED TO: [   PSEUDO: "); Serial.print(pseudoIDs[pseudo_cnt]); Serial.println("  ]   STATUS:  [  READY FOR HOMING  ]  SUCCESS");
            homeExecution = true;      
        }
        else
        {
            //MASTER_SPI.statusLEDblink(4, 250);
            Serial.print("[   MASTER:  ]"); Serial.print(" TALKED TO: [   PSEUDO: "); Serial.print(pseudoIDs[pseudo_cnt]); Serial.println("  ]   STATUS:  [  READY FOR HOMING  ]  FAILED");
            homeExecution = false;
        }
        
        } // END FOR READ STATE

        if(homeExecution){
          for (int pseudo_cnt = 0; pseudo_cnt < TOTAL_PSEUDOS_CONNECTED; pseudo_cnt++) 
          {   
  
              return_function_state = MASTER_SPI.setPreHomePositionStateMaster(pseudoIDs[pseudo_cnt], ssPins, &CURRENT_STATE_MASTER[pseudo_cnt]);
              if (return_function_state)
              {
                  Serial.print("[   MASTER:  ]"); Serial.print(" TALKED TO: [   PSEUDO: "); Serial.print(pseudoIDs[pseudo_cnt]); Serial.println("  ]   STATUS:  [      READY     ]  SUCCESS");
              }
              else
              {
                  Serial.print("[   MASTER:  ]"); Serial.print(" TALKED TO: [   PSEUDO: "); Serial.print(pseudoIDs[pseudo_cnt]); Serial.println("  ]   STATUS:  [      READY     ]  FAILED");
              }   
  
              return_function_state = MASTER_SPI.unlockPseudoMaster(pseudoIDs[pseudo_cnt], ssPins, &CURRENT_STATE_MASTER[pseudo_cnt]);
              if (return_function_state)
              {
                  Serial.print("[   MASTER:  ]"); Serial.print(" TALKED TO: [   PSEUDO: "); Serial.print(pseudoIDs[pseudo_cnt]); Serial.println("  ]   STATUS:  [    UNLOCKED    ]  SUCCESS");
              }
              else
              {
                  Serial.print("[   MASTER:  ]"); Serial.print(" TALKED TO: [   PSEUDO: "); Serial.print(pseudoIDs[pseudo_cnt]); Serial.println("  ]   STATUS:  [    UNLOCKED    ]  FAILED");
              }   
              
              return_function_state = MASTER_SPI.go2HomePositionMaster(pseudoIDs[pseudo_cnt], ssPins, &CURRENT_STATE_MASTER[pseudo_cnt]);
              if (return_function_state)
              {
                  Serial.print("[   MASTER:  ]"); Serial.print(" TALKED TO: [   PSEUDO: "); Serial.print(pseudoIDs[pseudo_cnt]); Serial.println("  ]   STATUS:  [     HOMED     ]  SUCCESS");
              }
              else
              {
                  Serial.print("[   MASTER:  ]"); Serial.print(" TALKED TO: [   PSEUDO: "); Serial.print(pseudoIDs[pseudo_cnt]); Serial.println("  ]   STATUS:  [     HOMED     ]  FAILED");
              }
                          
              return_function_state = MASTER_SPI.lockPseudoMaster(pseudoIDs[pseudo_cnt], ssPins, &CURRENT_STATE_MASTER[pseudo_cnt]);
              if (return_function_state)
              {
                  Serial.print(F("[   MASTER:  ]")); Serial.print(F(" TALKED TO: [   PSEUDO: ")); Serial.print(pseudoIDs[pseudo_cnt]); Serial.println(F("  ]   STATUS:  [     LOCKED     ]  SUCCESS"));
  
                  // IF LOCKED SUCCESS CHANGE NEW POSITION
                  CURRENT_ANATOMY[pseudo_cnt] = home_ci;
              } 
              else
              {
                  Serial.print(F("[   MASTER:  ]")); Serial.print(F(" TALKED TO: [   PSEUDO: ")); Serial.print(pseudoIDs[pseudo_cnt]); Serial.println(F("  ]   STATUS:  [     LOCKED     ]  FAILED"));
              }
  
              Serial.print("State before save EEPROM:"); Serial.println(CURRENT_STATE_MASTER[pseudo_cnt]);
              return_function_state = MASTER_SPI.saveEEPROMsettingsMaster(pseudoIDs[pseudo_cnt], ssPins, &CURRENT_STATE_MASTER[pseudo_cnt]);
              if (return_function_state)
              {
                  Serial.print(F("[   MASTER:  ]")); Serial.print(F(" TALKED TO: [   PSEUDO: ")); Serial.print(pseudoIDs[pseudo_cnt]); Serial.println(F("  ]   STATUS:  [   SAVED EEPROM   ]  SUCCESS"));
              } 
              else
              {
                  Serial.print(F("[   MASTER:  ]")); Serial.print(F(" TALKED TO: [   PSEUDO: ")); Serial.print(pseudoIDs[pseudo_cnt]); Serial.println(F("  ]   STATUS:  [   SAVED EEPROM   ]  FAILED"));
              }
              
          }

          homeExecution = false;
        }
        
        END_METAMORPHOSIS = false;
        END_ACTION        = false;
        END_HOME          = true;
  } // END IF HOME MODE


  END_METAMORPHOSIS = false;
  END_ACTION        = false;
  END_HOME          = false;
    
  //delay(250);
  
  time_now_micros = micros();
  while(micros() < time_now_micros + 500){}
  
} // END LOOP  
