/*
 *  Local Git Directory: ~/Arduino/MetaLunokhod101_Development/SPI/using_ISR_simple_complete/using_ISR_simple_complete_pseudo
 *  Hard linked using  : ln ~/Arduino/SPI/using_ISR_simple_complete/using_ISR_simple_complete_pseudo/using_ISR_simple_complete_pseudo.ino ./using_ISR_simple_complete_pseudo.ino
 */
#include <SPI.h>
#include "PseudoSPIcommMetamorphicManipulator.h"

/*
 * CONFIGURE all the system-dependent-arrays
 */
int pseudoID  = PSEUDO1_ID;                 // ID must change regarding pseudojoint!!
int ssPins[1] = {SSpinPseudo1};

/*
 * EXTERN LIBRARY DEFINED VARIABLES
 */
bool result;
bool return_function_state;

 unsigned long time_now_micros;
/* 
 *  Variables used in ISR
 */
 // COMMAND FLAGES:     ISR->LOOP (MASTER->SLAVE)
volatile byte command;
volatile bool MOVE_MOTOR            = false;
volatile bool LOCK_MOTOR            = false;
volatile bool UNLOCK_MOTOR          = false;
volatile bool GIVE_CS               = false;
volatile bool CONNECT2MASTER        = false;
volatile bool SET_GOAL_POS          = false;
volatile bool SAVE_GLOBALS_TO_EEPROM  = false;
volatile bool INDICATE_META_REPEATS   = false;

//  STATES INFO:  LOOP->ISR (SLAVE->MASTER)
volatile bool motor_finished        = false;
volatile bool motor_locked          = false;
volatile bool motor_unlocked        = false;
volatile bool current_state_sent    = false;
volatile bool connected2master      = false;
volatile bool goal_position_set     = false;
volatile bool globals_saved_to_eeprom     = false;
volatile bool leds_indicated_meta_repeats = false;

volatile byte motor_new_state;       
volatile byte goal_ci;
volatile byte slaveID;

/*
 *   MOTOR MOVEMENT GLOBAL VARIABLES - ALWAYS INITIALIZED BY READING EEPROM AT setup()
 */
byte  currentDirStatusPseudo;
int  currentMoveRelPseudo           = 0;    // can be initialized to 0 and change after 1st execution
int  currentAbsPosPseudo;
byte currentAbsPosPseudo_ci;
int  RELATIVE_STEPS_TO_MOVE;

int  theta_p_current_steps;
float theta_p_goal;
/*
 * CONSTRUCT CLASS OBJECT FOR SPI COMMUNICATION BETWEEN DEVICES
 */
PseudoSPIcommMetamorphicManipulator SLAVE1_SPI(Rx, pseudoID, statusLED_Pin,  MOSI_NANO, MISO_NANO, SCK_NANO, TXled_Pin, RXled_Pin, ssPins);

void setup (void)
{
  Serial.begin (115200);

// SLAVE PINMODE
  pinMode(SCK_NANO, OUTPUT);
  pinMode(MOSI_NANO, INPUT);
  pinMode(MISO_NANO, OUTPUT);
  for (size_t i = 0; i < sizeof(ssPins); i++)
  {
    pinMode(ssPins[i], INPUT);
  }  
// SLAVE PINMODE FOR STEPPER
  pinMode(stepPin_NANO, OUTPUT);
  pinMode(dirPin_NANO, OUTPUT);
  pinMode(enabPin_NANO, OUTPUT);
  pinMode(hallSwitch_Pin, INPUT);
  pinMode(RELAY_lock_Pin, OUTPUT);
  
  digitalWrite(stepPin_NANO, LOW);
  digitalWrite(enabPin_NANO, LOW);
  digitalWrite(dirPin_NANO, LOW);  
  
// SLAVE ISR CONFIGURATION
  SPCR |= _BV(SPE);           // turn on SPI in slave mode

  SPCR |= _BV(SPIE);          // turn on interrupts

  digitalWrite(TXled_Pin,LOW); digitalWrite(RXled_Pin,LOW);

  //SLAVE1_SPI.setupEEPROMslave( pseudoID, PI/2, -PI/2, 0.2617994);   // LAST EXECUTION Wed 18.3.2020

  //Serial.println("INITIAL STEPPER VALUES:");
  //Serial.print("motor_new_state        = "); Serial.println(motor_new_state);
  //Serial.print("currentDirStatusPseudo = "); Serial.println(currentDirStatusPseudo);
  //Serial.print("currentAbsPosPseudo    = "); Serial.println(currentAbsPosPseudo);
  
// HOMING only for Debugging => No lock/unlock considered!!!
/*  digitalWrite(dirPin_NANO, LOW);
  SLAVE1_SPI.setHomePositionSlave(&currentAbsPosPseudo, &currentAbsPosPseudo_ci);
  delay(1000);
  currentDirStatusPseudo = HIGH;
  digitalWrite(dirPin_NANO, currentDirStatusPseudo); 
  SLAVE1_SPI.setHomePositionSlave(&currentAbsPosPseudo, &currentAbsPosPseudo_ci);
  
// NOW THAT I KNOW THAT I AM AT HOME POSITION MUST WRITE TO EEPROM
  EEPROM.update(CD_EEPROM_ADDR, HIGH);
  EEPROM.update(CP_EEPROM_ADDR, 7);
  EEPROM.update(CS_EEPROM_ADDR, META_FINISHED);
// NOW I READ EEPROM AND INITIALIZE THE GLOBAL VARIABLES FROM HOME POSITION
//*/  
  SLAVE1_SPI.readEEPROMsettingsSlave(pseudoID, &motor_new_state ,  &currentAbsPosPseudo_ci,  &currentDirStatusPseudo, &currentAbsPosPseudo);

}  // end of setup


void loop (void)
{
///*
  if (digitalRead (ssPins[0]) == HIGH)
  {
    Serial.println("RESET VALUES");
    command = 0;
    //motor_finished = false;
    //motor_locked = false;
  }

  if ( CONNECT2MASTER )
  {
        // Calls function connectPseudoSlave()  that reads from EEPROM and return slave ID   
        slaveID  = SLAVE1_SPI.connectPseudoSlave();
        Serial.print("Slave sent this ID to Master:"); Serial.println(slaveID);
        
        // ID values must agree in order connaection to be achieved
        if (slaveID == pseudoID)
        {
          connected2master = true;          // Reading ID from EEPROM completed...
        }
        else
        {
          connected2master = false;          // Reading ID from EEPROM completed...
        }
        delay(1);
  }

  if ( GIVE_CS )
  {
        // Here just the current value of motor_new_state is sent
        Serial.print("[   PSEUDO:"); Serial.print(pseudoID); Serial.print("   ]   [   CURRENT STATUS:"); Serial.print(motor_new_state); Serial.println("   ]");
        current_state_sent = true;
  }

  if ( SET_GOAL_POS )
  {
        //motor_new_state = STATE_LOCKED;
        // Calls function setGoalPositionSlave
        return_function_state = SLAVE1_SPI.setGoalPositionSlave2( &goal_ci, &currentAbsPosPseudo_ci, &RELATIVE_STEPS_TO_MOVE, &currentDirStatusPseudo, &motor_new_state );
        if (return_function_state)
        {
          goal_position_set = true;
          Serial.print("RELATIVE_STEPS_TO_MOVE = "); Serial.println(RELATIVE_STEPS_TO_MOVE); 
        }
        else
        {
          goal_position_set = false;
        }
  }
  
  if ( MOVE_MOTOR )
  {
        // Calls function movePseudoSlave
        return_function_state = SLAVE1_SPI.movePseudoSlave(&motor_new_state , &RELATIVE_STEPS_TO_MOVE);
        if (return_function_state)
        {
          Serial.print("[   PSEUDO:"); Serial.print(pseudoID); Serial.println("   ]   [   CURRENT STATUS:"); Serial.print(STATE_IN_POSITION_STRING); Serial.println("   ]   SUCCESS");          
          motor_finished = true;          
        }
        else
        {
          Serial.print("[   PSEUDO:"); Serial.print(pseudoID); Serial.println("   ]   [   CURRENT STATUS:"); Serial.print(STATE_IN_POSITION_STRING); Serial.println("   ]   FAILED");          
          motor_finished = false;
        }
  }
  
  if ( LOCK_MOTOR )
  {
        // Calls function lockPseudoSlave
        return_function_state = SLAVE1_SPI.lockPseudoSlave(&motor_new_state );
       
        if (return_function_state)
        {
          Serial.print("[   PSEUDO:"); Serial.print(pseudoID); Serial.print("   ]   [   CURRENT STATUS:"); Serial.print(STATE_LOCKED_STRING); Serial.println("   ]   SUCCESS");          
          motor_locked = true;
          // here motor_new_state is returned from the function call !!!              
        }
        else
        {
          Serial.print("[   PSEUDO:"); Serial.print(pseudoID); Serial.print("   ]   [   CURRENT STATUS:"); Serial.print(STATE_LOCKED_STRING); Serial.println("   ]   FAILED");     
          motor_locked = false;              
        }        
  }

  if ( UNLOCK_MOTOR )
  {
        // Calls function unlockPseudoSlave
        return_function_state = SLAVE1_SPI.unlockPseudoSlave(&motor_new_state );
        
        if (return_function_state)
        {
          Serial.print("[   PSEUDO:"); Serial.print(pseudoID); Serial.print("   ]   [   CURRENT STATUS:"); Serial.print(STATE_UNLOCKED_STRING); Serial.println("   ]   SUCCESS");  
          motor_unlocked = true; 
          // here motor_new_state is returned from the function call !!!       
        }
        else
        {
          Serial.print("[   PSEUDO:"); Serial.print(pseudoID); Serial.print("   ]   [   CURRENT STATUS:"); Serial.print(STATE_UNLOCKED_STRING); Serial.println("   ]   FAILED");            
          motor_unlocked = false; 
        }           
  }

  if ( SAVE_GLOBALS_TO_EEPROM )     // saves global variables to EEPROM and indicates meta_exits
  {
        // Calls function saveEEPROMsettingsSlave
        //motor_new_state = META_FINISHED;
        //return_function_state = true;
        return_function_state = SLAVE1_SPI.saveEEPROMsettingsSlave(&motor_new_state, &currentAbsPosPseudo_ci , &currentDirStatusPseudo);

        if (return_function_state)
        {  
          // LED indicate that Slave saved glabals to EEPROM (TxRx, 2X1000)
          Serial.print("[   PSEUDO:"); Serial.print(pseudoID); Serial.print("   ]   [   CURRENT STATUS:"); Serial.print(EXIT_METAMORPHOSIS); Serial.println("   ]   SUCCESS");  
          //SLAVE1_SPI.txrxLEDSblink(2, 1000);
          globals_saved_to_eeprom = true;
        }
        else
        {
          Serial.print("[   PSEUDO:"); Serial.print(pseudoID); Serial.print("   ]   [   CURRENT STATUS:"); Serial.print(EXIT_METAMORPHOSIS); Serial.println("   ]   FAILED");  
          //SLAVE1_SPI.txrxLEDSblink(4, 500);
          globals_saved_to_eeprom = false;
        }
  }

  if ( INDICATE_META_REPEATS )      // only indicates meta repeats, global not need to be saved - no fn executed
  {
        // LED indicate that Slave repeats (TxRx, 4X1000)
        //SLAVE1_SPI.txrxLEDSblink(3, 1500);
        return_function_state = SLAVE1_SPI.repeatMetaSlave(&motor_new_state);
        if (return_function_state)
        {
          leds_indicated_meta_repeats = true;
        }
        else
        {
          leds_indicated_meta_repeats = false;
        }      
  }
//*/
  delay(100);           // specify frequency slave receives/responds
}  // end of loop
