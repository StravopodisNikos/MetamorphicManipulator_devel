/*
 *  Local Git Directory: ~/Arduino/MetaLunokhod101_Development/SPI/using_ISR_simple_complete/using_ISR_simple_complete_pseudo
 *  Hard linked using  : ln ~/Arduino/SPI/using_ISR_simple_complete/using_ISR_simple_complete_pseudo/using_ISR_simple_complete_pseudo.ino ./using_ISR_simple_pseudo.ino
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
byte current_slave_state;

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

//  STATES INFO:  LOOP->ISR (SLAVE->MASTER)
volatile bool motor_finished        = false;
volatile bool motor_locked          = false;
volatile bool motor_unlocked        = false;
volatile bool current_state_sent    = false;
volatile bool connected2master      = false;
volatile bool goal_position_set     = false;

volatile byte motor_new_state = STATE_LOCKED;       // Initial state assumed for tests
volatile byte goal_ci;
volatile byte slaveID;

/*
 *   MOTOR MOVEMENT VARIABLES
 */
uint32_t  currentDirStatusPseudo  = HIGH;
int  currentMoveRelPseudo         = 100;
int  currentAbsPosPseudo          = PI/2;
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
  
  SPCR |= _BV(SPE);           // turn on SPI in slave mode

  SPCR |= _BV(SPIE);          // turn on interrupts

  digitalWrite(TXled_Pin,LOW); digitalWrite(RXled_Pin,LOW);

  //SLAVE1_SPI.setupEEPROMslave( pseudoID, PI/2, -PI/2, 0.2617994);   // LAST EXECUTION Wed 18.3.2020

}  // end of setup


void loop (void)
{

  if (digitalRead (ssPins[0]) == HIGH)
  {
    Serial.println("RESET VALUES");
    command = 0;
    motor_finished = false;
    motor_locked = false;
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
        // Calls function setGoalPositionSlave
        return_function_state = SLAVE1_SPI.setGoalPositionSlave( &goal_ci, &RELATIVE_STEPS_TO_MOVE, &motor_new_state );
        goal_position_set = true;
        Serial.print("RELATIVE_STEPS_TO_MOVE = "); Serial.println(RELATIVE_STEPS_TO_MOVE); 
        // here motor_new_state is returned from the function call !!!
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
        Serial.print("state when lock_motor = "); Serial.println(motor_new_state);
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
        Serial.print("state when unlock_motor = "); Serial.println(motor_new_state);
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

  delay(100);
}  // end of loop
