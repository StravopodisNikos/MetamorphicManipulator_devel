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
 // COMMAND FLAGS:     ISR->LOOP (MASTER->SLAVE)
volatile bool MOVE_MOTOR;
volatile bool LOCK_MOTOR;
volatile bool UNLOCK_MOTOR;
volatile bool HOME_MOTOR;
volatile bool GIVE_CS;
volatile bool GIVE_CP;
volatile bool CONNECT2MASTER;
volatile bool SET_GOAL_POS;
volatile bool SAVE_GLOBALS_TO_EEPROM;
volatile bool INDICATE_META_REPEATS;
volatile bool SAVE_EEPROM;

//  STATES INFO:  LOOP->ISR (SLAVE->MASTER)
volatile bool motor_finished        = false;
volatile bool motor_locked          = false;
volatile bool motor_unlocked        = false;
volatile bool motor_homed           = false;
volatile bool current_state_sent    = false;
volatile bool current_position_sent = false;
volatile bool connected2master      = false;
volatile bool goal_position_set     = false;
volatile bool globals_saved_to_eeprom     = false;
volatile bool leds_indicated_meta_repeats = false;
volatile bool home_saved_to_eeprom     = false;

volatile bool homingHallActivated   = false;
volatile bool limitHallActivated    = false;

// bytes accessed by loop + ISR
volatile byte motor_new_state;
volatile byte motor_current_ci;
volatile byte CURRENT_Ci_IDENTITY;       
volatile byte goal_ci;
volatile byte slaveID;

/*
 *   MOTOR MOVEMENT GLOBAL VARIABLES - ALWAYS INITIALIZED BY READING EEPROM AT setup()
 */
byte  currentDirStatusPseudo        = LOW;
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

  Serial.println("STARTING SLAVE...");
  
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
  pinMode(RELAY_lock_Pin, OUTPUT);
  pinMode(RELAY_lock_Pin2, OUTPUT);
  
  pinMode(hallSwitch_Pin, INPUT_PULLUP);
  pinMode(pseudoLimitSwitch_Pin, INPUT_PULLUP);
  
  digitalWrite(stepPin_NANO, LOW);
  digitalWrite(enabPin_NANO, LOW);
  digitalWrite(dirPin_NANO, LOW);

  digitalWrite(RELAY_lock_Pin, HIGH);                   //  remains locked when when NO connected 
  digitalWrite(RELAY_lock_Pin2, HIGH);                  //  remains locked when when NO connected

// SLAVE EXTERNAL INTERRUPTS CONFIGURATION
  //attachInterrupt(digitalPinToInterrupt(pseudoLimitSwitch_Pin), changePseudoDirInterrupt, RISING);
  //attachInterrupt(digitalPinToInterrupt(hallSwitch_Pin), beginHomingCalibrationInterrupt, RISING);
  
  homingHallActivated = false;
  limitHallActivated  = false;
  
// SLAVE INTERNAL ISR CONFIGURATION
  SPCR |= _BV(SPE);           // turn on SPI in slave mode

  SPCR |= _BV(SPIE);          // turn on interrupts
  //SPI.attachInterrupt();
  //digitalWrite(TXled_Pin,LOW); digitalWrite(RXled_Pin,LOW);
  
// READ SETTINGS FROM EEPROM
  //SLAVE1_SPI.setupEEPROMslave( pseudoID, PI/2, -PI/2, 0.2617994);   // LAST EXECUTION Wed 18.3.2020

  //SLAVE1_SPI.readEEPROMsettingsSlave( pseudoID, &motor_new_state ,  &currentAbsPosPseudo_ci,  &currentDirStatusPseudo, &currentAbsPosPseudo);

  //return_function_state = SLAVE1_SPI.setHomePositionSlave( &motor_new_state, &currentAbsPosPseudo, &currentAbsPosPseudo_ci, &currentDirStatusPseudo, &homingHallActivated, &limitHallActivated);
 
  //return_function_state = SLAVE1_SPI.saveEEPROMsettingsSlave( &motor_new_state, &currentAbsPosPseudo_ci , &currentDirStatusPseudo);

  //SLAVE1_SPI.readEEPROMsettingsSlave( pseudoID, &motor_new_state ,  &currentAbsPosPseudo_ci,  &currentDirStatusPseudo, &currentAbsPosPseudo);

}  // end of setup

void loop (void)
{

  for (size_t i = 0; i < sizeof(ssPins); i++)
  {
      if (digitalRead (ssPins[i]) == HIGH)
      {
          //Serial.println("RESET VALUES");
          //command = 0;
          connected2master = false;
          current_state_sent = false;
          goal_position_set = false;
          motor_finished = false;
          motor_locked = false;
          motor_unlocked = false;
          motor_homed    = false;
          globals_saved_to_eeprom = false;
          leds_indicated_meta_repeats = false;
          home_saved_to_eeprom = false;
          current_position_sent = false;
      }
  } 

  
  if ( CONNECT2MASTER )
  {
        // Calls function connectPseudoSlave()  that reads from EEPROM and return slave ID   
        slaveID  = SLAVE1_SPI.connectPseudoSlave();
        Serial.print("[   INFO   ]  Slave sent this ID to Master:"); Serial.println(slaveID); 
        
        if (slaveID == pseudoID)            // ID values must agree in order connection to be achieved
        {
          connected2master = true;          // Reading ID from EEPROM completed...
          Serial.println("[   INFO   ]  CONNECTED");
        }
        else
        {
          connected2master = false;          // Reading ID from EEPROM completed...
          Serial.println("[   INFO   ]  DISCONNECTED");
        }
        //delay(1);
        time_now_micros = micros();
        while(micros() < time_now_micros + 500){}
  }

  if ( GIVE_CS )
  {
        return_function_state = SLAVE1_SPI.readCurrentStateSlave(&motor_new_state);
        if (return_function_state)
        {
          current_state_sent = true;
          Serial.print("[   PSEUDO:"); Serial.print(pseudoID); Serial.print("   ]   [   CURRENT STATE:"); Serial.print(motor_new_state); Serial.println("   ]   ACCEPTED"); 
        }
        else
        {
          Serial.print("[   PSEUDO:"); Serial.print(pseudoID); Serial.print("   ]   [   CURRENT STATE:"); Serial.print(motor_new_state); Serial.println("   ]   DECLINED"); 
          current_state_sent = false;
        }
        time_now_micros = micros();
        while(micros() < time_now_micros + 500){}   }


  if ( GIVE_CP )
  {
        return_function_state = SLAVE1_SPI.readCurrentAnatomySlave(&motor_current_ci, &CURRENT_Ci_IDENTITY);
        if (return_function_state)
        {
          current_position_sent = true;
          Serial.print("[   PSEUDO:"); Serial.print(pseudoID); Serial.print("   ]   [   CURRENT POSITION:"); Serial.print(motor_current_ci); Serial.println("   ]   ACCEPTED"); 
        }
        else
        {
          Serial.print("[   PSEUDO:"); Serial.print(pseudoID); Serial.print("   ]   [   CURRENT POSITION:"); Serial.print(motor_current_ci); Serial.println("   ]   DECLINED"); 
          current_position_sent = false;
        }
        currentAbsPosPseudo_ci = motor_current_ci;
        time_now_micros = micros();
        while(micros() < time_now_micros + 500){}   }
   
  if ( SET_GOAL_POS )
  {
        //motor_new_state = STATE_LOCKED;
        // Calls function setGoalPositionSlave2
 
        noInterrupts();
        byte PSEUDO_GOAL_POSITION = goal_ci;
        interrupts();

        Serial.print("[INFO]:   [   PSEUDO:"); Serial.print(pseudoID); Serial.print("  ]   CURRENT  Ci:  [  "); Serial.print(currentAbsPosPseudo_ci); Serial.println("  ]");
        Serial.print("[INFO]:   [   PSEUDO:"); Serial.print(pseudoID); Serial.print("  ]   GOAL     Ci:  [  "); Serial.print(PSEUDO_GOAL_POSITION); Serial.println("  ]");
        
        return_function_state = SLAVE1_SPI.setGoalPositionSlave2( &PSEUDO_GOAL_POSITION, &currentAbsPosPseudo_ci, &RELATIVE_STEPS_TO_MOVE, &currentDirStatusPseudo, &motor_new_state );
        if (return_function_state)
        {
          goal_position_set = true;
          Serial.print("RELATIVE_STEPS_TO_MOVE = "); Serial.println(RELATIVE_STEPS_TO_MOVE); 
        }
        else
        {
          goal_position_set = false;
        }
        time_now_micros = micros();
        while(micros() < time_now_micros + 500){}  }

  if ( MOVE_MOTOR )
  {
        // Calls function movePseudoSlave
        return_function_state = SLAVE1_SPI.movePseudoSlave(&motor_new_state , &RELATIVE_STEPS_TO_MOVE, &limitHallActivated);
        if (return_function_state)
        {
          Serial.print("[   PSEUDO:"); Serial.print(pseudoID); Serial.print("   ]   [   CURRENT STATUS:"); Serial.print(STATE_IN_POSITION_STRING); Serial.println("   ]   SUCCESS");          
          motor_finished = true;          
        }
        else
        {
          Serial.print("[   PSEUDO:"); Serial.print(pseudoID); Serial.print("   ]   [   CURRENT STATUS:"); Serial.print(STATE_IN_POSITION_STRING); Serial.println("   ]   FAILED");          
          motor_finished = false;
        }
        time_now_micros = micros();
        while(micros() < time_now_micros + 500){}  }

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
        time_now_micros = micros();
        while(micros() < time_now_micros + 500){}  }

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
        time_now_micros = micros();
        while(micros() < time_now_micros + 500){}  }

  if ( HOME_MOTOR )
  {
        // Calls function lockPseudoSlave
        return_function_state = SLAVE1_SPI.setHomePositionSlave( &motor_new_state, &currentAbsPosPseudo, &currentAbsPosPseudo_ci, &currentDirStatusPseudo, &homingHallActivated, &limitHallActivated);
       
        if (return_function_state)
        {
          Serial.print("[   PSEUDO:"); Serial.print(pseudoID); Serial.print("   ]   [   CURRENT STATUS:"); Serial.print(STATE_HOMED_STRING); Serial.println("   ]   SUCCESS");          
          motor_homed = true;
          // here motor_new_state is returned from the function call !!!              
        }
        else
        {
          Serial.print("[   PSEUDO:"); Serial.print(pseudoID); Serial.print("   ]   [   CURRENT STATUS:"); Serial.print(STATE_HOMED_STRING); Serial.println("   ]   FAILED");     
          motor_homed = false;              
        }

        Serial.print("[ INFO ] CURRENT Ci: [  "); Serial.print(currentAbsPosPseudo_ci); Serial.println(" ]");
        time_now_micros = micros();
        while(micros() < time_now_micros + 500){}  }

  if ( SAVE_GLOBALS_TO_EEPROM )     // saves global variables to EEPROM and indicates meta_exits
  {
        // Calls function saveEEPROMsettingsSlave
        return_function_state = SLAVE1_SPI.saveEEPROMsettingsSlave(&motor_new_state, &currentAbsPosPseudo_ci , &currentDirStatusPseudo);

        if (return_function_state)
        {  
          // LED indicate that Slave saved glabals to EEPROM (TxRx, 2X1000)
          Serial.print("[   PSEUDO:"); Serial.print(pseudoID); Serial.print("   ]   [   CURRENT STATUS:"); Serial.print(EXIT_METAMORPHOSIS); Serial.println("   ]   SUCCESS");  
          globals_saved_to_eeprom = true;
        }
        else
        {
          Serial.print("[   PSEUDO:"); Serial.print(pseudoID); Serial.print("   ]   [   CURRENT STATUS:"); Serial.print(EXIT_METAMORPHOSIS); Serial.println("   ]   FAILED");  
          globals_saved_to_eeprom = false;
        }
        time_now_micros = micros();
        while(micros() < time_now_micros + 500){}  }

  if ( INDICATE_META_REPEATS )      // only indicates meta repeats, global not need to be saved - no fn executed
  {
        // LED indicate that Slave repeats (TxRx, 4X1000)
        //SLAVE1_SPI.txrxLEDSblink(3, 1500);
        //Serial.print("State here must be locked(100) : "); Serial.println(motor_new_state);
        return_function_state = SLAVE1_SPI.saveCurrentStateSlave(&motor_new_state);
        if (return_function_state)
        {
          leds_indicated_meta_repeats = true;
        }
        else
        {
          leds_indicated_meta_repeats = false;
        }
        time_now_micros = micros();
        while(micros() < time_now_micros + 500){}  }

  if ( SAVE_EEPROM )     
  {
        Serial.print("Here state must be 100:"); Serial.println(motor_new_state);
        return_function_state = SLAVE1_SPI.saveEEPROMsettingsSlave( &motor_new_state, &currentAbsPosPseudo_ci , &currentDirStatusPseudo);
        if (return_function_state)
        {
          home_saved_to_eeprom = true;
        }
        else
        {
          home_saved_to_eeprom = false;
        }
        time_now_micros = micros();
        while(micros() < time_now_micros + 500){}  }
/*
 * DEBUGGING MESSAGES
 */
 /*
  // Print new global variables
  Serial.println("[   SLAVE LOOP   ]   SUCCESS"); 
  Serial.println("-------------------------------------");
  Serial.print("[    CURRENT_POS_ci    ]    [   "); Serial.print(currentAbsPosPseudo_ci); Serial.println("     ]");
  Serial.print("[    CURRENT STATE     ]    [   "); Serial.print(motor_new_state); Serial.println("     ]");
  //Serial.print("SET_GOAL_POS must be 0: "); Serial.println(SET_GOAL_POS);
  
  Serial.println("[   CHECK IF BOOLS    ]   ");  
  Serial.println("-------------------------------------");
  Serial.print("CONNECT2MASTER :"); Serial.println(CONNECT2MASTER);
  Serial.print("GIVE_CS :"); Serial.println(GIVE_CS);
  Serial.print("SET_GOAL_POS :"); Serial.println(SET_GOAL_POS);
  Serial.print("MOVE_MOTOR :"); Serial.println(MOVE_MOTOR);
  Serial.print("LOCK_MOTOR :"); Serial.println(LOCK_MOTOR);
  Serial.print("UNLOCK_MOTOR :"); Serial.println(UNLOCK_MOTOR);
  Serial.print("SAVE_GLOBALS_TO_EEPROM :"); Serial.println(SAVE_GLOBALS_TO_EEPROM);
  Serial.print("INDICATE_META_REPEATS :"); Serial.println(INDICATE_META_REPEATS);   
 */
 
/*
 * WELCOME TO PARANOIA - MAKES SURE NO DUPLICATED BOOL VALUES GIVEN TO SLAVE
 */
  MOVE_MOTOR        = false;
  LOCK_MOTOR        = false;
  UNLOCK_MOTOR      = false;
  HOME_MOTOR        = false;
  GIVE_CS           = false;
  GIVE_CP           = false;
  CONNECT2MASTER    = false;
  SET_GOAL_POS      = false;
  SAVE_GLOBALS_TO_EEPROM  = false;
  INDICATE_META_REPEATS   = false;
  SAVE_EEPROM       = false;
  
  //delay(250);           // specify frequency slave receives/responds
  
  time_now_micros = micros();
  while(micros() < time_now_micros + 500){}
}  // end of loop

/*
 *  INTERRUPT FUNCTIONS USED
 */
void changePseudoDirInterrupt()
{
  limitHallActivated = true;
}

void beginHomingCalibrationInterrupt()
{
  homingHallActivated = true;
}
