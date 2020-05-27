void metamorphosis_execution(PseudoSPIcommMetamorphicManipulator MASTER_SPI, bool END_METAMORPHOSIS, bool metaExecution, byte * desiredAnatomy, int * pseudoIDs, int *ssPins, byte * CURRENT_STATE, const int TOTAL_PSEUDOS_CONNECTED)
{

while( (!END_METAMORPHOSIS) ) {

Serial.println("Begin METAMORPHOSIS...");
/*
    * II.1 READ INITIAL STATE OF ALL PSEUDOS CONNECTED 
    */
for (int pseudo_cnt = 0; pseudo_cnt < TOTAL_PSEUDOS_CONNECTED; pseudo_cnt++) 
{
    
        return_function_state = MASTER_SPI.readCurrentStateMaster(pseudoIDs[pseudo_cnt], ssPins, &CURRENT_STATE[pseudo_cnt]);
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
        return_function_state = MASTER_SPI.setGoalPositionMaster(pseudoIDs[pseudo_cnt], ssPins, &desiredAnatomy[pseudo_cnt], &CURRENT_STATE[pseudo_cnt] );
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
                Serial.print("[   MASTER:  ]"); Serial.print(" TALKED TO: [   PSEUDO: "); Serial.print(pseudoIDs[pseudo_cnt]); Serial.println("  ]   STATUS:  [ REPEATS METAMORPHOSIS ]  START");
                // Repeats Anatomy metamorphosis for the new Ci given by user
                for (size_t id_count = 0; id_count < TOTAL_PSEUDOS_CONNECTED; id_count++)
                {
                    Serial.print("Give desired Ci for Pseudojoint["); Serial.print(id_count+1); Serial.println("] :");
                    while (Serial.available() == 0) {};
                    desiredAnatomy[id_count] = (byte) Serial.read();
                }
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

} // IF metaExecution

} // WHILE

}