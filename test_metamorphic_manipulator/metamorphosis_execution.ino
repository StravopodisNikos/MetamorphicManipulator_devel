bool metamorphosis_execution(PseudoSPIcommMetamorphicManipulator MASTER_SPI, bool metaExecution, byte * desiredAnatomy, int * pseudoIDs, int *ssPins, byte * CURRENT_STATE, int pseudoJoint_to_set_index, const int TOTAL_PSEUDOS_CONNECTED)
{
bool return_function_state;

if(metaExecution){
      
      for (int pseudo_cnt = 0; pseudo_cnt < TOTAL_PSEUDOS_CONNECTED; pseudo_cnt++) 
      {
        /*
         * II.2 IF CURRENT_STATE = LOCKED => SET GOAL POSITION
         */
        return_function_state = MASTER_SPI.setGoalPositionMaster(pseudoIDs[pseudo_cnt], ssPins, &desiredAnatomy[pseudoJoint_to_set_index], &CURRENT_STATE[pseudo_cnt] );
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

}

}