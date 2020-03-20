/*
 *  Local Git Directory: ~/Arduino/MetaLunokhod101_Development/SPI/using_ISR_simple_complete/using_ISR_simple_complete_pseudo
 *  Hard linked using  : ln ~/Arduino/SPI/using_ISR_simple_complete/using_ISR_simple_complete_pseudo/ISR.ino ./ISR.ino
 */
/*
 *  ISR always listens to MASTERS bytes sent!
 *  Accordingly changes flags for main loop code to execute!
 */

ISR (SPI_STC_vect)
{
  byte c = SPDR;                  // BYTE sent from MASTER 

  //Serial.print("command in ISR = "); Serial.println(c);
  
  switch (c)
  {
      // ALL AVAILABLE COMMANDS MUST BE CHECKED
      case 0:
        command = c;
        SPDR = 0;
        break;
        
       case CMD_CONNECT:
        // ISR -> loop
        CONNECT2MASTER = true;        // change flag for loop execution 
        
        // ISR <- loop      
        if(!connected2master)         // read flag from loop that relates to new state
        {
            SPDR = IS_TALKING;
        }
        else
        {
            SPDR = slaveID;            // (exception) MUST return Slave ID to MASTER NOT STATE!
            CONNECT2MASTER = false;    // change ISR to loop flag to stop loop execution  
        }
        break; 
            
      case CMD_GIVE_CS:
        // ISR -> loop
        GIVE_CS = true;               // change flag for loop execution 
        
        // ISR <- loop      
        if(!current_state_sent)       // read flag from loop that relates to new state
        {
            SPDR = IS_TALKING;
        }
        else
        {
            SPDR = motor_new_state;
            GIVE_CS = false;          // change ISR to loop flag to stop loop execution  
        }
        break;

      case c1 ... c13:                // this case is when user sets goal position
        // ISR -> loop
        SET_GOAL_POS = true;

        goal_ci = c;
        
        // ISR <- loop  
        if(!goal_position_set)        // read flag from loop that relates to new state
        {
            SPDR = IS_TALKING;
        }
        else
        {
            SPDR = motor_new_state;
            SET_GOAL_POS = false;          // change ISR to loop flag to stop loop execution  
        }            
        break;

      case CMD_MOVE:

        // write flag to execute loop commands
        MOVE_MOTOR = true;
        
        // read flag from loop that relate to motor status
        if(!motor_finished)
        {
            SPDR = IS_MOVING;
        }
        else
        {
            SPDR = motor_new_state;
            MOVE_MOTOR = false;           
        }
        break;
        
      case CMD_LOCK:
        LOCK_MOTOR = true;
        
        // read flag from loop that relate to motor status
        if(!motor_locked)
        {
            SPDR = IS_TALKING;
        }
        else
        {
            SPDR = motor_new_state;
            LOCK_MOTOR = false;           
        }
        break;        

      case CMD_UNLOCK:
        UNLOCK_MOTOR = true;
        
        // read flag from loop that relate to motor status
        if(!motor_unlocked)
        {
            SPDR = IS_TALKING;
        }
        else
        {
            SPDR = motor_new_state;
            UNLOCK_MOTOR = false;           
        }
        break; 

      case CMD_EXIT_META_EXEC:
        SAVE_GLOBALS_TO_EEPROM = true;

        if (!globals_saved_to_eeprom)
        {
            SPDR = IS_TALKING;
        }
        else
        {
            SPDR = motor_new_state;
            SAVE_GLOBALS_TO_EEPROM = false;
        }
        break;

      case CMD_CONT_META_EXEC:
        INDICATE_META_REPEATS = true;

        if (!leds_indicated_meta_repeats)
        {
            SPDR = IS_TALKING;
        }
        else
        {
            SPDR = motor_new_state;
            INDICATE_META_REPEATS = false;
        }
        break;
        
      default:
        break;
  } //switch

} //ISR
