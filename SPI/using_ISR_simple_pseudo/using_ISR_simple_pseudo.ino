#include <SPI.h>
#include "PseudoSPIcommMetamorphicManipulator.h"

// variables used in ISR
volatile byte command;
volatile bool MOVE_MOTOR;
volatile bool LOCK_MOTOR;

volatile bool motor_finished  = false;
volatile bool motor_locked    = false;

volatile int  motor_new_state = STATE_UNLOCKED;
int ssPins[1] = {SSpinPseudo1};

void setup (void)
{
  Serial.begin (115200);
  // have to send on master in, *slave out*
  pinMode(MISO, OUTPUT);
  pinMode(ssPins[0], INPUT);
  
  // turn on SPI in slave mode
  SPCR |= _BV(SPE);

  // turn on interrupts
  SPCR |= _BV(SPIE);
  
  digitalWrite(TXled_Pin,LOW); digitalWrite(RXled_Pin,LOW);
}  // end of setup


// SPI interrupt routine
ISR (SPI_STC_vect)
{
  byte c = SPDR;

  Serial.print("command in ISR = "); Serial.println(c);
  
  switch (c)
  {
      case 0:
        command = c;
        SPDR = 0;
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
        MOVE_MOTOR = false;
        LOCK_MOTOR = true;
        
        // read flag from loop that relate to motor status
        if(!motor_locked)
        {
            SPDR = STATE_UNLOCKED;
        }
        else
        {
            SPDR = motor_new_state;
            LOCK_MOTOR = false;           
        }
        break;        
         
      default:
        break;
  } //switch

} //ISR

void loop (void)
{

  if (digitalRead (ssPins[0]) == HIGH)
  {
    Serial.println("RESET VALUES");
    command = 0;
    motor_finished = false;
    motor_locked = false;
  }

  if (MOVE_MOTOR)
  {
        // MOTOR STEPPING SIMULATION
        //motor_finished = false;
        
        for(int motor_step = 0; motor_step < 50; motor_step++){
          delay(100);
          Serial.println("Stepper moving");
          Serial.print("current step = "); Serial.println(motor_step);
        }
        
        // Motor finished and new state reached
        motor_finished = true;
        motor_new_state = IN_POSITION;

  }
  
  if (LOCK_MOTOR)
  {
        //motor_locked = false;
        delay(1000);

        Serial.println("Motor is locked");
  
        // Motor locked and new state reached
        motor_locked = true;
        motor_new_state = STATE_LOCKED;
  }

  delay(2000);
}  // end of loop
