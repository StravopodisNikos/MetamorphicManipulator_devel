#include <SPI.h>
#include "PseudoSPIcommMetamorphicManipulator.h"

#define motor_execution_time 10
int ssPins[] = {SSpinPseudo1, SSpinPseudo2, SSpinPseudo3};

int CURRENT_STATE;
byte state_receive_from_slave;

void setup (void)
{
  Serial.begin (115200);
  Serial.println ();

  pinMode(SCK_NANO, OUTPUT);
  pinMode(MOSI_NANO, OUTPUT);
  pinMode(MISO_NANO, INPUT);
  for (size_t i = 0; i < sizeof(ssPins); i++)
  {
    pinMode(ssPins[i], OUTPUT);
    pinMode(ssPins[i], HIGH);
  }
  
  SPI.begin ();

  // Slow down the master a bit
  SPI.setClockDivider(SPI_CLOCK_DIV128);

  digitalWrite(TXled_Pin,LOW); digitalWrite(RXled_Pin,LOW);
}  // end of setup

byte transferAndWait (byte what)
{
  byte a = SPI.transfer (what);
  delayMicroseconds (10);
  return a;
} 

void loop (void)
{

  // enable Slave Select
  digitalWrite(SSpinPseudo1, LOW);    


  do{
    state_receive_from_slave = transferAndWait ((byte)CMD_MOVE);

    CURRENT_STATE = (int) state_receive_from_slave;

    delay(100);
    
  }while(! (CURRENT_STATE == IN_POSITION));
  Serial.print ("New Motor State:"); Serial.println (state_receive_from_slave, DEC);

  transferAndWait (0);
  
  // disable Slave Select
  digitalWrite(SSpinPseudo1, HIGH);
  
  

  if((CURRENT_STATE == IN_POSITION))
  {
  Serial.println("NEW COMMAND");
  // enable Slave Select
  digitalWrite(SSpinPseudo1, LOW);    

  do{
    
    state_receive_from_slave = transferAndWait ((byte)CMD_LOCK);

    CURRENT_STATE = (int) state_receive_from_slave;

    delay(100);
    
  }while(! (CURRENT_STATE == STATE_LOCKED) );
  
  Serial.print ("New Motor State:"); Serial.println (state_receive_from_slave, DEC);

  
  // disable Slave Select
  transferAndWait (0);
  digitalWrite(SSpinPseudo1, HIGH);
  }
  
  delay (2000);  
}  
