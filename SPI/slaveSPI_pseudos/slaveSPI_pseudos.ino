#include <SPI.h>
#include "PseudoSPIcommMetamorphicManipulator.h"

/*
 * IMPORTANT DEFINE PSEUDO ID HERE AND DEFAULT SS PINS FOR ALL SLAVES
 */
int pseudoID = PSEUDO_NUMBER1; // this is pseudo 1
int ssPins[] = {SSpinPseudo1, SSpinPseudo2, SSpinPseudo3};
/*
 * This is the datat type struct that resembles to msgs received
 */
typedef struct aliasPacketReceived{
    unsigned long   micros_talked;
    int             pseudoID_talked;
    int             command_state;      
}packets;

typedef union aliasPacketReceivedUnion{
  packets packet_received;
  unsigned char bytes[sizeof(packets)];
}packets_u;

/*
 *  Define variables used by ISR
 */
 volatile byte index;
 volatile boolean packet_received_success;
 int ISR_buffer_size  = 1*sizeof( unsigned long) + 2 * sizeof(int);
 char buf[sizeof(aliasPacketReceived)];
 bool newMessage = false;
 
/*
 * Construct struct object
 */
 aliasPacketReceived PACKET;
 
void setup() {
    pinMode(TXled_Pin,OUTPUT);
    pinMode(RXled_Pin,OUTPUT);

    digitalWrite(TXled_Pin,LOW);
    digitalWrite(RXled_Pin,LOW);
  
    Serial.begin(SERIAL_BAUDRATE);
    pinMode(MISO_NANO, OUTPUT);
    
    constructEptyPacket(&PACKET);
    
    SPCR |= _BV(SPE);
    
    index = 0;                            // turn on SPI in slave mode  
    packet_received_success = false;
    
    SPI.attachInterrupt();                // turn on interrupt
}

void loop (void) {
    if (newMessage) {
      Serial.println("Message received");
      newMessage = false;
    }
  
    if (packet_received_success) {
      digitalWrite(RXled_Pin,HIGH);
      delay(500);
      buf[index] = 0;
      aliasPacketReceivedUnion PACKET_UNION;

      for (size_t index_count = 0; index_count < sizeof(packets); index_count++)
      {
          PACKET_UNION.bytes[index_count] = buf[index_count];
      }

      PACKET = PACKET_UNION.packet_received;
      Serial.print("Received command = "); Serial.println(PACKET.command_state);

      // Here takes action
      if(PACKET.command_state == 1)
      {
        Serial.println("Ela geiaa1");
      }
      else
      {
        Serial.println("Ela geiaa2");
      }
      
      newMessage              = true;
      packet_received_success = false;  //reset the process
      index                   = 0;      //reset button to zero
    }

    digitalWrite(TXled_Pin,LOW);
    digitalWrite(RXled_Pin,LOW);

    delay(1000);
}

ISR (SPI_STC_vect){ 
   byte c = SPDR;
   buf[index] = c;
   index++;
   
   if (index >= sizeof(packets))
   {
      packet_received_success = true;
   }
}

void constructEptyPacket(aliasPacketReceived *PACKET){
    PACKET->micros_talked    = 0;
    PACKET->pseudoID_talked  = 0;
    PACKET->command_state    = 0;
}
