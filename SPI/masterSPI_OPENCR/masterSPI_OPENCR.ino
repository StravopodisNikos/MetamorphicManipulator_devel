#include <SPI.h>
#include "PseudoSPIcommMetamorphicManipulator.h"

/*
 * IMPORTANT DEFINE DEFAULT SS PINS FOR ALL SLAVES
 */
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
  packets       packet_received;
  unsigned char bytes[sizeof(packets)];
}packets_u;

/*
 *  Define variables used by ISR
 */
 volatile byte index;
 volatile bool packet_received_success;

bool return_function_state = false;

void setup() {

  pinMode(TXled_Pin,OUTPUT);
  pinMode(RXled_Pin,OUTPUT);

  digitalWrite(TXled_Pin,LOW);
  digitalWrite(RXled_Pin,LOW);
    
  Serial.begin(SERIAL_BAUDRATE);
  SPI.begin();

  for(int id_count = 0; id_count < sizeof(ssPins); id_count++){
    digitalWrite(ssPins[id_count], HIGH);           // disable all slaves
  }
  
  SPI.setClockDivider(SPI_CLOCK_DIV128);

  
}
aliasPacketReceived PACKET;

void loop() {    
    int user_command = 1000;
    
    // Construct packet
    constructPacket(&PACKET, PSEUDO_NUMBER1, user_command);
    Serial.print("construct_packet@ : "); Serial.println(PACKET.micros_talked);
    
    // Write/Read Packets
    return_function_state = executeTxRxMasterBlock(PACKET, PSEUDO_NUMBER1 , ssPins);
    if(return_function_state)
    {
        Serial.println("[   MASTER  ]   WRITE Command to SLAVE: SUCCESS");
        digitalWrite(TXled_Pin,HIGH);
        delay(500);
    }
    else
    {
        Serial.println("[   MASTER  ]   WRITE Command to SLAVE: FAILED");
    }
    
    digitalWrite(TXled_Pin,LOW);
    digitalWrite(RXled_Pin,LOW);

    delay(1000);
}

void constructPacket(aliasPacketReceived *PACKET, int pseudoID, int command_code){
  PACKET->micros_talked    = micros();
  PACKET->pseudoID_talked  = pseudoID;
  PACKET->command_state    = command_code;
}

bool executeTxRxMasterBlock(aliasPacketReceived PACKET, int pseudoID , int ssPins[]){

  aliasPacketReceivedUnion PACKET_UNION;

  PACKET_UNION.packet_received = PACKET;
  
  digitalWrite(ssPins[pseudoID-1], LOW);              // enable Slave Select

  for (size_t index_count = 0; index_count < sizeof(packets); index_count++)
  {
      SPI.transfer(PACKET_UNION.bytes[index_count]);
  }
  // End Write byte command
  delay(50);
  
  digitalWrite(ssPins[pseudoID-1], HIGH);             // disable Slave Select

  return true;
}
