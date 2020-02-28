#include <SPI.h>
#include <RF24.h>
#include <nRF24L01.h>
#include <RF24_config.h>
#include <printf.h>

#define CSN_Pin   8
#define CE_Pin    7
#define MOSI_Pin  11
#define SCK_Pin   13
#define MISO_Pin  12

/*
 * Create radio object for Master         
 */
RF24 OPENCR_MASTER(CE_Pin, CSN_Pin);                  // GIVE PINS: CE, CSN

/*
 * Give name to addresses between master and pseudos  
 */
const byte address[6] = "CR2P1";                      // OPENCR -> PSEUDO1
// const byte addresses[][6] = {"CR2P1","CR2P2"};     // for multiple ports  

/*      
 * Set the RADIO NUMBER TO IDENTIFY THE TRANCEIVER         
 */
bool radioNumber = 0;

// Used to control whether this node is sending or receiving //  0 -> SENDING
bool role = 0;

/*
 * Data Handling
 */
struct dataStruct{
  unsigned long _micros;
  float value;
}myData;


void setup() {
  Serial.begin(9600);
  OPENCR_MASTER.begin();
  OPENCR_MASTER.openWritingPipe(address);
  OPENCR_MASTER.setPALevel(RF24_PA_MIN);
  OPENCR_MASTER.stopListening();
}

void loop() {
  const char text[] = "Hello World";
  OPENCR_MASTER.write(&text, sizeof(text));
  Serial.println(text);
  delay(1000);
}
