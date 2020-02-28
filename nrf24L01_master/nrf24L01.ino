#include <SPI.h>
#include <RF24.h>
#include <nRF24L01.h>
#include <RF24_config.h>
//#include <printf.h>

#define CSN_Pin   8
#define CE_Pin    7
#define MOSI_Pin  11
#define SCK_Pin   13
#define MISO_Pin  12


RF24 radio(CE_Pin, CSN_Pin); // CE, CSN

const byte address[6] = "00001";

void setup() {
  
  //pinMode(SCK_Pin,OUTPUT);
  //pinMode(MOSI_Pin,OUTPUT);
  //pinMode(CSN_Pin,OUTPUT);
  
  Serial.begin(9600);
  radio.begin();
  radio.openWritingPipe(address);
  radio.setPALevel(RF24_PA_MIN);
  radio.stopListening();
}

void loop() {
  const char text[] = "Hello World";
  radio.write(&text, sizeof(text));
  Serial.println(text);
  delay(1000);
}
