#include <nRF24L01.h>
#include <RF24.h>
#include <RF24_config.h>
#include <SPI.h>
/*
This is the corresponding sketch to the 'basicSend' sketch.
the nrf24l01 will listen for numbers 0-255, and light the red LED
whenever a number in the sequence is missed.  Otherwise,
it lights the green LED
*/
int msg[1];
RF24 radio(9,10);
const uint64_t pipe = 0xE8E8F0F0E1LL;
 
void setup(void){
  Serial.begin(9600);
  radio.begin();
  radio.openReadingPipe(1,pipe);
  radio.startListening();

}
 
void loop(void){
  if (radio.available()){    
    Serial.println("Available");
      radio.read(msg, 2); 
      Serial.println(msg[0]);
  }
  //Serial.println("not available");
}
