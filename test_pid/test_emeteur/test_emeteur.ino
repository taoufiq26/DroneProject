#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"

#define ADD_KP_PITCH 1
#define SUB_KP_PITCH 2
#define ADD_KP_ROLL 3
#define SUB_KP_ROLL 4


#define ADD_KD_PITCH 5
#define SUB_KD_PITCH 6
#define ADD_KD_ROLL 7
#define SUB_KD_ROLL 8

#define ADD_KI_PITCH 9
#define SUB_KI_PITCH 10
#define ADD_KI_ROLL 11
#define SUB_KI_ROLL 12

#define ADD_KP_YAW 13
#define SUB_KP_YAW 14

#define ADD_KD_YAW 15
#define SUB_KD_YAW 16

#define ADD_KI_YAW 17
#define SUB_KI_YAW 18

#define ADD_SPEED 19
#define SUB_SPEED 20

#define RESTART_QUAD 21
#define STOP_QUAD 22

//PITCH PID
double KPP=0; 
double KDP=0; 
double KIP=0;

//ROLL PID
double KPR=0; 
double KDR=0; 
double KIR=0;

//YAW PID
double KPY=0; 
double KDY=0; 
double KIY=0;

double SPEED=700;
int msg[1];
RF24 radio(9,10);
const uint64_t pipe = 0xE8E8F0F0E1LL;
int SW1 = 7;
boolean test;

void setup(void){
 Serial.begin(9600);
 radio.begin();
 radio.openWritingPipe(pipe);
 }

void loop(void){
  if(Serial.available()){
     int r =Serial.parseInt();
     emet(r);
  }
 printPID(); 
}
 
 void emet(int r){
   if(r==0) return; // DANGER VERY IMPORTANT
    msg[0] = r;
    test=radio.write(msg, 1);
    if(test){
      msg[0]=0;
      radio.write(msg,1);
    }
     Serial.print("send ");
     Serial.print(r);
     Serial.print(" ack ");
     Serial.println(test);
    if(r==1)
    KPP++;
  if(r==2)
    KPP--;
  if(r==3)
    KPR++;
  if(r==4)
    KPR--;
  if(r==5)
    KDP++;
  if(r==6)
    KDP--;
  if(r==7)
    KDR++;
  if(r==8)
    KDR--;
  if(r==9)
    KIP++;
  if(r==10)
    KIP--;
  if(r==11)
    KIR++;
  if(r==12)
    KIR--;
  if(r==13)
    KPY++;
  if(r==14)
    KPY--;
  if(r==15)
    KDY++;
  if(r==16)
    KDY--;
  if(r==17)
    KIY++;
  if(r==18)
    KIY--;
  if(r==19)
    SPEED++;
  if(r==20)
    SPEED--;
  if(r==21){
    SPEED=700; KPP=0; KDP=0; KIP=0;
    KPR=0; KDR=0; KIR=0;
    KPY=0; KDY=0; KIY=0;
  }
  if(r==22){
    SPEED=0;
  }
  
 }
 
 void printPID(){
  Serial.print("SPEED : ");
  Serial.print(SPEED);
 Serial.print(" KPP ");
 Serial.print(KPP);
  Serial.print(" KDP ");
 Serial.print(KDP);
  Serial.print(" KIP ");
 Serial.print(KIP);
 
 Serial.print(" KPR ");
 Serial.print(KPR);
  Serial.print(" KDR ");
 Serial.print(KDR);
  Serial.print(" KIR ");
 Serial.print(KIR);
 
  Serial.print(" KPY ");
 Serial.print(KPY);
  Serial.print(" KDY ");
 Serial.print(KDY);
  Serial.print(" KIY ");
 Serial.println(KIY);
 
 }
