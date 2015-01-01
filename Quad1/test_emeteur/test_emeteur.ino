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

#define PITCH_MODE 23
#define ROLL_MODE 24

#define ADD_ROFFSET 25
#define SUB_ROFFSET 26
#define ADD_POFFSET 27
#define SUB_POFFSET 28

#define ADD100 29
#define SUB100 30

#define CLR_ROFFSET 31
#define CLR_POFFSET 32


// END COMMANDS //
boolean PitchMode=false;
boolean RollMode= false;

float rOffset=0;
float pOffset=0;
//PITCH PID
float KPP=3.4;
float KDP=100; 
float KIP=0.01;

//ROLL PID
float KPR=3.4; 
float KDR=100; 
float KIR=0.01;

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
 Serial.begin(115200);
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
    radio.flush_tx();
    
    if(test){
      msg[0]=0;
      radio.write(msg,1);
      radio.flush_tx();
    }
     
     Serial.print("send ");
     Serial.print(r);
     Serial.print(" ack ");
     Serial.println(test);
  if(r==1)
    KPP+=0.1;
  if(r==2)
    KPP-=0.1;
  if(r==3)
    KPR+=0.1;
  if(r==4)
    KPR-=0.1;
  if(r==5)
    KDP+=1;
  if(r==6)
    KDP-=1;
  if(r==7)
    KDR+=1;
  if(r==8)
    KDR-=1;
  if(r==9)
    KIP+=0.01;
  if(r==10)
    KIP-=0.01;
  if(r==11)
    KIR+=0.01;
  if(r==12)
    KIR-=0.01;
  if(r==13)
    KPY+=0.1;
  if(r==14)
    KPY-=0.1;
  if(r==15)
    KDY+=0.1;
  if(r==16)
    KDY-=0.1;
  if(r==17)
    KIY+=0.1;
  if(r==18)
    KIY-=0.1;
    
  if(r==19)
    SPEED+=10;
  if(r==20)
    SPEED-=10;
    
   if(r==ADD100)
    SPEED+=100;
  
  if(r==SUB100)
    SPEED-=100;
    
  if(r==21){
    SPEED=700; KPP=0; KDP=0; KIP=0;
    KPR=0; KDR=0; KIR=0;
    KPY=0; KDY=0; KIY=0;
    PitchMode=false;
    RollMode=false;
  }
  if(r==22){ 
    SPEED=700;
    KPP=3.4; KDP=100; KIP=0.01;
    KPR=3.4; KDR=100; KIR=0.01;
    PitchMode=false;
    RollMode=false;
    rOffset=0;
    pOffset=0;
  }
  if(r==23){
    PitchMode= !PitchMode;
  }
  if(r==24){
    RollMode= !RollMode;
  }
  if(r==ADD_ROFFSET){
    rOffset+=0.5;
  }
  if(r==SUB_ROFFSET){
    rOffset-=0.5;
  }
  if(r==ADD_POFFSET){
    pOffset+=0.5;
  }
   if(r==SUB_POFFSET){
    pOffset-=0.5;
  }
  if(r==CLR_POFFSET){
    pOffset=0;
  }
  if(r==CLR_ROFFSET){
    rOffset=0;
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
 Serial.print(KIY);
 
 Serial.print(" Roll Mode ");
 Serial.print(RollMode);
 Serial.print(" Pitch Mode ");
 Serial.print(PitchMode);
 
 Serial.print(" OR ");
 Serial.print(rOffset);
 Serial.print(" OP ");
 Serial.println(pOffset);
 }
