#include <Servo.h>

#define MAX_SIGNAL 2000
#define MIN_SIGNAL 700
#define PP 4
#define PM 5
#define RP 6
#define RM 7

Servo motorPP;
Servo motorPM;
Servo motorRP;
Servo motorRM;

void setup() {
  Serial.begin(9600);
  motorPP.attach(PP);
  motorPM.attach(PM);
  motorRP.attach(RP);
  motorRM.attach(RM);
  motorPP.writeMicroseconds(MIN_SIGNAL);
  motorPM.writeMicroseconds(MIN_SIGNAL);
  motorRP.writeMicroseconds(MIN_SIGNAL);
  motorRM.writeMicroseconds(MIN_SIGNAL);
}
int s=MIN_SIGNAL;
void loop() {  
  motorPP.writeMicroseconds(s);
  motorPM.writeMicroseconds(s);
  motorRP.writeMicroseconds(s);
  motorRM.writeMicroseconds(s);
  if(Serial.available()){
    char a=Serial.read();
    if(a=='U')
      s++;
    if(a=='D')
      s--;
    if(a=='X')
      s=MIN_SIGNAL;
  }
   Serial.println(s);
}

