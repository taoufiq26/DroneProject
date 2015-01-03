
#include "I2Cdev.h"
#include <Servo.h>
#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"
#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

MPU6050 mpu;
#define OUTPUT_READABLE_YAWPITCHROLL

/* START COMMANDS VALUES */
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
/* END COMMAND VALUES */

#define MAX_SIGNAL 2000
#define MIN_SIGNAL 700
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
#define PP 5
#define PM 4
#define RP 6
#define RM 7
bool blinkState = false;

boolean PitchMode=false;
boolean RollMode= false;
// Radio variables
int msg[1];
RF24 radio(9,10);
const uint64_t pipe = 0xE8E8F0F0E1LL;
//END RADIO VARIABLES
// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;
VectorFloat gravity; 
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo

uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };



// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}



// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

//PITCH PID
float KPP=4.1; 
float KDP=90; 
float KIP=0.1;

//ROLL PID
float KPR=4.1; 
float KDR=90; 
float KIR=0.1;

//YAW PID
float KPY=0; 
float KDY=0; 
float KIY=0;

float rOffset=-2;
float pOffset=0;
//MOTORS CURRENT SPEED
int SPEED=MIN_SIGNAL;

Servo motorPP;
Servo motorPM;
Servo motorRP;
Servo motorRM;
void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize serial communication
    // (115200 chosen because it is required for Teapot Demo output, but it's
    // really up to you depending on your project)
    Serial.begin(115200);
    while (!Serial); // wait for Leonardo enumeration, others continue immediately

    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // wait for ready
    Serial.println(F("\nSend any character to begin DMP programming and demo: "));
    //while (Serial.available() && Serial.read()); // empty buffer
    //while (!Serial.available());                 // wait for data
    //while (Serial.available() && Serial.read()); // empty buffer again

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();
    Serial.println(devStatus);
    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
    
    //Radio Initialize
    radio.begin();
    radio.openReadingPipe(1,pipe);
    radio.startListening();
   // Radio Initialize
    // configure LED for output
    //pinMode(LED_PIN, OUTPUT);
    motorPP.attach(PP);
    motorPM.attach(PM);
    motorRP.attach(RP);
    motorRM.attach(RM);
    motorPP.writeMicroseconds(MIN_SIGNAL);
    motorPM.writeMicroseconds(MIN_SIGNAL);
    motorRP.writeMicroseconds(MIN_SIGNAL);
    motorRM.writeMicroseconds(MIN_SIGNAL);
}



// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
    // if programming failed, don't try to do anything
    if (!dmpReady) return;

    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) {
        // other program behavior stuff here
        // .
        // .
        // .
        // if you are really paranoid you can frequently test in between other
        // stuff to see if mpuInterrupt is true, and if so, "break;" from the
        // while() loop to immediately process the MPU data
        // .
        // .
        // .
    }

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;
             
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            //Serial.print("ypr\t");
            //Serial.print(ypr[0] * 180/M_PI);
            //Serial.print("\t");
            //Serial.print(ypr[1] * 180/M_PI);
            //Serial.print(" ");
            //Serial.println(ypr[2] * 180/M_PI);
            //receiveCommands();
            
            receiveCommands();
            if(!PitchMode && !RollMode){
              motorRP.writeMicroseconds(MIN_SIGNAL);
              motorRM.writeMicroseconds(MIN_SIGNAL); 
              motorPP.writeMicroseconds(MIN_SIGNAL);
              motorPM.writeMicroseconds(MIN_SIGNAL);
              Serial.println("Roll and Pitch off");
            }
            else if(PitchMode && !RollMode){
              regulateP();
              motorRP.writeMicroseconds(MIN_SIGNAL);
              motorRM.writeMicroseconds(MIN_SIGNAL);
            }else if(!PitchMode && RollMode){
              regulateR();
              motorPP.writeMicroseconds(MIN_SIGNAL);
              motorPM.writeMicroseconds(MIN_SIGNAL);
            }else if(PitchMode && RollMode){
              regulateP();
              regulateR();
            }
            //regulateP();
            //regulateR();
            
           
           Serial.println("");
        // blink LED to indicate activity
        //blinkState = !blinkState;
        //digitalWrite(LED_PIN, blinkState);
    }
     //printPid();
}

void atterisage(){
 while(SPEED>60){
   SPEED--;
   delay(800);
 } 
}
float getErrorP(){
  return (ypr[1]*180/M_PI)+pOffset;
}

float getErrorR(){
  return (ypr[2]*180/M_PI)+rOffset;
}

float lastErrorP=0;
float lastErrorR=0;
float errorP=0;
float errorR=0;

float IP=0;
void regulateP(){
  errorP=getErrorP();
  float P=0,D=0;
  P=KPP*errorP;
  D=KDP*(errorP-lastErrorP);
  IP=IP+KIP*errorP;
  
  int speedPP=SPEED+(P+IP+D);
  int speedPM=SPEED-(P+IP+D);
  

  
  if(speedPP<MIN_SIGNAL) speedPP=MIN_SIGNAL;
  if(speedPM<MIN_SIGNAL) speedPM=MIN_SIGNAL;
  if(speedPP>MAX_SIGNAL) speedPP=MAX_SIGNAL;
  if(speedPM>MAX_SIGNAL) speedPM=MAX_SIGNAL;
  
  motorPP.writeMicroseconds(speedPP);
  motorPM.writeMicroseconds(speedPM);
  
   
  //Serial.print(" Error: ");
  //Serial.print(errorP);
 
  
  lastErrorP=errorP;
  Serial.print(" P Error: ");
  Serial.print(errorP);
  Serial.print("Last Error ");
  Serial.print(lastErrorP);
   Serial.print(" sPP: ");
  Serial.print(speedPP);


  Serial.print(" sPM: ");
  Serial.print(speedPM);
}
float IR=0;
void regulateR(){
  errorR=getErrorR();
  float P=0,D=0;
  P=KPR*errorR;
  D=KDR*(errorR-lastErrorR);
  IR=IR+KIR*errorR;
  
  int  speedRP=SPEED+(P+IR+D);
  int  speedRM=SPEED-(P+IR+D);
  

  
  if(speedRP<MIN_SIGNAL) speedRP=MIN_SIGNAL;
  if(speedRM<MIN_SIGNAL) speedRM=MIN_SIGNAL;
  if(speedRP>MAX_SIGNAL) speedRP=MAX_SIGNAL;
  if(speedRM>MAX_SIGNAL) speedRM=MAX_SIGNAL;
  
 //  Serial.print(" Error: ");
 // Serial.print(errorR);
 
  
  motorRP.writeMicroseconds(speedRP);
  motorRM.writeMicroseconds(speedRM);
  
 lastErrorR=errorR;
 
 Serial.print(" R Error: ");
 Serial.print(errorR);
 Serial.print("Last Error ");
 Serial.print(lastErrorR);
  Serial.print(" sRP: ");
  Serial.print(speedRP);
  
  
  Serial.print(" sRM: ");
  Serial.print(speedRM);
}
/*void printPid(){
  Serial.print("  Kp: ");
  Serial.print(KPP);
  Serial.print("  Kd: ");
  Serial.print(KDP);
  Serial.print("  Ki: ");
  Serial.print(KIP);
  Serial.print(" Roll Mode ");
  Serial.print(RollMode);
  Serial.print(" Pitch Mode ");
  Serial.println(PitchMode);
}*/

int last=0;
void receiveCommands(){
  if (radio.available()){
     radio.read(msg, 1);      
  }
  int r=msg[0];
  if(r==last)
    return;
  last=r;
  if(r==0) return ;
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
    KDY+=1;
  if(r==16)
    KDY-=1;
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
    SPEED=MIN_SIGNAL; KPP=0; KDP=0; KIP=0;
    KPR=0; KDR=0; KIR=0;
    KPY=0; KDY=0; KIY=0;
    IP=0;IR=0;
    PitchMode=false;
    RollMode=false;
    rOffset=-2;
    pOffset=0;
  }
  if(r==22){
    SPEED=MIN_SIGNAL;
    KPP=4.1; KDP=90; KIP=0.1;
    KPR=4.1; KDR=90; KIR=0.1;
    IP=0; IR=0;
    PitchMode=false;
    RollMode=false;
    rOffset=-2;
    pOffset=0;
  }
  if(r==23){
    PitchMode= !PitchMode;
  }
  if(r==24){
    RollMode= !RollMode;
  }
  if(r==ADD_ROFFSET){
    rOffset+=1.5;
  }
  if(r==SUB_ROFFSET){
    rOffset-=1.5;
  }
  if(r==ADD_POFFSET){
    pOffset+=1.5;
  }
   if(r==SUB_POFFSET){
    pOffset-=1.5;
  }
  if(r==CLR_ROFFSET){
    rOffset=-2;
  }
  if(r==CLR_POFFSET){
    pOffset=0;
  }
  
}


