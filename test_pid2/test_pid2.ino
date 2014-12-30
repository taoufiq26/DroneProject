
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

/* END COMMAND VALUES */

#define MAX_SIGNAL 2000
#define MIN_SIGNAL 700
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
#define PP 5
#define PM 4
#define RP 6
#define RM 7
bool blinkState = false;

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


//MOTORS CURRENT SPEED
double SPEED=760;

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
    pinMode(LED_PIN, OUTPUT);
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
            /*Serial.print("ypr\t");
            Serial.print(ypr[0] * 180/M_PI);
            Serial.print("\t");
            Serial.print(ypr[1] * 180/M_PI);
            Serial.print("\t");
            Serial.println(ypr[2] * 180/M_PI);*/
            //receiveCommands();
            
            receiveCommands();
            regulateP();
            regulateR();
            printPid();
           // Serial.println("");
        // blink LED to indicate activity
        blinkState = !blinkState;
        digitalWrite(LED_PIN, blinkState);
    }
}

void atterisage(){
 while(SPEED>60){
   SPEED--;
   delay(800);
 } 
}
double getErrorP(){
  return ypr[1]*180/M_PI;
}

double getErrorR(){
  return ypr[2]*180/M_PI;
}

double lastErrorP=0;
double lastErrorR=0;
double errorP=0;
double errorR=0;
void regulateP(){
  errorP=getErrorP();
  double P,D,I;
  P=KPP*errorP;
  D=KDP*(errorP-lastErrorP);
  I=I+KIP*errorP;
  
  double speedPP=SPEED+(P+I+D);
  double speedPM=SPEED-(P+I+D);
  
 
  Serial.print(" Error: ");
  Serial.print(errorP);
  Serial.print(" sPP: ");
  Serial.print(speedPP);


  Serial.print(" sPM: ");
  Serial.print(speedPM);
  motorPP.writeMicroseconds(speedPP);
  motorPM.writeMicroseconds(speedPM);
  lastErrorP=errorP;
}

void regulateR(){
  errorR=getErrorR();
  double P,D,I;
  P=KPR*errorR;
  D=KDR*(errorR-lastErrorR);
  I=I+KIR*errorR;
  
  double speedRP=SPEED+(P+I+D);
  double speedRM=SPEED-(P+I+D);
  
  Serial.print(" Error: ");
  Serial.print(errorR);
  Serial.print(" sPP: ");
  Serial.print(speedRP);


  Serial.print(" sPM: ");
  Serial.print(speedRM);
  
  motorRP.writeMicroseconds(speedRP);
  motorRM.writeMicroseconds(speedRM);
  lastErrorR=errorR;
}
void printPid(){
  Serial.print("  Kp: ");
  Serial.print(KPP);
  Serial.print("  Kd: ");
  Serial.print(KDP);
  Serial.print("  Ki: ");
  Serial.println(KIP);
}

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
    KDP+=0.1;
  if(r==6)
    KDP-=0.1;
  if(r==7)
    KDR+=0.1;
  if(r==8)
    KDR-=0.1;
  if(r==9)
    KIP+=0.1;
  if(r==10)
    KIP-=0.1;
  if(r==11)
    KIR+=0.1;
  if(r==12)
    KIR-=0.1;
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
    SPEED++;
    
  if(r==20)
    SPEED--;
    
  if(r==21){
    SPEED=760; KPP=0; KDP=0; KIP=0;
    KPR=0; KDR=0; KIR=0;
    KPY=0; KDY=0; KIY=0;
  }
  if(r==22){
    SPEED=0;
  }
}


