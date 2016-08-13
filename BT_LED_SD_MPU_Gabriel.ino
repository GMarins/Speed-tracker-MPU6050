
#include<SPI.h>
#include <SdFat.h>
#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

MPU6050 mpu;

#define OUTPUT_READABLE_WORLDACCEL

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector

bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high

SdFat sd;
SdFile myFile;

const int chipSelect = 4;


const int  redLed= 7;
const int greenLed = 5;
const int blueLed = 6;

int startButtonState = 0;
const int startButtontPin = 8; //# Faltou setar como output
int isRecording = 1; // if odd, then it is not recording; if even it is.
bool isStartButtonHigh = false; // to prevent repeated HIGH detecttion for the button pin //#
int accelX;
int accelY;
int accelZ;
char data[64]; //MPU data to be written to SD card //#

char dataTime[64]; // A separator for millis() data  //#
unsigned long ellapsedTime = 0; //Contais millis() data //#
int ledTime = 0; //Contais an increment for LED     
int ledInterval = 500; //How much time the red led will be HIGH

bool fifoOverflow = false; //Red light.
bool canRecord = false; //Blue light. Timer will confirm if it is ok to start recording
                      //isRecording is green light 

void setup() {
  Serial.begin(115200);
  pinMode(blueLed,OUTPUT);   
  pinMode(greenLed,OUTPUT);   
  pinMode(redLed,OUTPUT);
  pinMode(startButtontPin,INPUT); //# Incluído

  ledControl(); //Warning users they should wait, as data will not be recorded
  
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
      Wire.begin();
      TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz). Comment this line if having compilation difficulties with TWBR.
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
      Fastwire::setup(400, true);
  #endif

  mpu.initialize();
  mpu.testConnection();

  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(90);
  mpu.setYGyroOffset(-42);
  mpu.setZGyroOffset(21);
  
  mpu.setXAccelOffset(-4553);
  mpu.setYAccelOffset(1441);
  mpu.setZAccelOffset(1341);

  if (devStatus == 0) {
      mpu.setDMPEnabled(true);

      attachInterrupt(0, dmpDataReady, RISING);
      mpuIntStatus = mpu.getIntStatus();

      dmpReady = true;

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
   
 
}

void loop() {
    Serial.println("loop");
    Serial.print("f canRecord: ");
    Serial.print(canRecord);
    Serial.print("startButtonState: ");
    Serial.print(startButtonState);
    Serial.print("isStartButtonHigh: ");
    Serial.print(isStartButtonHigh);
    Serial.print("isRecording: ");
    Serial.println(isRecording);
    Serial.println("\n");
    
    ellapsedTime = millis();
    ledTime += 1; //Start incrementing ledTime
    
    if(ellapsedTime > 1000) { //Wait until 60 seconds for MPU data stabilize //# Só espera 5 segundos
      canRecord=true; //Now data can be recorded      
    }

    if( ledTime > ledInterval){
      fifoOverflow = false; 
      ledControl();    
    }

    //MPU CODE STARTS HERE
    if (!dmpReady) return;

    while (!mpuInterrupt && fifoCount < packetSize) {
    }

    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    fifoCount = mpu.getFIFOCount();

    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        mpu.resetFIFO();
        
        fifoOverflow = true;
        ledTime=0;
        ledControl();
        Serial.println("OVERFLOW");

    } else if (mpuIntStatus & 0x02) {
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        fifoCount -= packetSize;

        #ifdef OUTPUT_READABLE_WORLDACCEL
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetAccel(&aa, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
            mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
        #endif

      if( canRecord ){ //do not record until at least 60 seconds are passed
           startButtonState = digitalRead(startButtontPin);
           
          if (startButtonState == HIGH && isStartButtonHigh) {
              isStartButtonHigh = false;
              isRecording += 1;  
              handleFile(isRecording);    
           }
           else if (startButtonState == LOW) {
               isStartButtonHigh = true;
           }
    
           accelX = aaWorld.x;
           accelY = aaWorld.y;
           accelZ = aaWorld.z;
           
           sprintf(data,"%d\t%d\t%d",accelX, accelY, accelZ);
          
           writteToFile(isRecording, data); 
           ledControl(); 
        }
    }  
    Serial.print("f canRecord: ");
    Serial.print(canRecord);
    Serial.print("startButtonState: ");
    Serial.print(startButtonState);
    Serial.print("isStartButtonHigh: ");
    Serial.print(isStartButtonHigh);
    Serial.print("isRecording: ");
    Serial.println(isRecording);
    Serial.println("\n");
    
    
}

void dmpDataReady() {
    mpuInterrupt = true;
}

void ledControl(){
  //Turn all leds off
  digitalWrite(greenLed,LOW);
  digitalWrite(redLed,LOW);
  digitalWrite(blueLed,LOW);

  //Check what should be on
  if( !canRecord ){
    digitalWrite(blueLed,HIGH); //Wait for MPU data to stabilize. Nothing is being recorded.
  }
  
  if( fifoOverflow ){
    digitalWrite(redLed,HIGH); //FIFO overflow!
  }
  
  if( !fifoOverflow ){
    digitalWrite(redLed,LOW);
  }

  if( !fifoOverflow && canRecord &&  isRecording % 2 == 0 ){
    digitalWrite(greenLed,HIGH); //Recording
  }
  
}

void handleFile(int isRecording){
   
  //if isRecording is even
  if(isRecording % 2 == 0){  

      if (!sd.begin(chipSelect, SPI_HALF_SPEED)) sd.initErrorHalt();
  
      if (!myFile.open("MPU.txt", O_RDWR | O_CREAT | O_AT_END)) {
          sd.errorHalt("opening for write failed");
       }
       sprintf(dataTime,"Starting record: %lu",ellapsedTime); //#
       myFile.println( dataTime );
  }else{  

      sprintf(dataTime,"Stopping record: %lu",ellapsedTime);      //#
      myFile.println( dataTime );
      myFile.close();
  }
}

void writteToFile(int isRecording, char* data){
  if(isRecording % 2 == 0){ 
    myFile.println(data);    
  }
}
