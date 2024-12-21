
// importing the 2 downloaded libraries and the servo library
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include<Servo.h>


// Arduino Wire library is required for  I2Cdev I2CDEV_ARDUINO_WIRE implementation
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

// making MPU6050 object 
MPU6050 mpu;

// making 2 servo objects for roll and pitch
Servo servoX;
Servo servoY;

//initializing servo connection pins
int pin1 = 5;
int pin2 = 6;

//gives the yaw/pitch/roll values in degrees and maps the angles in degrees to give to the servos
#define OUTPUT_READABLE_YAWPITCHROLL

//used for processing IDE
//#define OUTPUT_TEAPOT


//defining pins
#define INTERRUPT_PIN 2  
#define LED_PIN 13 
bool blinkState = true;



// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };




// indicates whether MPU interrupt pin has gone high
volatile bool mpuInterrupt = false;    
void dmpDataReady() {
    mpuInterrupt = true;
}

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer



void setup() {
    //attaching servo pins    
    servoX.attach(pin1);
    servoY.attach(pin2);

    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize serial communication  
    Serial.begin(9600);
    while (!Serial); 
    // initialize device    
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();

    // verify connection
    pinMode(INTERRUPT_PIN, INPUT);
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // wait for ready
    Serial.println(F("\nSend any character to begin DMP programming and demo: "));
    while (Serial.available() && Serial.read()); // empty buffer
    while (!Serial.available());                 // wait for data
    while (Serial.available() && Serial.read()); // empty buffer again

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    //gyro offsets
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788);

    //if connection is successful
    if (devStatus == 0) {
        
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

       
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        
        packetSize = mpu.dmpGetFIFOPacketSize();


        // ERROR! if connection is unsuccessful
    } else {

        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

    //configure led for output
    pinMode(LED_PIN, OUTPUT);
}



// main loop
void loop() {
    // if programming failed, do nothing   
    
    if (!dmpReady) return;

    
    while (!mpuInterrupt && fifoCount < packetSize) {
      
    }

    //reset interrupt flag
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

     // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));

    
    } else if (mpuIntStatus & 0x02) {
        
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        fifoCount -= packetSize;

       

       

        
        
        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees and give input to servo motors
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            float pitch = ypr[1] * 180/M_PI;
            float roll = ypr[2] * 180/M_PI;
            Serial.print("pitch/roll\t");
            Serial.print(pitch);
            Serial.print("\t");
            Serial.println(roll);
            pitch = map(pitch, -90, 90, 0, 179);
            roll = map(roll, -90, 90, 179, 0);
            servoX.write(pitch);
            servoY.write(roll);            
        #endif

        
        #ifdef OUTPUT_TEAPOT
            // display quaternion values in InvenSense Teapot demo format:
            teapotPacket[2] = fifoBuffer[0];
            teapotPacket[3] = fifoBuffer[1];
            teapotPacket[4] = fifoBuffer[4];
            teapotPacket[5] = fifoBuffer[5];
            teapotPacket[6] = fifoBuffer[8];
            teapotPacket[7] = fifoBuffer[9];
            teapotPacket[8] = fifoBuffer[12];
            teapotPacket[9] = fifoBuffer[13];
            Serial.write(teapotPacket, 14);
            teapotPacket[11]++; // packetCount, loops at 0xFF on purpose
        #endif

       
        digitalWrite(LED_PIN, blinkState);
    }
}
