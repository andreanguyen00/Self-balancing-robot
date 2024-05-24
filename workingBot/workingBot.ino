// Aim:  leaning forwards, rotate the wheels forwards; leaning back, 
// rotate the wheels backward with appropriate speed
// rate of rotation as a function of the angle at which the robot is leaning over
// angle of leaning = reading of gyro + accelerometers
// restoration action calculated in PID controller - yaw/pitch/roll angles


// MPU gives gravity vector and quaternion values to calculate yaw/pitch/roll angles

//Libraries
#include <PID_v1.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <Wire.h>
#include <i2Cdev.h>
#include "math.h"

#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)


// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer


volatile bool mpuInterrupt = false; // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}

// orientation/motion variables from MPU library
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
// float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector


// PID controller settings initial parameters
double setpoint=177; // balance position, run angleInclination.ino to calibrate

double Kp=7.4; // Proportional gain , 12
double Kd=0.33; // Derivative gain
double Ki=85; // Integral gain 

//19.5, 0.6, 0.005; (12,0.5,80)


// Inititialise PID algorithm
double input, output; // input = yaw, output calculated by PID
PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);


// MPU setting (accelerometers and gyros(angle) and communicates) through an I2C interface
MPU6050 mpu;

void setup() {
  
  /*if (!mpu.begin()) {
    Serial.println("Failed to start MPU6050");
    while (1);
  }*/

 // join I2C bus (I2Cdev library doesn't do this automatically)
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
      Wire.begin();
      //Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
      Fastwire::setup(400, true);
  #endif 

  Serial.begin(19200);
  while (!Serial); // wait for Leonardo enumeration, others continue immediately

  // initialize MPU-6050
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);

  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // // wait for ready
  // Serial.println(F("\nSend any character to begin DMP programming and demo: "));
  // while (Serial.available() && Serial.read()); // empty buffer
  // while (!Serial.available());                 // wait for data
  // while (Serial.available() && Serial.read()); // empty buffer again

  //configure Digital Motion Program
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();
  //wait for  the MPU to load
  delay(1000);

  // setup MPU by offset (need to modify) for min sentitivity
  mpu.setXGyroOffset(147);
  mpu.setYGyroOffset(86);
  mpu.setZGyroOffset(-28);
  mpu.setZAccelOffset(1059);


  // Checking the bot is functional make sure it worked (returns 0 if so)
  if (devStatus == 0) {

      // turn on the DMP, now that it's ready
      Serial.println(F("Enabling DMP..."));
      mpu.setDMPEnabled(true);

      // enable Arduino interrupt detection
      Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
      Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
      Serial.println(F(")..."));
      attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
      mpuIntStatus = mpu.getIntStatus();

      // set our DMP Ready flag so the main loop() function knows it's okay to use it
      Serial.println(F("DMP ready! Waiting for first interrupt..."));
      dmpReady = true;

      // get expected DMP packet size for later comparison
      packetSize = mpu.dmpGetFIFOPacketSize();

      // setup PID
      pid.SetMode(AUTOMATIC);
      pid.SetSampleTime(20);
      pid.SetOutputLimits(-255, 255);  //- limit for pid as the speed is constraint between -255 and 255
  } 
  else {
      // ERROR!
      // 1 = initial memory load failed
      // 2 = DMP configuration updates failed
      // (if it's going to break, usually the code will be 1)
      Serial.print(F("DMP Initialization failed (code "));
      Serial.print(devStatus);
      Serial.println(F(")"));
  }  

  // Initialise digital PWM pins to control motors
  pinMode (6, OUTPUT);
  pinMode (9, OUTPUT);
  pinMode (10, OUTPUT);
  pinMode (11, OUTPUT);

  //Output LOW by default
  analogWrite(6,0);
  analogWrite(9,0);
  analogWrite(10,0);
  analogWrite(11,0);

  // configure LED for output
  //pinMode(LED_PIN, OUTPUT);
}


// getting feedback and change accordingly
void loop() {
  // Check if the program is ready
  if (!dmpReady) return; // not ready then don't do anything

  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
    // start the pid
    pid.Compute(); // outpput is (+) then bot is falling forward, (-) means bot falling backwards

    // Print Input and Output to check how it is working
    // Serial.print(input); Serial.print(" =>"); Serial.println(output);

    // If the Bot is falling in threshold
    if (input>140 && input<210 && input != 180) {
      if (output>0) {//Falling towards front 
        Forward(); // Rotate the wheels forward 
      }
      else if (output<0) {//Falling towards back, Rotate the wheels backward 
        Reverse(); 
      }
    }
    // If Bot not falling or if it is falling out of range to recover
    else {
      Stop(); //Hold the wheels still
    }
    mpu.dmpGetQuaternion(&q, fifoBuffer); //get value for q
    mpu.dmpGetGravity(&gravity, &q); //get value for gravity
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity); //get value for ypr

    input = ypr[1] * 180/M_PI + 180;
  }
}


// Code for rotation depending on output value calculated from PID; test if they rotate at the right direction
void Forward() { 
  analogWrite(6,0);
  analogWrite(9,output);
  analogWrite(10,0);
  analogWrite(11,output);
  // Serial.println("F"); // Wheel driving forward
}

void Reverse() {
  analogWrite(6,-output);
  analogWrite(9,0); 
  analogWrite(10,-output*0.8);
  analogWrite(11,0);
  // Serial.println("R"); // Wheel eeversing
}

void Stop() {
  analogWrite(6,LOW);
  analogWrite(9,LOW);
  analogWrite(10,LOW);
  analogWrite(11,LOW);
  // Serial.println("S"); // Stop the bot
}