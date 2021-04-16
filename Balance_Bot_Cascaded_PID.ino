/*
* PROJECT DETAILS:
* Project Name: Bipedal_Bot_2550
* File Name: Bipedal_Bot_PID.ino
* Release Date: 05-May-17 6:14:00 AM
*
* Created: 27-Apr-20 1:00:00 PM
* Author : Rahul
* Email: rahulk200013@gmail.com
*
* GitHub: https://github.com/rahulk200013/Self-Balancing-Bot
*
* Team ID: eYRC-BB#2550
* Theme: Bipedal Bot
*
* Completed all the tasks at National Level e-Yantra Robotics Competition 2019-20
* conducted by IIT Bombay, India and sponsored by MHRD, Government of India
* 
* PURPOSE:
* Program to control a balance bot using Cascaded PID control architecture
* Four PID controllers are implemented to control the position, velocity
* rotation and the tilt angle of the robot.
*
* USAGE:
* Refer the wiki for further details on the working of the robot.
*
* LICENSE:
* MIT License
*
* Copyright (c) 2020 Rahul
*/

// MPU6050 Library for DMP Created by Kai-Chieh (Kenneth) Huang
// <kaichieh.kenneth.huang@gmail.com>
// Date 2019-05-10

// =============  Author's Notes  ======================
// To use this sketch, you must first intall I2Cdev and MPU6050 as libraries
// These libraries should be found here: https://github.com/jrowberg/i2cdevlib
//
// This is how you should wire MPU6050 http://301o583r8shhildde3s0vcnh.wpengine.netdna-cdn.com/wp-content/uploads/2014/11/conn.png
// (the SCL, SDA, and INT pins on MPU6050 connects to pins A4, A5, and 2 pins on the Arduino Uno.)
// (You will not find the connection declarations for the SCL and SDA pins on this sketch.)
//
// This sketch serves as a template for creating projects using the MPU6050.
//
// WARNING: Avoid using delay in the loop() function.
// A long delay may cause an overflow on MPU6050's buffer, and you will not be able to read from it.
// If your code in the loop() function takes a long time to run,
// try calling the readMPUFIFOBuffer() function more than once in the loop() function.
//
// The initial output from the Digital Motion Processor(DMP) right after reset is not reliable.
// So it is advised to start using them after a little while.
//
//

// =====  Improvements Over Previous Reference Sketches  =========
// I used the two refenence sketches below and made the following refinements:
// 1. It is only safe to access memory(buffers) on MPU6050 immediately after the MPU6050 signals that data is ready
//    via the interrupt pin. (As far as my testing goes, both RISING and FALLING signals indicate such an event.)
//    Therefore, I added a line of code to ensure this happens, which fixes freezing issues when using the reference sketches below.
// 2. I removed code that I determined redundant, making the sketch more readable. Please refer to the reference
//    sketches below if you feel that I removed something you need.

// =============  Attribution ==========================
// Provided by "HC" aka "zhomeslice" on forum.arduino.cc
// at https://forum.arduino.cc/index.php?PHPSESSID=h4c6487i42hbb7uh6rjk0eadp1&topic=446713.msg3073854#msg3073854
// The above work most likely is based off of Jeff Rowberg's <jeff@rowberg.net> MPU6050_DMP6
// at https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/MPU6050
// =====================================================

/* ============================================
  I2Cdev device library code is placed under the MIT license
  Copyright (c) 2012 Jeff Rowberg

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in
  all copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
  THE SOFTWARE.
  ===============================================
*/

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"
MPU6050 mpu;


#define LED_PIN 13

// You may use MPU6050_calibration.ino (https://github.com/Protonerd/DIYino/blob/master/MPU6050_calibration.ino)
// to find the offest values for your MPU6050.
// If you require high precision, you may wish to fine tune it by printing out some parameters.
//                       XA      YA      ZA      XG      YG      ZG
int MPUOffsets[6] = {  263,  36,    810,     -31,     61,     -4};


// ================================================================
// ===                      i2c SETUP Items                     ===
// ================================================================
void i2cSetup() {
  // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif
}

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has changed
void dmpDataReady() {
  mpuInterrupt = true;
}

// ================================================================
// ===                      MPU DMP SETUP                       ===
// ================================================================
// MPU control/status vars
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
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// ================================================================
// ===                  User Defined Variables                  ===
// ================================================================

volatile byte b[18];
byte check;
byte flag = 1;
byte trueFrame = 1;
int X=0, Y=0, digitalByte = 0;
bool digitalMask[8] = {0,0,0,0,0,0,0,0}, motion = 0;
char joyStick = 'N', lastJoystick = 'N';

float yaw_error = 0, last_yaw_error = 0;
int yawOffset = 0, Kp_yaw = 1600, Kd_yaw = 1000;

int recData[6] = {0, 0, 0, 0, 0, 0};
byte garbage;

int loopCount = 0;

//Low Pass Filter
float alpha = 0.90, filteredData = 0.00f;

long lastByteTime = 0;
bool incompleteData = 0, completeData = 0;

volatile int leftEncoderCount = 0;
volatile int rightEncoderCount = 0;
const int pinA_left = 3; //Right motor encoder pins
const int pinB_left = 4;
int leftPWMoffset = 0, rightPWMoffset = 0;

const int pinA_right = 18;  //Left motor encoder pins
const int pinB_right = 17;

float Pos_error = 0, last_Pos_error = 0, pos_error_sum = 0, Pos_setpoint = 0, Pos_offset = 0;
float Vel_setpoint = 0, last_Vel_error = 0, total_velocity = 0, Vel_error = 0, Pitch_offset = 0, vel_error_sum = 0;
float Kp_pos = 10, Kd_pos = 8, Ki_pos = 0;

float Pitch = 0.00f, lastPitch = 0.00f;
float Pitch_error = 0, last_Pitch_error = 0, Pitch_setpoint = 0.00f, pitch_error_sum = 0.00f;
int Kp_pitch = 45, Kd_pitch = 100, Ki_pitch = 0;
int Kp_vel = 20, Kd_vel = 20, Ki_vel = 0;  //13,10
int PWM = 0;
bool Bot_ready = 0;

float distanceLeft = 0.00f, distanceRight = 0.00f, last_distanceLeft = 0.00f, last_distanceRight = 0.00f;

void MPU6050Connect() {
  static int MPUInitCntr = 0;
  // initialize device
  mpu.initialize();
  // load and configure the DMP
  devStatus = mpu.dmpInitialize();

  if (devStatus != 0) {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)

    char * StatStr[5] { "No Error", "initial memory load failed", "DMP configuration updates failed", "3", "4"};

    MPUInitCntr++;

    Serial.print(F("MPU connection Try #"));
    Serial.println(MPUInitCntr);
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(StatStr[devStatus]);
    Serial.println(F(")"));

    if (MPUInitCntr >= 10) return; //only try 10 times
    MPU6050Connect(); // Lets try again
    return;
  }
  mpu.setXAccelOffset(MPUOffsets[0]);
  mpu.setYAccelOffset(MPUOffsets[1]);
  mpu.setZAccelOffset(MPUOffsets[2]);
  mpu.setXGyroOffset(MPUOffsets[3]);
  mpu.setYGyroOffset(MPUOffsets[4]);
  mpu.setZGyroOffset(MPUOffsets[5]);

  Serial.print(F("Enabling DMP..."));
  mpu.setDMPEnabled(true);
  // enable Arduino interrupt detection
  Serial.println(F("Enabling interrupt detection (Arduino external interrupt pin 2 on the Uno)..."));
  Serial.print("mpu.getInterruptDrive=  ");
  Serial.println(mpu.getInterruptDrive());
  attachInterrupt(0, dmpDataReady, CHANGE); // pin 2 on the Uno. Please check the online Arduino reference for more options for connecting this interrupt pin
  // get expected DMP packet size for later comparison
  packetSize = mpu.dmpGetFIFOPacketSize();
  mpu.resetFIFO(); // Clear fifo buffer
  mpuInterrupt = false; // wait for next interrupt
}

// ================================================================
// ===                    MPU DMP Get Data                      ===
// ================================================================
bool readMPUFIFOBuffer() {
  fifoCount = mpu.getFIFOCount();
  mpuInterrupt = false; // We caught an interrupt reset for next time

  /************************************
     Check for incorrect packet size any size that is not divisible by 42 (Note: this will only occur when we overflow )
     Check for overflow  (1024 bytes FiFo Buffer divided by 42 bytes Packet Size leaves a remainder of 16)
     If the fifoCount is > Zero we have data so get it
   ************************************/
  if ((!fifoCount) || (fifoCount % packetSize)) { // something's wrong. reset and try again.
    Serial.print(F("Wrong packet size, or overflow! packetSize= "));
    Serial.println(packetSize);
    digitalWrite(LED_PIN, LOW); // lets turn off the blinking light so we can see we are failing.
    mpu.resetFIFO();// clear the buffer and start over
    return (false); //no or corrupt data return false

  } else {

    /************************************
       You will want to empty the fifo buffer because the last packet is the latest reading
       If more than one packet exists then the first packet is already at least 10 MS old!!!
     ************************************/
    while (fifoCount  >= packetSize) { // Get the packets until we have the latest!
      mpu.getFIFOBytes(fifoBuffer, packetSize); // lets do the magic and get the data
      fifoCount -= packetSize;
    }
    digitalWrite(LED_PIN, !digitalRead(LED_PIN)); // Blink the LED to indicate activity
    return (true);
  }
}


// ================================================================
// ===                    Output Functions                      ===
// ================================================================
// add these functions to your code as needed

// get quaternion components in a [w, x, y, z] format
// very useful when Euler and YPR angles cannot satisfy your application
// refer to https://en.wikipedia.org/wiki/Quaternion for more information
void getQuaternion()
{
  mpu.dmpGetQuaternion(&q, fifoBuffer);
}
void printQuaternion()
{
  // display quaternion values in easy matrix form: w x y z

  Serial.print("quat\t");
  Serial.print(q.w);
  Serial.print("\t");
  Serial.print(q.x);
  Serial.print("\t");
  Serial.print(q.y);
  Serial.print("\t");
  Serial.println(q.z);
}

// The Euler angles are in radians. Divide it by M_PI then multiply it by 180 to get angles in degrees.
// Note that Euler angles suffer from gimbal lock (for more info, see http://en.wikipedia.org/wiki/Gimbal_lock)
// Try calculating your parameters from quaternions if you experience gimbal lock.
void getEuler()
{
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetEuler(euler, &q);
}
void printEuler()
{
  // display Euler angles in degrees

  Serial.print("euler\t");
  Serial.print(euler[0] * 180 / M_PI);
  Serial.print("\t");
  Serial.print(euler[1] * 180 / M_PI);
  Serial.print("\t");
  Serial.println(euler[2] * 180 / M_PI);
}

// Yaw, pitch, and roll angles are in radians. Divide it by M_PI then multiply it by 180 to get angles in degrees.
// Note that these angles suffer from gimbal lock (for more info, see http://en.wikipedia.org/wiki/Gimbal_lock)
// Try calculating your parameters from quaternions if you experience gimbal lock.
void getYawPitchRoll()
{
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
}
float getPitch()
{
  // display yaw, pitch, roll in degrees

  //Serial.print("ypr\t");
  //Serial.print(ypr[0] * 180 / M_PI);
  //Serial.print("\t");
  return(ypr[1] * 180 / M_PI);
  //Serial.print("\t");
  //Serial.println(ypr[2] * 180 / M_PI);
}

// Use this if you want to see acceleration
// components with gravity removed. This acceleration reference frame is
// not compensated for orientation, so +X is always +X according to the
// sensor, just without the effects of gravity. If you want acceleration
// compensated for orientation, use getWorldAccel() instead.
void getRealAccel()
{
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetAccel(&aa, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
}
void printRealAccel()
{

  Serial.print("areal\t");
  Serial.print(aaReal.x);
  Serial.print("\t");
  Serial.print(aaReal.y);
  Serial.print("\t");
  Serial.println(aaReal.z);
}

// Use this if you want to see acceleration
// components with gravity removed and adjusted for the world frame of
// reference (yaw is relative to initial orientation, since no magnetometer
// is present in this case). Could be quite handy in some cases.
void getWorldAccel()
{
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetAccel(&aa, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
  mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
}
void printWorldAccel()
{

  Serial.print("aworld\t");
  Serial.print(aaWorld.x);
  Serial.print("\t");
  Serial.print(aaWorld.y);
  Serial.print("\t");
  Serial.println(aaWorld.z);
}

// ================================================================
// ===                         Setup                            ===
// ================================================================
void setup() {
  Serial.begin(9600); //115200
  Serial3.begin(9600);  //XBee
  while (!Serial);
  Serial.println("i2cSetup");
  i2cSetup();
  Serial.println("MPU6050Connect");
  MPU6050Connect();
  Serial.println("Setup complete");
  pinMode(LED_PIN, OUTPUT);
  Serial3.begin(9600);
  pinMode(22, OUTPUT);
  pinMode(24, OUTPUT);
  pinMode(26, OUTPUT);
  pinMode(28, OUTPUT);
  pinMode(44, OUTPUT);
  pinMode(46, OUTPUT);
  pinMode(pinB_left, INPUT_PULLUP);
  pinMode(pinB_right, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(pinA_left), leftEncoder, RISING);
  attachInterrupt(digitalPinToInterrupt(pinA_right), rightEncoder, RISING);
}
// ================================================================
// ===                          Loop                            ===
// ================================================================
void loop() {
  // ---- read from the FIFO buffer ---- //
  // The interrupt pin could have changed for some time already.
  // So set mpuInterrupt to false and wait for the next interrupt pin CHANGE signal.
  //  Wait until the next interrupt signal. This ensures the buffer is read right after the signal change.
  if (mpuInterrupt) {
    if (readMPUFIFOBuffer()) {
      // Calculate variables of interest using the acquired values from the FIFO buffer
      // getQuaternion();
      // getEuler();
      getYawPitchRoll();
      // getRealAccel();
      // getWorldAccel();

      static unsigned long SpamTimer;

      // ==========  Your code goes here that uses the MPU6050 readings ========= //

      //Wait for 10 seconds for the MPU6050 to stabilise and DMP to give stable values
      if(millis()>10000){
        Bot_ready = 1;      
      }
      
      if(Bot_ready){
  
      Pitch = getPitch();   

      //Calculating distance travelled by each motor
      distanceLeft += (getLeftEncoderCount()*0.00076);       
      distanceRight += (getRightEncoderCount()*0.00076);

      //Difference between the distance travelled by each motor is taken as error for yaw control
      yaw_error = distanceLeft - distanceRight;

      yawOffset = Kp_yaw*yaw_error + Kd_yaw*(yaw_error - last_yaw_error);

      //Turn off yaw correction when turning left or right
      if(joyStick == 'L' || joyStick == 'R'){
        yawOffset = 0;
      }

      //Calculating the velocity of the bot by taking average of velocity of each motor
      total_velocity = ((getLeftEncoderCount()*0.00076) + (getRightEncoderCount()*0.00076))/0.02;

      //Passing the velocity through a low pass filter to smoothen the discrete nature
      total_velocity = lowPassFilter(total_velocity);

      //Resetting encoder count so that they don't overflow and can be stored in integer type variable
      leftEncoderCount = 0;
      rightEncoderCount = 0;

      // ==========  Position Control Section ========= //

      //Position PID loop runs at 25Hz
      if(loopCount == 10){

      //When JoyStick is in Neutral position
      if(joyStick == 'N'){                                   
         if(lastJoystick == 'F' || lastJoystick == 'B'){
            distanceLeft = 0;
            distanceRight = 0;
          if(Pos_error>0.05){
              //Gradually decreasing the error to stop smoothly
              Pos_error -= 0.04;
            }else if(Pos_error<-0.05){
              //Gradually decreasing the error to stop smoothly
              Pos_error += 0.04;
            }else{
              lastJoystick = 'N';
              motion = 1;
            }
        }else if(lastJoystick == 'L' || lastJoystick == 'R'){
          distanceLeft = 0;
          distanceRight = 0;
          last_yaw_error = 0;
          lastJoystick = 'N';
        }else{
        Pos_error = Pos_setpoint - (distanceLeft + distanceRight)/2;
        }
      } else if(joyStick == 'F'){       
        if(digitalMask[1] == 0){              //Normal Speed Mode
        if(Pos_error < 0.24){
        Pos_error += 0.02;
        }
        distanceLeft = 0;
        distanceRight = 0;
        } else {
          if(Pos_error < 0.36){           //High Speed Mode
            Pos_error += 0.04;
        }
        distanceLeft = 0;
        distanceRight = 0;
        }
      }else if(joyStick == 'B'){
        if(digitalMask[1] == 0){        //Normal Speed Mode
        if(Pos_error > -0.24){
        Pos_error -= 0.02;
        }
        distanceLeft = 0;
        distanceRight = 0;
        } else {
          if(Pos_error > -0.36){      //High Speed Mode 
            Pos_error -= 0.04;
        }
        distanceLeft = 0;
        distanceRight = 0;
        }
      }else{
        Pos_error = Pos_setpoint - (distanceLeft + distanceRight)/2;
      }
      if(motion == 1){
        distanceLeft = 0;
        distanceRight = 0;
        Pos_error = 0;
        last_Pos_error =0;
      }

      //When bot is almost still after movement set motion flag to 0 to enable Position Hold
      if(abs(total_velocity) < 0.04  && motion == 1){
        motion = 0;
      }

      
      Vel_setpoint = (Kp_pos/10)*Pos_error + (Kd_pos/10)*(Pos_error - last_Pos_error) + Ki_pos*pos_error_sum;
      Vel_setpoint = constrain(Vel_setpoint, -0.83,0.83);
      last_Pos_error = Pos_error;
      }

      // ==========  Velocity Control Section ========= //

      //Velocity Control PID loop runs at 50Hz
      if(loopCount%5 == 0){
      Vel_error =   Vel_setpoint - total_velocity;
      Pitch_offset = (Kp_vel*Vel_error - Kd_vel*(Vel_error - last_Vel_error) + Ki_vel*vel_error_sum);
      Pitch_offset = constrain(Pitch_offset, -20, 20);
      vel_error_sum += Vel_error;
      vel_error_sum = constrain(vel_error_sum, -20,20);
      last_Vel_error = Vel_error;
      }


      // ==========  Pitch Control Section ========= //

      //Pitch Control PID loop runs at 100Hz
      Pitch_error = (-2.7 + Pitch_offset)  - Pitch;  
      PWM = Kp_pitch*Pitch_error + Kd_pitch*(Pitch_error - last_Pitch_error) + Ki_pitch*pitch_error_sum;
      last_Pitch_error = Pitch_error;
      lastPitch = Pitch;
      PWM = (int)constrain(PWM, -220, 220); 

      //Apply calculated PWM to motors
      MotorLeft(-PWM*0.90);                //0.90 is multiplied to compensate for the difference in motors
      MotorRight(-PWM);

      //Reset Loop counter
      if(loopCount >= 10){
        //Reset Loop counter
        loopCount = 0;
      }else{
      loopCount++;
      }
     }
    }
  }
 
  // ==========  Wireless Xbee Controller Section ========= //

  //First waiting for the start delimiter 0x7E and then waiting for the rest of the
  //API frame to arrive and then storing the bytes of the frame in array b
    if(flag == 0){
    if(Serial3.read() == 0x7E){
  flag = 1;
    }
   }
  if(flag){
    if(Serial3.available()>=17){
      flag = 0;
      b[0] = 0x7E;
      for(int i=0; i<17; i++){
        b[i+1] = Serial3.read();
      }
    }

    //Checking if the recieved frame is correct and not corrupted
    for(int j=1;j<18; j++){
      if(b[j] == 0x7E){
      trueFrame = 0;
      break;
      }else{
        trueFrame = 1;
      }
    }
  }

 //Converting the byte holding digital mask of digital pins on Xbee to binary bits and storing the bits in the array digitalMask
 if(trueFrame == 1){
    digitalByte = b[12];
    for(int n=7; n>=0; n--){
      digitalMask[n] = digitalByte%2;
      digitalByte = digitalByte/2;      
    }
  }
  
  //Combining High and Low bytes to get JoyStick Analog values
  X = b[14] + (b[13]*256);
  Y = b[16] + (b[15]*256);

  if((X >=950 && X<=1023) && (Y >=950 && Y<=1023)){        //Neutral
    leftPWMoffset = 0;
    rightPWMoffset = 0;
    if(joyStick != 'N'){
      lastJoystick = joyStick;
      joyStick = 'N';
    }
  } else if((X >=500 && X<=650) && (Y >=950 && Y<=1023)){   //Forward
    leftPWMoffset = 0;
    rightPWMoffset = 0;
    joyStick = 'F';
  } else if((X >=0 && X<=20) && (Y >=950 && Y<=1023)){     //Backward
    leftPWMoffset = 0;
    rightPWMoffset = 0;
    joyStick = 'B';
  }else if((X >=950 && X<=1023) && (Y >=0 && Y<=20)){      //Left
    joyStick = 'L';
    leftPWMoffset = -60;
    rightPWMoffset = 60;
  }else if((X >=950 && X<=1023) && (Y >=500 && Y<=650)){   //Right
    joyStick = 'R';
    leftPWMoffset = 60;
    rightPWMoffset = -60;
  }
}

/*
* Function Name: MotorLeft
* Input: PWM to be applied to the left motor
* Output: None
* Logic: Depending on the sign of PWM the motor will rotate in clockwise or anticlock wise direction
*        Yaw and Rotation offset is also introduced here and PWM is constrained to -255,255
* Example Call: MotorLeft(leftPWM);
*/
void MotorLeft(int PWM) {
  if (PWM < 0) {
    PWM = abs(PWM)+leftPWMoffset - yawOffset;
    PWM = constrain(PWM, 0, 220);
    digitalWrite(24, HIGH);
    digitalWrite(22, LOW);
    analogWrite(44, PWM+35);
  } else {
    PWM = abs(PWM)-leftPWMoffset + yawOffset;
    PWM = constrain(PWM, 0, 220);
    digitalWrite(24, LOW);
    digitalWrite(22, HIGH);
    analogWrite(44, PWM+35);
  }
}

/*
* Function Name: MotorRight
* Input: PWM to be applied to the right motor
* Output: None
* Logic: Depending on the sign of PWM the motor will rotate in clockwise or anticlock wise direction
*        Yaw and Rotation offset is also introduced here and PWM is constrained to -255,255
* Example Call: MotorLeft(rightPWM);
*/
void MotorRight(int PWM) {
  if (PWM < 0) {
    PWM = abs(PWM)+rightPWMoffset + yawOffset;
    PWM = constrain(PWM, 0, 220);
    digitalWrite(26, HIGH);
    digitalWrite(28, LOW);
    analogWrite(46, PWM+35);
  } else {
    PWM = abs(PWM)-rightPWMoffset - yawOffset;
    PWM = constrain(PWM, 0, 220);
    digitalWrite(26, LOW);
    digitalWrite(28, HIGH);
    analogWrite(46, PWM+35);
  }
}

/*
* Function Name: leftEncoder
* Input: None
* Output: None
* Logic: Increase or decrease number of encoder counts when pulse from encoder is detected
* Example Call: leftEncoder();
*/
void leftEncoder() {
  if (digitalRead(pinB_left) == HIGH) {
    leftEncoderCount++;
  } else {
    leftEncoderCount--;
  }
}

/*
* Function Name: rightEncoder
* Input: None
* Output: None
* Logic: Increase or decrease number of encoder counts when pulse from encoder is detected
* Example Call: rightEncoder();
*/
void rightEncoder() {
  if (digitalRead(pinB_right) == HIGH) {
    rightEncoderCount--;
  } else {
    rightEncoderCount++;
  }
}

/*
* Function Name: getLeftEncoderCount
* Input: None
* Output: Number of encoder counts of left motor
* Logic: reurn leftEncoderCount when the function is called
* Example Call: getLeftEncoderCount();
*/
int getLeftEncoderCount(){
  return leftEncoderCount;
}

/*
* Function Name: getRightEncoderCount
* Input: None
* Output: Number of encoder counts of right motor
* Logic: reurn rightEncoderCount when the function is called
* Example Call: getRightEncoderCount();
*/
int getRightEncoderCount(){
  return rightEncoderCount;
}

/*
* Function Name: lowPassFilter
* Input: Raw data to be filtered
* Output: Filtered data after passing raw data through low pass filter
* Logic: Simple first order low pass filter is used 
* Example Call: lowPassFilter(raw);
*/
float lowPassFilter(float raw){
  filteredData = (1-alpha)*raw + alpha*filteredData;
  return filteredData;
}
