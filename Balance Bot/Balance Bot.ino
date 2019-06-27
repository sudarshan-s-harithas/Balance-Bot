/*
 * Code for Balance Bot , using DMP (Digital Motoion Processing)
 * using Gyroscope to measure the yaw,pitch,roll
 */

#include "I2Cdev.h"
#include "math.h"
#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file


#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

MPU6050 mpu;


#define OUTPUT_READABLE_YAWPITCHROLL





#define INTERRUPT_PIN 2  
#define LED_PIN 13
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
int motrf=11;
int motlf=4;
int motrb=10;
int motlb=9;
int en1=3;
int en2=5;
#define Kp 20
#define Kd 0
#define Ki 0
#define sample_time 0.005      
#define target_angle 2.5
int16_t accy,accz, gyrox,gyrorate;
volatile int motorpwm;
float gyroAngle=0,pitch;
 float accAngle,  currentAngle, prevAngle=0, error, prevError=0, errorSum=0;
// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };

void bye() {  
  // initialize Timer1
  cli();          // disable global interrupts
  TCCR1A = 0;     // set entire TCCR1A register to 0
  TCCR1B = 0;     // same for TCCR1B    
  // set compare match register to set sample time 5ms
  OCR1A = 9999;    
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS11 bit for prescaling by 8
  TCCR1B |= (1 << CS11);
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);
  sei();          // enable global interrupts
}

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

void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif


    Serial.begin(115200);
    while (!Serial); 

    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));


 pinMode(motrf,OUTPUT);
   pinMode(motrb,OUTPUT);
    pinMode(motlf,OUTPUT);
     pinMode(motlb,OUTPUT);
      pinMode(en1,OUTPUT);
       pinMode(en2,OUTPUT);
       
       mpu.initialize();
    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();
bye();
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
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
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

    // configure LED for output
    pinMode(LED_PIN, OUTPUT);
}



// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
    // if programming failed, don't try to do anything
    motorpwm = constrain(motorpwm, -255, 255);
    motor_update(motorpwm);
    Serial.println("hnfjkweh");
    //Serial.println(motorpwm);
    if (!dmpReady) return;

    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) {

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

        #ifdef OUTPUT_READABLE_QUATERNION
            // display quaternion values in easy matrix form: w x y z
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            Serial.print("quat\t");
            Serial.print(q.w);
            Serial.print("\t");
            Serial.print(q.x);
            Serial.print("\t");
            Serial.print(q.y);
            Serial.print("\t");
            Serial.println(q.z);
        #endif

        #ifdef OUTPUT_READABLE_EULER
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetEuler(euler, &q);
            Serial.print("euler\t");
            Serial.print(euler[0] * 180/M_PI);
            Serial.print("\t");
            Serial.print(euler[1] * 180/M_PI);
            Serial.print("\t");
            Serial.println(euler[2] * 180/M_PI);
        #endif

        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            Serial.print("ypr\t");
            Serial.print(ypr[0] * 180/M_PI);
            Serial.print("\t");
            pitch=ypr[1]*180/M_PI;
            Serial.print(pitch);
            Serial.print("\t");
            Serial.println(ypr[2] * 180/M_PI);
        #endif

        #ifdef OUTPUT_READABLE_REALACCEL
            // display real acceleration, adjusted to remove gravity
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetAccel(&aa, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
            Serial.print("areal\t");
            Serial.print(aaReal.x);
            Serial.print("\t");
            Serial.print(aaReal.y);
            Serial.print("\t");
            Serial.println(aaReal.z);
        #endif

        #ifdef OUTPUT_READABLE_WORLDACCEL
            // display initial world-frame acceleration, adjusted to remove gravity
            // and rotated based on known orientation from quaternion
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetAccel(&aa, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
            mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
            Serial.print("aworld\t");
            Serial.print(aaWorld.x);
            Serial.print("\t");
            Serial.print(aaWorld.y);
            Serial.print("\t");
            Serial.println(aaWorld.z);
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

        // blink LED to indicate activity
        blinkState = !blinkState;
        digitalWrite(LED_PIN, blinkState);
    }
}

/* ******************************
 * Function name : ISR (Interrupt Service Routine) 
 * Functionality : To compute the motor pwm value using PID algoritm 
 * Arguments     : None
 * Return Value  : motorpwm
 ******************************
 */

ISR(TIMER1_COMPA_vect)
{
   error = target_angle-pitch;
  errorSum = errorSum + error;  
  errorSum = constrain(errorSum, -255, 255);
  //calculate output from P, I and D values
  motorpwm = Kp*(error) + Ki*(errorSum)*sample_time - Kd*(currentAngle-prevAngle)/sample_time;
  prevAngle = pitch;

}



/* ******************************
 * Function name : motor_update
 * Functionality : To update the pwm value using w.r.t to the start value of motor 
 * Arguments     : motorpwm
 * Return Value  : none ;pwm is updated
 ******************************
 */
void motor_update(float motorpwm)
{
  float left_pwm=0;
  float right_pwm=0;
  left_pwm=constrain(motorpwm+63,-255,+255);
  right_pwm=constrain(motorpwm+55,-255,+255);
  driver(1,left_pwm,63);  // insert the pwm  value at which the left motor starts to rotate, 63 in this case  , use trial and error technique 
  driver(2,right_pwm,55); // insert the pwm  value at which the right motor starts to rotate, 55 in this case
}


/* ******************************
 * Function name : driver
 * Functionality : To map and determine motor direction 
 * Arguments     : motor,pwm_value,min_value
 * Return Value  : none ; motor direction is determined
 ******************************
 */
void driver(int motor,float pwm_value,int min_value)
 {
  
  if(pwm_value==0)
  {

    digitalWrite(motrf,LOW);
    digitalWrite(motrb,LOW);
    digitalWrite(motlf,LOW);
    digitalWrite(motlb,LOW);
  }
  
  if(pwm_value>0)
  {
    pwm_value=map(pwm_value,0,255,min_value,255);
    set_motor_pwm(motor,(unsigned char)pwm_value);
    forward();    
  }

  
 if(pwm_value<0)
 {
  pwm_value=map(pwm_value,-255,0,255,min_value);
  set_motor_pwm(motor,(unsigned char)pwm_value);
  backward();
 }
 
 }


/* ******************************
 * Function name : set_motor_pwm
 * Functionality : To write the pwm_value to motor
 * Arguments     : motor,pwm_value
 * Return Value  : none ; motor pwm is written
 ******************************
 */
 
 void  set_motor_pwm(int motor, unsigned char pwm_value)
 {
  if(motor==1)
  {
    analogWrite(en1,pwm_value);
  }
  if(motor==2)
  {
    analogWrite(en2,pwm_value);
  }
 }


/* ******************************
 * Function name : forward
 * Functionality : To rotate the motors in forward direction 
 * Arguments     : none
 * Return Value  : none ; motor is rotated forward
 ******************************
 */
 
 void forward()
{
    digitalWrite(motrf,HIGH);
    digitalWrite(motrb,LOW);
    digitalWrite(motlf,HIGH);
    digitalWrite(motlb,LOW);
}


/* ******************************
 * Function name : backward
 * Functionality : To rotate the motors in backward direction 
 * Arguments     : none
 * Return Value  : none ; motor is rotated backward
 ******************************
 */
void backward()
{
    digitalWrite(motrf,LOW);
    digitalWrite(motrb,HIGH);
    digitalWrite(motlf,LOW);
    digitalWrite(motlb,HIGH);
}
