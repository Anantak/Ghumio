// Ghumio Beacon controller
// Manuj Naman
// Apr 2015

// This is adapted from Jeff Rowberg's I2Cdevlib available at https://github.com/jrowberg/i2cdevlib
// Under i2cdevlib/Arduino/MPU6050/Examples/MPU6050_DMP6/MPU6050_DMP6.ino

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high

/* =========================================================================
   NOTE: In addition to connection 3.3v, GND, SDA, and SCL, this sketch
   depends on the MPU-6050's INT pin being connected to the Arduino's
   external interrupt #0 pin. On the Arduino Uno and Mega 2560, this is
   digital I/O pin 2. On Due this can be set to whatever pin needed.
 * ========================================================================= */

/* =========================================================================
   NOTE: Power reset pin is setup that provides power to the IMU. This is
   brought low at startup. IMU is then switched back on and initialized.
   This makes the IMU startup much more reliable.
 * ========================================================================= */

/* =========================================================================
   NOTE: IMU's VIO pin is set to HIGH. This is necessary to make it work.
 * ========================================================================= */

#define INTERRUPT_PIN 2
#define POWER_RESET_PIN 3

#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
//Quaternion q;           // [w, x, y, z]         quaternion container
//VectorInt16 aa;         // [x, y, z]            accel sensor measurements
//VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
//VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
//VectorFloat gravity;    // [x, y, z]            gravity vector
//float euler[3];         // [psi, theta, phi]    Euler angle container
//float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// raw vectors of information
int16_t dmp_q[4];
int16_t dmp_a[3];
int16_t dmp_r[3];
//int16_t temperature;  // getTemperature() does not exist

// packet structure for tranmitting IMU readings
uint8_t imuPacket[20] = { '$', 
                                           0x00,                       // Message number
                                           0,0, 0,0, 0,0, 0,0,     // Quaternion 4 int16
                                           0,0, 0,0, 0,0,            // Acceleration 3 int16
                                           0x00,                       // Packet number
                                           0x00, '\r', '\n' };

uint8_t send_every_nth_message = 2;
uint8_t send_counter = 0;

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
        //TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize serial communication
    // (115200 chosen because it is required for Teapot Demo output, but it's
    // really up to you depending on your project)
    //Serial.begin(115200);
    Serial.begin(57600);
    while (!Serial); // wait for Leonardo enumeration, others continue immediately
    
    Serial1.begin(57600);     // initialize serial communication over the header port
    while (!Serial1) {} // wait for port to be ready

    // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3v or Ardunio
    // Pro Mini running at 3.3v, cannot handle this baud rate reliably due to
    // the baud timing being too misaligned with processor ticks. You must use
    // 38400 or slower in these cases, or use some kind of external separate
    // crystal solution for the UART timer.
    
    // Reset power on the IMU chip - this is important for reliable initiation
    pinMode(POWER_RESET_PIN, OUTPUT);
    Serial.println(F("Powering down and waiting..."));
    digitalWrite(POWER_RESET_PIN, LOW);
    delay(1000);
    Serial.println(F("Powering up."));
    digitalWrite(POWER_RESET_PIN, HIGH);

    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // wait for ready
    //Serial.println(F("\nSend any character to begin DMP programming and demo: "));
    //while (Serial.available() && Serial.read()); // empty buffer
    //while (!Serial.available());                 // wait for data
    //while (Serial.available() && Serial.read()); // empty buffer again

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // If we need to set gyro offsets in hardware, do it here, scaled for min sensitivity
    //mpu.setXGyroOffset(0);
    //mpu.setYGyroOffset(0);
    //mpu.setZGyroOffset(0);
    //mpu.setZAccelOffset(0);

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(INTERRUPT_PIN, dmpDataReady, RISING);
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
    if (!dmpReady) {

        char reading;
        if (Serial.available()) {
          reading = Serial.read();
        }
        if (reading == 'r') {
          Serial.println(F("Resetting device"));
          setup();
        }
    } else {
        
      // wait for MPU interrupt or extra packet(s) available
      while (!mpuInterrupt && fifoCount < packetSize) {
          // other program behavior stuff here
          // if you are really paranoid you can frequently test in between other
          // stuff to see if mpuInterrupt is true, and if so, "break;" from the
          // while() loop to immediately process the MPU data
          Serial.println("Wait.");
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
          
          // Copy quaternion data to imuPacket
          imuPacket[2] = fifoBuffer[0];
          imuPacket[3] = fifoBuffer[1];
          imuPacket[4] = fifoBuffer[4];
          imuPacket[5] = fifoBuffer[5];
          imuPacket[6] = fifoBuffer[8];
          imuPacket[7] = fifoBuffer[9];
          imuPacket[8] = fifoBuffer[12];
          imuPacket[9] = fifoBuffer[13];

          // Copy acceleration data to imuPacket
          imuPacket[10] = fifoBuffer[28];
          imuPacket[11] = fifoBuffer[29];
          imuPacket[12] = fifoBuffer[32];
          imuPacket[13] = fifoBuffer[33];
          imuPacket[14] = fifoBuffer[36];
          imuPacket[15] = fifoBuffer[37];
          
          imuPacket[1]++; // messageCount, loops at 0xFF on purpose
            
          // send data to the serial port
          send_counter = (send_counter+1) % send_every_nth_message;
          if (send_counter==0) {
            Serial1.write(imuPacket, 20);
            imuPacket[16]++; // packetCount, loops at 0xFF on purpose
          }

          // send all captured data to the serial port
          /*mpu.dmpGetQuaternion(dmp_q, fifoBuffer);
          mpu.dmpGetAccel(dmp_a, fifoBuffer);
          mpu.dmpGetGyro(dmp_r, fifoBuffer);
          
          Serial.print("BeaconIMU ");
          Serial.print(dmp_q[1]); Serial.print(",");
          Serial.print(dmp_q[2]); Serial.print(",");
          Serial.print(dmp_q[3]); Serial.print(",");
          Serial.print(dmp_q[0]); Serial.print(",");
          Serial.print(dmp_a[0]); Serial.print(",");
          Serial.print(dmp_a[1]); Serial.print(",");
          Serial.print(dmp_a[2]); Serial.print(",");
          Serial.print(dmp_r[0]); Serial.print(",");
          Serial.print(dmp_r[1]); Serial.print(",");
          Serial.println(dmp_r[2]);*/

          /*Serial1.print("BeaconIMU ");
          Serial1.print(dmp_q[1]); Serial1.print(",");
          Serial1.print(dmp_q[2]); Serial1.print(",");
          Serial1.print(dmp_q[3]); Serial1.print(",");
          Serial1.print(dmp_q[0]); Serial1.print(",");
          Serial1.print(dmp_a[0]); Serial1.print(",");
          Serial1.print(dmp_a[1]); Serial1.print(",");
          Serial1.print(dmp_a[2]); Serial1.print(",");
          Serial1.print(dmp_r[0]); Serial1.print(",");
          Serial1.print(dmp_r[1]); Serial1.print(",");
          Serial1.println(dmp_r[2]); Serial1.print("\n");*/

          // blink LED to indicate activity
          //blinkState = !blinkState;
          //digitalWrite(LED_PIN, blinkState);
      }
   }
}
