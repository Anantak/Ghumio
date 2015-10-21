// machineController 
// controls 2 motors and 2 servos using PWM and Servo
// reads 4 encoders using SPI
// gets commands from the controller interface
// transmits commands back for synchronization
// implements limits on signals and maximum allowed changes

// include libraries
#include <Servo.h>
#include <SPI.h>

// communication - in command is of the form 99,255,255,1500,1500
int loopnum = 0;          // loop counter
String inString = "";     // this will hold the incoming data
char terminalChar = '\n'; // this terminates the command
String inCommandID = "";  // this identifies the incoming command
char loopNumStr[3] = "00"; 
int loopStatus = 0;

//// Motors ////
int fullMotorSwingTime = 500;  // 500ms min for a full swing. this sets the max change
int maxMotorChange = 10;
int minMotor = -255;
int maxMotor = 255;
int zeroMotor = 0;

// right motor
const int rearRightMotorPWMLPin = 13;
const int rearRightMotorPWMHPin = 12;
const int rearRightMotorDirPin  = 11;
char rearRightMotorStr[4] = "255";
int  rearRightMotorRead = zeroMotor;
int  rearRightMotor = zeroMotor;
int  last_rearRightMotor = zeroMotor;

// left motor
const int rearLeftMotorPWMLPin = 8;
const int rearLeftMotorPWMHPin = 7;
const int rearLeftMotorDirPin  = 6;
char rearLeftMotorStr[4] = "255";
int  rearLeftMotorRead = zeroMotor;
int  rearLeftMotor = zeroMotor;
int  last_rearLeftMotor = zeroMotor;

//// Servos ////
int fullServoSwingTime = fullMotorSwingTime;
int maxServoChange = 25;
int minServo = 900;
int maxServo = 2100;
int zeroServo = 1500;

// right servo
Servo frontRightServoObject;
const int frontRightServoPin = 9;
char frontRightServoStr[5] = "1500";
int frontRightServoRead = zeroServo;
int frontRightServo = zeroServo;
int last_frontRightServo = zeroServo;

// left servo
Servo frontLeftServoObject;
const int frontLeftServoPin = 10;
char frontLeftServoStr[5] = "1500";
int frontLeftServoRead = zeroServo;
int frontLeftServo = zeroServo;
int last_frontLeftServo = zeroServo;

//// Encoders ////
const int rearRightEncoderSelectPin  = 39;
const int rearLeftEncoderSelectPin   = 41;
const int frontRightEncoderSelectPin = 38;
const int frontLeftEncoderSelectPin  = 40;
unsigned int rearRightEncoderValue = 0;
unsigned int rearLeftEncoderValue = 0;
unsigned int frontRightEncoderValue = 0;
unsigned int frontLeftEncoderValue = 0;

// time calculators
unsigned long current_millis, old_millis, last_msg_time;
unsigned long point1_millis, point2_millis, point3_millis;
long dtime, dtime1, dtime2, dtime3;
long machine_cutoff_time = 500; // stop machine if not heard from controller interface
long time_since_last_msg = 0;

// motor current sensing analog pins
const int rearRightCurrentSensorPin = 11;
const int rearLeftCurrentSensorPin  = 10;
const int frontRightCurrentSensorPin = 9;
const int frontLeftCurrentSensorPin  = 8;
int rearRightCurrentReading = 0;
int rearLeftCurrentReading = 0;
int frontRightCurrentReading = 0;
int frontLeftCurrentReading = 0;
int rearRightCurrent = 0;
int rearLeftCurrent = 0;
int frontRightCurrent = 0;
int frontLeftCurrent = 0;

// temperature sensor
const int temperatureSensorPin = 12;
int temperatureReading = 0;
int temperature = 0;

void setup() {
  
  // start communication over the Serial port
  Serial.begin(115200);
  
  // communication strings
  inString.reserve(25);
  inCommandID.reserve(5);
  
  // setup pwm control
  pinMode(rearRightMotorPWMLPin, OUTPUT);
  pinMode(rearRightMotorPWMHPin, OUTPUT);
  pinMode(rearRightMotorDirPin, OUTPUT);
  pinMode(rearLeftMotorPWMLPin, OUTPUT);
  pinMode(rearLeftMotorPWMHPin, OUTPUT);
  pinMode(rearLeftMotorDirPin, OUTPUT);
  
  // attach servos
  frontRightServoObject.attach(frontRightServoPin);
  frontLeftServoObject.attach(frontLeftServoPin);
  
  // set encoder select pins to output
  pinMode(rearRightEncoderSelectPin, OUTPUT);
  pinMode(rearLeftEncoderSelectPin, OUTPUT);
  pinMode(frontRightEncoderSelectPin,  OUTPUT);
  pinMode(frontLeftEncoderSelectPin,  OUTPUT);

  // initiate encoders' SPI communication
  SPI.begin();
  SPI.setClockDivider(SPI_CLOCK_DIV4);
  SPI.setBitOrder(MSBFIRST);
  SPI.setDataMode(SPI_MODE0);
  
  // timings
  current_millis = millis();
  old_millis = current_millis;
  last_msg_time = current_millis;
  dtime = 20;
  
}

void loop() {
  
  current_millis = millis();
  loopnum += 1; if (loopnum > 99) {loopnum=0;}
  itoa(loopnum, loopNumStr, 10); 
  if (loopnum < 10) {loopNumStr[2]='\0'; loopNumStr[1]=loopNumStr[0]; loopNumStr[0]='0';}

  maxMotorChange = (dtime * (maxMotor - minMotor)) / fullMotorSwingTime;
  maxServoChange = (dtime * (maxServo - minServo)) / fullServoSwingTime;
  
  inString = Serial.readStringUntil(terminalChar);
  if (inString.length() >= 4) {
    
    // parse the command
    inCommandID = inString.substring(0,2);
    inString.substring(3,6).toCharArray(rearRightMotorStr, 4);
    inString.substring(7,10).toCharArray(rearLeftMotorStr, 4);
    inString.substring(11,15).toCharArray(frontRightServoStr, 5);
    inString.substring(16,20).toCharArray(frontLeftServoStr, 5);
    
    // convert to integers
    rearRightMotorRead  = atoi(rearRightMotorStr) - 255;
    rearLeftMotorRead   = atoi(rearLeftMotorStr) - 255;
    frontRightServoRead = atoi(frontRightServoStr);
    frontLeftServoRead  = atoi(frontLeftServoStr);
    
    // time of the last message
    last_msg_time = millis();
  }
  
  time_since_last_msg = millis() - last_msg_time;
  if (time_since_last_msg > machine_cutoff_time) {
    
    // if waiting to hear from controller interface for too long, stop the machine
    rearRightMotorRead  = zeroMotor;
    rearLeftMotorRead   = zeroMotor;
    frontRightServoRead = zeroServo;
    frontLeftServoRead  = zeroServo;
    
  }
  
  // change values safely
  rearRightMotor = safeChange(rearRightMotorRead, last_rearRightMotor, maxMotorChange, minMotor, maxMotor);
  rearLeftMotor = safeChange(rearLeftMotorRead, last_rearLeftMotor, maxMotorChange, minMotor, maxMotor);
  frontRightServo = safeChange(frontRightServoRead, last_frontRightServo, maxServoChange, minServo, maxServo);
  frontLeftServo = safeChange(frontLeftServoRead, last_frontLeftServo, maxServoChange, minServo, maxServo);    
  
  // transfer current values to last values
  last_rearRightMotor  = rearRightMotor;
  last_rearLeftMotor   = rearLeftMotor;
  last_frontRightServo = frontRightServo;
  last_frontLeftServo  = frontLeftServo;
  
  point1_millis = millis();
  
  // write commands
  runRearRightMotor(rearRightMotor);
  runRearLeftMotor(rearLeftMotor);
  frontRightServoObject.writeMicroseconds(frontRightServo);
  frontLeftServoObject.writeMicroseconds(frontLeftServo);
  
  point2_millis = millis();
  
  // read encoders
  rearRightEncoderValue  = AMT203_Read( rearRightEncoderSelectPin ); 
  rearLeftEncoderValue   = AMT203_Read( rearLeftEncoderSelectPin ); 
  frontRightEncoderValue = AMT203_Read( frontRightEncoderSelectPin ); 
  frontLeftEncoderValue  = AMT203_Read( frontLeftEncoderSelectPin ); 
  
  point3_millis = millis();

  // read analog pins for currents
  rearRightCurrentReading = analogRead(rearRightCurrentSensorPin);
  rearLeftCurrentReading  = analogRead(rearLeftCurrentSensorPin);
  frontRightCurrentReading = analogRead(frontRightCurrentSensorPin);
  frontLeftCurrentReading  = analogRead(frontLeftCurrentSensorPin);
  rearRightCurrent = (int) map(rearRightCurrentReading, 0, 1023, -3000, 3000);
  rearLeftCurrent  = (int) map(rearLeftCurrentReading, 0, 1023, -3000, 3000);
  frontRightCurrent = (int) map(frontRightCurrentReading, 0, 1023, -3000, 3000);
  frontLeftCurrent  = (int) map(frontLeftCurrentReading, 0, 1023, -3000, 3000);
  
  // read temperature
  temperatureReading = analogRead(temperatureSensorPin);
  temperature = (int) ( (temperatureReading * 5.0 / 1024.0 - 0.5) * 100);

  // time measurements
  dtime1 = (long) (point1_millis - current_millis);
  dtime2 = (long) (point2_millis - current_millis);
  dtime3 = (long) (point3_millis - current_millis);
  dtime = (long) (current_millis - old_millis);
  old_millis = current_millis;  

  // report  
  Serial.print(loopNumStr);
  Serial.print(","); Serial.print(inCommandID);
  Serial.print(","); Serial.print(rearRightEncoderValue, HEX);
  Serial.print(","); Serial.print(rearLeftEncoderValue, HEX);
  Serial.print(","); Serial.print(frontRightEncoderValue, HEX);
  Serial.print(","); Serial.print(frontLeftEncoderValue, HEX);
    
  //Serial.print(","); Serial.print(dtime1, DEC);
  //Serial.print(","); Serial.print(dtime2, DEC);
  //Serial.print(","); Serial.print(dtime3, DEC);
  Serial.print(","); Serial.print(dtime); 
  
  //Serial.print(","); Serial.print(rearRightMotorStr);
  Serial.print(","); Serial.print(rearRightCurrent, DEC);
  Serial.print(","); Serial.print(rearLeftCurrent, DEC);
  Serial.print(","); Serial.print(frontRightCurrent, DEC);
  Serial.print(","); Serial.print(frontLeftCurrent, DEC);
  Serial.print(","); Serial.print(temperature, DEC);
  //Serial.print(","); Serial.print(maxServoChange, DEC);
  
  Serial.println();
  
} // loop


int safeChange(int newValue, int lastValue, int maxChange, int minValue, int maxValue) {
  int value;
  int change = newValue - lastValue;
  if (abs(change) <= maxChange) {
    value = newValue;
  } else {
    value = lastValue + maxChange * abs(change)/change;
  }
  value = max(min(value, maxValue), minValue);
  return value;
}

/*
  Motor can be run in multiple operationModes. after testing, we only run in drive-brake mode.
  Drive-brake: PWML high/dc, PWMH pwm, DIR high/low; during pwm high V+, pwm low Gnd
  Drive-coast: PWML pwm , PWMH pwm, Dir high/low; during pwm high V+, pwm low disconnected
  Brake-coast: PWML pwm , PWMH low/dc, Dir no effect; during pwm high Gnd, pwm low disconnected
  Locked-anitphase: PWML high/dc, PWMH high, Dir pwm; motor stops at 50% pwm
*/

/* Rear right motor control 
 * speed is determined by the duty cycle (0, 255)
 * positive direction Dir pin is high */
void runRearRightMotor(int dutyCycle) {
  int motorDirection = 255;
  if (dutyCycle >=0) { 
    motorDirection = 255;
  } else {
    motorDirection = 0;
    dutyCycle *= -1;
  }
  //now write to the motor pins
  analogWrite(rearRightMotorPWMLPin, 255);
  analogWrite(rearRightMotorPWMHPin, dutyCycle);
  analogWrite(rearRightMotorDirPin, motorDirection);
}

/* Rear left motor control 
 * speed is determined by the duty cycle (0, 255)
 * positive direction Dir pin is high */
void runRearLeftMotor(int dutyCycle) {
  int motorDirection = 255;
  if (dutyCycle >=0) { 
    motorDirection = 255;
  } else {
    motorDirection = 0;
    dutyCycle *= -1;
  }
  //now write to the motor pins
  analogWrite(rearLeftMotorPWMLPin, 255);
  analogWrite(rearLeftMotorPWMHPin, dutyCycle);
  analogWrite(rearLeftMotorDirPin, motorDirection);
}

unsigned int AMT203_NoOp(int SPIchipSelectPin) {
  unsigned int result = 0;   // result to return
  int delaytime = 20; //delay after setting the pin to high
  byte dataToSend = 0x00;
  // take the chip select low to select the device:
  digitalWrite(SPIchipSelectPin, LOW);
  // send AMT203 a NoOp command. Expect a 0xA5
  result = SPI.transfer(dataToSend);
  // take the chip select high to de-select
  digitalWrite(SPIchipSelectPin, HIGH);
  // return the result:
  delayMicroseconds(delaytime);
  return(result);
}

unsigned int AMT203_Read(int SPIchipSelectPin) {
  int cycler = 10;     // maximum number of wait cycles
  byte inByte = 0;     // byte read from the encoder
  byte msb = 0;        // byte read from the encoder
  byte lsb = 0;        // byte read from the encoder
  int delaytime = 200; //delay after setting the pin to high
  unsigned int result = 0; // reading
  
  // issue the read command
  byte dataToSend = 0x10;
  digitalWrite(SPIchipSelectPin, LOW);
  inByte = SPI.transfer(dataToSend);
  digitalWrite(SPIchipSelectPin, HIGH);
  delayMicroseconds(delaytime);
  //Serial.print("Ack > "); Serial.print(inByte, HEX); Serial.print(", ");
  // cycle through till data appears
  dataToSend = 0x00;
  while (cycler > 0 && inByte != 0x10) {
    digitalWrite(SPIchipSelectPin, LOW);
    inByte = SPI.transfer(dataToSend);
    digitalWrite(SPIchipSelectPin, HIGH);
    delayMicroseconds(delaytime);
    
    //Serial.print(" > "); Serial.print(inByte, HEX); Serial.print(", ");
    cycler--;
  }
  
  digitalWrite(SPIchipSelectPin, LOW);
  msb = SPI.transfer(dataToSend);
  digitalWrite(SPIchipSelectPin, HIGH);
  delayMicroseconds(delaytime); 
  //Serial.print("MSB > "); Serial.print(inByte, HEX); Serial.print(", ");
  
  digitalWrite(SPIchipSelectPin, LOW);
  lsb = SPI.transfer(dataToSend);
  digitalWrite(SPIchipSelectPin, HIGH);
  delayMicroseconds(delaytime);
  result = (msb << 8) | lsb;
  
  //Serial.print("MSB = "); Serial.print(msb, HEX); Serial.print(", ");
  //Serial.print("LSB = "); Serial.print(lsb, HEX); Serial.print(", ");
  //Serial.print("Reading = "); Serial.println(result, HEX);
  
  // return the result
  return(result);
}

// show loopStatus using LEDs on the controller
void showLoopStatus() {
  
}


