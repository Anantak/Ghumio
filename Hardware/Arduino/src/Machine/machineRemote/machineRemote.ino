
#include <Esplora.h>
#include <SPI.h>

int loopnum = 0;          // loop counter
unsigned long curr_time, last_time, d_time;
int loop_delay = 35;

int xValue = 0;
int yValue = 0;
int slider = 0;

unsigned long button_read_wait = 200;

int buttonState = HIGH;
boolean stopMode = true;
unsigned long button_read_time = 0;

int buttonState2 = HIGH;
boolean onMode2 = false;
unsigned long button_read_time2 = 0;

int buttonState3 = HIGH;
boolean onMode3 = false;
unsigned long button_read_time3 = 0;

int buttonState4 = HIGH;
boolean onMode4 = false;
unsigned long button_read_time4 = 0;

//// Encoders ////
const int rightEncoderSelectPin = 7;
const int leftEncoderSelectPin = 8;
unsigned int rightEncoderValue = 0;
unsigned int leftEncoderValue = 0;


void setup()
{
  
  Esplora.writeRed(0);
  Esplora.writeGreen(0);
  Esplora.writeBlue(0);
  
  Serial.begin(9600);       // initialize serial communication with your computer
  //while (!Serial) {
  //  Esplora.writeRed(255);    
  //}
  Esplora.writeRed(0);    
  
  Serial1.begin(57600);     // initialize serial communication over the header port
  while (!Serial1) {
    Esplora.writeBlue(255);
  }
  Esplora.writeBlue(0);
  
  curr_time = millis();
  last_time = curr_time;
  button_read_time = 0;
  button_read_time2 = 0;
  button_read_time3 = 0;
  button_read_time4 = 0;
  
  pinMode(rightEncoderSelectPin, OUTPUT);
  pinMode(leftEncoderSelectPin, OUTPUT);

  // initiate encoders' SPI communication
  SPI.begin();
  SPI.setClockDivider(SPI_CLOCK_DIV4);
  SPI.setBitOrder(MSBFIRST);
  SPI.setDataMode(SPI_MODE0);

  
} 

void loop()
{
  //readAndPrintControls();
  loopnum += 1; if (loopnum > 99) {loopnum=0;}
  curr_time = millis();
  d_time = curr_time - last_time;
  last_time = curr_time;
  
  // send Joystick X value over the Serial1 port
  xValue = Esplora.readJoystickX();           // read the joystick's X position
  yValue = Esplora.readJoystickY();           // read the joystick's X position
  slider = Esplora.readSlider();              // read the slider
  buttonState  = Esplora.readButton(SWITCH_1); // read the button 1
  buttonState2 = Esplora.readButton(SWITCH_2); // read the button 1
  buttonState3 = Esplora.readButton(SWITCH_3); // read the button 1
  buttonState4 = Esplora.readButton(SWITCH_4); // read the button 1
  
  if (buttonState == LOW) {
    if ((millis() - button_read_time) > button_read_wait) {
      button_read_time = millis();
      stopMode = !stopMode;
    }
  }

  if (buttonState2 == LOW) {
    if ((millis() - button_read_time2) > button_read_wait) {
      button_read_time2 = millis();
      onMode2 = !onMode2;
    }
  }
  
  if (buttonState3 == LOW) {
    if ((millis() - button_read_time3) > button_read_wait) {
      button_read_time3 = millis();
      onMode3 = !onMode3;
    }
  }

  if (buttonState4 == LOW) {
    if ((millis() - button_read_time4) > button_read_wait) {
      button_read_time4 = millis();
      onMode4 = !onMode4;
    }
  }

  if (stopMode || (abs(xValue) >= 510) || (abs(yValue) >= 510)) {
    Esplora.writeRed(50);
    Esplora.writeGreen(0);
    Esplora.writeBlue(0);
  } else {
    if (onMode4) {
      Esplora.writeRed(0);
      Esplora.writeGreen(50);
      Esplora.writeBlue(0);
    } else {
      if (onMode2) {
        Esplora.writeRed(0);
        Esplora.writeGreen(0);
        Esplora.writeBlue(50);
      } else {
        if (onMode3) {
          Esplora.writeRed(20);
          Esplora.writeGreen(20);
          Esplora.writeBlue(0);
        } else {
          Esplora.writeRed(20);
          Esplora.writeGreen(20);
          Esplora.writeBlue(20);
        }
      }
    }
  }
  
  // read encoders
  //leftEncoderValue   = AMT203_NoOp( leftEncoderSelectPin );
  rightEncoderValue  = AMT203_Read( rightEncoderSelectPin ); 
  leftEncoderValue   = AMT203_Read( leftEncoderSelectPin ); 

  //Esplora.writeGreen(10);
  Serial1.print(loopnum);
  Serial1.print(",");Serial1.print(xValue);
  Serial1.print(",");Serial1.print(yValue);
  Serial1.print(",");Serial1.print(stopMode);
  Serial1.print(",");Serial1.print(slider);
  Serial1.print(",");Serial1.print(onMode2);
  Serial1.print(",");Serial1.print(onMode3);
  Serial1.print(",");Serial1.print(onMode4);
  Serial1.print(",");Serial1.print(rightEncoderValue, HEX);
  Serial1.print(",");Serial1.print(leftEncoderValue, HEX);
  Serial1.print("\n");
  //Esplora.writeGreen(0);
  
  delay(loop_delay);                             // a short delay before moving again
}

void readAndPrintControls() {
  int xValue = Esplora.readJoystickX();        // read the joystick's X position
  int yValue = Esplora.readJoystickY();        // read the joystick's Y position
  int button = Esplora.readJoystickSwitch();   // read the joystick pushbutton
  int slider = Esplora.readSlider();           // read the slider
  Serial.print("Joystick X: ");                // print a label for the X value
  Serial.print(xValue);                        // print the X value
  Serial.print("\tY: ");                       // print a tab character and a label for the Y value
  Serial.print(yValue);                        // print the Y value
  Serial.print("\tSlider: ");                  // print a tab character and a label for the button
  Serial.print(slider);                        // print the button value
  Serial.println("");
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
  Serial.println(result, HEX);
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
    
    //Serial.print(">"); Serial.print(inByte, HEX); Serial.print(", ");
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

