// Ghumio rangers code
//  Two types of rangers are sensed: Maxbotix sonars and PulsedLight lidars

// Libraries to be included

// Sonars use SoftI2CMaster library
#define SCL_PIN 0              // Arduino Mega pin 21
#define SCL_PORT PORTD 
#define SDA_PIN 1              // Arduino Mega pin 20
#define SDA_PORT PORTD
#define I2C_TIMEOUT 100        //Define a timeout of 100 ms -- do not wait for clock stretching longer than this time
//#define I2C_CPUFREQ (F_CPU/8)//Useful if you plan on doing any clock switching
//#define I2C_FASTMODE 1         //Run in fast mode (400 kHz)
//#define I2C_SLOWMODE 1         //If you do not define the mode it will run at 100kHz with this define set to 1 it will run at 25kHz
#include <SoftI2CMaster.h>     //You will need to install this library

// Sensor addresses
// Each sonar has a I2C address
uint8_t num_sonars = 0;
uint8_t sonar_id[] = {6, 7, 8, 9};
uint8_t sonar_address[] = {222, 224, 226, 228};
uint32_t sonar_delay = 100;    // 100 milliseconds delay for the sonar
uint32_t sonar_trigger_time = 0;

// Each Lidar is sensed using PWM signal so has two pins. Lidars are also cycled using power pins
uint8_t num_lidars = 5;
uint8_t lidar_id[] = {1, 2, 3, 4, 5};
uint16_t lidar_trigger_pin[] = {2, 4, 6, 8, 10};
uint16_t lidar_signal_pin[] = {3, 5, 7, 9, 11};
uint16_t lidar_power_pin[] = {45, 47, 49, 51, 53};
uint32_t lidar_max_delay = 20000; // maximum time to wait in microsec
uint32_t lidar_trigger_time = 0;
uint8_t lidar_reset_counter_mask = B00111111;

// Codes to tell reciever success/ failure etc.
const byte SUCCESS = 0x00;
const byte FAIL = 0x01;
const byte SONAR_TRIGGER_FAIL = 0x02;

// Define this to send data as a packet. Comment it out to send readable text on serial monitor.
#define TRANSMIT_PACKET 0

// Transmission packet variables
uint8_t transmit_counter = 0;
uint8_t data_packet[15] = { '$',                         // Start marker
                                           0x00,                       // Packet number
                                           0x00,                       // Sensor id
                                           0,0, 0,0,                   // Range uint32_t
                                           0,0, 0,0,                   // Delay since trigger uint32_t
                                           0x00,                       // Code - success, fail etc.
                                           0x00, '\r', '\n' };        // Packet termination marker

// Loop variables
uint8_t loop_counter = 0;

void setup() {
  // Start serial communications
  Serial.begin(115200); 
  
  // Initiate the I2C library for sonars
  i2c_init();
  
  // Setup lidar pins
  SetupAllLidars();
  
  // Trigger all sonars
  TriggerAllSonars();
}

void loop() {
  // Read each lidar one-by-one. In between, check for Sonars
  for (uint8_t i=0; i<num_lidars; i++) {
    ReadLidar(i);       // Read ith Lidar
    ReadAllSonars();  // Read sonars after every sonar delay
  }
  loop_counter++;
  ResetAllLidars();    // Reset Lidars after 
}


// Transmit a reading from a ranger
//  Construct a packet, transmit it
void TransmitReading(uint8_t id, uint32_t range, uint32_t delay_since_trigger, byte code) {
#ifdef TRANSMIT_PACKET
  // Transmit as a packet
  data_packet[1] = transmit_counter;
  data_packet[2] = id;
  data_packet[6] = range & 0xFF;
  range = range >> 8; data_packet[5] = range & 0xFF;
  range = range >> 8; data_packet[4] = range & 0xFF;
  range = range >> 8; data_packet[3] = range & 0xFF;
  data_packet[10] = delay_since_trigger & 0xFF;
  delay_since_trigger = delay_since_trigger >> 8; data_packet[9] = delay_since_trigger & 0xFF;
  delay_since_trigger = delay_since_trigger >> 8; data_packet[8] = delay_since_trigger & 0xFF;
  delay_since_trigger = delay_since_trigger >> 8; data_packet[7] = delay_since_trigger & 0xFF;
  data_packet[11] = code;
  Serial.write(data_packet, 15);
#else
  // Transmit as text
  Serial.print(id); Serial.print(',');
  Serial.print(range); Serial.print(',');
  Serial.print(delay_since_trigger); Serial.print(',');
  Serial.print(code); Serial.print(',');
  Serial.println(transmit_counter);
  //Serial.println(loop_counter);
#endif
  transmit_counter++;
}

// Report Sonar error
void ReportSonarError(uint8_t num, uint8_t code) {
  TransmitReading(sonar_id[num], 0, 0, code);
}

// Trigger all sonars
//  Trigger readings at each sonar
bool TriggerAllSonars() {
  for (uint8_t i=0; i<num_sonars; i++) {
    bool rc = TriggerSonar(sonar_address[i]);
    if (rc) ReportSonarError(i, SONAR_TRIGGER_FAIL);
  }
  sonar_trigger_time = millis();
  return true;
}

// Read all sonars
//  Reads each sonar, constructs and transmits a message
bool ReadAllSonars() {
  uint32_t time_since_sonar_trigger = millis() - sonar_trigger_time;
  if (time_since_sonar_trigger >= sonar_delay) {
    // Read each sonar, transmit its reading and trigger them again
    for (uint8_t i=0; i<num_sonars; i++) {
      // Read each sonar
      uint32_t range = 0;
      range = uint32_t(ReadSonar(sonar_address[i]));
      // Transmit sonar reading
      uint32_t delay_since_trigger = millis() - sonar_trigger_time;
      TransmitReading(sonar_id[i], range, delay_since_trigger, SUCCESS);
    }
    // Trigger all sonars again
    TriggerAllSonars();
  }
  return true;
}

// Start a range reading on the sensor
//  Collects and reports an error bit where: 1 = there was an error or 0 = there was no error.
//  INPUTS: byte bit8address = the address of the sensor that we want to command a range reading
//  OUPUTS: bit  errorlevel = reports if the function was successful in taking a range reading: 1 = the function
//	had an error, 0 = the function was successful
bool TriggerSonar(uint8_t bit8address) {
  bool errorlevel = 0;
  bit8address = bit8address & B11111110;               //Do a bitwise 'and' operation to force the last bit to be zero -- we are writing to the address.
  errorlevel = !i2c_start(bit8address) | errorlevel;   //Run i2c_start(address) while doing so, collect any errors where 1 = there was an error.
  errorlevel = !i2c_write(81) | errorlevel;            //Send the 'take range reading' command. (notice how the library has error = 0 so I had to use "!" (not) to invert the error)
  i2c_stop();
  return errorlevel;
}


// Read the range from the sensor at the specified address //
//  Uses the I2C library to read a sensor at the given address
//  Collects errors and reports an invalid range of "0" if there was a problem.
//  INPUTS: byte  bit8address = the address of the sensor to read from
//  OUPUTS: int   range = the distance in cm that the sensor reported; if "0" there was a communication error
uint16_t ReadSonar(uint8_t bit8address) {
  bool errorlevel = 0;
  uint16_t range = 0;
  uint8_t range_highbyte = 0;
  uint8_t range_lowbyte = 0;
  bit8address = bit8address | B00000001;  //Do a bitwise 'or' operation to force the last bit to be 'one' -- we are reading from the address.
  errorlevel = !i2c_start(bit8address) | errorlevel;
  range_highbyte = i2c_read(0);           //Read a byte and send an ACK (acknowledge)
  range_lowbyte  = i2c_read(1);           //Read a byte and send a NACK to terminate the transmission
  i2c_stop();
  range = (range_highbyte * 256) + range_lowbyte;  //compile the range integer from the two bytes received.
  if (errorlevel) {
    return 0;
  } else {
    return range;
  }
}


// Setup lidar
void SetupLidar(uint8_t num) {
  pinMode(lidar_trigger_pin[num], OUTPUT);   // Set trigger pin
  pinMode(lidar_signal_pin[num], INPUT);        // Set signal/monitor pin
  pinMode(lidar_power_pin[num], OUTPUT);    // Set power enable pin
  digitalWrite(lidar_power_pin[num], HIGH);     // Turn Lidar on
  //digitalWrite(2, HIGH);    // Set trigger LOW for continuous read
}

// Setup all Lidars
void SetupAllLidars() {
  for (uint8_t i=0; i<num_lidars; i++) {
    SetupLidar(i);
  }
}

// Read Lidar, transmit the range reading
bool ReadLidar(uint8_t num) {
  // Read Lidar
  unsigned long pulse_width;
  digitalWrite(lidar_trigger_pin[num], LOW); // Set trigger LOW for reading
  lidar_trigger_time = millis();
  pulse_width = pulseIn(lidar_signal_pin[num], HIGH, lidar_max_delay); // Count how long the pulse is high in microseconds
  pulse_width = pulse_width/10; // 10usec = 1 cm of distance for LIDAR-Lite
  digitalWrite(lidar_trigger_pin[num], HIGH); // Unset trigger
  // Transmit reading
  uint32_t delay_since_trigger = millis() - lidar_trigger_time;
  TransmitReading(lidar_id[num], uint32_t(pulse_width), delay_since_trigger, SUCCESS);
  return true;
}

void ResetLidar(uint8_t num) {
  digitalWrite(lidar_power_pin[num], LOW); // Turn off the sensor
  delay(1);// Wait 1ms
  digitalWrite(lidar_power_pin[num], HIGH); // Turn on te sensor
  delay(1);// Wait 1ms
}

void ResetAllLidars() {
  if ((loop_counter & lidar_reset_counter_mask) == 0) {
    //Serial.println("Lidars reset -----------------------");
    for (uint8_t i=0; i<num_lidars; i++) {
      ResetLidar(i);
    }
  }
  //else {Serial.println("No reset");}
}



