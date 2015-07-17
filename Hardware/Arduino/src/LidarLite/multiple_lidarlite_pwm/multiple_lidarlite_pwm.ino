/*
http://pulsedlight3d.com
LIDARLite_PWM_Fix:
A fix for the PWM mode of units shipped prior to January 15, 2015
Problem: 
Some users have noticed a problem with the PWM read mode of LIDAR-Lite 
in the 5-6cm range. When an object is in that range it can cause the 
unit to read a really really really long value that effectively locks 
up the unit. On Arduino's this produces a "0" value that takes a long 
time to read.
Software Solution: 
The software solution for this is to reset the sensor via the PWR_EN 
line when a zero value is read from the sensor. See details below.
Hardware Solution: 
If the software solution is not a good fit for your application simply 
send your unit back to PulsedLight Inc and we'll reprogram it and send 
it back, please be sure to include a return address.
PulsedLight Inc. 
P.O. Box 691 
Bend, OR 97709
*/

const unsigned long max_delay = 20000;    // Setting max_timeout to 20m

unsigned long pulse_width;
unsigned long pulse_width2;
byte cam1_counter = B0;

void setup()
{
  Serial.begin(115200); // Start serial communications
  pinMode(2, OUTPUT); // Set pin 2 as trigger pin
  pinMode(3, INPUT); // Set pin 3 as monitor pin
  pinMode(4, OUTPUT); // Set pin 4 to control power enable line
  digitalWrite(4,HIGH); //Turn sensor on
  digitalWrite(2, HIGH); // Set trigger LOW for continuous read

  pinMode(5, OUTPUT); // Set pin 2 as trigger pin
  pinMode(6, INPUT); // Set pin 3 as monitor pin
  pinMode(7, OUTPUT); // Set pin 4 to control power enable line
  digitalWrite(7,HIGH); //Turn sensor on
  digitalWrite(5, HIGH); // Set trigger LOW for continuous read

}

void loop()
{
  digitalWrite(2, LOW); // Set trigger LOW for continuous read
  pulse_width = pulseIn(3, HIGH, max_delay); // Count how long the pulse is high in microseconds
  pulse_width = pulse_width/10; // 10usec = 1 cm of distance for LIDAR-Lite
  digitalWrite(2, HIGH); // Set trigger LOW for continuous read
  
  digitalWrite(5, LOW); // Set trigger LOW for continuous read
  pulse_width2 = pulseIn(6, HIGH, max_delay); // Count how long the pulse is high in microseconds
  pulse_width2 = pulse_width2/10; // 10usec = 1 cm of distance for LIDAR-Lite
  digitalWrite(5, HIGH); // Set trigger LOW for continuous read
  
  Serial.print(pulse_width);
  Serial.print(", ");
  Serial.println(pulse_width2);
  
  if ((cam1_counter & B01111111) == 0) {
    Serial.println("Resetting 1");
    digitalWrite(4,LOW); // Turn off the sensor
    delay(1);// Wait 1ms
    digitalWrite(4,HIGH); //Turn on te sensor
    delay(1);// Wait 1ms
    
    Serial.println("Resetting 2");
    digitalWrite(7,LOW); // Turn off the sensor
    delay(1);// Wait 1ms
    digitalWrite(7,HIGH); //Turn on te sensor
    delay(1);//Wait 1ms for it to turn on.
  }
  cam1_counter++;

  delay(1); //Delay so we don't overload the serial port
}

  //if (pulse_width != 0){ // If we get a reading that isn't zero, let's print it
  //      pulse_width = pulse_width/10; // 10usec = 1 cm of distance for LIDAR-Lite
  //	//Serial.println(pulse_width); // Print the distance
  //}else{ // We read a zero which means we're locking up. 
  //  Serial.println("Resetting 1");
  //  digitalWrite(4,LOW); // Turn off the sensor
  //  delay(1);// Wait 1ms
  //  digitalWrite(4,HIGH); //Turn on te sensor
  //  delay(1);//Wait 1ms for it to turn on.
  //}
  //delay(1); //Delay so we don't overload the serial port

  //if (pulse_width2 != 0){ // If we get a reading that isn't zero, let's print it
  //      pulse_width2 = pulse_width2/10; // 10usec = 1 cm of distance for LIDAR-Lite
  //	//Serial.println(pulse_width2); // Print the distance
  //}else{ // We read a zero which means we're locking up. 
  //  Serial.println("Resetting 2");
  //  digitalWrite(7,LOW); // Turn off the sensor
  //  delay(1);// Wait 1ms
  //  digitalWrite(7,HIGH); //Turn on te sensor
  //  delay(1);//Wait 1ms for it to turn on.
  //}

