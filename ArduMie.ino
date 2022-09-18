/*
This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.

Copyright 2016 Achim Sack, Christian Scholz
*/

#include <Servo.h> 

// Constants
const int ServoBasePin = 9;      // Servo to move the base arm is attached here
const int ServoPolPin = 10;      // Servo to move the polarizer is attached here
const int PhotodiodeInPin = A0;  // Analog input pin where the photodiode is attached
const int LaserPin = 8;


// Variables
int Base = 90;             // variable to store the servo position: Base servo
int Pol = 90;              // variable to store the servo position: Polarizer servo
float PhotodiodeValue = 0;  // variable to store the brightness information
long accA = 0;              // Accumulator for average from analog in (signed!)
long accB = 0;              // Accumulator for average from analog in (signed!)

int NumAverage = 10;        // how many samples to acquire during one measurement

String inputString = "";         // a string to hold incoming data
boolean stringComplete = false;  // whether the string is complete

// Create objects
Servo ServoBase;  // create servo object to control a servo
Servo ServoPol;   // Polarizer Servo 

void setup() 
{ 
  analogReference(INTERNAL);
  pinMode(LaserPin, OUTPUT);      
  ServoBase.attach(9);  // attaches the servo on pin  9 to the servo object 
  ServoPol.attach(10);  // attaches the servo on pin 10 to the servo object 
  Serial.begin(9600);    // Attach serial Port
  inputString.reserve(20);
  
} 
 
 
void loop() { 
  // when a newline arrives:
  if (stringComplete) {
    //Serial.println(inputString);
    command_handler(inputString);
    stringComplete = false; 
  }
 
} 


int command_handler(String command) {
  int retval = false;
  if (command.startsWith("p")) {
    Serial.print("ArduMie\n");
    retval = true;
  }
  
  else if (command.startsWith("x")) {
    Base = command.substring(1).toInt();
    update_position(Base,Pol);
    Serial.print("OK\n");
    retval = true;
  }
  
  else if(command.startsWith("y")) {
    Pol = command.substring(1).toInt();
    update_position(Base,Pol);
    Serial.print("OK\n");
    retval = true;
  }
  
  else if(command.startsWith("O")) {
    digitalWrite(LaserPin, HIGH);
    Serial.print("OK\n");
    retval = true;
  }
  
  else if(command.startsWith("o")) {
    digitalWrite(LaserPin, LOW);
    Serial.print("OK\n");
    retval = true;
  }
  
  else if (command.startsWith("m")) {
    PhotodiodeValue = read_analog();
    Serial.print("Ambient: ");
    Serial.print(accA);
    Serial.print(" Laser: ");
    Serial.print(accB);
    Serial.print(" Diff: ");
    Serial.println(PhotodiodeValue);  
    retval = true;
  }
  
  else if(command.startsWith("N")) {
    NumAverage = command.substring(1).toInt();
    Serial.print("OK\n");
    retval = true;
  }
  
  else {
    Serial.print("NOT OK\n");
  }
  
  inputString = "";
  return retval;
  
}


void update_position(int x, int y) {
  ServoBase.write(x);             // update base servo
  ServoPol.write(y);              // update polarizer servo
}


long read_analog(){
  accA = 0;
  accB = 0;
  for (int i=0;i<NumAverage;i++){
    digitalWrite(LaserPin, LOW);
    delay(3);
    accA += analogRead(PhotodiodeInPin);
    digitalWrite(LaserPin, HIGH);
    delay(3);
    accB += analogRead(PhotodiodeInPin);
  }
  
  digitalWrite(LaserPin, LOW);
  return accB-accA;
  
}


/*
  SerialEvent occurs whenever a new data comes in the
 hardware serial RX.  This routine is run between each
 time loop() runs, so using delay inside loop can delay
 response.  Multiple bytes of data may be available.
 */
void serialEvent() {
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read(); 
    // add it to the inputString:
    inputString += inChar;
    // if the incoming character is a newline, set a flag
    // so the main loop can do something about it:
    if (inChar == '\r') {
      stringComplete = true;
    } 
  }
}
