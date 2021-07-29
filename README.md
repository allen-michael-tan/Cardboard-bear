# Cardboard bear

This repository showcases how a cardboard sculpture made by [Mr Bartholomew Ting](https://www.instagram.com/butterNmilk/) was modified and integrated with electronics to welcome students who step into Singapore Polytechnic's Makerspace. 

insert original cardboard sculpture pic

insert updated bear

## Concept

insert gif of concept

## Design

The elctrical components used in this project are listed below. 

1. Arduino Uno [1]
2. SM-S3317B servo motors [2] 
2. PIR sensor [1]
3. DFR0299 MP3 player module [1] 
4. GF1002 amplifier module [1]
5. 3W speakers [2]
6. Button switch [1]
7. Blue LED bulb [1]
8. Red LED bulb [1]
9. 1k ohm resistor [3]

### Servo motor

![SM-S3317B]()

Both ears are individually actuated by servo motors. A mount for the servo motors were designed and 3D printed. 

![Servo motor mount]()

Two acrylic servo arms were laser cut, they were used in conjunction with nylon string to translate the motion of the servo motors to the ears.

![Servo arm]()

### PIR sensor

![PIR sensor]()

To attach the PIR sensor to the bottom of the "chin" of the bear, a 3D printed mount was made.

![PIR mount]()

### GF1002 Amplifier module

![GF1002]()

The purpose of this module is to allow users to easily control the volume of the speakers without the use of code. This module was installed inside the body of the bear. Thus, a 3D printed holder was made to ensure that it could be securely installed in the bear.

![GF1002 mount]()

To make it easier for users to adjust the volume of the speakers, a volume knob was designed and 3D printed. It was slotted into the potentiometer of the module. 

![Volume knob]()

### Interface board

An interface board is made to show what state the bear is in. Two different coloured LED bulbs - blue and red, were used to indicate the different states. To make it easy to install, a 3D printed cover with the relevant cutouts was made to cover the board. 

![Interface board cover]()

## Wiring

![Schematic diagram]()

### Circuit boards

## Assembly

### Setup

### Disassembly

## Code 

All the codes used for the project can be found [here](https://github.com/allen-michael-tan/Cardboard-bear/tree/main/Code). 

### Servo motor 

	#include <Servo.h>
	
	Servo left;
	Servo right;
	
	int leftAngle = 20;
	int rightAngle = 25;
	
	void setup() {
	  left.attach(5);
	  right.attach(4);
	  left.write(20);
	  right.write(25);
	}
	
	void loop() {
	  for (rightAngle == 25; leftAngle < 90; leftAngle ++) {
	    left.write(leftAngle);
	    delay(7);
	  }
	  for (leftAngle == 90; rightAngle < 90; rightAngle ++){
	    right.write(rightAngle);
	    delay(7);    
	  }
	  for (rightAngle == 90; leftAngle > 20; leftAngle --) {
	    left.write(leftAngle);
	    delay(7);
	  }
	  for (leftAngle == 20; rightAngle > 25; rightAngle --) {
	    right.write(rightAngle);
	    delay(7);
	  }
	 }

### PIR

	int calibrationTime = 10; // Calibration time (10 - 60 secs)
	
	long unsigned int lowIn; // The time when the sensor outputs a low impulse
	long unsigned int pause = 5000; // The amount of milliseconds the sensor has to be low before assuming all motion has stopped
	
	boolean lockLow = true;
	boolean takeLowTime;
	
	int pirPin = 8; // Connects to PIR output pin
	int pirPos = 9; // Connects to PIR Vcc pin
	int LED = 7; // Connects to arduino pin 7
	
	void setup() {
	  Serial.begin(9600);
	  
	  pinMode(pirPin, INPUT);
	  pinMode(pirPos, OUTPUT);
	  digitalWrite(pirPos, HIGH);
	
	  Serial.println(); // Give sensor time to calibrate
	  Serial.println("Calibrating sensor "); 
	  for (int i = 0; i < calibrationTime; i++){
	    Serial.print(calibrationTime - i);
	    Serial.print("-");
	    delay(1000);
	  }
	  Serial.println();
	  Serial.println("Sensor calibration done!");
	
	  // To prevent the PIR's output from going HIGH immediately after calibrating, this while loop waits until the PIR's output is LOW before ending setup
	  while (digitalRead(pirPin) == HIGH) {
	    delay(500);
	    Serial.print(".");
	  }
	  Serial.print("SENSOR ACTIVE");
	}
	
	void loop() {
	  if(digitalRead(pirPin) == HIGH){ // If the PIR output is HIGH, LED will light up 
	
	    digitalWrite(LED, HIGH);
	 
	    if(lockLow){ // Makes sure the PIR waits for a transition to LOW before any further outputs are made
	      lockLow = false;
	      Serial.println("---");
	      Serial.print("Motion detected at ");
	      Serial.print(millis()/1000);
	      Serial.println(" sec");
	      delay(50);
	    }
	   takeLowTime = true;
	  }
	
	  if(digitalRead(pirPin) == LOW);{
	
	    digitalWrite(LED, LOW);
	
	    if(takeLowTime){
	      lowIn = millis(); // Saves the time of the transistion from HIGH to LOW
	      takeLowTime = false;
	    }
	
	    // If the sensor is LOW for more than the given pause, the PIR will assume all motion has stopped
	    if(!lockLow && millis() - lowIn > pause){ // Make sure this block of code is only executed again after a new motion sequence has been detected
	      lockLow = true;
	      Serial.print("Motion ended at "); // Output
	      Serial.print(millis() - pause/1000);
	      Serial.println(" sec");
	      delay(50);
	    }
	  }
	}

### MP3 

	#include <NeoSWSerial.h>
	#include "DFRobotDFPlayerMini.h"
	
	NeoSWSerial mp3(12,13);
	DFRobotDFPlayerMini myDFPlayer;
	
	void printDetail(uint8_t type, int value);
	
	void setup() {
	  Serial.begin(9600);
	  Serial.println("DFPlyaer with AltSoftSerial Test Begin");
	  mp3.begin(9600);
	  Serial.println("DFRobot DFPlayer Mini Demo");
	  Serial.println("Initializing Player ... (May take 3~5 seconds)");
	
	  if (!myDFPlayer.begin(mp3)) {
	    Serial.println("Unable to begin!");
	    Serial.println("1. Please recheck the connection!");
	    Serial.println("2. Please insert the SD card!");
	    Serial.println(myDFPlayer.readState());
	    Serial.println(myDFPlayer.readFileCounts());
	    while(true){
	      delay(0);
	    }
	  }
	  Serial.println("DFPlayer Mini online.");
	
	  myDFPlayer.volume(30); // Volume settings (from 0 - 30)
	}
	
	void loop() {
	  if (myDFPlayer.available()) {
	    printDetail(myDFPlayer.readType(), myDFPlayer.read()); // Print the detail message from DFPlayer to handle different errors and state.
	  }
	  myDFPlayer.randomAll(); // Randomly play all the tracks in the SD card
	}
	
	void printDetail(uint8_t type, int value){
	  switch (type) {
	    case TimeOut:
	      Serial.println(F("Time Out!"));
	      break;
	    case WrongStack:
	      Serial.println(F("Stack Wrong!"));
	      break;
	    case DFPlayerCardInserted:
	      Serial.println(F("Card Inserted!"));
	      break;
	    case DFPlayerCardRemoved:
	      Serial.println(F("Card Removed!"));
	      break;
	    case DFPlayerCardOnline:
	      Serial.println(F("Card Online!"));
	      break;
	    case DFPlayerUSBInserted:
	      Serial.println("USB Inserted!");
	      break;
	    case DFPlayerUSBRemoved:
	      Serial.println("USB Removed!");
	      break;
	    case DFPlayerPlayFinished:
	      Serial.print(F("Number:"));
	      Serial.print(value);
	      Serial.println(F(" Play Finished!"));
	      break;
	    case DFPlayerError:
	      Serial.print(F("DFPlayerError:"));
	      switch (value) {
	        case Busy:
	          Serial.println(F("Card not found"));
	          break;
	        case Sleeping:
	          Serial.println(F("Sleeping"));
	          break;
	        case SerialWrongStack:
	          Serial.println(F("Get Wrong Stack"));
	          break;
	        case CheckSumNotMatch:
	          Serial.println(F("Check Sum Not Match"));
	          break;
	        case FileIndexOut:
	          Serial.println(F("File Index Out of Bound"));
	          break;
	        case FileMismatch:
	          Serial.println(F("Cannot Find File"));
	          break;
	        case Advertise:
	          Serial.println(F("In Advertise"));
	          break;
	        default:
	          break;
	      }
	      break;
	    default:
	      break;
	  }
	}

Initially the Software.Serial library was used, however, 

### Full code

	#include <Servo.h>
	#include "Arduino.h"
	#include <NeoSWSerial.h>
	#include "DFRobotDFPlayerMini.h"  
	
	
	/*DFPlayer Mini definitions*/ 
	
	NeoSWSerial mp3(12,13); 
	DFRobotDFPlayerMini myDFPlayer;
	
	void printDetail(uint8_t type, int value);
	
	int mp3track;
	
	boolean audioPlay = true;
	
	/*Servo definitions*/
	
	Servo servo1;
	Servo servo2;
	
	// Starting positions of both servos (angle)
	int Angle1 = 20;
	int Angle2 = 25;
	
	/*PIR sensor definitions*/
	
	int calibrationTime = 10; // Calibration time (10 - 60 secs)
	
	long unsigned int lowIn; // The time when the sensor outputs a low impulse
	long unsigned int pause = 5000; // The amount of milliseconds the sensor has to be low before assuming all motion has stopped
	
	boolean lockLow = true;
	boolean takeLowTime;
	
	int pirPin = 8; // Connects to PIR output pin
	int pirPos = 9; // Connects to PIR Vcc pin
	int LEDB = 7; // Connects blue LED to arduino pin 7
	int LEDR = 6; // Connects red LED to arduino pin 6
	
	void setup() {
	  Serial.begin(9600);
	
	  digitalWrite(LEDR, HIGH);
	  delay(3000);
	  digitalWrite(LEDR, LOW);
	
	  /*DFPlayer Mini setup*/
	  
	  mp3.begin(9600);
	  Serial.println();
	  Serial.println(F("Initializing DFPlayer ... (May take 3~5 seconds)"));
	
	  if (!myDFPlayer.begin(mp3)) { // Use softwareSerial to communicate with mp3
	    Serial.println(F("Unable to begin:"));
	    Serial.println(F("1. Please recheck the connection!"));
	    Serial.println(F("2. Please insert the SD card!"));
	    Serial.println(myDFPlayer.readState()); // Read mp3 state
	    Serial.println(myDFPlayer.readFileCounts()); // Read all file counts in SD card
	    while (!myDFPlayer.begin(mp3)) { // Red LED will blink every 1s if mp3 player does not start
	      digitalWrite(LEDR, HIGH);
	      delay(1000);
	      digitalWrite(LEDR, LOW);
	    }
	  }
	  Serial.println(F("DFPlayer Mini online."));
	
	  myDFPlayer.volume(20); // Volume settings (from 0 - 30)
	  
	  /*PIR sensor setup*/
	
	  pinMode(pirPin, INPUT);
	  pinMode(pirPos, OUTPUT);
	  digitalWrite(pirPos, HIGH);
	
	  Serial.println(); // Give sensor time to calibrate
	  Serial.println("Calibrating sensor ");
	  for (int i = 0; i < calibrationTime; i++) {
	    Serial.print(calibrationTime - i);
	    Serial.print("-");
	    delay(1000);
	  }
	  Serial.println();
	  Serial.println("Sensor calibration done!");
	
	  // To prevent the PIR's output from going HIGH immediately after calibrating, this while loops waits until the PIR's output is LOW before ending setup
	  while (digitalRead(pirPin) == HIGH) {
	    delay(500);
	    Serial.print(".");
	  }
	  Serial.print("SENSOR ACTIVE");
	
	  /*Servo motors setup*/  
	
	  servo1.attach(5);
	  servo2.attach(4);
	  servo1.write(Angle1);
	  servo2.write(Angle2);
	
	  myDFPlayer.play(1); //Start up sound
	  digitalWrite(LEDB, HIGH); // Blue LED lights up for 5s when all setups are successful
	  delay(5000);
	  digitalWrite(LEDB, LOW);
	}
	
	void playaudio() { // Function for playing mp3
	  if (audioPlay) {
	    mp3track = random(1,23); // Play a random mp3 file in the sd card (from 1 to how many files there are)
	    myDFPlayer.play(mp3track);
	    audioPlay = false;
	  }
	  return;
	}
	
	void servocontrol() { // Function for controlling servos
	  if (digitalRead(pirPin) == HIGH) {
	    // servo1 will flap down first, then followed by servo2; after which servo1 will go back to its original position, then followed by servo2
	    for (Angle1 == 20; Angle1 < 90; Angle1 ++) {
	      servo1.write(Angle1);
	      delay(7); // Higher number = slower speed
	    }
	    for (Angle1 == 90; Angle2 < 90; Angle2 ++) {
	      servo2.write(Angle2);
	      delay(7);
	    }
	    for (Angle2 == 90; Angle1 > 20; Angle1 --) {
	      servo1.write(Angle1);
	      delay(7);
	    }
	    for (Angle1 == 20; Angle2 > 25; Angle2 --) {
	      servo2.write(Angle2);
	      delay(7);
	    }
	  }    
	  return;
	}
	
	void loop() {
	  if (myDFPlayer.available()) {
	    printDetail(myDFPlayer.readType(), myDFPlayer.read()); // Print the detail message from DFPlayer to handle different errors and state.
	  }
	
	  if(digitalRead(pirPin) == HIGH) { // If the PIR output is HIGH, control servo and play mp3
	
	    digitalWrite(LEDB, HIGH);
	    playaudio();
	    servocontrol();
	
	    if(lockLow) { // Makes sure the PIR waits for a transition to LOW before any further outputs are made
	      lockLow = false;
	      Serial.println("---");
	      Serial.print("Motion detected at ");
	      Serial.print(millis()/1000);
	      Serial.println(" sec");
	      delay(50);
	    }
	    takeLowTime = true;
	  }
	
	  if(digitalRead(pirPin) == LOW) {
	    digitalWrite(LEDB, LOW);
	    audioPlay = true;
	
	    if(takeLowTime) {
	      lowIn = millis(); // Saves the time of the transistion from HIGH to LOW
	      takeLowTime = false; // Make sure this is only done at the start of a LOW phase
	    }
	
	    // If the sensor is LOW for more than the given pause, the PIR will assume all motion has stopped
	    if(!lockLow && millis() - lowIn > pause) { // Make sure this block of code is only executed again after a new motion sequence has been detected
	      lockLow = true;
	      Serial.print("Motion ended at "); // Output
	      Serial.print((millis() - pause)/1000);
	      Serial.println(" sec");
	      delay(50);      
	    }
	  }
	}
	
	void printDetail(uint8_t type, int value) {
	  switch (type) {
	    case TimeOut:
	      Serial.println(F("Time Out!"));
	      break;
	    case WrongStack:
	      Serial.println(F("Stack Wrong!"));
	      break;
	    case DFPlayerCardInserted:
	      Serial.println(F("Card Inserted!"));
	      break;
	    case DFPlayerCardRemoved:
	      Serial.println(F("Card Removed!"));
	      break;
	    case DFPlayerCardOnline:
	      Serial.println(F("Card Online!"));
	      break;
	    case DFPlayerUSBInserted:
	      Serial.println("USB Inserted!");
	      break;
	    case DFPlayerUSBRemoved:
	      Serial.println("USB Removed!");
	      break;
	    case DFPlayerPlayFinished:
	      Serial.print(F("Number:"));
	      Serial.print(value);
	      Serial.println(F(" Play Finished!"));
	      break;
	    case DFPlayerError:
	      Serial.print(F("DFPlayerError:"));
	      switch (value) {
	        case Busy:
	          Serial.println(F("Card not found"));
	          break;
	        case Sleeping:
	          Serial.println(F("Sleeping"));
	          break;
	        case SerialWrongStack:
	          Serial.println(F("Get Wrong Stack"));
	          break;
	        case CheckSumNotMatch:
	          Serial.println(F("Check Sum Not Match"));
	          break;
	        case FileIndexOut:
	          Serial.println(F("File Index Out of Bound"));
	          break;
	        case FileMismatch:
	          Serial.println(F("Cannot Find File"));
	          break;
	        case Advertise:
	          Serial.println(F("In Advertise"));
	          break;
	        default:
	          break;
	      }
	      break;
	    default:
	      break;
	  } 
	}

To help with troubleshooting

## Operation

## Final Product