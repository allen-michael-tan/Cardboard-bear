# Cardboard bear

This repository showcases how a cardboard sculpture made by [Mr Bartholomew Ting](https://www.instagram.com/butterNmilk/) was modified and integrated with electronics to welcome students who step into Singapore Polytechnic's Makerspace. 

![Cardboard lion](https://github.com/allen-michael-tan/Cardboard-bear/blob/main/Images/Cardboard%20lion.jpeg) 

This was how the original cardboard sculpture looked like. It was a lion, standing at a staggering height of over three meters! However, it was too tall to be placed in Makerspace which has a ceiling height of less than three meters. Therefore, modifications to the sculpture had to be made in order for it to be transported and placed in Makerspace.

Hence, the decision was made to remove the mane of the lion which happened to be enough height reduction for the sculpture to fit in Makerspace.

![Cardboard bear](https://github.com/allen-michael-tan/Cardboard-bear/blob/main/Images/Cardboard%20bear.jpeg) 

This was how the cardboard sculpture after its "haircut". Without its mane, the lion looked very similar to a bear. Thus, the cardboard bear sculpture was born.

## Concept

![Overview](https://github.com/allen-michael-tan/Cardboard-bear/blob/main/Images/Overview.jpg)

The concept of this project is when someone walks past the bear, it will be able to sense it through a sensor. After which, it will "interact" with the person by flapping its ear and playing a short audio track towards the person. 

Below is a short illustration of the concept.

![Concept](https://github.com/allen-michael-tan/Cardboard-bear/blob/main/Images/Concept.gif)

## Design

All CAD models were modelled using Autodesk Inventor, the files can be found [here](https://github.com/allen-michael-tan/Kinematic-clock/tree/main/Inventor%20files).

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

![SM-S3317B](https://github.com/allen-michael-tan/Cardboard-bear/blob/main/Images/SM-S3317B.jpg)

Both ears are individually actuated by servo motors. A mount for the servo motors were designed and 3D printed. 

![Servo mount](https://github.com/allen-michael-tan/Cardboard-bear/blob/main/Images/Servo%20mount.jpg)

Two acrylic servo arms of 5mm thickness were laser cut, they were used in conjunction with nylon string to translate the motion of the servo motors to the ears.

![Servo arm](https://github.com/allen-michael-tan/Cardboard-bear/blob/main/Images/Servo%20arm.jpg)

****

### PIR sensor

![PIR sensor](https://github.com/allen-michael-tan/Cardboard-bear/blob/main/Images/PIR%20sensor.jpg)

To attach the PIR sensor to the bottom of the "chin" of the bear, a 3D printed mount was made.

![PIR mount](https://github.com/allen-michael-tan/Cardboard-bear/blob/main/Images/PIR%20mount.jpeg)

****

### GF1002 Amplifier module

![GF1002](https://github.com/allen-michael-tan/Cardboard-bear/blob/main/Images/GF1002.jpg)

The purpose of this module is to allow users to easily control the volume of the speakers without the use of code. This module was installed inside the body of the bear. Thus, a 3D printed holder was made to ensure that it could be securely installed in the bear.

![GF1002 mount](https://github.com/allen-michael-tan/Cardboard-bear/blob/main/Images/GF1002%20mount.jpeg)

To make it easier for users to adjust the volume of the speakers, a volume knob was designed and 3D printed. It was slotted into the potentiometer of the module. 

![Volume knob](https://github.com/allen-michael-tan/Cardboard-bear/blob/main/Images/Volume%20knob.jpeg)

****

### Interface board

An interface board is made to show what state the bear is in. Two different coloured LED bulbs - blue and red, were used to indicate the different states. To make it easy to install, a 3D printed cover with the relevant cutouts was made to cover the board. 

![Interface board cover](https://github.com/allen-michael-tan/Cardboard-bear/blob/main/Images/Interface%20board%20cover.jpeg)

## Electrical Wiring

Below is full schematic diagram of all components used.

![Schematic diagram](https://github.com/allen-michael-tan/Cardboard-bear/blob/main/Images/Schematic%20diagram.jpg)

The amplifier module and servo motors will not draw power from the Arduino because the combined load of all components exceeded what the Arduino can output. Instead, they will draw power from an external source that outputs 5V DC.

![Power plug](https://github.com/allen-michael-tan/Cardboard-bear/blob/main/Images/Power%20plug.jpeg)

A multi-USB phone charger wall plug was the choice of power delivery. It converts AC power to DC power and has enough current in each USB socket to power the Arduino, amplifier module, and the servo motors.  

### Circuit boards

![Uno board](https://github.com/allen-michael-tan/Cardboard-bear/blob/main/Images/Uno%20board.jpeg) 

*Arduino Uno expansion board*

![MP3 board](https://github.com/allen-michael-tan/Cardboard-bear/blob/main/Images/MP3%20board.jpeg)

*MP3 module expansion board* 

![Interface board](https://github.com/allen-michael-tan/Cardboard-bear/blob/main/Images/Interface%20board.jpeg)

The interface board consists of a button switch, red and blue LED bulbs. The switch is a reset switch for the entire circuit while the LEDs are indicating lights.

Extension cables of varying lengths were made, as for some components such as the servo motors require long wires (at least 2m) to be connected to the Arduino and power plug.

## Assembly

### Servo motors and ears

![Servo assembly](https://github.com/allen-michael-tan/Cardboard-bear/blob/main/Images/Servo%20assembly.jpg)

As shown was the assembly of the ear movement system. The servo motor (blue box) was bolted onto the servo mount with bolts and nuts. Next, holes were drilled into the top of the head so that the mount could be bolted down. At the same time, the acrylic servo arm was also securely attached to the servo motor with screws. Lastly, nylon thread [red box] was used to connect the servo arm to the ear through holes in the servo arm and holes that were pre-drilled into the ear.   

The process for the right ear was exactly the same. 

****

### GF1002 and speakers

![GF1002 assembly](https://github.com/allen-michael-tan/Cardboard-bear/blob/main/Images/GF1002%20assembly.jpeg)

This was how the amplifier module was installed onto its mount. 

![GF1002 installed](https://github.com/allen-michael-tan/Cardboard-bear/blob/main/Images/GF1002%20installed.jpg)

To attach the mount onto the body, double-sided foam tape was used (red arrow). A hole was pre-drilled to let the potentiometer protude out so that the volume knob can be pushed in from the front. 

![Speaker w tape](https://github.com/allen-michael-tan/Cardboard-bear/blob/main/Images/Speaker%20w%20tape.jpeg)

The speakers were adhered onto the bear with the use of double-sided foam tape placed around the speaker as shown above. 

![Speaker placement](https://github.com/allen-michael-tan/Cardboard-bear/blob/main/Images/Speaker%20placement.jpg)

The speakers were attached behind the bow tie of the bear in the following circled locations.

![Speaker installed](https://github.com/allen-michael-tan/Cardboard-bear/blob/main/Images/Speaker%20installed.jpg) 

This was how the speaker (red box) looked like from behind the bow tie. 

Furthermore, the volume knob (blue box) was also pushed into the potentiometer. 

****

### Arduino Uno and MP3 module

To allow for easy access to the pins of the arduino, a hinged cut out was made to the top of the body of the bear. This way, in case any maintainence or troubleshooting had to be done, the head would only need to be tilted to make room for the cut out to be lifted up.

![Board mounting](https://github.com/allen-michael-tan/Cardboard-bear/blob/main/Images/Board%20mounting.jpeg)

The Arduino and MP3 expansion boards are adhered to the underside of the cut out using double-sided foam tape as shown above (Arduino and MP3 module had already been installed here). 

****

### PIR sensor

![PIR assembly](https://github.com/allen-michael-tan/Cardboard-bear/blob/main/Images/PIR%20assembly.jpg)

In the assembly of the PIR sensor to its mount, two screws (blue arrows) were used.

The PIR sensor was mounted on the underside of the bear's chin (blue box). However, velcro strips were used in place of bolts and nuts in the attachment of the sensor. This was because should the bear be transported elsewhere, it would be easier to pull the sensor away from the mount rather than to remove screws.

![Velcro](https://github.com/allen-michael-tan/Cardboard-bear/blob/main/Images/Velcro.jpg)

****

### Interface board

The interface board was covered with its 3D printed cover.

![Interface mounting](https://github.com/allen-michael-tan/Cardboard-bear/blob/main/Images/Interface%20mounting.jpeg)

To attach this board, two circular holes and a square hole above the bow tie were respectively drilled and cut out. Subsequently, the interface board with its covered was attached from the inside.  

![Interface installed](https://github.com/allen-michael-tan/Cardboard-bear/blob/main/Images/Interface%20installed.jpg)

Double-sided foam tape was used to adhere the board (blue arrow) to the bear. 

****

### Putting them together

Before the head was placed on top of the body, all wires were connected to theirs corresponding ports except for the data wire of both servo motors. This was because those particular wires had to be connected to the GPIO pins of the Arduino which was situated in the body whereas the data wires were coming in from the head. 

Thus, as previously mentioned, a custom soldered wire was created, were the data wires were intertwined and made especially long. 

Steps taken to place the head onto the body:

1. After all other wires were connected, the head was lifted and placed on top of the body.
2. The data wires (encircled with yellow) of the servo motors were shoved into the gap between the head and face of the bear as shown below.
![Servo wire through head 1](https://github.com/allen-michael-tan/Cardboard-bear/blob/main/Images/Servo%20wire%20through%20head%201.jpg)
3. After that, the wires were routed through the gap to the bottom of the head. *(A white line was drawn on the image below for illustration.)*
![Servo wire through head 2](https://github.com/allen-michael-tan/Cardboard-bear/blob/main/Images/Servo%20wire%20thorugh%20head%202.jpg)
4. Finally, the cut out in the body was lifted up, and the data wires were connected to the Arduino expansion board as shown (red box). *(Note, the shiny part of the dupont head should be face outside)
![Servo wire connected](https://github.com/allen-michael-tan/Cardboard-bear/blob/main/Images/Servo%20wire%20connected.jpg)  

### Disassembly

For disassembly, there is a couple steps.

1. Disconnect the data pins of the servo motors from the Arduino expansion board and remove them from the body.
2. Detach the PIR sensor from the chin.
3. Remove the head from the body by carrying it away.

Now, the bear sculpture is in two pieces and can be transported safely and easily.

## Code 

All the codes used for the project can be found [here](https://github.com/allen-michael-tan/Cardboard-bear/tree/main/Code). 

### Servo motor code

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

### PIR sensor code

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

### MP3 module code

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

Initially the <SoftwareSerial.h> library to control the MP3 module was used, however, that library conflicted with the <servo.h> library used to control the servos. It caused the servo motors to jitter and sometimes made them go spastic. Furthermore, the conflict also made the MP3 module unable to play any error.

Therefore, a solution to this was to use the <NeoSWSerial.h> library to control the MP3 module. 

Although there were other solutions such as using a servo motor driver to drive the servo motors or changeingto a different microcontroller such as an Arduino Mega that has hardware serial already built into the board. The <NeoSWSerial.h> library solution was ultimately chosen due to it being the easiest and lowest cost solution to implement.

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

As mentioned before, apart of being used to reset the entire circuit, the other function of the interface board is to indicate the different state at which the bear is at. There are four states.

**First state: System start up**

- When the bear is first powered on, the red LED light up for 3 seconds.

**Second state: MP3 startup failure**

- If the MP3 failed to start, the red LED will blink every 1 second.

**Third state: Successful start up**

- Upon a successful system start up, the blue LED will light up for 5 seconds. In addition to that, the speakers will play a welcome message to indicate the bear is working.

**Fourth state: Motion detected**

- When the PIR sensor detects any motion i.e. a student walking past, the blue LED will turn on as long as there is any motion detected until all motion has ended five seconds after the last detected motion. 

## Operation

**Start up problem**

There is this problem where the speakers will produce satic noise immediately after the bear was powered on. To find out more about this problem, the link is below.

[![Problem video](https://img.youtube.com/vi/13f0RLJsI6g/0.jpg)](https://youtu.be/13f0RLJsI6g)

The solution is to turn off and on the bear repeatedly until the static noise goes away upon powering on. It is important to wait for the plug to power down completely before switching it back on again as the plug may retain power for awhile even after the switch is turned off. To find out how to implement this solution, click on the video below.

[![Solution video](https://img.youtube.com/vi//0.jpg)](https://youtu.be/)

**Adding/removing sound tracks**

To add or remove sound tracks, the steps are as follows:

1. Remove the micro SD card from the MP3 module.
2. Open the mp3 folder in the SD card.
3. Add or remove sound tracks in the folder, follow the naming convention as the other tracks in the folder.
![MP3 folder](https://github.com/allen-michael-tan/Cardboard-bear/blob/main/Images/MP3%20folder.jpg) **Note: It is very important to not remove or change the sound track named "001" and keep the sound track within 5 seconds and the format is in ".mp3".**
4. Insert the SD card back to the MP3 module.
5. For this line of code - " mp3track = random(1,23);" change the number "23" to the number of the last sound track. For example if the final sound track is named "031.mp3", change "23" to "31". 
![Code ss](https://github.com/allen-michael-tan/Cardboard-bear/blob/main/Images/Code%20ss.jpg)

## Final Product

Below are an image of the finished cardboard bear project and a short video of the bear in operation.

![Final product](https://github.com/allen-michael-tan/Cardboard-bear/blob/main/Images/Final%20product.jpeg)

*Finished project*

[![Product video](https://img.youtube.com/vi/WaNZcLU6RV4/0.jpg)](https://youtu.be/WaNZcLU6RV4)

*Short video of the bear in action*