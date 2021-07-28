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
