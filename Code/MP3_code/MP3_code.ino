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
