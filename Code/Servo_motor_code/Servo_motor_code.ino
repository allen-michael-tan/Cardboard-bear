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
