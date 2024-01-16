#include <Adafruit_PWMServoDriver.h>
#include <Wire.h>
#include "math.h"

#define SERVO_FREQ 50  // servo frequency
#define SERVOMIN  105  // the minimun physical limit(after testing)
#define SERVOMAX  505 

Adafruit_PWMServoDriver board1 = Adafruit_PWMServoDriver(0x40);




void setup() {
  Serial.begin(9600);
  board1.begin();
  board1.setPWMFreq(SERVO_FREQ);
}


void loop() {
  sweep();
}

void limit_test() {
  if (Serial.available()) {
    char command = Serial.read();
    if (command == '+') {
      SERVOMIN += 5;
      Serial.println(SERVOMIN);
    }
    if (command == '-') {
      SERVOMIN -= 5;
      Serial.println(SERVOMIN);
    }
  }
  float angle = 0;
  int setPWM_value = map(angle, 0, M_PI, SERVOMIN, SERVOMAX);
  board1.setPWM(2, 0, setPWM_value);
}

void sweep() {
  board1.setPWM(2, 0, SERVOMIN);
  Serial.printf("SERVOMIN:%d\n", SERVOMIN);
  delay(1000);
  board1.setPWM(2, 0, SERVOMAX);
  Serial.printf("SERVOMAX:%d\n", SERVOMAX);
  delay(1000);
}