#include <Adafruit_PWMServoDriver.h>
#include <Wire.h>
#include "math.h"

#define SERVO_FREQ 50  // servo frequency
#define SERVOMIN 97    // the minimun physical limit(after testing)
#define SERVOMAX 505

Adafruit_PWMServoDriver board1 = Adafruit_PWMServoDriver(0x40);



void setup() {
  Serial.begin(9600);
  board1.begin();
  board1.setPWMFreq(SERVO_FREQ);
}


void loop() {
  // sweep(2);
  // assemble();
  angle(2);
}

void limit_test() {
  if (Serial.available()) {
    char command = Serial.read();
    if (command == '+') {
      // SERVOMIN += 5; 
      Serial.println(SERVOMIN);
    }
    if (command == '-') {
      // SERVOMIN -= 5;
      Serial.println(SERVOMIN);
    }
  }
  float angle = 0;
  int setPWM_value = map(angle, 0, M_PI, SERVOMIN, SERVOMAX);
  board1.setPWM(2, 0, setPWM_value);
}

void sweep(int servo) {
  board1.setPWM(servo, 0, SERVOMIN);
  Serial.printf("SERVOMIN:%d\n", SERVOMIN);
  delay(1000);
  board1.setPWM(servo, 0, SERVOMAX);
  Serial.printf("SERVOMAX:%d\n", SERVOMAX);
  delay(1000);
}

void assemble() {
  for (int servo_num = 0; servo_num < 15; servo_num++) {
    board1.setPWM(servo_num, 0, SERVOMAX);
    Serial.printf("servo_%d set to SERVOMAX\n", servo_num);
  }
}

void angle(int servo_num) {
  if (Serial.available()) {
      char command = Serial.read();
      if (command == '0') {
        board1.setPWM(servo_num, 0, SERVOMIN);  // SERVOMIN(J1:-90, J2:90, J3:-180)
        Serial.println("SERVOMIN(J1:-90, J2:90, J3:-180)");
      }
      if (command == '1') {
        board1.setPWM(servo_num, 0, (SERVOMIN+SERVOMAX) / 2);  // (J1:0, J2:0, J3:-90)
        Serial.println("(J1:0, J2:0, J3:-90)");
      }
      if (command == '2') {
        board1.setPWM(servo_num, 0, SERVOMAX);  // (J1:90, J2:-90, J3:0)
        Serial.println("SERVOMAX(J1:90, J2:-90, J3:0)");
      }
      
      
  }
}