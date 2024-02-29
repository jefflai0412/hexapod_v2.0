#include <Adafruit_PWMServoDriver.h>
#include <Wire.h>
#include "math.h"

#define SERVO_FREQ 50  // servo frequency
#define SERVOMIN 98    // the minimun physical limit(after testing)
#define SERVOMAX 490

Adafruit_PWMServoDriver board1 = Adafruit_PWMServoDriver(0x40);
Adafruit_PWMServoDriver board2 = Adafruit_PWMServoDriver(0x41);



void setup() {
  Serial.begin(9600);
  board1.begin();
  board1.setPWMFreq(SERVO_FREQ);
  board2.begin();
  board2.setPWMFreq(SERVO_FREQ);
}


void loop() {
  // sweep(2);
  assemble();
  // angle(2);
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
  // j1
  for (int servo_num = 0; servo_num < 9; servo_num+=3) {
    board1.setPWM(servo_num, 0, (SERVOMIN + SERVOMAX) / 2);
    board2.setPWM(servo_num, 0, (SERVOMIN + SERVOMAX) / 2);
    Serial.printf("j1 set to (SERVOMIN + SERVOMAX) / 2)\n");
  }
  // j2
    for (int servo_num = 1; servo_num < 9; servo_num+=3) {
    board1.setPWM(servo_num, 0, (SERVOMIN + SERVOMAX) / 2);
    board2.setPWM(servo_num, 0, (SERVOMIN + SERVOMAX) / 2);
    Serial.printf("j1 set to (SERVOMIN + SERVOMAX) / 2)\n");
  }
  // j3
    for (int servo_num = 2; servo_num < 9; servo_num+=3) {
    board1.setPWM(servo_num, 0, (SERVOMIN + SERVOMAX) / 2);
    board2.setPWM(servo_num, 0, (SERVOMIN + SERVOMAX) / 2);
    Serial.printf("j1 set to (SERVOMIN + SERVOMAX) / 2)\n");
  }
}

void angle(int servo_num) {
  if (Serial.available()) {
    char command = Serial.read();
    if (command == '0') {
      board1.setPWM(servo_num, 0, SERVOMIN);  // right legs(J1:-90, J2:90, J3:-180)
      board2.setPWM(servo_num, 0, SERVOMIN);  // left legs(J1:90, J2:-90, J3:0)
      Serial.println("right legs(J1:-90, J2:90, J3:-180)");
      Serial.println("left legs(J1:90, J2:-90, J3:0)");
    }
    if (command == '1') {
      board1.setPWM(servo_num, 0, (SERVOMIN + SERVOMAX) / 2);  // right legs(J1:0, J2:0, J3:-90)
      board2.setPWM(servo_num, 0, (SERVOMIN + SERVOMAX) / 2);  // left legs(J1:0, J2:0, J3:-90)
      Serial.println("right legs(J1:0, J2:0, J3:-90)");
      Serial.println("left legs(J1:0, J2:0, J3:-90)");
    }
    if (command == '2') {
      board1.setPWM(servo_num, 0, SERVOMAX);  // right legs(J1:90, J2:-90, J3:0)
      board2.setPWM(servo_num, 0, SERVOMAX);  // left legs(J1:-90, J2:90, J3:-180)
      Serial.println("right legs(J1:90, J2:-90, J3:0)");
      Serial.println("left legs(J1:-90, J2:90, J3:-180)");
    }
  }
}