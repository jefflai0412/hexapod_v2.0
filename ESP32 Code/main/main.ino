#include "main.h"
#include <math.h>
#include <Adafruit_PWMServoDriver.h>
#include <Wire.h>

//==============================================================================
//                             servo defines
//==============================================================================
#define SERVO_FREQ 50  // servo frequency
#define SERVOMIN 98    // the minimun physical limit(after testing)
#define SERVOMAX 490
//==============================================================================
//                                  END
//==============================================================================
//==============================================================================
//                               全域變數
//==============================================================================
// 待修改 (末端到j1的相對座標)
float initial_point[6][3] = {
  { 57.385, 0, -76.814 },  // right legs
  { -57.385, 0, -76.814 }  // left legs
};

float step_size = 15;    //代表每隻腳的圓的半徑
float step_height = 20;  //每次腳要抬高的高度

float l1 = 0;       // 第一個link的長度
float l2 = 50;      // 第二個link的長度
float l3 = 60.612;  // 第三個link的長度

float moving_cycle_lenth = 4;  // 整個循環的長度

int interpolate_num = 20.00;  // 將每步切成幾部分

float radian_dir = -M_PI / 2;  // 行進方向

int step_delay = 100;  //  每一步之間的間隔

Adafruit_PWMServoDriver board1 = Adafruit_PWMServoDriver(0x40);
Adafruit_PWMServoDriver board2 = Adafruit_PWMServoDriver(0x41);

//==============================================================================
//                                  END
//==============================================================================

//==============================================================================
//                         Functions Declaration
//==============================================================================
void move(float dir);
void interpolate(int step_num, float radian_dir);
void coor2angle(int leg_num, float X, float Y, float Z);
void angle2servo(int servo_num, float j1, float j2, float j3);
//==============================================================================
//                                  END
//==============================================================================

void setup() {
  Serial.begin(9600);
  board1.begin();
  board1.setPWMFreq(SERVO_FREQ);
  board2.begin();
  board2.setPWMFreq(SERVO_FREQ);
}

void loop() {
  move(radian_dir);  // 直走
}



//==============================================================================
//                                動作循環
//==============================================================================
void move(float dir) {
  for (int step = 0; step < moving_cycle_lenth; step++) {
    // Serial.printf("-----------------------------------------------\n-----------------------%d-----------------------\n-----------------------------------------------\n", step);
    delay(500);
    interpolate(step, radian_dir);
    delay(step_delay);
  }
}
//==============================================================================
//                                  END
//==============================================================================

//==============================================================================
//                                內插法
//==============================================================================
void interpolate(int step_num, float radian_dir) {
  // 更新動作組(because of the change of radian_dir)
  float moving_cycle[4][3] = {
    { step_size * cos(radian_dir), step_size * sin(radian_dir), 0 },
    { 0, 0, 0 },
    { -step_size * cos(radian_dir), -step_size * sin(radian_dir), 0 },
    { 0, 0, step_height }
  };

  // 判斷上一步是第幾步
  int last_step_num = step_num - 1;
  if (last_step_num == -1)  // 0的上一步是3
    last_step_num = 3;

  // 計算兩步之間的差值
  float difference[3] = {
    moving_cycle[step_num][0] - moving_cycle[last_step_num][0],
    moving_cycle[step_num][1] - moving_cycle[last_step_num][1],
    moving_cycle[step_num][2] - moving_cycle[last_step_num][2]
  };
  // interpolation
  for (int i = 1; i <= interpolate_num; i++) {
    for (int leg_num = 0; leg_num < 6; leg_num++) {
      if (leg_num < 3) {
        coor2angle(leg_num,
                   initial_point[0][0] + moving_cycle[last_step_num][0] + difference[0] * (i / static_cast<float>(interpolate_num)),

                   initial_point[0][1] + moving_cycle[last_step_num][1] + difference[1] * (i / static_cast<float>(interpolate_num)),

                   initial_point[0][2] + moving_cycle[last_step_num][2] + difference[2] * (i / static_cast<float>(interpolate_num)));
      } else {
        coor2angle(leg_num,
                   initial_point[1][0] + moving_cycle[last_step_num][0] + difference[0] * (i / static_cast<float>(interpolate_num)),

                   initial_point[1][1] + moving_cycle[last_step_num][1] + difference[1] * (i / static_cast<float>(interpolate_num)),

                   initial_point[1][2] + moving_cycle[last_step_num][2] + difference[2] * (i / static_cast<float>(interpolate_num)));
      }
    }
    yield();  // 等所有servo都到定點再繼續下個動作
  }
}
//==============================================================================
//                                  END
//==============================================================================

//==============================================================================
//                                關節角度
//==============================================================================
void coor2angle(int leg_num, float X, float Y, float Z) {
  if (leg_num < 3) {  // right legs
    double j1 = atan(Y / X);
    double X_prime = (X / cos(j1)) - l1;
    double j3 = -((acos((pow(X_prime, 2) + pow(Z, 2) - pow(l2, 2) - pow(l3, 2)) / (2 * l2 * l2))));
    double j2 = (atan(Z / X_prime) - atan((l3 * sin(j3)) / (l2 + l3 * cos(j3))));
    angle2servo(leg_num, j1, j2, j3);
  } else {
    double j1 = atan(Y / X) + M_PI;
    double X_prime = -(X / cos(j1)) + l1;
    double j3 = -M_PI + ((acos((pow(X_prime, 2) + pow(Z, 2) - pow(l2, 2) - pow(l3, 2)) / (2 * l2 * l2))));
    double j2 = (atan(Z / X_prime) + M_PI - atan((l3 * sin(j3)) / (-l2 + l3 * cos(j3))));
    // Serial.printf("X:%f, Y:%f, Z:%f\n", X, Y, Z);
    // Serial.printf("X_prime:%f, j1:%f, j2:%f, j3:%f\n", X_prime, j1, j2, j3);
    angle2servo(leg_num, j1, j2, j3);
  }
}
//==============================================================================
//                                  END
//==============================================================================

//==============================================================================
//                                馬達角度
//==============================================================================

void angle2servo(int leg_num, float j1, float j2, float j3) {
  if (leg_num < 3) {  // right legs
    int pwm_j1 = (j1 + M_PI / 2) / (M_PI) * (SERVOMAX - SERVOMIN) + SERVOMIN;
    int pwm_j2 = (j2 + M_PI / 2) / (M_PI) * (SERVOMIN - SERVOMAX) + SERVOMAX;
    int pwm_j3 = (j3 + M_PI) / (M_PI) * (SERVOMAX - SERVOMIN) + SERVOMIN;
    board1.setPWM(0 + leg_num * 3, 0, pwm_j1);
    board1.setPWM(1 + leg_num * 3, 0, pwm_j2);
    board1.setPWM(2 + leg_num * 3, 0, pwm_j3);
    // Serial.printf("%d|| j1:%f, j2:%f, j3:%f\n", leg_num, j1, j2, j3);
  } else {  // left legs
    int pwm_j1 = (j1 - M_PI / 2) / (M_PI) * (SERVOMAX - SERVOMIN) + SERVOMIN;
    int pwm_j2 = (j2 - M_PI / 2) / (M_PI) * (SERVOMIN - SERVOMAX) + SERVOMAX;
    int pwm_j3 = (j3 + M_PI) / (M_PI) * (SERVOMAX - SERVOMIN) + SERVOMIN;
    board2.setPWM(0 + (leg_num - 3) * 3, 0, pwm_j1);
    board2.setPWM(1 + (leg_num - 3) * 3, 0, pwm_j2);
    board2.setPWM(2 + (leg_num - 3) * 3, 0, pwm_j3);
    // Serial.printf("%d|| j1:%f, j2:%f, j3:%f\n", leg_num, j1, j2, j3);
  }
}
//==============================================================================
//                                  END
//==============================================================================
