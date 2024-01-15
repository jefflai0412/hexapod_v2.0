#include "main.h"
#include <math.h>
#include <Adafruit_PWMServoDriver.h>
#include <Wire.h>

//==============================================================================
//                             servo defines
//==============================================================================
#define SERVO_FREQ 50  // servo frequency
#define SERVOMIN  105  // the minimun physical limit(after testing)
#define SERVOMAX  505

#define servo1_1 0
#define servo1_2 1
#define servo1_3 2
#define servo2_1 3
#define servo2_2 4
#define servo2_3 5
#define servo3_1 6
#define servo3_2 7
#define servo3_3 8
#define servo4_1 9
#define servo4_2 10
#define servo4_3 11
#define servo5_1 12
#define servo5_2 13
#define servo5_3 14
#define servo6_1 15
#define servo6_2 16
#define servo6_3 17
//==============================================================================
//                                  END
//==============================================================================
//==============================================================================
//                               全域變數
//==============================================================================
// 待修改 (末端到j1的相對座標)
float initial_point[6][3] = {
  { 52.141, 5.735, -41.747 },
  { 0, 0, 0 },
  { 0, 0, 0 },
  { 0, 0, 0 },
  { 0, 0, 0 },
  { 0, 0, 0 }
};

float step_size = 10;   //代表每隻腳的圓的半徑
float step_height = 5;  //每次腳要抬高的高度

float l1 = 0;       // 第一個link的長度
float l2 = 50;      // 第二個link的長度
float l3 = 60.612;  // 第三個link的長度

float moving_cycle_lenth = 4;  // 整個循環的長度

int interpolate_num = 10.00;  // 將每步切成幾部分

float radian_dir = M_PI / 2;  // 行進方向

int step_delay = 1000;  //每一步之間的間隔
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
}

void loop() {
  move(M_PI / 2);  // 直走
}



//==============================================================================
//                                動作循環
//==============================================================================
void move(float dir) {
  for (int step = 0; step < moving_cycle_lenth; step++) {
    interpolate(step, radian_dir);
    Serial.println(step);
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
    { initial_point[0][0] + step_size * cos(radian_dir), initial_point[0][1] + step_size * sin(radian_dir), initial_point[0][2] },
    { initial_point[0][0], initial_point[0][1], initial_point[0][2] },
    { initial_point[0][0] - step_size * cos(radian_dir), initial_point[0][1] - step_size * sin(radian_dir), initial_point[0][2] },
    { initial_point[0][0], initial_point[0][1], initial_point[0][2] + step_height }
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

      coor2angle(leg_num,
                 moving_cycle[last_step_num][0] + difference[0] * (i / static_cast<float>(interpolate_num)),
                 moving_cycle[last_step_num][1] + difference[1] * (i / static_cast<float>(interpolate_num)),
                 moving_cycle[last_step_num][2] + difference[2] * (i / static_cast<float>(interpolate_num)));
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
  // Serial.printf("leg:%d, X:%f, Y:%f, Z:%f  ||  \n",leg_num, X, Y, Z);
  float j1 = atan(Y / X);
  float X_prime = (X / cos(j1)) - l1;
  float j3 = acos((pow(X_prime, 2) + pow(Z, 2) - pow(l2, 2) - pow(l3, 2)) / (2 * l2 * l2));
  float j2 = atan(Z / X_prime) - atan((l3 * sin(j3)) / (l2 + l3 * cos(j3)));
  Serial.printf("j1:%f, j2:%f, j3:%f\n", j1, j2, j3);
  angle2servo(leg_num, j1, j2, j3);
}
//==============================================================================
//                                  END
//==============================================================================

//==============================================================================
//                                馬達角度
//==============================================================================

void angle2servo(int leg_num, float j1, float j2, float j3) {

  // Serial.printf("%d|| j1:%f, j2:%f, j3:%f\n", leg_num, j1, j2, j3);
}
//==============================================================================
//                                  END
//==============================================================================
