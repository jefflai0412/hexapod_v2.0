#include "main.h"
#include <math.h>

//==============================================================================
//                               全域變數
//==============================================================================
// 待修改 (末端到j1的相對座標)
float initial_point[6][3] = {
  { 0, 0, 0 },
  { 0, 0, 0 },
  { 0, 0, 0 },
  { 0, 0, 0 },
  { 0, 0, 0 },
  { 0, 0, 0 }
};

float step_size = 0;    //代表每隻腳的圓的半徑
float step_height = 0;  //每次腳要抬高的高度

float l1 = 0;  // 第一個link的長度
float l2 = 0;  // 第二個link的長度
float l3 = 0;  // 第三個link的長度

float moving_cycle_lenth = 4;  // 整個循環的長度

int interpolate_num = 10;  // 將每步切成幾部分

float radian_dir = M_PI / 2;  // 行進方向
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
  // put your setup code here, to run once:
}

void loop() {
  // put your main code here, to run repeatedly:
}



//==============================================================================
//                                動作循環
//==============================================================================
void move(float dir) {
  for (int step = 0; step < moving_cycle_lenth; step++) {
    interpolate(step, radian_dir);
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
  float leg_cycle[4][3] = { { initial_point[0][0] + step_size * cos(radian_dir), initial_point[0][1] + step_size * sin(radian_dir), initial_point[0][2] },
                            { initial_point[0][0], initial_point[0][1], initial_point[0][2] },
                            { initial_point[0][0] - step_size * cos(radian_dir), initial_point[0][1] - step_size * sin(radian_dir), initial_point[0][2] },
                            { initial_point[0][0], initial_point[0][1], initial_point[0][2] + step_height } };

  // 判斷上一步是第幾步
  int last_step_num = step_num - 1;
  if (step_num == -1)  // 0的上一步是3
    last_step_num = 3;

  // 計算兩步之間的差值
  float difference[3] = { leg_cycle[step_num][0] - leg_cycle[last_step_num][0],
                          leg_cycle[step_num][1] - leg_cycle[last_step_num][1],
                          leg_cycle[step_num][2] - leg_cycle[last_step_num][2]};
  // interpolation
  for (int i = 0; i < interpolate_num; i++) {
    for (int leg_num = 0; leg_num < 6; leg_num++) {
      coor2angle(leg_num, difference[0] * (i / interpolate_num), difference[1] * (i / interpolate_num), difference[2] * (i / interpolate_num));
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
  float j1 = atan(Y / X);
  float X_prime = (X / cos(j1)) - l1;
  float j3 = acos((pow(X_prime, 2) + pow(Z, 2) - pow(l2, 2) - pow(l3, 2)) / (2 * l2 * l2));
  float j2 = atan(Z / X_prime) - atan((l3 * sin(j3)) / (l2 + l3 * cos(j3)));

  angle2servo(leg_num, j1, j2, j3);
}
//==============================================================================
//                                  END
//==============================================================================

//==============================================================================
//                                馬達角度
//==============================================================================

void angle2servo(int servo_num, float j1, float j2, float j3) {
}
//==============================================================================
//                                  END
//==============================================================================
