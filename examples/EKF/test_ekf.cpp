#include <Arduino.h>
#include <RobotLocalization/EKF.h>

// x, y, th, x', y', th'
float x[6] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
// v, arx, ary, w
float y[4] = {0.0f, 0.0f, 0.0f, 0.0f};
float P[6] = {0.01f, 0.01f, 0.01f, 0.01f, 0.01f, 0.01f};
float Q[6] = {0.01f, 0.01f, 0.01f, 0.01f, 0.01f, 0.01f};
float R[3] = {0.01f, 0.01f, 0.01f};
float x_new[6] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

unsigned long last_time = 0;

RobotLocalization::EKF ekf(
  x, // [x, y, th, x', y', th']
  P, // P
  Q, // Q
  R // R
);


void setup() {
  Serial.begin(115200);

  last_time = millis();
}

void loop() {

  unsigned long now_time = millis();
  float dt = (now_time - last_time) / 1000.0f;

  y[0] = 0.25f;
  y[1] = 0.0f;
  y[2] = 0.0f;
  y[3] = 1.0f;

  memcpy(x, x_new, sizeof(float)*6);

  ekf.update(x, y, dt, x_new);

  Serial.print(dt); Serial.print(": ");
  Serial.print(x_new[0]); Serial.print(" ");
  Serial.print(x_new[1]); Serial.print(" ");
  Serial.print(x_new[2]); Serial.print(" ");
  Serial.println();

  last_time = now_time;

  delay(30);
}

