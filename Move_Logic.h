#ifndef MOVE_LOGIC_H
#define MOVE_LOGIC_H

#include "Config.h"
#include "IMU_Logic.h"

void wheel(int s1, int s2, int s3) {
  motor(1, s1);
  motor(2, s2);
  motor(3, s3);
}

void holonomic(float spd, float theta, float omega) {
  thetaRad = theta * degToRad;
  vx = spd * cos(thetaRad);
  vy = spd * sin(thetaRad);
  spd1 = vy * cos30 - vx * sin30 + omega;
  spd2 = -vy * cos30 - vx * sin30 + omega;
  spd3 = vx + omega;
  wheel(spd1, spd2, spd3);
}

void heading(float spd, float theta, float spYaw) {
  head_error = spYaw - pvYaw;
  head_i = constrain(head_i + head_error, -180, 180);
  head_d = head_error - head_pError;
  head_w = (head_error * head_Kp) + (head_i * head_Ki) + (head_d * head_Kd);
  head_w = constrain(head_w, -100, 100);
  holonomic(spd, theta, head_w);
  head_pError = head_error;
}

void resetHeadingPID() {
  head_i = 0;
  head_pError = 0;
}

// ฟังก์ชันตั้งลำด้วยสายตา (Robot-Ball-Goal Alignment)
void visualAlign(int ballX, int goalX, int targetPower) {
  // 1. คุมการหมุน (Omega) ด้วยตำแหน่งประตู เพื่อให้หน้าหุ่นจ้องประตู
  float vis_rot_error = 160 - goalX;
  float vis_rot_w = vis_rot_error * visRot_Kp; 
  vis_rot_w = constrain(vis_rot_w, -60, 60);

  // 2. คุมการสไลด์ข้าง (Vx) ด้วยตำแหน่งบอล เพื่อให้บอลอยู่ตรงกลางง่ามยิง
  float vis_strafe_error = ballX - 160; 
  float vis_vx = vis_strafe_error * visStrafe_Kp;
  vis_vx = constrain(vis_vx, -60, 60);

  // 3. เดินหน้าประคอง (Vy)
  float vis_vy = targetPower;

  // แปลงค่ากลับไปเป็นระบบ Holonomic
  vx = vis_vx;
  vy = vis_vy;
  float omega = vis_rot_w;

  spd1 = vy * cos30 - vx * sin30 + omega;
  spd2 = -vy * cos30 - vx * sin30 + omega;
  spd3 = vx + omega;
  wheel(spd1, spd2, spd3);
}

// ฟังก์ชันล็อคตำแหน่งประตู (Axis Lock)
void axisLockPush(int power, int duration) {
  resetHeadingPID();
  unsigned long start_time = millis();
  while (millis() - start_time < duration) {
    getIMU();
    heading(power, 90, 0); 
  }
  wheel(0, 0, 0);
}

#endif
