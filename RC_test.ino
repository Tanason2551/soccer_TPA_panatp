#include "Config.h"
#include "IMU_Logic.h"
#include "Move_Logic.h"
#include "Shoot_Logic.h"

// Define Global Variables
POP32_Huskylens huskylens;
float rot_error, rot_pError, rot_i, rot_d, rot_w;
int ballPosX;
float spFli = spFli_val;
float fli_error, fli_pError, fli_i, fli_d, fli_spd;
int ballPosY;
float vecCurve, radCurve;
int discoveState = 1;
float head_error, head_pError, head_w, head_d, head_i;
float thetaRad, vx, vy, spd1, spd2, spd3;
float pvYaw, lastYaw;
uint8_t rxCnt = 0, rxBuf[8];

void setup() {
  reload();
  while (!huskylens.begin(Wire)) {
    oled.text(1, 0, "Begin failed!");
    oled.show();
    delay(100);
  }
  delay(3000);
  pvYaw = 90.0f;
  Auto_zero();
  oled.text(2, 0, "SW_A => Test_Shoot");
  oled.text(3, 0, "SW_B => RUN");
  oled.show();
  while (!SW_B()) {
    if (SW_A()) {
      shoot();
      reload();
    }
    getIMU();
    heading(0, 0, 0);
  }
  oled.clearDisplay();
  oled.text(0, 0, "RUN");
  oled.show();
}

void loop() {
  if (SW_OK()) {
    beep();
    oled.clearDisplay();
    oled.text(2, 0, "SW_A => Test_Shoot");
    oled.text(3, 0, "SW_B => RUN");
    oled.show();
    while (!SW_B()) {
      if (SW_A()) { shoot(); reload(); }
      getIMU();
      heading(0, 0, 0);
    }
    oled.clearDisplay();
    oled.text(0, 0, "MA DI WA I LOVE YOU");
    oled.show();
  }

  if (huskylens.updateBlocks()) {
    int ballX = -1, goalX = -1;
    
    if (huskylens.blockSize[1] > 0) {
      ballX = huskylens.blockInfo[1][0].x;
      ballPosY = huskylens.blockInfo[1][0].y;
    }
    if (huskylens.blockSize[2] > 0) {
      goalX = huskylens.blockInfo[2][0].x;
    }

    if (ballX != -1) {
      ballPosX = ballX;
      
      if (analog(A1) > 3040) { wheel(0, 0, 0); delay(200); holonomic(40, 270, 0); delay(500); }
      if (analog(A2) > 3230) { wheel(0, 0, 0); delay(200); holonomic(40, 270, 0); delay(500); }
      if (analog(A3) > 3420) { wheel(0, 0, 0); delay(200); holonomic(40, 90, 0); delay(500); }

      for (int i = 0; i < 8; i++) getIMU();

      if (discoveState) {
        rot_error = sp_rot - ballPosX;
        rot_i = constrain(rot_i + rot_error, -100, 100);
        rot_d = rot_error - rot_pError;
        rot_pError = rot_error;
        rot_w = constrain((rot_error * rot_Kp) + (rot_i * rot_Ki) + (rot_d * rot_Kd), -100, 100);

        fli_error = spFli - ballPosY;
        fli_i = constrain(fli_i + fli_error, -100, 100);
        fli_d = fli_error - fli_pError;
        fli_pError = fli_error;
        fli_spd = constrain(fli_error * fli_Kp + fli_i * fli_Ki + fli_d * fli_Kd, -100, 100);

        holonomic(fli_spd, 90, rot_w);
        if ((abs(rot_error) < rotErrorGap) && (abs(fli_error) < flingErrorGap)) {
          wheel(0, 0, 0);
          lastYaw = pvYaw;
          discoveState = 0;
        }
      } else {
        if (goalX != -1) {
          visualAlign(ballX, goalX, 30);
          
          if (abs(160 - goalX) < visualErrorGap && abs(160 - ballX) < rotErrorGap) {
            axisLockPush(100, 200); 
            shoot();
            delay(300); reload();
            delay(300); reload();
            axisLockPush(40, 100);
            discoveState = 1;
          }
        } else {
          if (lastYaw < 0) { vecCurve = 0; radCurve = 15; } 
          else { vecCurve = 180; radCurve = -15; }
          
          holonomic(60, vecCurve, radCurve);
          if (abs(pvYaw) < alignErrorGap) {
            axisLockPush(100, 200); 
            shoot();
            delay(300); reload();
            delay(300); reload();
            axisLockPush(40, 100);
            discoveState = 1;
          }
        }
      }
    } else {
      int sideRot = (sp_rot - ballPosX);
      holonomic(0, 0, (sideRot == 0 ? 1 : sideRot / abs(sideRot)) * idleSpd);
      discoveState = 1;
    }
  } else {
    int sideRot = (sp_rot - ballPosX);
    holonomic(0, 0, (sideRot == 0 ? 1 : sideRot / abs(sideRot)) * idleSpd);
    discoveState = 1;
  }
}
