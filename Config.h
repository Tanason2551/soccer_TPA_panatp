#ifndef CONFIG_H
#define CONFIG_H

#include <POP32.h>
#include <POP32_Huskylens.h>

extern POP32_Huskylens huskylens;

#define degToRad 0.0174f
#define sin30 sin(30.f * degToRad)
#define cos30 cos(30.f * degToRad)

// PID Rotation (Tuned)
#define rot_Kp 0.25
#define rot_Ki 0.0
#define rot_Kd 0.1
#define sp_rot 160
#define rotErrorGap 15
#define idleSpd 25

// PID Following (Tuned)
#define fli_Kp 1.2
#define fli_Ki 0.0
#define fli_Kd 0.0
#define flingErrorGap 20
#define spFli_val 180

// Alignment
#define alignErrorGap 4
#define visualErrorGap 15

// PID Visual Alignment (Goal-Tracking Rotation)
#define visRot_Kp 0.4
#define visRot_Kd 0.1

// PID Visual Alignment (Ball-Tracking Strafe)
#define visStrafe_Kp 0.6

// PID Heading
#define head_Kp 2.3f
#define head_Ki 0.0f
#define head_Kd 0.5f

// Mechanism
#define limPin A0
#define reloadSpd 80

// Global Variables
extern float rot_error, rot_pError, rot_i, rot_d, rot_w;
extern int ballPosX;
extern float spFli;
extern float fli_error, fli_pError, fli_i, fli_d, fli_spd;
extern int ballPosY;
extern float vecCurve, radCurve;
extern int discoveState;
extern float head_error, head_pError, head_w, head_d, head_i;
extern float thetaRad, vx, vy, spd1, spd2, spd3;
extern float pvYaw, lastYaw;
extern uint8_t rxCnt, rxBuf[8];

#endif
