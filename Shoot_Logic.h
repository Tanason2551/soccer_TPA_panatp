#ifndef SHOOT_LOGIC_H
#define SHOOT_LOGIC_H

#include "Config.h"

void shoot() {
  motor(4, reloadSpd);
  delay(150);
  motor(4, 0);
  delay(50);
}

void reload() {
  motor(4, reloadSpd);
  int timer_count = 0;
  for (int i = 0; i < 2000; i++) {
    timer_count++;
    if (analogRead(limPin) > 700) break;
    delay(1);
  }
  if (timer_count == 2000) {
    motor(4, -reloadSpd);
    delay(500);
    motor(4, reloadSpd);
    for (int i = 0; i < 2000; i++) {
      if ((analogRead(limPin) > 700)) break;
      delay(1);
    }
  }
  motor(4, 0);
}

#endif
