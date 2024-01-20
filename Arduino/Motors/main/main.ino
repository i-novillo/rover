#include "motor_controller.h"

const int motor_1[] = {10, 9, 8};

void setup() {
  motor_setup(motor_1);
}

void loop() {
  for(int speed=0; speed<30; speed++) {
    move_motor(motor_1, speed);
    delay(100);
  }
}
