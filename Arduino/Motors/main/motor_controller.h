#ifndef MOTOR_CONTROLLER
#define MOTOR_CONTROLLER

void motor_setup(int motor_n[]){
  pinMode (motor_n[0], OUTPUT);
  pinMode (motor_n[1], OUTPUT);
  pinMode (motor_n[2], OUTPUT);
}

int rotation_direction(int speed){
  return speed > 0 ? HIGH : LOW;
}

void move_motor(int motor_n[], int speed){
  int motor_direction = rotation_direction(speed);
  digitalWrite (motor_n[1], motor_direction);
  digitalWrite (motor_n[2], !motor_direction);
  analogWrite (motor_n[0], map(abs(speed), 0, 30, 1, 255));
}

#endif