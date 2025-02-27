#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

struct Motor {
  int enable_pin;
  int motor_pins[2];
};

int rotation_direction(int speed){
  return speed > 0 ? HIGH : LOW;
}

class Motor_Controller {
  public:
    Motor_Controller(const Motor* motors, size_t motor_count) : motors(motors), motor_count(motor_count) {};

    void motor_setup() {
      for (int i = 0; i < this->motor_count; i++) {
        pinMode (motors[i].motor_pins[0], OUTPUT);
        pinMode (motors[i].motor_pins[1], OUTPUT);
        pinMode (motors[i].enable_pin, OUTPUT);
      }
    }

    void move_motor(int motor_number, int speed) {
      int direction = rotation_direction(speed);
      int normalized_speed = abs(map(speed, 0, 100, 0, 255));

      digitalWrite(motors[motor_number - 1].motor_pins[0], direction);
      digitalWrite(motors[motor_number - 1].motor_pins[1], !direction);
      
      analogWrite(motors[motor_number - 1].enable_pin, abs(speed));
    }

  private:
    const Motor* motors;
    size_t motor_count;

};

#endif