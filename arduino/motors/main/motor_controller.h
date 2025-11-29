#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

struct Motor {
  int motor_pins[2]; // Pin 0 Forward Pin 1 Backward
};

int time_to_target = 50; // milliseconds
int dt = 5; // milliseconds

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
        digitalWrite(motors[i].motor_pins[0], LOW);
        digitalWrite(motors[i].motor_pins[1], LOW);
      }
    }

    void set_motor_speed(int motor_number, int speed) {
      if (motor_number > motor_count) {
        Serial.print("Invalid motor number: ");
        Serial.println(motor_number);
        return;
      }

      int pin_forward = motors[motor_number - 1].motor_pins[0];
      int pin_backward = motors[motor_number - 1].motor_pins[1];

      if (speed > 0) {
        analogWrite(pin_forward, map(abs(speed), 0, 100, 0, 255));
        digitalWrite(pin_backward, LOW);
      } else if (speed < 0) {
        analogWrite(pin_backward, map(abs(speed), 0, 100, 0, 255));
        digitalWrite(pin_forward, LOW);
      } else {
        digitalWrite(pin_forward, LOW);
        digitalWrite(pin_backward, LOW);
      }
    }

    void move_motors(int speeds[4]) {
      double accelerations[4] = {0, 0, 0, 0};
      int steps = time_to_target / dt;

      for (int i = 0; i < 4; i++) {
        accelerations[i] = (speeds[i] - latest_speeds[i]) / time_to_target;
      }
      
      for (int step = 0; step < steps; step++) {
        for (int i = 0; i < 4; i++) {
          latest_speeds[i] += (int)(accelerations[i] * dt);
          set_motor_speed(i + 1, latest_speeds[i]);
        }
        delay(dt);
      }
    }

  private:
    const Motor* motors;
    size_t motor_count;
    int latest_speeds[4] = {0, 0, 0, 0};

};

#endif