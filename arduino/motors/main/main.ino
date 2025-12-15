#include "motor_controller.h"
#include "i2c_manager.h"

const Motor motor_1 = {11, 10};
const Motor motor_2 = {12, 13};
const Motor motor_3 = {7, 6};
const Motor motor_4 = {8, 9};

const Motor motors[] = {motor_1, motor_2, motor_3, motor_4};
const int i2c_slave_address = 0x01;

const Motor_Controller motor_controller = Motor_Controller(&motors[0], 4);
const I2C_Manager i2c_manager = I2C_Manager(i2c_slave_address, motor_controller);

void setup() {
  motor_controller.motor_setup();
  i2c_manager.i2c_setup();

  Serial.begin(115200);
  Serial.println("Aduino As Motor Controller demonstration");
}

void loop() {
  delay(10);
}
