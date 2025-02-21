#include "motor_controller.h"
//#include "i2c_controller.h"
#include <Wire.h>

const int motor_1[] = {10, 9, 8};
const int i2cAddress = 0x01;
int motor_speed = 0;

void setup() {
  motor_setup(motor_1);
  i2c_setup(i2cAddress);

  Serial.begin(115200);
  Serial.println("Aduino As Motor Controller demonstration");
}

void loop() {
  move_motor(motor_1, motor_speed);
  delay(10);
}

void i2c_setup(int device_address){
  Wire.begin(i2cAddress);

  // Function to run when data received from master
  Wire.onReceive(receiveEvent);
}

void receiveEvent(int howMany) {
  int i = 0;
  int number = 0;
  int speeds_received[4];
  while (Wire.available()) { // loop through all but the last
    char c = Wire.read();    // receive byte as a character
    number = number << (8*i);
    number += c;  
  }

  if (number < 0) {
    motor_speed = -motor_speed;
  }
  motor_speed = number / 10;

  Serial.print("Motor commands received: ");
  Serial.print(number);
  Serial.println(); // to newline
}