#include <Wire.h>
#include <AS5600.h>

AS5600 encoder;

// ===== CONFIG =====
#define GEAR_RATIO 270.0        // next higher ratio after 188
#define COUNTS_PER_REV 4096.0   // AS5600 resolution

// ===== STATE =====
long motor_revs = 0;
uint16_t last_raw = 0;
float output_zero_offset = 0.0;

void setup() {
  Serial.begin(115200);
  Wire.begin();

  delay(200);

  // Initialize encoder state
  last_raw = encoder.readAngle();

  // Establish zero reference at startup
  output_zero_offset =
    (last_raw * 360.0 / COUNTS_PER_REV) / GEAR_RATIO;
}

void loop() {
  uint16_t raw = encoder.readAngle();

  // ----- Detect wraparound on motor shaft -----
  int diff = raw - last_raw;

  if (diff > 2048) {
    motor_revs--;
  }
  else if (diff < -2048) {
    motor_revs++;
  }

  last_raw = raw;

  // ----- Continuous motor angle (degrees) -----
  float motor_angle_deg =
    (motor_revs * 360.0) +
    (raw * 360.0 / COUNTS_PER_REV);

  // ----- Output shaft angle (degrees) -----
  float output_angle_deg =
    (motor_angle_deg / GEAR_RATIO) - output_zero_offset;

  // ----- Wrap to 0–360 for plotting -----
  float plot_angle = fmod(output_angle_deg, 360.0);
  if (plot_angle < 0) plot_angle += 360.0;

  // ----- Force Serial Plotter Y-axis -----
  Serial.print(plot_angle);
  Serial.print(" ");
  Serial.print(0);
  Serial.print(" ");
  Serial.println(360);

  delay(10);
}
