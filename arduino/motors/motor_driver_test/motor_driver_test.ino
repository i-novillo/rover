/**
 * Motor Driver Speed Test Sketch (ZK-5AD / TA6586)
 * * This sketch cycles the motor in two directions:
 * 1. Spin in Direction 1 (Ramp Up, Hold, Ramp Down).
 * 2. Stop (Brief Wait).
 * 3. Spin in Direction 2 (Ramp Up, Hold, Ramp Down).
 * * It uses the two input pins (D0 and D1) to manage both speed (via PWM) and direction.
 */

// Define the Arduino pins connected to the ZK-5AD module inputs D0 and D1.
// We will use Digital Pin 9 (PWM) for IN1 (connected to D0 on the driver).
const int IN1 = 11;   
// We will use Digital Pin 8 (PWM capable) for IN2 (connected to D1 on the driver).
const int IN2 = 10; 

// Control variables
int currentSpeed = 0;   // Current PWM value (0 to 255)
int step = 5;           // How much to change the speed each time
int delayTime = 25;     // Delay in milliseconds to make the ramp visible
const int MAX_PWM = 255; // Max PWM limit at 50% (127/255)
const int MIN_START_PWM = 100; // New: Minimum PWM to overcome static friction (0 to 255)

void setup() {
  // 1. Set the pins as outputs
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  
  // 2. Initial state: Stop the motor
  analogWrite(IN1, 0); 
  analogWrite(IN2, 0); 
  
  // Initialize Serial Monitor for debugging
  Serial.begin(9600);
  Serial.println("ZK-5AD Motor Reversing Test Starting...");
  Serial.print("Maximum PWM speed limit: ");
  Serial.println(MAX_PWM);
  Serial.print("Minimum PWM start threshold: ");
  Serial.println(MIN_START_PWM);
}

/**
 * @brief Performs a complete ramp cycle (up, hold, down) for the motor.
 * * This function handles both acceleration and deceleration phases.
 * * @param pwmPin The pin that will receive the PWM signal (speed control).
 * @param lowPin The pin that will be held LOW (sets the direction).
 */
void rampCycle(int pwmPin, int lowPin) {
  // 1. Lock the direction by setting the non-PWM pin LOW
  digitalWrite(lowPin, LOW); 
  
  // --- Phase A: Increase Speed (Ramp Up) ---
  Serial.print("Ramping UP on PWM pin ");
  Serial.println(pwmPin);
  // Start the ramp from MIN_START_PWM to skip the noisy zone.
  for (currentSpeed = MIN_START_PWM; currentSpeed <= MAX_PWM; currentSpeed += step) {
    analogWrite(pwmPin, currentSpeed);
    Serial.print("Speed: ");
    Serial.println(currentSpeed);
    delay(delayTime);
  }
  
  // Wait at maximum speed for a moment
  Serial.println("Holding max speed.");
  delay(1000); 
  
  // --- Phase B: Decrease Speed (Ramp Down) ---
  Serial.print("Ramping DOWN on PWM pin ");
  Serial.println(pwmPin);
  // Ramp down to 0 for a complete stop.
  for (currentSpeed = MAX_PWM; currentSpeed >= 0; currentSpeed -= step) {
    // Ensure speed doesn't drop below 0 if step size is large
    int outputSpeed = max(0, currentSpeed); 
    analogWrite(pwmPin, outputSpeed);
    Serial.print("Speed: ");
    Serial.println(outputSpeed);
    delay(delayTime);
  }
  
  // Ensure the motor is fully stopped
  analogWrite(pwmPin, 0);
  digitalWrite(lowPin, LOW);
  Serial.println("Motor stopped.");
}


void loop() {
  // --- Cycle 1: Direction 1 ---
  Serial.println("\n--- STARTING DIRECTION 1 ---");
  // IN1 (D9) controls PWM speed, IN2 (D8) is held LOW.
  rampCycle(IN1, IN2);
  
  // Wait for a bit (2 seconds) before switching directions
  Serial.println("Waiting before direction change...");
  delay(2000); 
  
  // --- Cycle 2: Direction 2 ---
  Serial.println("\n--- STARTING DIRECTION 2 ---");
  // IN2 (D8) controls PWM speed, IN1 (D9) is held LOW.
  rampCycle(IN2, IN1);
  
  // Wait before restarting the loop
  Serial.println("Cycle complete. Waiting to restart...");
  delay(3000); 
}