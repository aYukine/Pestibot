#include <Servo.h>

// Servo objects
Servo servo1;  // Vertical rotation (up/down)
Servo servo2;  // Keep camera perpendicular

// ==========================================================
// Pin map per board
// Recommended board: ESP32 (38-pin DevKit) for better PWM and expansion.
// ==========================================================
#if defined(ARDUINO_ARCH_ESP32)
const int SERVO1_PIN = 18;
const int SERVO2_PIN = 19;
const int PUMP_PIN = 23;

const int LEFT_MOTOR_PWM_PIN = 25;
const int LEFT_MOTOR_IN1_PIN = 26;
const int LEFT_MOTOR_IN2_PIN = 27;

const int RIGHT_MOTOR_PWM_PIN = 33;
const int RIGHT_MOTOR_IN1_PIN = 32;
const int RIGHT_MOTOR_IN2_PIN = 14;

const int LEFT_PWM_CHANNEL = 0;
const int RIGHT_PWM_CHANNEL = 1;
const int PWM_FREQ = 20000;
const int PWM_RESOLUTION = 8;
#else
// Arduino UNO / NANO
const int SERVO1_PIN = 9;
const int SERVO2_PIN = 10;
const int PUMP_PIN = 8;

const int LEFT_MOTOR_PWM_PIN = 5;
const int LEFT_MOTOR_IN1_PIN = 4;
const int LEFT_MOTOR_IN2_PIN = 7;

const int RIGHT_MOTOR_PWM_PIN = 6;
const int RIGHT_MOTOR_IN1_PIN = 12;
const int RIGHT_MOTOR_IN2_PIN = 13;
#endif

// Timing
const unsigned long SPRAY_TIMEOUT = 1000; // 1 second in milliseconds
unsigned long last_message_time = 0;

// Motor control state
uint8_t left_wheel_cmd = 127;
uint8_t right_wheel_cmd = 127;
uint8_t servo1_pos = 90;
uint8_t servo2_pos = 90;
bool pump_state = false;

void setWheelOutput(
  uint8_t cmd,
  int pwmPin,
  int in1Pin,
  int in2Pin
#if defined(ARDUINO_ARCH_ESP32)
  , int pwmChannel
#endif
) {
  int speed = (int)cmd - 127;
  int pwm = abs(speed) * 2;
  if (pwm > 255) {
    pwm = 255;
  }

  if (speed > 0) {
    digitalWrite(in1Pin, HIGH);
    digitalWrite(in2Pin, LOW);
  } else if (speed < 0) {
    digitalWrite(in1Pin, LOW);
    digitalWrite(in2Pin, HIGH);
  } else {
    digitalWrite(in1Pin, LOW);
    digitalWrite(in2Pin, LOW);
  }

#if defined(ARDUINO_ARCH_ESP32)
  ledcWrite(pwmChannel, pwm);
#else
  analogWrite(pwmPin, pwm);
#endif
}

void setup() {
  // Initialize serial communication (115200 baud to match hardware bridge)
  Serial.begin(115200);
  
  // Attach servos
  servo1.attach(SERVO1_PIN);
  servo2.attach(SERVO2_PIN);
  
  // Initialize pump pin
  pinMode(PUMP_PIN, OUTPUT);
  digitalWrite(PUMP_PIN, LOW);

  // Initialize motor driver pins
  pinMode(LEFT_MOTOR_IN1_PIN, OUTPUT);
  pinMode(LEFT_MOTOR_IN2_PIN, OUTPUT);
  pinMode(RIGHT_MOTOR_IN1_PIN, OUTPUT);
  pinMode(RIGHT_MOTOR_IN2_PIN, OUTPUT);

#if defined(ARDUINO_ARCH_ESP32)
  ledcSetup(LEFT_PWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(RIGHT_PWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(LEFT_MOTOR_PWM_PIN, LEFT_PWM_CHANNEL);
  ledcAttachPin(RIGHT_MOTOR_PWM_PIN, RIGHT_PWM_CHANNEL);
#else
  pinMode(LEFT_MOTOR_PWM_PIN, OUTPUT);
  pinMode(RIGHT_MOTOR_PWM_PIN, OUTPUT);
#endif
  
  // Set initial servo positions
  servo1.write(servo1_pos);
  servo2.write(servo2_pos);

  setWheelOutput(
    left_wheel_cmd,
    LEFT_MOTOR_PWM_PIN,
    LEFT_MOTOR_IN1_PIN,
    LEFT_MOTOR_IN2_PIN
#if defined(ARDUINO_ARCH_ESP32)
    , LEFT_PWM_CHANNEL
#endif
  );
  setWheelOutput(
    right_wheel_cmd,
    RIGHT_MOTOR_PWM_PIN,
    RIGHT_MOTOR_IN1_PIN,
    RIGHT_MOTOR_IN2_PIN
#if defined(ARDUINO_ARCH_ESP32)
    , RIGHT_PWM_CHANNEL
#endif
  );
  
  last_message_time = millis();
}

void loop() {
  unsigned long current_time = millis();
  
  // Check for serial data (MotorControl bridge packet)
  if (Serial.available() >= 7) {
    // Expected format:
    // [left_wheel(1), right_wheel(1), servo1(1), servo2(1), pump(1), checksum_low(1), checksum_high(1)]
    uint8_t lw = Serial.read();
    uint8_t rw = Serial.read();
    uint8_t s1 = Serial.read();
    uint8_t s2 = Serial.read();
    uint8_t pump = Serial.read();
    uint16_t checksum = (uint16_t) Serial.read() | ((uint16_t) Serial.read() << 8);
    
    // Simple checksum validation
    uint16_t calculated_checksum = (uint16_t)lw + (uint16_t)rw + (uint16_t)s1 + (uint16_t)s2 + (uint16_t)pump;
    if (checksum == calculated_checksum) {
      left_wheel_cmd = lw;
      right_wheel_cmd = rw;
      servo1_pos = s1;
      servo2_pos = s2;
      pump_state = (pump != 0);
      last_message_time = current_time;
    }
  }
  
  // Safety timeout: disable spray if no message received for 1 second
  if (current_time - last_message_time > SPRAY_TIMEOUT) {
    left_wheel_cmd = 127;
    right_wheel_cmd = 127;
    pump_state = false;
  }

  // Update wheel outputs
  setWheelOutput(
    left_wheel_cmd,
    LEFT_MOTOR_PWM_PIN,
    LEFT_MOTOR_IN1_PIN,
    LEFT_MOTOR_IN2_PIN
#if defined(ARDUINO_ARCH_ESP32)
    , LEFT_PWM_CHANNEL
#endif
  );
  setWheelOutput(
    right_wheel_cmd,
    RIGHT_MOTOR_PWM_PIN,
    RIGHT_MOTOR_IN1_PIN,
    RIGHT_MOTOR_IN2_PIN
#if defined(ARDUINO_ARCH_ESP32)
    , RIGHT_PWM_CHANNEL
#endif
  );
  
  // Update servo positions
  servo1.write(servo1_pos);
  servo2.write(servo2_pos);
  
  // Update pump state
  digitalWrite(PUMP_PIN, pump_state ? HIGH : LOW);
  
  // Small delay to prevent overwhelming the loop
  delay(20);
}
