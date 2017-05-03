#include <Wire.h>
#include <Adafruit_MotorShield.h>

// Create the motor shield object with the default I2C address.
Adafruit_MotorShield AFMS = Adafruit_MotorShield();

// Initialize motors.
Adafruit_DCMotor *leftMotor = AFMS.getMotor(2);
Adafruit_DCMotor *rightMotor = AFMS.getMotor(1);

// Initialize pins.
const int leftSwitchInputPin = 2;
const int rightSwitchInputPin = 4;
const int frontSwitchInputPin = 6;
const int frontSwitchPowerPin = 8;

// Motor speeds.
const int defaultMotorSpeed = 100;
const int correctionMotorSpeed = 150;

// Switch states.
const int pressed = LOW;
const int unpressed = HIGH;

void setup() {
  Serial.begin(9600); // Set up Serial library at 9600 bps.
  AFMS.begin(); // Create with the default frequency 1.6KHz.

  // Setup pins.
  pinMode(leftSwitchInputPin, INPUT);
  pinMode(rightSwitchInputPin, INPUT);
  pinMode(frontSwitchInputPin, INPUT);
  pinMode(frontSwitchPowerPin, OUTPUT);

  // Write to output pins.
  digitalWrite(frontSwitchPowerPin, HIGH);

  // Initial motor speeds.
  leftMotor->setSpeed(defaultMotorSpeed);
  rightMotor->setSpeed(defaultMotorSpeed);
}

void loop() {
  int leftSwitchPressed = digitalRead(leftSwitchInputPin);
  int rightSwitchPressed = digitalRead(rightSwitchInputPin);
  int frontSwitchPressed = digitalRead(frontSwitchInputPin);

  if (frontSwitchPressed == LOW) {
    if (leftSwitchPressed == pressed && rightSwitchPressed == pressed) {
      Serial.print("F");
      leftMotor->setSpeed(defaultMotorSpeed);
      rightMotor->setSpeed(defaultMotorSpeed);
      leftMotor->run(FORWARD);
      rightMotor->run(FORWARD);
    }
    else if (leftSwitchPressed == unpressed && rightSwitchPressed == pressed) {
      Serial.print("L");
      leftMotor->setSpeed(correctionMotorSpeed);
      rightMotor->setSpeed(defaultMotorSpeed);
      leftMotor->run(FORWARD);
      rightMotor->run(FORWARD);
    }
    else if (leftSwitchPressed == pressed && rightSwitchPressed == unpressed) {
      Serial.print("R");
      leftMotor->setSpeed(defaultMotorSpeed);
      rightMotor->setSpeed(correctionMotorSpeed);
      leftMotor->run(FORWARD);
      rightMotor->run(FORWARD);
    }
    else if (leftSwitchPressed == unpressed && rightSwitchPressed == unpressed) {
      Serial.print("S");
      leftMotor->run(RELEASE);
      rightMotor->run(RELEASE);
    }
  }
  else if (frontSwitchPressed == HIGH) {
    // TODO.
  }
}

void turnLeft() {
  Serial.print("L");
}

void turnRight() {
  Serial.print("R");
}

