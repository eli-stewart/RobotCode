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
const int correctionMotorSpeed = 255;

// Switch states.
const int pressed = LOW;
const int unpressed = HIGH;

// Recent whisker.
int recentAction = -1;

// Turning.
bool turning = false;

// Timers.
unsigned long nextTimeLeft;
unsigned long nextTimeRight;
unsigned long nextTimeTurn;
bool timingRight = false;
bool timingLeft = false;
bool timingTurn = false;

void setup() {
  Serial.begin(9600); // Set up Serial library at 9600 bps.
  AFMS.begin(); // Create with the default frequency 1.6KHz

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

  if (!turning ) {
    if (leftSwitchPressed == pressed && rightSwitchPressed == pressed) {
      //turning = false;
      driveForward();
    }
    else if (leftSwitchPressed == unpressed && rightSwitchPressed == pressed) {
      // turning = false;
      correctLeft();
      timerLeft();
    }
    else if (leftSwitchPressed == pressed && rightSwitchPressed == unpressed) {
      //turning = false;
      correctRight();
      timerRight();
    }
    else if (leftSwitchPressed == unpressed && rightSwitchPressed == unpressed) {
      if (recentAction == 2) {
        driveForward();
      }
      else if (recentAction == 0) {
        // Most recent whisker is left. (Robot was correcting right.)
        pivotLeft();
      }
      else if (recentAction == 1) {
        // Most recent whisker is right. (Robot was correcting left.)
        pivotRight();
      }

    }
  }
  // Turning.
  if (frontSwitchPressed == HIGH) {

    turning = true;
  }
  if (turning) {
    turn(leftSwitchPressed, rightSwitchPressed, frontSwitchPressed);
  }
}

void turn(int leftSwitchPressed, int rightSwitchPressed, int frontSwitchPressed) {
  timerTurn();
  if (timingTurn) {
    if (leftSwitchPressed == pressed) {
      turnRight();
    }
    else if (rightSwitchPressed == pressed) {
      turnLeft();
    }
    else if (frontSwitchPressed == HIGH && rightSwitchPressed == unpressed && leftSwitchPressed == unpressed){
        if (recentAction == 0) {
        // Most recent whisker is left. (Robot was correcting right.)
        pivotLeft();
      }
      else if (recentAction == 1) {
        // Most recent whisker is right. (Robot was correcting left.)
        pivotRight();
      }
    }
 
    else {

      if (recentAction == 1) {
        // Most recent whisker is right. (Robot was correcting left.)
        pivotLeft();

      }
      else if (recentAction == 0) {
        // Most recent whisker is left. (Robot was correcting right.)
        pivotRight();
      }
    }
  }
}

void turnRight() {
  Serial.print("[TR]");
  releaseMotors();
  leftMotor->setSpeed(255);
  leftMotor->run(FORWARD);
  rightMotor->setSpeed(120);
  rightMotor->run(BACKWARD);
}

void turnLeft() {
  Serial.print("[TL]");
  releaseMotors();
  leftMotor->setSpeed(255);
  leftMotor->run(BACKWARD);
  rightMotor->setSpeed(255);
  rightMotor->run(FORWARD);
}

void pivotLeft() {
  Serial.print("[PL]");
  releaseMotors();
  leftMotor->setSpeed(255);
  leftMotor->run(BACKWARD);
  rightMotor->setSpeed(120);
  rightMotor->run(FORWARD);
}

void pivotRight() {
  Serial.print("[PR]");
  releaseMotors();
  leftMotor->setSpeed(255);
  leftMotor->run(FORWARD);
  rightMotor->setSpeed(120);
  rightMotor->run(BACKWARD);
}

void driveForward() {
  Serial.print("[F]");
  leftMotor->setSpeed(defaultMotorSpeed);
  rightMotor->setSpeed(defaultMotorSpeed);
  leftMotor->run(FORWARD);
  rightMotor->run(FORWARD);
}

void correctRight() {
  Serial.print("[CR]");
  releaseMotors();
  leftMotor->setSpeed(correctionMotorSpeed);
  rightMotor->setSpeed(defaultMotorSpeed);
  leftMotor->run(FORWARD);
  rightMotor->run(FORWARD);
}

void correctLeft() {
  releaseMotors();
  Serial.print("[CL]");
  leftMotor->setSpeed(defaultMotorSpeed);
  rightMotor->setSpeed(correctionMotorSpeed);
  leftMotor->run(FORWARD);
  rightMotor->run(FORWARD);
  //Serial.print (recentAction);
  // 1 means was correcting left
}

void releaseMotors() {
  leftMotor->run(RELEASE);
  rightMotor->run(RELEASE);
}

void timerRight() {
  unsigned long currentTime = millis();
  if (!timingRight) {
    nextTimeRight = currentTime + 1000;
    timingRight = true;
    //Serial.println("Started timer.");
  }
  else if (timingRight) {
    if (currentTime > nextTimeRight - 100 && currentTime < nextTimeRight + 100) {
      //Serial.println("Stopped timer.");
      // 0 means robot was correcting right.
      recentAction = 0;
      //Serial.print(recentAction);
      timingRight = false;
    }
    else if (currentTime > nextTimeRight + 100) {
      timingRight = false;
    }
  }
}

void timerLeft() {
  unsigned long currentTime = millis();
  if (!timingLeft) {
    nextTimeLeft = currentTime + 5000;
    timingLeft = true;
    //Serial.println("Started timer.");
  }
  else if (timingLeft) {
    if (currentTime > nextTimeLeft - 100 && currentTime < nextTimeLeft + 100) {
      //Serial.println("Stopped timer.");
      // 0 means robot was correcting right.
      recentAction = 1;
      //Serial.print(recentAction);
      timingLeft = false;
    }
    else if (currentTime > nextTimeLeft + 100) {
      timingLeft = false;
    }
  }
}

void timerTurn() {
  unsigned long currentTime = millis();
  if (!timingTurn) {
    nextTimeTurn = currentTime + 1000;
    timingTurn = true;
    //Serial.println("Started timer.");
  }
  else if (timingTurn) {
    if (currentTime > nextTimeTurn + 100) {
      timingTurn = false;
      turning = false;
          recentAction = 2;
    }
  }
}

