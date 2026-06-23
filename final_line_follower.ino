#include <AFMotor.h>
#include <Wire.h>
#include <Adafruit_TCS34725.h>

// ── Motors ───────────────────────────────────────────────────
AF_DCMotor motorLeft (1);
AF_DCMotor motorRight(3);

// ── IR Sensors ───────────────────────────────────────────────
#define IR_1   9
#define IR_2   A0
#define IR_3   A1
#define IR_4   A2
#define IR_5   A3
#define IR_6   10

// ── Ultrasonic ───────────────────────────────────────────────
#define TRIG_PIN            2
#define ECHO_PIN            13
#define OBSTACLE_DIST_CM    15
// ── PD Tuning ────────────────────────────────────────────────
#define BASE_SPEED        150
#define MIN_SPEED         40
#define SPEED_REDUCTION   20
#define KP                25
#define KD                20

// ── Obstacle scan parameters ─────────────────────────────────
#define SCAN_SPEED        150
#define TURN_90_MS        500
#define SPIN_180_MS       800
#define MOTOR_SETTLE_MS   150

// ── Color Sensor ─────────────────────────────────────────────
#define STRAIGHT_SPEED    70
#define SNAKE_BASE_SPEED  70
#define SNAKE_CORRECTION  15

// ── State ─────────────────────────────────────────────────────
int lastError        = 0;
int lastCorrection   = 0;
int preferredTurnDir = 1;

// ── TCS34725 ─────────────────────────────────────────────────
Adafruit_TCS34725 tcs = Adafruit_TCS34725(
  TCS34725_INTEGRATIONTIME_50MS,
  TCS34725_GAIN_4X
);

enum Color { C_BLACK, C_GREEN, C_RED, C_WHITE };
enum SnakeState { GOING_RIGHT, GOING_LEFT };
SnakeState snakeDir = GOING_RIGHT;

// ─────────────────────────────────────────────────────────────

void setMotor(AF_DCMotor &motor, int speed) {
  if (speed < 0) {
    motor.setSpeed(constrain(-speed, 0, 255));
    motor.run(BACKWARD);
  } else if (speed > 0) {
    motor.setSpeed(constrain(speed, 0, 255));
    motor.run(FORWARD);
  } else {
    motor.run(RELEASE);
  }
}

void motorSet(AF_DCMotor &m, int spd) {
  if (spd > 0) { m.setSpeed(constrain(spd,0,255)); m.run(FORWARD);  }
  else if (spd < 0) { m.setSpeed(constrain(-spd,0,255)); m.run(BACKWARD); }
  else { m.run(RELEASE); }
}

void stopMotors() {
  motorLeft.run(RELEASE);
  motorRight.run(RELEASE);
  delay(MOTOR_SETTLE_MS);
}

long readDistanceCM() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  long duration = pulseIn(ECHO_PIN, HIGH, 30000);
  if (duration == 0) return 999;
  return duration / 58;
}

int readError() {
  int s1 = (digitalRead(IR_1) == LOW);
  int s2 = (digitalRead(IR_2) == LOW);
  int s3 = (digitalRead(IR_3) == LOW);
  int s4 = (digitalRead(IR_4) == LOW);
  int s5 = (digitalRead(IR_5) == LOW);
  int s6 = (digitalRead(IR_6) == LOW);
  return (-7*s1) + (-3*s2) + (-1*s3) + (1*s4) + (3*s5) + (7*s6);
}

void pivot(int direction, unsigned long ms) {
  if (direction > 0) {
    setMotor(motorLeft,   SCAN_SPEED);
    setMotor(motorRight, -SCAN_SPEED);
  } else {
    setMotor(motorLeft,  -SCAN_SPEED);
    setMotor(motorRight,  SCAN_SPEED);
  }
  delay(ms);
  stopMotors();
}

void spin180() {
  setMotor(motorLeft,   SCAN_SPEED);
  setMotor(motorRight, -SCAN_SPEED);
  delay(SPIN_180_MS);
  stopMotors();
}

bool sweepScan(int direction, int startError) {
  stopMotors();
  unsigned long start = millis();
  if (direction > 0) {
    setMotor(motorLeft,   SCAN_SPEED);
    setMotor(motorRight, -SCAN_SPEED);
  } else {
    setMotor(motorLeft,  -SCAN_SPEED);
    setMotor(motorRight,  SCAN_SPEED);
  }
  while (millis() - start < TURN_90_MS) {
    if (readError() != startError) {
      stopMotors();
      return true;
    }
  }
  stopMotors();
  return false;
}

void handleObstacle() {
  stopMotors();
  int dir        = preferredTurnDir;
  int attempts   = 0;
  int startError = readError();

  while (attempts < 6) {
    if (sweepScan(dir, startError)) {
      preferredTurnDir = dir;
      lastError        = 0;
      lastCorrection   = 0;
      return;
    }
    pivot(-dir, TURN_90_MS);
    dir = -dir;
    attempts++;
  }
  spin180();
  lastError      = 0;
  lastCorrection = 0;
}

// ── Color sensor functions ────────────────────────────────────

Color readColor() {
  uint16_t r, g, b, c;
  tcs.getRawData(&r, &g, &b, &c);

  if (c < 400)  return C_BLACK;
  if (c > 1200) return C_WHITE;

  float rg = (float)r / g;
  if (rg < 1.1) return C_GREEN;
  if (rg > 1.4) return C_RED;

  return C_WHITE;
}

void followGreenSnake() {
  Color lastColor = C_GREEN;
  snakeDir = GOING_RIGHT;

  unsigned long whiteStart  = 0;
  bool          turningOnWhite = false;
 
  while (true) {
    Color c = readColor();
 
    if (c == C_BLACK) return;
    if (c == C_RED)   return;
  
    // ── Green→White transition: flip direction, start timer ──
    if (c == C_WHITE && lastColor == C_GREEN) {
      snakeDir = (snakeDir == GOING_RIGHT) ? GOING_LEFT : GOING_RIGHT;
      whiteStart    = millis();
      turningOnWhite = true;
    }
 
    // ── Back on green: cancel the timer ──────────────────────
    if (c == C_GREEN) {
      turningOnWhite = false;
    }
 
    // ── Timeout: green not found after 500ms → flip and search indefinitely ──
    if (turningOnWhite && c == C_WHITE &&
        millis() - whiteStart > 800) {
      snakeDir      = (snakeDir == GOING_RIGHT) ? GOING_LEFT : GOING_RIGHT;
      turningOnWhite = false;           // disables timeout — won't flip again until next green→white
    }
 
    lastColor = c;
 
    if (c == C_WHITE) {
      if (snakeDir == GOING_RIGHT) {
        motorSet(motorLeft,  SNAKE_BASE_SPEED + 70);
        motorSet(motorRight, SNAKE_BASE_SPEED - 50);
      } else {
        motorSet(motorLeft,  SNAKE_BASE_SPEED - 70);
        motorSet(motorRight, SNAKE_BASE_SPEED + 50);
      }
    } else {
      if (snakeDir == GOING_RIGHT) {
        motorSet(motorLeft,  SNAKE_BASE_SPEED + SNAKE_CORRECTION);
        motorSet(motorRight, SNAKE_BASE_SPEED - SNAKE_CORRECTION);
      } else {
        motorSet(motorLeft,  SNAKE_BASE_SPEED - SNAKE_CORRECTION);
        motorSet(motorRight, SNAKE_BASE_SPEED + SNAKE_CORRECTION);
      }
    }
  }
}


void handleRed() {
  while (true) {
    motorSet(motorLeft,  STRAIGHT_SPEED);
    motorSet(motorRight, STRAIGHT_SPEED);
 
    Color c = readColor();
    if (c == C_GREEN) { followGreenSnake(); return; }
    if (c == C_BLACK) return;
  }
}
 
bool handleColorSensor() {
  Color c = readColor();
  if (c == C_RED)   { handleRed();        return true; }
  if (c == C_GREEN) { followGreenSnake(); return true; }
  return false;
}
// ─────────────────────────────────────────────────────────────

void setup() {
  Serial.begin(9600);
  Wire.begin();

  pinMode(IR_1, INPUT); pinMode(IR_2, INPUT);
  pinMode(IR_3, INPUT); pinMode(IR_4, INPUT);
  pinMode(IR_5, INPUT); pinMode(IR_6, INPUT);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  motorLeft.setSpeed(0);  motorRight.setSpeed(0);
  motorLeft.run(RELEASE); motorRight.run(RELEASE);

  if (!tcs.begin()) {
    Serial.println("COLOR SENSOR NOT FOUND");
    while (1);
  }

  Serial.println("Ready.");
  delay(2000);
}

void loop() {
  // ── Color sensor first ───────────────────────────────────
  if (handleColorSensor()) return;

  // ── Obstacle check ───────────────────────────────────────
  if (readDistanceCM() < OBSTACLE_DIST_CM) {
    handleObstacle();
    return;
  }

  // ── Sensor read ──────────────────────────────────────────
  int s1 = (digitalRead(IR_1) == LOW);
  int s2 = (digitalRead(IR_2) == LOW);
  int s3 = (digitalRead(IR_3) == LOW);
  int s4 = (digitalRead(IR_4) == LOW);
  int s5 = (digitalRead(IR_5) == LOW);
  int s6 = (digitalRead(IR_6) == LOW);

  bool allWhite = (s1 && s2 && s3 && s4 && s5 && s6);

  // ── Lost line recovery ───────────────────────────────────
  if (allWhite) {
    if (abs(lastError) <7) {
      setMotor(motorLeft,  100);
      setMotor(motorRight, 100);
      return;
    }
    if (abs(lastCorrection) < 100)
      lastCorrection = 100 * (lastCorrection / abs(lastCorrection));
    if (lastCorrection > 0) {
      setMotor(motorLeft, lastCorrection);
      motorRight.run(RELEASE);
    } else {
      setMotor(motorRight, -lastCorrection);
      motorLeft.run(RELEASE);
    }
    return;
  }

  // ── PD follow ────────────────────────────────────────────
  int error = (-7*s1) + (-3*s2) + (-1*s3) + (1*s4) + (3*s5) + (7*s6);

  int correction = (KP * error) + (KD * (error - lastError));
  lastError      = error;
  lastCorrection = (KP + 10) * error;

  int dynamicSpeed = constrain(BASE_SPEED - (abs(error) * SPEED_REDUCTION),
                               MIN_SPEED, BASE_SPEED);

  setMotor(motorLeft,  dynamicSpeed + correction);
  setMotor(motorRight, dynamicSpeed - correction);
}
