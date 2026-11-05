#include <PeanutKingSoccerV4.h>
#include <math.h>

const int MOTOR_SPEED_SLOW   = 50;
const int MOTOR_SPEED_MEDIUM = 90;
const int MOTOR_SPEED_FAST   = 140;

// PID Tuning Parameters [COMPASS CODE - UNCHANGED]
float Kp = 1.2;   // Proportional gain
float Ki = 0.02;  // Integral gain
float Kd = 0.6;   // Derivative gain

float compassIntegral = 0.0;   // Accumulates past errors over time
float compassPrevError = 0.0;  // Stores previous error to calculate rate of change
unsigned long lastTime = 0;    // Tracks loop time difference (dt)

// Case Duration Lock Parameters (1.25 Seconds)
const unsigned long CASE_HOLD_DURATION = 1250; // 1.25s in milliseconds
unsigned long caseHoldStartTime = 0;
int lockedSector = -1;
float lockedAngle = -1.0;
bool forceStraight = false;

// Counter to prevent infinite side-kicking/veering
int sideAttackCounter = 0;     

static PeanutKingSoccerV4 robot;

void motors(int RF_spd, int RB_spd, int LB_spd, int LF_spd) {  // RF=M1, RB=M2, LB=M3, LF=M4
  robot.motorSetSpeed(RightFront, RF_spd);
  robot.motorSetSpeed(RightBack,  RB_spd);
  robot.motorSetSpeed(LeftBack,   LB_spd);
  robot.motorSetSpeed(LeftFront,  LF_spd);
}  

void attack(int speed) { motors(-speed, -speed, speed, speed); }
void seek2(int speed)  { motors(speed, 0, -speed, 0); }
void seek6(int speed)  { motors(0, speed, 0, -speed); }
void seek1(int speed)  { motors(speed, -speed, -speed, speed); }
void seek7(int speed)  { motors(-speed, speed, speed, -speed); }
void seek011(int speed){ motors(0, speed, 0, -speed); }
void seek89(int speed) { motors(speed, 0, -speed, 0); }
void seek10(int speed) { motors(speed, -speed, -speed, speed); }
void turnRight(int speed) { motors(speed, speed, speed, speed); }
void turnLeft(int speed)  { motors(-speed, -speed, -speed, -speed); }
void attackleft(int speed)  { motors(-speed, 0, speed, 0); }
void attackright(int speed) { motors(0, -speed, 0, speed); }

// Calculates precise ball angle (0.0° to 359.9°) using vector sum of 12 IR sensors
float getAccurateBallAngle(uint8_t* ir, int minThreshold = 10) {
  float sumX = 0.0;
  float sumY = 0.0;
  int maxVal = 0;

  for (int i = 0; i < 12; i++) {
    if (ir[i] > maxVal) maxVal = ir[i];
    
    float angleRad = (i * 30.0) * (PI / 180.0);
    sumX += ir[i] * cos(angleRad);
    sumY += ir[i] * sin(angleRad);
  }

  if (maxVal < minThreshold) return -1.0;

  float angleDeg = atan2(sumY, sumX) * (180.0 / PI);
  if (angleDeg < 0) angleDeg += 360.0;

  return angleDeg;
}

void setup() {
  robot.init();
  Serial.begin(115200);
  lastTime = millis();
}

void loop() {
  // --- 1. CALCULATE TIME DELTA (dt) --- [COMPASS CODE - UNCHANGED]
  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0;
  if (dt <= 0.0) dt = 0.001;
  lastTime = now;

  // --- 2. HEADING ERROR CALCULATION --- [COMPASS CODE - UNCHANGED]
  int rawHeading = robot.compassRead();
  int compassDeadZone = 5;

  float headingError = rawHeading;
  if (headingError > 180) {
    headingError -= 360;
  }

  bool isCompassAligned = (fabs(headingError) <= compassDeadZone);

  if (!isCompassAligned) {
    // --- 3. PID COMPASS CORRECTION --- [COMPASS CODE - UNCHANGED]
    float P = Kp * headingError;

    compassIntegral += headingError * dt;
    compassIntegral = constrain(compassIntegral, -100.0, 100.0);
    float I = Ki * compassIntegral;

    float derivative = (headingError - compassPrevError) / dt;
    float D = Kd * derivative;

    float pidOutput = P + I + D;
    compassPrevError = headingError;

    int pidTurnSpeed = abs((int)pidOutput);
    pidTurnSpeed = constrain(pidTurnSpeed, MOTOR_SPEED_SLOW, MOTOR_SPEED_FAST);

    if (headingError > 0) {
      turnLeft(pidTurnSpeed);
    } else {
      turnRight(pidTurnSpeed);
    }

    sideAttackCounter = 0;
    lockedSector = -1; // Reset action lock when compass corrects

  } else {
    // [COMPASS CODE - UNCHANGED]
    compassIntegral = 0.0;
    compassPrevError = 0.0;

    // --- 4. BALL CATCHING MOVEMENTS WITH 1.25s CASE LOCK ---
    // Update case decision ONLY if 1.25 seconds passed or no case is currently running
    if (lockedSector == -1 || (millis() - caseHoldStartTime >= CASE_HOLD_DURATION)) {
      caseHoldStartTime = millis();

      uint8_t* ir = robot.compoundEyeReadAll();
      lockedAngle = getAccurateBallAngle(ir);

      if (lockedAngle >= 0) {
        lockedSector = ((int)(lockedAngle + 15.0) / 30) % 12;
      } else {
        lockedSector = -1;
      }

      // Track side-attack counter per 1.25s action cycle
      if (lockedSector == 3 && lockedAngle < 90.0) {
        sideAttackCounter++;
      } else if (lockedSector == 5 && lockedAngle > 150.0) {
        sideAttackCounter++;
      } else {
        sideAttackCounter = 0;
      }

      // Force straight attack if side-attack occurs 10 consecutive cycles
      if (sideAttackCounter >= 10) {
        forceStraight = true;
        sideAttackCounter = 0;
      } else {
        forceStraight = false;
      }
    }

    // Execute locked case for 1.25 seconds
    switch (lockedSector) {        
      case 4:
        attack(MOTOR_SPEED_FAST);
        break;

      case 3:
        if (forceStraight) {
          attack(MOTOR_SPEED_FAST);
        } else {
          int calculatedSpeed = map((int)lockedAngle, 75, 105, MOTOR_SPEED_MEDIUM, MOTOR_SPEED_FAST);
          calculatedSpeed = constrain(calculatedSpeed, MOTOR_SPEED_MEDIUM, MOTOR_SPEED_FAST);

          if (lockedAngle < 90.0) {
            attackleft(calculatedSpeed);
          } else {
            motors(-MOTOR_SPEED_FAST, -calculatedSpeed, MOTOR_SPEED_FAST, calculatedSpeed);
          }
        }
        break;

      case 5:
        if (forceStraight) {
          attack(MOTOR_SPEED_FAST);
        } else {
          int calculatedSpeed = map((int)lockedAngle, 165, 135, MOTOR_SPEED_MEDIUM, MOTOR_SPEED_FAST);
          calculatedSpeed = constrain(calculatedSpeed, MOTOR_SPEED_MEDIUM, MOTOR_SPEED_FAST);

          if (lockedAngle > 150.0) {
            attackright(calculatedSpeed);
          } else {
            motors(-calculatedSpeed, -MOTOR_SPEED_FAST, calculatedSpeed, MOTOR_SPEED_FAST);
          }
        }
        break;

      case 2:
        seek2(MOTOR_SPEED_FAST);
        break;
      case 1:
        seek1(MOTOR_SPEED_FAST);
        break;
      case 6:
        seek6(MOTOR_SPEED_FAST);
        break;
      case 8:
      case 9:
        seek89(MOTOR_SPEED_FAST);
        break;
      case 7:
        seek7(MOTOR_SPEED_FAST);
        break;                 
      case 0:
      case 11:
        seek011(MOTOR_SPEED_FAST);
        break;
      case 10:
        seek10(MOTOR_SPEED_FAST);
        break;
      default:
        motors(0, 0, 0, 0);
        break;
    }
  }
}