#include <PeanutKingSoccerV4.h>
#include <math.h>

const int MOTOR_SPEED_SLOW   = 50;
const int MOTOR_SPEED_MEDIUM = 90;
const int MOTOR_SPEED_FAST   = 140;
float Kp = 1.2;   // Proportional gain
float Ki = 0.02;  // Integral gain
float Kd = 0.6;   // Derivative gain
float compassIntegral = 0.0;   // Accumulates past errors over time
float compassPrevError = 0.0;  // Stores previous error to calculate rate of change
unsigned long lastTime = 0;    // Tracks loop time difference (dt)

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
void attackleft(int speed) { motors(-speed, 0, speed, 0); }
void attackright(int speed)  { motors(0, -speed, 0, speed); }
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
  int rawHeading = robot.compassRead();
  int compassDeadZone = 10;

  int headingError = rawHeading;
  if (headingError > 180) {
    headingError -= 360;
  }

  bool isCompassAligned = (abs(headingError) <= compassDeadZone);

  if (!isCompassAligned) {
    int dynamicTurnSpeed = map(abs(headingError), compassDeadZone, 180, MOTOR_SPEED_SLOW, MOTOR_SPEED_FAST);
    dynamicTurnSpeed = constrain(dynamicTurnSpeed, MOTOR_SPEED_SLOW, MOTOR_SPEED_FAST);

    if (headingError > 0) {
      turnLeft(dynamicTurnSpeed);
    } else {
      turnRight(dynamicTurnSpeed);
    }
  } else {
    uint8_t* ir = robot.compoundEyeReadAll();
    float accurateAngle = getAccurateBallAngle(ir);

    int ballSector = -1;
    if (accurateAngle >= 0) {
      ballSector = ((int)(accurateAngle + 15.0) / 30) % 12;
    }

    switch (ballSector) {        
      case 4:
        attack(MOTOR_SPEED_FAST);
        break;

      case 3:
        // Sector 3 (75° to 105°): Split at 90° midpoint
        if (accurateAngle < 90.0) {
          attackleft(MOTOR_SPEED_FAST);   // Leans towards Sector 2
        } else {
          attack(MOTOR_SPEED_FAST);  // Leans towards Sector 4
        }
        break;

      case 5:
        // Sector 5 (135° to 165°): Split at 150° midpoint
        if (accurateAngle < 150.0) {
          attack(MOTOR_SPEED_FAST);  // Leans towards Sector 4
        } else {
          attackright(MOTOR_SPEED_FAST);   // Leans towards Sector 6
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