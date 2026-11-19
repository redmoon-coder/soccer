#include <PeanutKingSoccerV4.h>
#include <math.h>

const int MOTOR_SPEED_SLOW   = 50;
const int MOTOR_SPEED_MEDIUM = 90;
const int MOTOR_SPEED_FAST   = 140;

// PID Tuning Parameters (UNCHANGED)
float Kp = 1.2;   // Proportional gain
float Ki = 0.02;  // Integral gain
float Kd = 0.6;   // Derivative gain

float compassIntegral = 0.0;   // Accumulates past errors over time
float compassPrevError = 0.0;  // Stores previous error to calculate rate of change
unsigned long lastTime = 0;    // Tracks loop time difference (dt)

// 12-Eye angle table centered at Eye 4 = 0.0° (Dead Front)
// Negative = Left of front (-1° to -180°), Positive = Right of front (+1° to +180°)
// You can fine-tune individual angles here if your physical sensor spacing is uneven
const int NUM_EYES = 12;
const float eyeAngles[NUM_EYES] = {
  -120.0, // Eye 0  (Back-Left)
  -90.0,  // Eye 1  (Left)
  -60.0,  // Eye 2  (Front-Left)
  -30.0,  // Eye 3  (Slightly Left)
   0.0,   // Eye 4  (DEAD FRONT)
   30.0,  // Eye 5  (Slightly Right)
   60.0,  // Eye 6  (Front-Right)
   90.0,  // Eye 7  (Right)
   120.0, // Eye 8  (Back-Right)
   150.0, // Eye 9  (Rear-Right)
   180.0, // Eye 10 (Dead Back)
  -150.0  // Eye 11 (Rear-Left)
};

static PeanutKingSoccerV4 robot;

void motors(int RF_spd, int RB_spd, int LB_spd, int LF_spd) {  // RF=M1, RB=M2, LB=M3, LF=M4
  robot.motorSetSpeed(RightFront, RF_spd);
  robot.motorSetSpeed(RightBack,  RB_spd);
  robot.motorSetSpeed(LeftBack,   LB_spd);
  robot.motorSetSpeed(LeftFront,  LF_spd);
}  

void turnRight(int speed) { motors(speed, speed, speed, speed); }
void turnLeft(int speed)  { motors(-speed, -speed, -speed, -speed); }

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

  } else {
    // [COMPASS CODE - UNCHANGED]
    compassIntegral = 0.0;
    compassPrevError = 0.0;

    // --- 4. 3-EYE NEIGHBOR VECTOR BALL TRACKING ---
    uint8_t* ir      = robot.compoundEyeReadAll();
    int maxEye       = robot.compoundMaxEyeRead();
    int maxStrength  = robot.compoundMaxEyeValueRead();

    // Filter out noise when the ball is off the field
    if (maxStrength < 50 || maxEye < 0 || maxEye >= NUM_EYES) {
      motors(0, 0, 0, 0);
      return;
    }

    // Find neighboring eyes using modulo-12 wrapping
    int leftEye  = (maxEye - 1 + NUM_EYES) % NUM_EYES;
    int rightEye = (maxEye + 1) % NUM_EYES;

    // Signal strength of the center eye and its two neighbors
    int sCenter = maxStrength;
    int sLeft   = ir[leftEye];
    int sRight  = ir[rightEye];

    // Compute 2D vector sum using only the 3 active eyes
    float radCenter = eyeAngles[maxEye]   * PI / 180.0;
    float radLeft   = eyeAngles[leftEye]  * PI / 180.0;
    float radRight  = eyeAngles[rightEye] * PI / 180.0;

    float totalX = sCenter * cos(radCenter) + sLeft * cos(radLeft) + sRight * cos(radRight);
    float totalY = sCenter * sin(radCenter) + sLeft * sin(radLeft) + sRight * sin(radRight);

    // Exact ball angle (-180° to +180°, where 0° = dead front)
    float ballAngle = atan2(totalY, totalX) * 180.0 / PI;

    Serial.print("MaxEye: ");
    Serial.print(maxEye);
    Serial.print(" | Exact Angle: ");
    Serial.println(ballAngle);

    // --- 5. OMNI-WHEEL MOVEMENT FROM EXACT ANGLE ---
    // If ball is behind the robot (> ±105°), add an orbit offset so it circles behind the ball
    float moveAngle = ballAngle;
    if (moveAngle > 105.0) {
      moveAngle += 55.0; // Circle around right side to get behind ball
    } else if (moveAngle < -105.0) {
      moveAngle -= 55.0; // Circle around left side to get behind ball
    }

    float moveRad = moveAngle * PI / 180.0;
    float forward = cos(moveRad); // +1.0 at 0° (straight front)
    float strafe  = sin(moveRad); // -1.0 Left, +1.0 Right

    // Normalize so at 0° (dead front) all motors run at full MOTOR_SPEED_FAST
    float maxWheel = max(fabs(forward - strafe), fabs(forward + strafe));
    if (maxWheel < 1.0) maxWheel = 1.0;

    int rfSpeed = (int)(MOTOR_SPEED_FAST * (-forward + strafe) / maxWheel);
    int rbSpeed = (int)(MOTOR_SPEED_FAST * (-forward - strafe) / maxWheel);
    int lbSpeed = (int)(MOTOR_SPEED_FAST * ( forward - strafe) / maxWheel);
    int lfSpeed = (int)(MOTOR_SPEED_FAST * ( forward + strafe) / maxWheel);

    motors(rfSpeed, rbSpeed, lbSpeed, lfSpeed);
  }
}