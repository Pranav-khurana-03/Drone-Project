#include <Wire.h>
#include <PPMReader.h>
#include <Servo.h>

bool speedy = true;
unsigned long lost = 0;
Servo frontRight;
Servo frontLeft;
Servo backRight;
Servo backLeft;

const int frontRightPin = 11; // White
const int frontLeftPin = 9; // Green
const int backRightPin = 10; // Yellow
const int backLeftPin = 8; // Orange 


const int minPower = 1180;
const int stopPower = 1000;
const int throttleCutOff = 1050;

float rollRate, yawRate, pitchRate;
float accX, accY, accZ;
float rollCalibrationRate, pitchCalibrationRate, yawCalibrationRate;
float rollAngle, pitchAngle;
float rollAngleCalibrationValue = 0, pitchAngleCalibrationValue = 0;

float desiredAngleRoll, desiredAnglePitch;
float desiredRateRoll, desiredRatePitch, desiredRateYaw;
float errorRateRoll, errorRatePitch, errorRateYaw;
float prevErrorRateRoll, prevErrorRatePitch, prevErrorRateYaw;
float prevITermRoll = 0, prevITermPitch = 0, prevITermYaw = 0;

float rollKStatic = 0; // 1000 to 2000
float pitchKStatic = 0;
float PRateRoll = 0.46; // 0.34
float PRatePitch = -0.4;
float PRateYaw = 0.005;

float IRateRoll = 0;  // 1.16
float IRatePitch = -0.25;
float IRateYaw = 0;

float DRateRoll = 0.007;  // 0.005
float DRatePitch = -0.007;
float DRateYaw = 0;

uint32_t loopTimer;
uint32_t printTimer;

float kalmanAngleRoll = 0;
float kalmanRollUncertainty = 2 * 2;
float kalmanAnglePitch = 0;
float kalmanPitchUncertainty = 2 * 2;
float kalmanOutput[] = { 0, 0 };


float receiverValues[] = { 0, 0, 0, 0 };
int channels = 4;
const int interruptPin = 2;
PPMReader receiverInput(interruptPin, channels);

void getReceiverInput() {
  for (int channel = 1; channel <= channels; channel++) {
    receiverValues[channel - 1] = receiverInput.latestValidChannelValue(channel, 0);
    // Serial.print(receiverValues[channel-1]); Serial.print(" ");
  }
  // Serial.println();
}

float getPIDEffort(float error, float P, float I, float D, float *prevError, float *prevITerm) {
  float pEffort = P * (error);
  float iEffort = *prevITerm + I * (error + *prevError) * 0.004 / 2;
  if (iEffort > 400) iEffort = 400;
  if (iEffort < -400) iEffort = -400;
  float dEffort = D * (error - *prevError) / 0.004;
  float pidOutput = pEffort + iEffort + dEffort;
  if (pidOutput > 400) pidOutput = 400;
  if (pidOutput < -400) pidOutput = -400;
  *prevITerm = iEffort;
  *prevError = error;
  return pidOutput;
}

void resetPID() {
  prevErrorRatePitch = prevErrorRateRoll = prevErrorRateYaw = 0;
  prevITermPitch = prevITermRoll = prevITermYaw = 0;
}


void kalman(float kalmanState, float kalmanUncertainty, float kalmanInput, float kalmanMeasurement) {
  kalmanState = kalmanState + 0.004 * kalmanInput;
  kalmanUncertainty = kalmanUncertainty + 0.004 * 0.004 * 4 * 4;
  float kalmanGain = kalmanUncertainty / (kalmanUncertainty + 3 * 3);
  kalmanState = kalmanState + kalmanGain * (kalmanMeasurement - kalmanState);
  kalmanUncertainty = (1 - kalmanGain) * kalmanUncertainty;
  kalmanOutput[0] = kalmanState;
  kalmanOutput[1] = kalmanUncertainty;
}







void gyroSignals() {
  Wire.beginTransmission(0x68);
  Wire.write(0x1A);
  Wire.write(0x05);
  Wire.endTransmission();

  Wire.beginTransmission(0x68);
  Wire.write(0x1C);
  Wire.write(0x10);
  Wire.endTransmission();

  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission();

  Wire.requestFrom(0x68, 6);

  int16_t accXLSB = Wire.read() << 8 | Wire.read();
  int16_t accYLSB = Wire.read() << 8 | Wire.read();
  int16_t accZLSB = Wire.read() << 8 | Wire.read();

  accX = (float)accXLSB / 4096 - 0.07;
  accY = (float)accYLSB / 4096 + 0.02;
  accZ = (float)accZLSB / 4096 - 0.1;

  pitchAngle = -atan(accY / sqrt(accX * accX + accZ * accZ)) / (3.142 / 180);
  rollAngle = atan(accX / sqrt(accY * accY + accZ * accZ)) / (3.142 / 180);
  rollAngle = rollAngle - rollAngleCalibrationValue;
  pitchAngle = pitchAngle - pitchAngleCalibrationValue;
  Wire.beginTransmission(0x68);
  Wire.write(0x1B);
  Wire.write(0x8);
  Wire.endTransmission();

  Wire.beginTransmission(0x68);
  Wire.write(0x43);
  Wire.endTransmission();

  Wire.requestFrom(0x68, 6);

  int16_t gyroX = Wire.read() << 8 | Wire.read();
  int16_t gyroY = Wire.read() << 8 | Wire.read();
  int16_t gyroZ = Wire.read() << 8 | Wire.read();

  rollRate = (float)gyroY / 65.5;
  pitchRate = (float)gyroX / 65.5;
  yawRate = (float)gyroZ / 65.5;
}


void calibratePitchRoll() {
  gyroSignals();
  rollAngleCalibrationValue = rollAngle;
  pitchAngleCalibrationValue = pitchAngle;
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(57600);
  delay(5);
  Serial.println("Starting...");
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
  delay(10);
  Serial.println("Gyro...");
  Wire.setClock(400000);
  Serial.println("Gyro...");
  Wire.begin();
  Serial.println("Gyro...");
  delay(250);
  Serial.println("Gyro...");

  Wire.beginTransmission(0x68);
  delay(10);
  Serial.println("Gyro...");
  Wire.write(0x6B);
  Serial.println("Gyro...");
  Wire.write(0x00);
  Serial.println("Gyro...");
  delay(2);
  Wire.endTransmission();

  Serial.println("GyroCalib...");

  for (int i = 0; i < 2000; i++) {
    gyroSignals();
    rollCalibrationRate += rollRate;
    pitchCalibrationRate += pitchRate;
    yawCalibrationRate += yawRate;
    delay(1);
  }
  Serial.println("Gyro Done.");

  rollCalibrationRate /= 2000;
  pitchCalibrationRate /= 2000;
  yawCalibrationRate /= 2000;
  calibratePitchRoll();
  digitalWrite(13, LOW);
  loopTimer = micros();
  frontRight.attach(frontRightPin, 1000, 2000);
  frontLeft.attach(frontLeftPin, 1000, 2000);
  backRight.attach(backRightPin, 1000, 2000);
  backLeft.attach(backLeftPin, 1000, 2000);
  frontRight.write(0);
  frontLeft.write(0);
  backRight.write(0);
  backLeft.write(0);
  printTimer = micros();

}

void loop() {
  if (!speedy) {
    lost++;
  }
  speedy = false;
  // put your main code here, to run repeatedly:
  gyroSignals();
  rollRate -= rollCalibrationRate;
  pitchRate -= pitchCalibrationRate;
  yawRate -= yawCalibrationRate;

  pitchRate = -pitchRate;
  rollRate = -rollRate;
  // Serial.print(rollRate); Serial.print(" ");
  // Serial.print(pitchRate); Serial.print(" ");
  // Serial.print(yawRate); Serial.println(" ");

  kalman(kalmanAngleRoll, kalmanRollUncertainty, rollRate, rollAngle);
  kalmanAngleRoll = kalmanOutput[0]; // VERY VERY SKETCHY SOL TO MAKE ROLL 0
  kalmanRollUncertainty = kalmanOutput[1];
  kalman(kalmanAnglePitch, kalmanPitchUncertainty, pitchRate, pitchAngle);
  kalmanAnglePitch = kalmanOutput[0];
  kalmanPitchUncertainty = kalmanOutput[1];
  // Serial.print("PitchAngle: "); Serial.print(kalmanAnglePitch);
  // Serial.print("RollAngle: "); Serial.println(kalmanAngleRoll);

  getReceiverInput();
  float throttle, roll, pitch, yaw;
  throttle = receiverValues[2];
  if (throttle > 1800) throttle = 1800;
  desiredAnglePitch = -0.1 * (receiverValues[1] - 1500);
  desiredAngleRoll = 0.1 * (receiverValues[0] - 1500);
  desiredRateYaw = 0.15 * (receiverValues[3] - 1500);


  desiredRatePitch = 2 * (desiredAnglePitch - kalmanAnglePitch);
  desiredRateRoll = 2 * (desiredAngleRoll - kalmanAngleRoll);
  // Serial.println(desiredRatePitch);

  errorRateRoll = desiredRateRoll - rollRate;
  errorRatePitch = pitchRate - desiredRatePitch;
  errorRateYaw = desiredRateYaw - yawRate;

  roll = getPIDEffort(errorRateRoll, PRateRoll, IRateRoll, DRateRoll, &prevErrorRateRoll, &prevITermRoll) + rollKStatic;
  pitch = getPIDEffort(errorRatePitch, PRatePitch, IRatePitch, DRatePitch, &prevErrorRatePitch, &prevITermPitch);
  yaw = getPIDEffort(errorRateYaw, PRateYaw, IRateYaw, DRateYaw, &prevErrorRateYaw, &prevITermYaw);

  float frontRightPower, frontLeftPower, backRightPower, backLeftPower;
  frontRightPower = throttle - roll + pitch + yaw;
  frontLeftPower = throttle + roll + pitch - yaw;
  backRightPower = throttle - roll - pitch - yaw;
  backLeftPower = throttle + roll - pitch + yaw;

  if (frontRightPower > 2000) frontRightPower = 2000;
  if (frontLeftPower > 2000) frontLeftPower = 2000;
  if (backRightPower > 2000) backRightPower = 2000;
  if (backLeftPower > 2000) backLeftPower = 2000;

  if (frontRightPower < minPower) frontRightPower = minPower;
  if (frontLeftPower < minPower) frontLeftPower = minPower;
  if (backRightPower < minPower) backRightPower = minPower;
  if (backLeftPower < minPower) backLeftPower = minPower;

  if (throttle < throttleCutOff) {
    frontRightPower = stopPower;
    frontLeftPower = stopPower;
    backRightPower = stopPower;
    backLeftPower = stopPower;
    resetPID();
    // Serial.println(lost);
  }
  frontRightPower = map(frontRightPower, 1000, 2000, 0, 180);
  frontLeftPower = map(frontLeftPower, 1000, 2000, 0, 180);
  backRightPower = map(backRightPower, 1000, 2000, 0, 180);
  backLeftPower = map(backLeftPower, 1000, 2000, 0, 180);

  // Serial.print("FrontLeft:");
  // Serial.print(frontLeftPower);
  // Serial.print(",FrontRight:");
  // Serial.println(frontRightPower);
  // Serial.print(",backLeft:");
  // Serial.print(backLeftPower);
  // Serial.print(",backRight:");
  // Serial.println(backRightPower);
  // Serial.print(",max:");
  // Serial.println(50);
  // Serial.print(",min:");
  // Serial.println(0);
  // Serial.print(",desiredPitch:");
  // Serial.println(desiredAnglePitch);
  // Serial.print(",pitchAngle:");
  // Serial.println(kalmanAnglePitch);
  // Serial.print(",desiredRoll:");
  // Serial.println(desiredAngleRoll);
  // Serial.print(",RollAngle:");
  // Serial.println(kalmanAngleRoll);
  // Serial.print(",RollEffort:");
  // Serial.println(roll);
  // Serial.print(",PitchEffort:");
  // Serial.println(pitch);

  if (micros() - printTimer > 100000) {
    // Serial.print("Pitch Error ");
    // Serial.print(errorRatePitch);
    // Serial.print("  Roll Error ");
    // Serial.println(errorRateRoll);
    // Serial.print(pitch); Serial.print("  "); Serial.println(roll);
    // Serial.print(frontLeftPower);
    // Serial.print("         ");
    // Serial.println(frontRightPower);
    // Serial.print(backLeftPower);
    // Serial.print("         ");
    // Serial.println(backRightPower);
    // // Serial.println(roll);
    // Serial.print(rollAngle); Serial.print(" ");
    // Serial.println(rollAngleCalibrationValue);
    // Serial.print(pitchAngle); Serial.print(" ");
    // Serial.println(pitchAngleCalibrationValue);
    // Serial.print(kalmanAngleRoll); Serial.print(" ");
    // Serial.println(kalmanAnglePitch);
    // // Serial.println(pitchAngle);
    // Serial.print(accX);
    // Serial.print(" ");
    // Serial.print(accY);
    // Serial.print(" ");
    // Serial.println(accZ);
    // Serial.println(yawRate);
    // Serial.println(desiredRateYaw);
    // Serial.print("DesiredYawRate: ");
    // Serial.println(desiredRateYaw);
    // Serial.print(receiverValues[0]);
    // Serial.print(" ");
    // Serial.print(receiverValues[1]);
    // Serial.print(" ");
    // Serial.print(receiverValues[2]);
    // Serial.print(" ");
    // Serial.print(receiverValues[3]);
    // Serial.println(" ");

    Serial.print(rollAngle); Serial.print(" ");
    
    Serial.print(rollRate); Serial.print(" ");

    
    Serial.println(kalmanAngleRoll);

    printTimer = micros();
  }


  frontRight.write(frontRightPower);
  frontLeft.write(frontLeftPower);
  backRight.write(backRightPower);
  backLeft.write(backLeftPower);
  while (micros() - loopTimer < 4000)
    speedy = true;
  loopTimer = micros();
  // delay(300);
}
