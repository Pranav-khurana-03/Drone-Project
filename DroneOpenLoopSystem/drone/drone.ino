#include <Servo.h>

const int vrA = 13;
int vrAVal = 0;

bool start = false;

Servo frontRight;     // create servo objects to control the ESCs
Servo frontLeft;     
Servo backRight;     
Servo backLeft;

const int throttle = 4; 
const int pitch = 5; 
const int roll = 2; 
const int yaw = 3;

const int frontRightTX = 11; // White
const int frontLeftTX = 9; // Green
const int backRightTX = 10; // Yellow
const int backLeftTX = 8; // Orange


unsigned long pulseWidth;  // value from the transmitter

void simpleScale(int* frontRight, int* frontLeft, int*backRight, int*backLeft, const int MIN, const int MAX) {  
  *frontRight = max(0, min(180, *frontRight));
  *frontLeft = max(0, min(180, *frontLeft));
  *backRight = max(0, min(180, *backRight));
  *backLeft = max(0, min(180, *backLeft));
}

void scale(int* frontRight, int* frontLeft, int*backRight, int*backLeft, const int MIN, const int MAX, int minFound, int maxFound) {  
  if (maxFound>MAX || minFound<MIN) {
    *frontRight = (*frontRight - minFound)/(maxFound-minFound) * (MAX-MIN) + MIN;
    *frontLeft = (*frontLeft - minFound)/(maxFound-minFound) * (MAX-MIN) + MIN;
    *backRight = (*backRight - minFound)/(maxFound-minFound) * (MAX-MIN) + MIN;
    *backLeft = (*backLeft - minFound)/(maxFound-minFound) * (MAX-MIN) + MIN;
  }
}


int findMax(int A, int B, int C, int D) {
  return max(A, max(B, max(C, D)));
}

int findMin(int A, int B, int C, int D) {
  return min(A, min(B, min(C, D)));
}


void setup() {
  pinMode(vrA, INPUT);
  pinMode(throttle, INPUT);
  pinMode(pitch, INPUT);
  pinMode(roll, INPUT);
  pinMode(yaw, INPUT);
  frontRight.attach(frontRightTX, 1000, 2000); // (pin, min pulse width, max pulse width in microseconds) 
  frontLeft.attach(frontLeftTX, 1000, 2000);
  backRight.attach(backRightTX, 1000, 2000);
  backLeft.attach(backLeftTX, 1000, 2000);
  frontRight.write(0);
  frontLeft.write(0);
  backLeft.write(0);
  backRight.write(0);
  Serial.begin(9600);
  vrAVal = 0;
}

void loop() {
  // while (!vrAVal && !start) {
  //   int vrAVal = map(pulseIn(vrA,HIGH), 990, 1984, 0, 1);
  //   if (vrAVal) start = true;
  // }

  // Get values from all 4 channels
  int throttleVal, pitchVal, rollVal, yawVal;
  pulseWidth = pulseIn(throttle, HIGH);
  throttleVal = map(pulseWidth, 970, 1982, 0, 180);
  
  pulseWidth = pulseIn(pitch, HIGH);
  pitchVal = map(pulseWidth, 970, 1982, -90, 90);
  
  pulseWidth = pulseIn(roll, HIGH);
  rollVal = map(pulseWidth, 970, 1982, -90, 90);
  
  pulseWidth = pulseIn(yaw, HIGH);
  yawVal = map(pulseWidth, 970, 1982, -90, 90);

  int frontRightPower, frontLeftPower, backLeftPower, backRightPower;
  
  
  frontRightPower = throttleVal - pitchVal - rollVal - yawVal;
  backRightPower  = throttleVal + pitchVal - rollVal + yawVal;  
  backLeftPower   = throttleVal + pitchVal + rollVal - yawVal;  
  frontLeftPower  = throttleVal - pitchVal + rollVal + yawVal;
  
  
  


  
  int maxPower = findMax(frontRightPower, frontLeftPower, backRightPower, backLeftPower);
  int minPower = findMin(frontRightPower, frontLeftPower, backRightPower, backLeftPower);
  simpleScale(&frontRightPower, &frontLeftPower, &backRightPower, &backLeftPower, 0, 180);
  // // frontRight.write(frontRightPower);
  // // Serial.println("going");
  Serial.print(frontLeftPower); Serial.print("         ");
  Serial.println(frontRightPower);
  Serial.print(backLeftPower); Serial.print("         ");
  Serial.println(backRightPower);

  frontRight.write(frontRightPower);
  frontLeft.write(frontLeftPower);
  backRight.write(backRightPower);
  backLeft.write(backLeftPower);

}
