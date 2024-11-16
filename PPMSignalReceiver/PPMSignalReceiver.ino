#include <PPMReader.h>

const int interruptPin = 2;
int channels = 4;

float desiredAngleRoll, desiredAnglePitch;

PPMReader receiverInput(interruptPin, channels);

float receiverValue[] = {0, 0, 0, 0};

void readReceiver() {
  for (int channel = 1; channel <= channels; ++channel) {
        receiverValue[channel-1] = receiverInput.latestValidChannelValue(channel, 0);
    }
}


void setup() {
  // put your setup code here, to run once:
  Serial.begin(57600);
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
}

void loop() {
  // put your main code here, to run repeatedly:
  desiredAnglePitch = 0.1*(receiverValue[1]-1500);
  desiredAngleRoll = 0.1*(receiverValue[0]-1500);
  float desiredRateYaw = 0.15*(receiverValue[3]-1500);
  readReceiver();
  Serial.print("Channels: ");
  Serial.print(channels);

  Serial.print("  Throttle: ");
  Serial.print(receiverValue[2]);
  Serial.print("  Pitch: ");
  Serial.print(desiredAnglePitch);

  Serial.print("  Roll: ");
  Serial.print(desiredAngleRoll);

  Serial.print("  Yaw: ");
  Serial.println(desiredRateYaw);

}
