#include<PPMReader.h>

int interruptPin = 2;
int channelAmount = 4;
PPMReader ppm(interruptPin, channelAmount);

void setup() {
    Serial.begin(9600);
}

void loop() {
    // Print latest valid values from all channels
    for (int channel = 1; channel <= channelAmount; ++channel) {
        unsigned long value = ppm.latestValidChannelValue(channel, 0);
        Serial.print(String(value) + " ");
    }
    delay(100);
    Serial.println();
}
