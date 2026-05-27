#include <Servo.h>

Servo servos[15];
int pins[] = {2, 3, 4, 5, 6, 7}; // Pins PWM a l'ESP32

void setup() {
  Serial.begin(115200);
  for(int i = 0; i < 6; i++) {
    servos[i].attach(pins[i]);
    servos[i].write(90); // Posició neutre de seguretat
  }
}

void loop() {
  if (Serial.available() > 0) {
    String data = Serial.readStringUntil('\n');
    int startIdx = 0;
    int commaIdx = data.indexOf(',');
    int count = 0;

    while (commaIdx != -1 && count < 5) {
      int angle = data.substring(startIdx, commaIdx).toInt();
      servos[count++].write(constrain(angle, 0, 180));
      startIdx = commaIdx + 1;
      commaIdx = data.indexOf(',', startIdx);
    }
    // Últim angle
    int lastAngle = data.substring(startIdx).toInt();
    servos[count].write(constrain(lastAngle, 0, 180));
  }
}
