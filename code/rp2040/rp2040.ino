void setup() {
  Serial.begin(115200);
  Serial1.setRX(1);
  Serial1.setTX(0);
  Serial1.begin(57600);
}

void loop() {
  while (Serial.available() > 0) {
    Serial1.write(Serial.read());
  }
  while (Serial1.available() > 0) {
    Serial.write(Serial1.read());
  }
}
