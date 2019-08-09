void setup() {
  // put your setup code here, to run once:
  pinMode(35, OUTPUT);

}

void loop() {
  // 100 us for 10khz
  digitalWrite(35, HIGH);
  delayMicroseconds(50);
  digitalWrite(35, LOW);
  delayMicroseconds(50);
}
