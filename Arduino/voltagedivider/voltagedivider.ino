int VoltageInputPin = A0;
float Input=0.0;
float InputR=0.0;

void setup() {
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  Input = analogRead(VoltageInputPin);
  InputR = (float)Input*0.0537/1.03;
  Serial.println(InputR);
  delay(100);
}
