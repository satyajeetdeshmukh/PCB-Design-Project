int PWMPin = 13;
float dutyCycle = 0.5;


void setup() {
  pinMode(PWMPin, OUTPUT);

  int myEraser = 7;             // this is 111 in binary and is used as an eraser
  TCCR0B &= ~myEraser;   // this operation (AND plus NOT),  set the three bits in TCCR2B to 0
  //TCCR0B = (TCCR0B&248)|1;
  int myPrescaler = 2;         // this could be a number in [1 , 6]. In this case, 3 corresponds in binary to 011.
  TCCR0B |= myPrescaler;  //this operation (OR), replaces the last three bits in TCCR2B with our new value 011
  

}

void someFunc(){
  delay(1000);
}

void loop() {
  // put your main code here, to run repeatedly:
  analogWrite(PWMPin, 255*dutyCycle);
  someFunc();
}
