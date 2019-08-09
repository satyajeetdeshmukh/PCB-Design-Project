
int VoltageInputPin = A0;
int PWMPin;
unsigned long lastTime = 0;
float Input, Output, Setpoint;
float ITerm=0, lastInput;
float kp, ki;
int SampleTime;
float outMin, outMax;
float VoltDivider;
float InputR;
float inputScale;
float ki_x;

void Compute()
{
  unsigned long now = millis();
  int timeChange = (now - lastTime);
  if (timeChange >= SampleTime)
  {
    // Take Input
    
//    Input = 0;
//    for (int i=0; i<5; i++) {
//      Input += analogRead(VoltageInputPin);
//    }
    Input = analogRead(VoltageInputPin);
    Input = (float)Input*0.054;
    Serial.println(Input);
    
    /*Compute all the working error variables*/
    float error = Setpoint - Input;
    
    ITerm += (ki * error)*timeChange/1000;
    
    if (ITerm > outMax) ITerm = outMax;
    else if (ITerm < outMin) ITerm = outMin;

    /*Compute PID Output*/
    Output = kp * error + ITerm;
    
    if (Output > outMax) Output = outMax;
    else if (Output < outMin) Output = outMin;
    
    Serial.println(Output);
    Serial.println('\n');

    /*Remember some variables for next time*/
    lastInput = Input;
    lastTime = now;
  }
}

void setup() {
  Setpoint = 5; // Reference Voltage
  Output  = 0.2; // Starting duty cycle
  
  VoltDivider = 11; //Ratio by which the voltage divider steps down the voltage across load

  // Set kp and ki
  kp = 0.01;
  ki = 0.01;
  

  // Limiting the Output duty cycle
  outMin = 0.2;
  outMax = 0.7;
  
  SampleTime = 8; //sampling time for the pi controller in ms * 8 as we have changed the clock speed of the pwm pins
  PWMPin = 13;
  
  // Setting clock speed of pins 13 and 4 to 7.8kHz
  Serial.begin(9600);
  pinMode(PWMPin, OUTPUT);
  int myEraser = 7;             // this is 111 in binary and is used as an eraser
  TCCR0B &= ~myEraser;   // this operation (AND plus NOT),  set the three bits in TCCR2B to 0
  //TCCR0B = (TCCR0B&248)|1;
  int myPrescaler = 2;         // this could be a number in [1 , 6]. In this case, 3 corresponds in binary to 011.
  TCCR0B |= myPrescaler;  //this operation (OR), replaces the last three bits in TCCR2B with our new value 011

  ki_x = ki * SampleTime / 1000;
  inputScale = 5/1023*VoltDivider;
}



void loop() {
  // put your main code here, to run repeatedly:
  Compute(); //computes new duty cycle every sampling time
  analogWrite(PWMPin, 255*(1-Output)); //keeps running the pwm signal
}
