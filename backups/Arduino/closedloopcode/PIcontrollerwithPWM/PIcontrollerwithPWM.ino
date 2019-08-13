// Global declarations and default values
unsigned long count = 0;
int VoltageInputPin = A0;
int PWMPin = 13;
unsigned long lastTime = 0;
float Input = 0, Output = 0, Setpoint = 0;
float ITerm = 0, lastInput;
float kp = 1, ki = 1;
float outMin = 0, outMax = 1;


void setup() {
  // User set parameters
  Setpoint = 5; // Reference Voltage
  VoltageInputPin = A0; // Pin being used to take voltage input
  PWMPin = 13; // Pin being used for PWM
  Output  = 0.2; // Starting duty cycle
  kp = 0.01; // Set kp and ki
  ki = 0.01;

  // Limiting the Output duty cycle
  outMin = 0.2;
  outMax = 0.7;

  // Setting clock speed of pins 13 and 4 to 7.8kHz on Atmega 2560
  pinMode(PWMPin, OUTPUT);
  int myEraser = 7; // this is 111 in binary and is used as an eraser
  TCCR0B &= ~myEraser; // this operation (AND plus NOT),  set the three bits in TCCR2B to 0
  //TCCR0B = (TCCR0B&248)|1;
  int myPrescaler = 2; // this could be a number in [1 , 6]. In this case, 2 corresponds in binary to 010.
  TCCR0B |= myPrescaler; //this operation (OR), replaces the last three bits in TCCR2B with our new value 010.

  Serial.begin(9600); // Initializing serial port
}

void Compute()
{
  unsigned long now = micros();
  unsigned long timeChange = (now - lastTime);

  Input = analogRead(VoltageInputPin); // Input is read as (1023-0) for (5,0) V
  //Input = (float)Input*0.054; // Input needs to scaled to get real value for calculating error
  
  float error = Setpoint - Input; // Calculate error term

  ITerm += (ki * error) * (timeChange / 1000000); // Calculate and limit Integration Term
  if (ITerm > outMax) ITerm = outMax;
  else if (ITerm < outMin) ITerm = outMin;

  Output = kp * error + ITerm; // Calculate new duty cycle

  if (Output > outMax) Output = outMax; // Limit new duty cycle
  else if (Output < outMin) Output = outMin;
  
  lastInput = Input; // Preparing for next calculation
  lastTime = now;

  // For debugging:
  //Serial.println(Input);
  //Serial.println(Output);
  //Serial.println('\n');
}

void loop() {
  
  count++;
  if (count % 10000 == 0) {
    Serial.println(count);
  }

  Compute(); //computes new duty cycle every sampling time
  analogWrite(PWMPin, 255 * (1-Output)); //keeps running the pwm signal, 1-output as the octocoupler inverts the signal!
}
