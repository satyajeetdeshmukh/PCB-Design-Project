
int VoltageInputPin = A3;
unsigned long lastTime = 0;
float Input, Output, Setpoint;
float ITerm=0, lastInput;
float kp, ki;
int SampleTime;
float outMin, outMax;
float VoltDivider;

void Compute()
{
  unsigned long now = millis();
  int timeChange = (now - lastTime);
  if (timeChange >= SampleTime)
  {
    // Take Input
    Input = analogRead(VoltageInputPin)/1024*5*VoltDivider; // input (as 0-1024) / 1024 * 5 * volt_divider_factor to get actual voltage
    
    /*Compute all the working error variables*/
    double error = Setpoint - Input;
    ITerm += (ki * error);
    if (ITerm > outMax) ITerm = outMax;
    else if (ITerm < outMin) ITerm = outMin;

    /*Compute PID Output*/
    Output = kp * error + ITerm;
    if (Output > outMax) Output = outMax;
    else if (Output < outMin) Output = outMin;

    /*Remember some variables for next time*/
    lastInput = Input;
    lastTime = now;
  }
}

void SetTunings(float Kp, float Ki)
{
  if (Kp < 0 || Ki < 0) return;
  float SampleTimeInSec = ((float)SampleTime) / 1000;
  kp = Kp;
  ki = Ki * SampleTimeInSec;
}

void SetSampleTime(float NewSampleTime)
{
  if (NewSampleTime > 0)
  {
    float ratio  = (float)NewSampleTime / (float)SampleTime;
    ki *= ratio;
    SampleTime = (float)NewSampleTime;
  }
}

void SetOutputLimits(float Min, float Max)
{
  if (Min > Max) return;
  outMin = Min;
  outMax = Max;

  if (Output > outMax) Output = outMax;
  else if (Output < outMin) Output = outMin;

  if (ITerm > outMax) ITerm = outMax;
  else if (ITerm < outMin) ITerm = outMin;
}

void PWM() {
  digitalWrite(13, HIGH);
  delayMicroseconds(50*Output);
  digitalWrite(13, LOW);
  delayMicroseconds(50*(1-Output));
}

void setup() {
  Setpoint = 15; //Required Voltage
  VoltDivider = 12; //Ratio by which the voltage divider steps down the voltage across load
  SetTunings(0.1, 100); //(Kp, ki*sample_time_in_sec)
  SetOutputLimits(0, 1); //(Min, Max); Output of PI
  pinMode(13, OUTPUT);
  SampleTime = 25; //sampling time for the pi controller in ms
}



void loop() {
  // put your main code here, to run repeatedly:
  Compute(); //computes new duty cycle every sampling time
  PWM(); //keeps running the pwm signal
}
