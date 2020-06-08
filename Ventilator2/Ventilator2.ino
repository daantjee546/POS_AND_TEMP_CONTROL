#include "PWM.h"
#include <Arduino.h>

const int fanControlPin = 3;
const int interruptPin = 2;

volatile byte state = LOW;
unsigned long ms, prevMs, dt, rpm; // dt = deltaTime
int32_t pwmFrequency = 25000;

unsigned long lastTime;
double Input, Output, Setpoint = 400;
double errSum, lastErr;
double kp = 0.1, ki = 0, kd = 0;
double error;

void handleTachoPulse()
{
  state = HIGH;
}

void Compute()
{
  unsigned long now = millis();
  double timeChange = (double)(now - lastTime);

  error = (Setpoint) - (Input);
  errSum += (error * timeChange);
  double dErr = (error - lastErr) / timeChange;

  Output = kp * error + ki * errSum + kd * dErr;
//  Serial.print("Output = ");
//  Serial.println(Output);

  lastErr = error;
  lastTime = now;
}

void setup()
{
  Serial.begin(9600);
  InitTimersSafe();
  SetPinFrequencySafe(fanControlPin, pwmFrequency);

  pinMode(fanControlPin, OUTPUT);
  pinMode(interruptPin, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(interruptPin), handleTachoPulse, CHANGE);
  ms = millis();

  prevMs = ms;

  pwmWrite(fanControlPin, 255);
}

void loop()
{
  for (int i = 0; i < 1000; i++)
  {
    for (int change = 0; change < 4; change++)
    {
      while (state == LOW)
      {
      }
      delay(10);
      state = LOW;
    }
    ms = millis();
    dt = (ms - prevMs);
    prevMs = ms;
    rpm = 60000UL / dt;  // RPM = (rotation per ms) *1000*60


    if (dt > 1)
    {
      Input = rpm;
      Compute();
            
      Serial.print("Input = ");
      Serial.println(Input);

      pwmWrite(fanControlPin, (Setpoint * 0.34) - (Output));
    }
  }
}
