#include <Arduino.h>
#include "Servo.h"

#define yawPin    9
#define pitchPin  10

#define LDR_topRight        A0
#define LDR_topLeft         A1
#define LDR_bottomRight     A2
#define LDR_bottomLeft      A3

#define controlLoopFreq      10                         //hertz
#define controlLoop_dt       1000 / controlLoopFreq     //milliseconds
#define invertedYawLoop     false
#define invertedPitchLoop   false


Servo yawServo;
Servo pitchServo;

unsigned long int ControlLoopTimer = 0;
double yaw_controlEffort = 0;
double pitch_controlEffort = 0;


void setup() {
  yawServo.attach(yawPin);
  pitchServo.attach(pitchPin);

}

void loop() 
{
  if(millis()-ControlLoopTimer>=controlLoop_dt)
  {
    controlLoop();
  }
  // put your main code here, to run repeatedly:
}

void controlLoop()
{
  ControlLoopTimer=millis();
  yaw_Controller();
  pitch_Controller();
  yawServo.writeMicroseconds(yaw_controlEffort);
  pitchServo.writeMicroseconds(pitch_controlEffort);
}

void yaw_Controller()
{
  double Ki = 1;
  double deadZone = 10;
  double r = 0;
  double y = (double)(analogRead(LDR_topRight)+analogRead(LDR_bottomRight)-analogRead(LDR_topLeft)-analogRead(LDR_bottomLeft));
  double du = Ki*((r-y)/100);

  if(invertedYawLoop==true)
  {
    du*=-1;
  }

  if(yaw_controlEffort>1000 && yaw_controlEffort<2000)
  {
    yaw_controlEffort += du;
  }
  else
  {
    // this function acts as saturation
  }
}

int pitch_Controller()
{
  double Kp = 1;
  double deadZone = 10;
  double r = 0;
  double y = (double)(analogRead(LDR_topRight)+analogRead(LDR_topLeft)-analogRead(LDR_bottomRight)-analogRead(LDR_bottomLeft));

  double du = Ki*((r-y)/100);

  if(invertedYawLoop==true)
  {
    du*=-1;
  }

  if(yaw_controlEffort>1000 && yaw_controlEffort<2000)
  {
    yaw_controlEffort += du;
  }
  else
  {
    // this function acts as saturation
  }
}

