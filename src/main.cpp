#include <Arduino.h>
#include "Servo.h"

#define yawPin    9
#define pitchPin  10

#define LDR_topRight      A0
#define LDR_topLeft       A1
#define LDR_bottomRight   A2
#define LDR_bottomLeft    A3

#define pitchLoopFreq     100 //hertz
#define yawLoopFreq       100 //hertz
#define yaw_dt_us         1000000 / yawLoopFreq
#define pitch_dt_us       1000000 / pitchLoopFreq


Servo yawSer;
Servo pitchSer;


void setup() {
  yawSer.attach(yawPin);
  pitchSer.attach(pitchPin);

}

void loop() 
{
  
  // put your main code here, to run repeatedly:
}

int yaw_Controller()
{

}

int pitch_Controller()
{
  
  double r = (double) delta
  double Kp = 1
  u = Kp*r

}