#include <Arduino.h>
#include "Servo.h"
#include <Wire.h>
#include "LiquidCrystal_I2C.h"

#define DEBUG                   true

#define yawPin                  9
#define pitchPin                10

#define LDR_topRight            A0
#define LDR_topLeft             A1
#define LDR_bottomRight         A2
#define LDR_bottomLeft          A3
#define V_Pin                   A6
#define I_pin                   A7

#define controlLoopFreq         10                         //hertz
#define controlLoop_dt          1000 / controlLoopFreq     //milliseconds
#define invertedYawLoop         false
#define invertedPitchLoop       true
#define maxServoMicroseconds    1900
#define minServoMicroseconds    1100


#define lcdFreq             1                         //hertz
#define lcd_dt              1000 / lcdFreq            //milliseconds

#define DEBUG_dt            250

#define Volt2Amp            0.4


Servo yawServo;
Servo pitchServo;

LiquidCrystal_I2C lcd(0x27,20,2);  // set the LCD address to 0x27 for a 16 chars and 2 line display

unsigned long int ControlLoopTimer = 0;
unsigned long int lcdTimer = 0;
unsigned long int debugTimer=0;
int yaw_Position = 1500;
int pitch_Position = 1500;
double deadZone = 0;

void yaw_Controller();
void pitch_Controller();
void controlLoop();
void power_Display();
void lcd_Setup();
double saturation_deadZone(double val, double dval, double topValue, double bottomValue, double deadZone);

void setup() 
{
  if(DEBUG)
  {
    Serial.begin(115200);
  }

  yawServo.attach(yawPin);
  yawServo.writeMicroseconds(1500);

  pitchServo.attach(pitchPin);
  pitchServo.writeMicroseconds(1500);

  lcd_Setup();
}
void loop() 
{
  unsigned long int currentTime = millis();
  if(currentTime-ControlLoopTimer >= controlLoop_dt)
  {
    ControlLoopTimer=millis();
    controlLoop();
  }

  if(currentTime-lcdTimer >= lcd_dt)
  {
    lcdTimer=millis();
    power_Display();
  }
  
  if(DEBUG && currentTime - debugTimer >= DEBUG_dt)
  {
    debugTimer=millis();
    Serial.println("yaw: "+String(yaw_Position)+" pitch: "+String(pitch_Position));
  }
}

void lcd_Setup()
{
  lcd.init();
  lcd.init();
  lcd.clear();
  lcd.backlight();
}
void power_Display()
{
  double I = ((((double)analogRead(I_pin)) * 5 / 1023)-2.5)*1/0.4;
  double V = ((double)analogRead(V_Pin)) * 5 / 1023;
  double P = V * I;

  lcd.setCursor(0,0);
  lcd.print("V: ");
  lcd.print(V,2);
  lcd.print(" I: ");
  lcd.print(I,2);
  lcd.setCursor(0,1);
  lcd.print("P: ");
  lcd.print(P,2);
  lcd.print(" [Watt]");
}
void controlLoop()
{ 
  yaw_Controller();
  pitch_Controller();
  yawServo.writeMicroseconds(yaw_Position);
  pitchServo.writeMicroseconds(pitch_Position);
}
void yaw_Controller()
{
  double Kp = 1200;
  double r = 0;
  double y = (double)(analogRead(LDR_topRight)+analogRead(LDR_bottomRight)-analogRead(LDR_topLeft)-analogRead(LDR_bottomLeft));
  double du = (Kp / controlLoop_dt)*(r-y);

  if(invertedYawLoop==true)
  {
    du*=-1;
  }

  du = saturation_deadZone(yaw_Position,du,maxServoMicroseconds,minServoMicroseconds,deadZone);
  yaw_Position += (int)du;
}
void pitch_Controller()
{
  double Kp = 700;
  double r = 0;
  double y = (double)(analogRead(LDR_topRight)+analogRead(LDR_topLeft)-analogRead(LDR_bottomRight)-analogRead(LDR_bottomLeft));

  double du = (Kp / controlLoop_dt)*(r-y);

  if(invertedPitchLoop==true)
  {
    du*=-1;
  }

  du = saturation_deadZone(pitch_Position,du,maxServoMicroseconds,minServoMicroseconds,deadZone);
  pitch_Position += (int)du;
}
double saturation_deadZone(double val, double dval, double topValue, double bottomValue, double deadZone)
{
  if(abs(dval) < deadZone)
  {
    // dead zone function
    return 0;
  }
  else if ((val <= bottomValue && dval < 0) || (val >= topValue && dval > 0))
  {
    //saturation function
    return 0;
  }
  else return dval;
}

