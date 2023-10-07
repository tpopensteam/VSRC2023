#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <PS2X_lib.h>
#include <cstdlib>
#define PS2_DAT 12
#define PS2_CMD 13
#define PS2_SEL 15
#define PS2_CLK 14
#define ENA
#define ENB 
#define pressures false
#define rumble false
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
PS2X ps2x;
bool shooter_value = false, servo_intake = false;
void setup() 
{
  pwm.begin();
  pwm.setOscillatorFrequency(2400000);
  pwm.setPWMFreq(60);
  Wire.setClock(400000);
  Serial.begin(115200);
  Serial.print("Ket noi voi tay cam PS2)");

  int error = -1;
  for (int i = 0; i < 10; i++) 
  {
    delay(200);
    error = ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, pressures, rumble);
    Serial.print(".");
  }
  switch (error) 
  {
    case (0):
      Serial.println(" Ket noi tay cam PS2 thanh cong");
      break;
    case (1):
      Serial.println(" LOI) Khong tim thay tay cam, hay kiem tra day ket noi vơi tay cam ");
      break;
    case (2):
      Serial.println(" LOI) khong gui duoc lenh");
      break;
    case (3):
      Serial.println(" LOI) Khong vao duoc Pressures mode ");
      break;
  }

}
// servo setup
void servo_clockwise(uint16_t Servo)
{
  pwm.setPWM(Servo, 0, 180);
}
void servo_anticlockwise(uint16_t Servo1)
{
  pwm.setPWM(Servo1, 90, 580);
}
void servo_clockwise180(uint16_t Servo3)
{
  pwm.setPWM(Servo3, 0, 512);
}
void servo_anticlockwise180(uint16_t Servo4)
{
  pwm.setPWM(Servo4, 0, 205);
}
void stop_servo(uint8_t Servo2)
{
  pwm.setPWM(Servo2, 0, 0);
}
// motor setup
void rightdc(uint16_t right, uint16_t right2)
{
  pwm.setPWM(12, 0, right);
  pwm.setPWM(13, 0, right2);
}
void middle(uint16_t middle1, uint16_t middle2)
{
  pwm.setPWM(14, 0, middle1);
  pwm.setPWM(15, 0, middle2);
}
void leftdc(uint16_t left0, uint16_t left1)
{
  pwm.setPWM(10,0,left0);
  pwm.setPWM(11,0,left1);
}
// shooter and collector setup
void shooter()
{
  pwm.setPWM(8,0,0);
  pwm.setPWM(9,0,4000);
}
void shooter_stop()
{
  pwm.setPWM(8,0,0);
  pwm.setPWM(9,0,0);
}
// control setup
void ps2Control() 
{
  ps2x.read_gamepad(false, false);
  int joyleft = ps2x.Analog(PSS_LY);
  int joyright = ps2x.Analog(PSS_RY);
  int njoyl = ps2x.Analog(PSS_LX);
  int njoyr = ps2x.Analog(PSS_RX);
  njoyr = map(njoyr,0, 255, 4095, -4095);
  njoyl = map(njoyl,0, 255, 4095, -4095);
  joyright = map(joyright,0, 255, 4095, -4095);
  joyleft = map(joyleft, 0, 255, 4095, -4095);
  
  if(joyleft > 17)
  {
    leftdc(0, 0+joyleft);
    Serial.print("joyleft: ");
    Serial.println(joyleft);
  }
  else if(joyleft < 17)
  {
    leftdc(0 + abs(joyleft), 0);
    Serial.print("joyleft: ");
    Serial.println(joyleft);
  }
  else
  {
    leftdc(0,0);
  }
  if(joyright > 17)
  {
    rightdc(0, 0+joyright);
    Serial.print("joyright: ");
    Serial.println(joyright);
  }
  else if(joyright < 17)
  {
    rightdc(0 + abs(joyright), 0);
    Serial.print("joyright: ");
    Serial.println(joyright);
  }
  else 
  {
    rightdc(0,0);
  }
  if(njoyl < 17 && njoyr < 17)
  {
    middle(0, 0 + abs(njoyl+njoyr) /2;
  }
  else if(njoyl > 17 &&  njoyr > 17)
  {    
    middle(0 + (njoyr + njoyl) / 2, 0);
  }
  else
  {
    middle(0,0);
  }
  if(ps2x.ButtonPressed(PSB_L1))
  {
    shooter_value = !shooter_value;
  }
  if(shooter_value)
  {
    shooter();
  }
  else
  {
    shooter_stop();
  } 
  delay(50);
}
void loop() {ps2Control();}
