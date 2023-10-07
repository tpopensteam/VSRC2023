#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <PS2X_lib.h>
#include <cstdlib>
#include <Servo.h>
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
bool shooter_value = false;
int servo_value = 0; 
void setup() 
{
  pwm.begin();
  pwm.setOscillatorFrequency(24000000);
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
void servo_90(uint16_t Servo)
{
  pwm.setPWM(Servo, 0, 308);
}
void servo_180(uint16_t Servo1)
{
  pwm.setPWM(Servo1, 0, 410);
}
void servo_0(uint8_t Servo2)
{
  pwm.setPWM(Servo2, 0, 205);
}
// motor setup
void vertical(uint16_t right, uint16_t right2, uint16_t left0, uint16_t left1)
{
  pwm.setPWM(12, 0, right);
  pwm.setPWM(13, 0, right2);
  pwm.setPWM(10,0,left0);
  pwm.setPWM(11,0,left1);
}
void horizon(uint16_t left0, uint16_t left1, uint16_t right, uint16_t right2)
{
  pwm.setPWM(12, 0, right);
  pwm.setPWM(13, 0, right2);
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
void intake(uint16_t intake, uint16_t intake2)
{
  pwm.setPWM(14,0,intake);
  pwm.setPWM(15,0,intake2);
  
}
// control setup
void ps2Control() 
{
  ps2x.read_gamepad(false, false);
  int joyleft = ps2x.Analog(PSS_LY);
  int njoyr = ps2x.Analog(PSS_RX);
  njoyr = map(njoyr,0, 255, 4095, -4095);
  joyleft = map(joyleft, 0, 255, 4095, -4095);
  servo_90();
  
  if(joyleft > 17)
  {
    vertical(0, 0+joyleft, 0, 0+joyleft);
    Serial.print("joyleft: ");
    Serial.println(joyleft);
  }
  else if(joyleft < 17)
  {
   vertical(0 + abs(joyleft), 0, 0+abs(joyleft), 0);
    Serial.print("joyleft: ");
    Serial.println(joyleft);
  }
  else
  {
    vertical(0,0,0,0);
  }
  if(njoyr > 17)
  {
    horizon(0+njoyr, 0 , 0 , 0+njoyr);
  }
  else if(njoyr < 17)
  {
    horizon(0, 0 + abs(njoyr), 0 + abs(njoyr), 0);
  }
  else 
  {
    horizon(0,0,0,0);
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
  if(ps2x.ButtonPressed(PSB_TRIANGLE))
  {
    intake(0,4000);
  }
  else if (ps2x.ButtonPressed(PSB_CROSS))
  {
    intake(4000,0);
  }
  else if(ps2x.ButtonPressed(PSB_CIRCLE))
  {
    intake(0,0);
  }
  if(ps2x.ButtonPressed(PSB_PAD_UP))
  {
    servo_0();
  }
  if(ps2x.ButtonPressed(PSB_PAD_DOWN))
  {
    servo_180();
  }
  
  delay(50);
}
void loop() {ps2Control();}
