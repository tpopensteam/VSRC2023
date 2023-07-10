#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <PS2X_lib.h>
  

#define PS2_DAT 12
#define PS2_CMD 13
#define PS2_SEL 15
#define PS2_CLK 14
#define SERVO 2
#define SERVO2 3
#define SERVO3 4
#define SERVO4 5
#define SERVO_FREQ_MIN 200
#define SERVO_FREQ_MAX 550
#define SERVO_MIN_DEGREE 0
#define SERVO_MAX_DEGREE 180

#define ENA
#define ENB 

#define pressures false
#define rumble false

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
PS2X ps2x;
bool mode_drive = false;
int servo1 = 0, servo2 = 0, servo3 = 0, servo4 = 0, collector_mode = 0, shooter_mode = 0;
void setup() 
{
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(50);

  pwm.setPWM(SERVO,0,0);

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
void servo_clockwise(uint8_t Servo)
{
  pwm.setPWM(Servo,0,180);
}
void servo_anticlockwise(uint8_t Servo1)
{
  pwm.setPWM(Servo1, 90, 680);
}
void stop_servo(uint8_t Servo2)
{
  pwm.setPWM(Servo2, 0, 0);
}
// motor setup
void forwarddc(uint16_t forward, uint16_t forward2)
{
  pwm.setPWM(10,0,forward);
  pwm.setPWM(11,0,forward2);
}
void forward2dc(uint16_t fur, uint16_t fur2)
{
  pwm.setPWM(12,0,fur);
  pwm.setPWM(13,0,fur2);
}
void rightdc(uint16_t right, uint16_t right2)
{
  pwm.setPWM(10,0,right);
  pwm.setPWM(11,0,right2);
}
void right2dc(uint16_t right0, uint16_t right1)
{
  pwm.setPWM(12,0,right0);
  pwm.setPWM(13,0,right1);
}
void leftdc(uint16_t left, uint16_t left2)
{
  pwm.setPWM(10,0,left);
  pwm.setPWM(11,0,left2);
}
void left2dc(uint16_t left0, uint16_t left1)
{
  pwm.setPWM(12,0,left0);
  pwm.setPWM(13,0,left1);
}
void backdc(uint16_t back, uint16_t back2)
{
  pwm.setPWM(10,0,back);
  pwm.setPWM(11,0,back2);
}
void back2dc(uint16_t back0, uint16_t back1)
{
  pwm.setPWM(12,0,back0);
  pwm.setPWM(13,0,back1);
}
// motor mode speed
void eco()
{
  if(ps2x.Analog(PSS_RY) == 0 && ps2x.Analog(PSS_LY) == 0)
  {
    forwarddc(0,1000);
    forward2dc(0,1000);
  }
  else if(ps2x.Analog(PSS_RY) == 255 && ps2x.Analog(PSS_LY) == 255)
  {
    backdc(1000,0);
    back2dc(1000,0);
  }
  else if(ps2x.Analog(PSS_RY) == 0 && ps2x.Analog(PSS_LY) == 255)
  {
    left2dc(0,1000);
    leftdc(1000,0);
  } 
  else if(ps2x.Analog(PSS_RY) == 255 && ps2x.Analog(PSS_LY) == 0)
  {
    rightdc(0,1000);
    right2dc(1000,0);
  }
  else if(ps2x.Analog(PSS_LY) == 0)
  {
    forwarddc(0,1000);
  }
  else if(ps2x.Analog(PSS_RY) == 0)
  {
    forward2dc(0,1000);
  }
  else if(ps2x.Analog(PSS_LY) == 255)
  {
    backdc(1000,0);
  }
  else if(ps2x.Analog(PSS_RY) == 255)
  {
    back2dc(1000,0);
  }
  else
  {
    forwarddc(0,0);
    forward2dc(0,0);
    backdc(0,0);
    back2dc(0,0);
  }
}
void sport()
{
  if(ps2x.Analog(PSS_RY) == 0 && ps2x.Analog(PSS_LY) == 0)
  {
    forwarddc(0,3000);
    forward2dc(0,3000);
  }
  else if(ps2x.Analog(PSS_RY) == 255 && ps2x.Analog(PSS_LY) == 255)
  {
    backdc(3000,0);
    back2dc(3000,0);
  }
  else if(ps2x.Analog(PSS_RY) == 0 && ps2x.Analog(PSS_LY) == 255)
  {
    left2dc(0,3000);
    leftdc(3000,0);
  } 
  else if(ps2x.Analog(PSS_RY) == 255 && ps2x.Analog(PSS_LY) == 0)
  {
    rightdc(0,3000);
    right2dc(3000,0);
  }
  else if(ps2x.Analog(PSS_LY) == 0)
  {
    forwarddc(0,3000);
  }
  else if(ps2x.Analog(PSS_RY) == 0)
  {
    forward2dc(0,3000);
  }
  else if(ps2x.Analog(PSS_LY) == 255)
  {
    backdc(3000,0);
  }
  else if(ps2x.Analog(PSS_RY) == 255)
  {
    back2dc(3000,0);
  }
  else
  {
    forwarddc(0,0);
    forward2dc(0,0);
    backdc(0,0);
    back2dc(0,0);
  }
}
// shooter and collector setup
void collector()
{
  pwm.setPWM(8,0,0);
  pwm.setPWM(9,0,3000);
}
void reverse_collector()
{
  pwm.setPWM(8,0,3000);
  pwm.setPWM(9,0,0);
}
void collector_stop()
{
  pwm.setPWM(8,0,0);
  pwm.setPWM(9,0,0);
}
void shooter()
{
  pwm.setPWM(14,0,0);
  pwm.setPWM(15,0,4000);
}
void shooter_stop()
{
  pwm.setPWM(14,0,0);
  pwm.setPWM(15,0,0);
}
// control setup
void ps2Control() 
{
  ps2x.read_gamepad(false, false);
// motor mode controller
  if(ps2x.ButtonPressed(PSB_TRIANGLE))
  {
    mode_drive =!mode_drive;
  }
  if(mode_drive)
  {
    eco();
  }
  else
  {
    sport();
  }
//control servo 360 clockwise and anticlockwise 
  if(ps2x.ButtonPressed(PSB_PAD_UP))
  {
    servo1++;
  }
  switch(servo1 % 3)
  {
    case (0):
    stop_servo(SERVO);
    stop_servo(SERVO2);
    break;
    case(1):
    servo_clockwise(SERVO);
    servo_clockwise(SERVO2);
    break;
    case(2):
    servo_anticlockwise(SERVO);
    servo_anticlockwise(SERVO2);
    break;
  }
// control servo 180 clockwise and anticlockwise
  if(ps2x.ButtonPressed(PSB_PAD_LEFT))
  {
    servo3++;
  }
  switch(servo3 % 3)
  {
    case (0):
    stop_servo(SERVO3);
    break;
    case(1):
    servo_clockwise(SERVO3);
    break;
    case(2):
    servo_anticlockwise(SERVO3);
    break;
  }
// control servo 180 (2) clockwise and anticlockwise
  if(ps2x.ButtonPressed(PSB_PAD_RIGHT))
  {
    servo4++;
  }
  switch(servo4 % 3)
  {
    case (0):
    stop_servo(SERVO4);
    break;
    case(1):
    servo_clockwise(SERVO4);
    break;
    case(2):
    servo_anticlockwise(SERVO4);
    break;
  }
//shooting motor
  if(ps2x.ButtonPressed(PSB_R1))
  {
    shooter_mode++;
  }
  switch(shooter_mode % 2)
  {
    case(0):
      shooter_stop();
      break;
    case(1):
      shooter();
      break;
  }
//ball collector motor
  if(ps2x.Button(PSB_L2))
  {
    reverse_collector();
  }
  if(ps2x.ButtonPressed(PSB_L1))
  {
    collector_mode++;
  }
  switch(collector_mode % 2)
  {
    case (0):
      collector_stop();
      break;
    case(1):
      collector();
  }
  delay(50);
}
void loop() {ps2Control();}
