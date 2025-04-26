#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#define MIN_PULSE_WIDTH       650
#define MAX_PULSE_WIDTH       2350
#define FREQUENCY             50

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

int potJoin3 = A2;  // join3舵机电位器连接到A1
int join3 = 2;      // join3舵机连接到驱动板的2号通道

void setup() 
{
  pwm.begin();                            // 初始化PWM舵机驱动器
  pwm.setPWMFreq(FREQUENCY);              // 设置PWM频率

  Serial.begin(9600);                     // 初始化串口通信
}

void moveMotor(int controlIn, int motorOut)
{
  int pulse_wide, pulse_width, potVal;

  potVal = analogRead(controlIn);         // 读取电位器的值

  // 将电位器值从0到1023映射到舵机角度0到360度
  int angle = map(potVal, 1023, 200, 0, 400);

  // 将角度转换为脉冲宽度
  pulse_wide = map(angle, 0, 360, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
  pulse_width = int(float(pulse_wide) / 1000000 * FREQUENCY * 4096);   // 将脉宽转换为占空比值

  pwm.setPWM(motorOut, 0, pulse_width);   // 设置舵机的角度
}

void loop() 
{
  moveMotor(potJoin3, join3);       // 控制join3舵机
}