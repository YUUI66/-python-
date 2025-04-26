#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#define MIN_PULSE_WIDTH       650
#define MAX_PULSE_WIDTH       2350
#define FREQUENCY             50

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// 舵机和电位器的定义
int potBase = A0;    // 底座舵机电位器连接到A0
int base = 0;        // 底座舵机连接到驱动板的0号通道
int potShoulder = A1;// 肩部舵机电位器连接到A1
int shoulder = 1;    // 肩部舵机连接到驱动板的1号通道
int potJoin3 = A2;   // joint3舵机电位器连接到A2
int join3 = 2;       // joint3舵机连接到驱动板的2号通道
int potJoint4 = A3;  // joint4舵机电位器连接到A3
int joint4 = 3;      // joint4舵机连接到驱动板的3号通道

void setup() 
{
  pwm.begin();                            // 初始化PWM舵机驱动器
  pwm.setPWMFreq(FREQUENCY);              // 设置PWM频率

  Serial.begin(9600);                     // 初始化串口通信
}

void moveMotor(int controlIn, int motorOut, int mapFromLow, int mapFromHigh, int mapToLow, int mapToHigh)
{
  int pulse_wide, pulse_width, potVal;

  potVal = analogRead(controlIn);         // 读取电位器的值

  // 将电位器值从指定范围映射到舵机角度范围
  int angle = map(potVal, mapFromLow, mapFromHigh, mapToLow, mapToHigh);

  // 将角度转换为脉冲宽度
  pulse_wide = map(angle, 0, 360, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
  pulse_width = int(float(pulse_wide) / 1000000 * FREQUENCY * 4096);   // 将脉宽转换为占空比值

  pwm.setPWM(motorOut, 0, pulse_width);   // 设置舵机的角度
}

void loop() 
{
  // 控制底座舵机
  moveMotor(potBase, base, 950, 100, 0, 360);

  // 控制肩部舵机
  moveMotor(potShoulder, shoulder, 823, 0, 0, 360);

  // 控制 join3 舵机
  moveMotor(potJoin3, join3, 1023, 200, 0, 400);

  // 控制 joint4 舵机
  moveMotor(potJoint4, joint4, 400, 1023, 0, 360);
}