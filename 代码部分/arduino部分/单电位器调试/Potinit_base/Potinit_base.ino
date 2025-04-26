#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#define MIN_PULSE_WIDTH       650
#define MAX_PULSE_WIDTH       2350
#define FREQUENCY             50

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

int potBase = A0;  // 底座舵机电位器连接到A0
int base = 0;      // 底座舵机连接到驱动板的0号通道

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
  Serial.print("Potentiometer Value: ");
  Serial.println(potVal);

  // 将电位器值从0到1023映射到舵机角度0到180度
  int angle = map(potVal, 1023, 250, 0, 180);
  Serial.print("Angle: ");
  Serial.println(angle);

  // 将角度转换为脉冲宽度
  pulse_wide = map(angle, 0, 180, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
  Serial.print("Pulse Width: ");
  Serial.println(pulse_wide);

  pulse_width = int(float(pulse_wide) / 1000000 * FREQUENCY * 4096);   // 将脉宽转换为占空比值
  Serial.print("Pulse Width (PCA9685): ");
  Serial.println(pulse_width);

  pwm.setPWM(motorOut, 0, pulse_width);   // 设置舵机的角度
}

void loop() 
{
  moveMotor(potBase, base);               // 控制底座舵机
}