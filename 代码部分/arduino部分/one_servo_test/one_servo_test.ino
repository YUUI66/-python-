#include <Servo.h>

Servo myServo;  // 创建舵机对象
int angle = 0;  // 当前角度

void setup() {
  Serial.begin(9600);  // 初始化串口通信，波特率与 Python 中一致
  myServo.attach(9);   // 假设舵机连接在数字引脚 9 上
}

void loop() {
  if (Serial.available() > 0) {  // 检查是否有数据可用
    angle = Serial.parseInt();  // 读取角度值
    if (angle >= 0 && angle <= 180) {  // 检查角度是否在有效范围内
      myServo.write(angle);  // 设置舵机角度
      Serial.println(angle);  // 可选：回传角度值，用于调试
    }
  }
}