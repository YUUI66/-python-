#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// 舵机配置结构体
struct ServoConfig {
  uint8_t channel;     // PWM通道（0-15）
  int initAngle;       // 初始角度
  int minPulseUs;      // 最小脉宽（微秒）
  int maxPulseUs;      // 最大脉宽（微秒）
};

// 配置数组对应舵机1-4 + 夹爪（通道4）
ServoConfig servos[] = {
  {0, 95, 500, 2500},   // MG996R 舵机1
  {1, 0, 500, 2500},    // 20kg   舵机2
  {2, 180, 500, 2500},  // MG996R 舵机3
  {3, 90, 500, 2500},   // MG996R 舵机4
  {4, 120, 650, 2350}    // 夹爪（通道4，初始关闭）
};

const int gripButtonPin = 13;  // 夹爪按钮引脚

void setup() {
  Serial.begin(9600);
  pwm.begin();
  pwm.setPWMFreq(50);
  
  // 初始化按钮引脚
  pinMode(gripButtonPin, INPUT_PULLUP);

  // 初始化所有舵机（包含夹爪）
  for (int i = 0; i < 5; i++) {
    setServoAngle(i, servos[i].initAngle);
    delay(300);
  }
  Serial.println("Ready! Waiting for angles from Python...");
}

void loop() {
  // 处理串口指令
  if (Serial.available() > 0) {
    handleSerialInput();
  }
  
  // 实时检测夹爪按钮
  controlGripper();
}

// 修改后的夹爪控制函数（切换模式）
void controlGripper() {
  static int lastButtonState = HIGH;  // 上一次按钮状态
  static int stableButtonState = HIGH; // 稳定后的按钮状态
  static unsigned long lastDebounceTime = 0;
  const int debounceDelay = 50;       // 消抖时间（毫秒）
  static bool isOpen = false;          // 夹爪状态跟踪

  int currentState = digitalRead(gripButtonPin);

   // 检测状态变化时重置消抖计时器
  if (currentState != lastButtonState) {
    lastDebounceTime = millis();
  }

  // 经过消抖时间后处理稳定状态
  if ((millis() - lastDebounceTime) > debounceDelay) {
    // 仅当按钮状态实际改变时更新
    if (currentState != stableButtonState) {
      stableButtonState = currentState;

      // 检测下降沿（按下动作）
      if (stableButtonState == LOW) {
        isOpen = !isOpen; // 切换状态
        setServoAngle(4, isOpen ? 170 : 110);
      }
    }
  }
  lastButtonState = currentState;
}

// 设置舵机角度
void setServoAngle(uint8_t servoIndex, int angle) {
  ServoConfig cfg = servos[servoIndex];
  angle = constrain(angle, 0, 180);

  // 计算脉宽（保持原始参数）
  long pulseUs = map(angle, 0, 180, cfg.minPulseUs, cfg.maxPulseUs);
  int pwmValue = pulseUs * 4096 / 20000;  // 20000μs = 20ms周期
  
  pwm.setPWM(cfg.channel, 0, pwmValue);

  // 调试信息（仅显示夹爪状态）
  if (servoIndex == 4) {
    Serial.print("Gripper[CH4] ");
    Serial.print(angle);
    Serial.println((angle == 180) ? "° Open" : "° Closed");
  }
}

// 处理串口输入（保持不变）
void handleSerialInput() {
  static char buffer[64];
  static int index = 0;

  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\n') {
      buffer[index] = '\0';
      parseAngles(buffer);
      index = 0;
    } else {
      buffer[index++] = c;
    }
  }
}

// 解析角度（仅处理前4个舵机）
void parseAngles(const char* data) {
  char* token;
  char* dataCopy = strdup(data);
  int angles[4] = {0};

  token = strtok(dataCopy, ",");
  for (int i = 0; i < 4 && token != NULL; i++) {
    angles[i] = (int) atof(token);
    token = strtok(NULL, ",");
  }

  free(dataCopy);

  // 设置前4个舵机
  for (int i = 0; i < 4; i++) {
    setServoAngle(i, angles[i]);
     delay(1000);  // 添加 0.5 秒的延迟
  }
  Serial.println("Arm movement completed.");
}
