#include <Adafruit_PWMServoDriver.h>
#include <Wire.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// ================= 运动控制优化 =================
const int STEPS = 150;         // 增加插值步数（原80 → 150）
const int STEP_DELAY = 30;     // 增加每步延时ms（原15 → 30）
bool isMoving = false;
int currentStep = 0;
float increments[4];
int startAngles[4];            // 起始角度记录
int targetAngles[4];           // 目标角度
unsigned long lastStepTime = 0;

// ================= 硬件配置 =================
struct ServoConfig {
  uint8_t channel;
  int minPulseUs;
  int maxPulseUs;
  int initAngle;
};

const ServoConfig servos[] = {
  // 关节  通道  最小脉宽  最大脉宽  初始角度
  {0, 500, 2500, 95},    // 关节1（注意检查硬件方向）
  {1, 500, 2500, 0},     // 关节2 
  {2, 500, 2500, 180},   // 关节3
  {3, 500, 2500, 90},    // 关节4
  {4, 650, 2350, 100}     // 夹爪
};

int currentAngles[4] = {95, 0, 180, 90}; // 当前角度跟踪
const int GRIP_BUTTON_PIN = 13;

// ================= 初始化 =================
void setup() {
  Serial.begin(9600);
  while(!Serial); // 等待串口连接
  pwm.begin();
  pwm.setPWMFreq(50);
  pinMode(GRIP_BUTTON_PIN, INPUT_PULLUP);
  
  // 初始化舵机位置
  for(int i=0; i<5; i++) {
    setServoAngle(i, servos[i].initAngle);
    delay(300);
  }
  Serial.println("[SYS_READY]"); // 系统就绪标志
}

// ================= 主循环 =================
void loop() {
  handleSerial();
  handleMovement();
  handleGripper();
}

// ================= 串口处理 =================
void handleSerial() {
  static char buffer[64];
  static int index = 0;

  while(Serial.available()) {
    char c = Serial.read();
    
    // 指令格式：#角度1,角度2,角度3,角度4!
    if(c == '#') {
      index = 0;
      buffer[index++] = c;
    } else if(c == '!') {
      buffer[index++] = c;
      buffer[index] = '\0';
      processCommand(buffer);
      index = 0;
    } else if(index < 63) {
      buffer[index++] = c;
    }
  }
}

// ================= 运动控制优化 =================
void processCommand(char* cmd) {
  if(isMoving || cmd[0]!='#' || cmd[strlen(cmd)-1]!='!') return;

  // 提取有效数据
  char* data = cmd+1;
  data[strlen(data)-1] = '\0';

  // 解析角度
  int angles[4];
  if(parseData(data, angles)) {
    // 记录起始角度
    memcpy(startAngles, currentAngles, sizeof(startAngles));
    memcpy(targetAngles, angles, sizeof(targetAngles));
    
    // 计算插值
    for(int i=0; i<4; i++) {
      increments[i] = (targetAngles[i] - startAngles[i])/(float)STEPS;
    }
    
    // 启动运动
    currentStep = 0;
    isMoving = true;
    lastStepTime = millis();
    Serial.println("[MOVE_START]");
  }
}

// ================= 运动执行优化 =================
void handleMovement() {
  if(!isMoving) return;

  if(currentStep <= STEPS) {
    if(millis()-lastStepTime >= STEP_DELAY) {
      // 使用浮点数计算保证精度
      for(int i=0; i<4; i++) {
        float exactAngle = startAngles[i] + increments[i] * currentStep;
        currentAngles[i] = round(exactAngle);
        setServoAngle(i, currentAngles[i]);
      }
      
      // 发送运动进度
      Serial.print("[PROGRESS]");
      Serial.println(map(currentStep, 0, STEPS, 0, 100));

      currentStep++;
      lastStepTime = millis();
    }
  } else {
    // 最终位置校准
    for(int i=0; i<4; i++) {
      setServoAngle(i, targetAngles[i]);
    }
    isMoving = false;
    Serial.println("[MOVE_FINISH]");
  }
}

// ================= 辅助函数 =================
bool parseData(char* data, int* angles) {
  char* token = strtok(data, ",");
  for(int i=0; i<4; i++) {
    if(!token) return false;
    angles[i] = constrain(atoi(token),0,180);
    token = strtok(NULL, ",");
  }
  return true;
}

void setServoAngle(uint8_t index, int angle) {
  angle = constrain(angle,0,180);
  const ServoConfig &cfg = servos[index];
  
  // 脉宽映射（带保护）
  long pulseWidth = map(angle, 0, 180, cfg.minPulseUs, cfg.maxPulseUs);
  pulseWidth = constrain(pulseWidth, cfg.minPulseUs, cfg.maxPulseUs);
  
  // 12-bit PWM计算
  int pwmValue = pulseWidth * 4096 / 20000; 
  pwm.setPWM(cfg.channel, 0, pwmValue);

  // 调试输出（关节1-4）
  if(index < 4) {
    Serial.print("J");
    Serial.print(index+1);
    Serial.print(":");
    Serial.print(angle);
    Serial.print("° ");
  }
}
// ================= 夹爪控制修正版 =================
void handleGripper() {
  static unsigned long lastTriggerTime = 0;
  static bool isOpen = false;
  const int debounceDelay = 50;       // 消抖延时(ms)
  const int triggerCooldown = 500;    // 操作冷却时间(ms)

  int currentState = digitalRead(GRIP_BUTTON_PIN);
  
  // 冷却期检测
  if (millis() - lastTriggerTime < triggerCooldown) return;

  // 简易消抖检测
  static int stableCounter = 0;
  const int stableThreshold = 3; // 连续3次检测
  
  if (currentState == LOW) {
    stableCounter++;
    if (stableCounter >= stableThreshold) {
      isOpen = !isOpen;
      setServoAngle(4, isOpen ? 170 : 100);
      Serial.println(isOpen ? "[GRIP_OPEN]" : "[GRIP_CLOSE]");
      lastTriggerTime = millis();
      stableCounter = 0; // 重置计数器
    }
  } else {
    stableCounter = 0;   // 重置计数器
  }
}
