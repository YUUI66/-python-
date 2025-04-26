#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// ================= 运动控制参数 =================
const int STEPS = 150;         // 插值步数
const int STEP_DELAY = 30;     // 每步延时(ms)
bool isMoving = false;         // 运动状态标志
int currentStep = 0;           // 当前步数
float increments[4];           // 角度增量数组
int startAngles[4];            // 起始角度
int targetAngles[4];           // 目标角度
unsigned long lastStepTime = 0;// 最后一步时间

// ================= 舵机配置 =================
struct ServoConfig {
  uint8_t channel;     // PWM通道
  int initAngle;       // 初始角度
  int minPulseUs;      // 最小脉宽
  int maxPulseUs;      // 最大脉宽
};

ServoConfig servos[] = {
  {0, 95, 500, 2500},   // 舵机1
  {1, 0, 500, 2500},    // 舵机2
  {2, 180, 500, 2500},  // 舵机3
  {3, 90, 500, 2500},   // 舵机4
  {4, 110, 650, 2350}    // 夹爪
};

int currentAngles[4] = {95, 0, 180, 90}; // 当前角度跟踪
const int gripButtonPin = 13;            // 夹爪按钮引脚

// 函数声明
void setServoAngle(uint8_t index, int angle, bool silent = false);

void setup() {
  Serial.begin(9600);
  pwm.begin();
  pwm.setPWMFreq(50);
  pinMode(gripButtonPin, INPUT_PULLUP);

  // 初始化舵机位置
  for(int i=0; i<5; i++){
    setServoAngle(i, servos[i].initAngle);
    delay(300);
  }
  Serial.println("Ready! Waiting for commands...");
}

void loop() {
  handleSerialInput();
  handleMovement();
  controlGripper();
}

// ================= 平滑运动控制 =================
void handleMovement() {
  if(!isMoving) return;

  if(currentStep <= STEPS) {
    if(millis()-lastStepTime >= STEP_DELAY){
      for(int i=0; i<4; i++){
        float exactAngle = startAngles[i] + increments[i]*currentStep;
        currentAngles[i] = round(exactAngle);
        setServoAngle(i, currentAngles[i], true);
      }
      currentStep++;
      lastStepTime = millis();
    }
  } else {
    // 最终位置校准
    for(int i=0; i<4; i++){
      setServoAngle(i, targetAngles[i], true);
    }
    isMoving = false;
    Serial.println("[MOVE_COMPLETE]");
  }
}

// ================= 舵机控制 =================
void setServoAngle(uint8_t index, int angle, bool silent) {
  angle = constrain(angle, 0, 180);
  ServoConfig cfg = servos[index];
  
  // 更新当前角度（仅关节1-4）
  if(index < 4) currentAngles[index] = angle;
  
  long pulseUs = map(angle, 0, 180, cfg.minPulseUs, cfg.maxPulseUs);
  int pwmValue = pulseUs * 4096 / 20000;
  pwm.setPWM(cfg.channel, 0, pwmValue);

  // 调试输出
  if(!silent && index == 4){
    Serial.print("Gripper[CH4] ");
    Serial.print(angle);
    Serial.println((angle > 120) ? "° Open" : "° Closed");
  }
}

// ================= 串口指令处理 =================
void handleSerialInput() {
  static char buffer[64];
  static int index = 0;

  while(Serial.available() && !isMoving){
    char c = Serial.read();
    
    if(c == '\n'){
      buffer[index] = '\0';
      parseAngles(buffer);
      index = 0;
    }else if(index < 63){
      buffer[index++] = c;
    }
  }
}

void parseAngles(const char* data) {
  char* token;
  char* dataCopy = strdup(data);
  int angles[4] = {0};

  token = strtok(dataCopy, ",");
  for(int i=0; i<4 && token!=NULL; i++){
    angles[i] = atoi(token);
    angles[i] = constrain(angles[i],0,180);
    token = strtok(NULL, ",");
  }

  // 初始化运动参数
  memcpy(startAngles, currentAngles, sizeof(startAngles));
  memcpy(targetAngles, angles, sizeof(targetAngles));
  
  for(int i=0; i<4; i++){
    increments[i] = (targetAngles[i]-startAngles[i])/(float)STEPS;
  }

  currentStep = 0;
  isMoving = true;
  lastStepTime = millis();
  free(dataCopy);
  Serial.println("[MOVING] Arm is moving...");
}

// ================= 夹爪控制 =================
void controlGripper() {
  static int lastState = HIGH;
  static unsigned long lastTime = 0;
  const int debounce = 50;
  static bool isOpen = false;

  int currentState = digitalRead(gripButtonPin);
  
  if(millis()-lastTime > debounce){
    if(currentState == LOW && lastState == HIGH){
      isOpen = !isOpen;
      setServoAngle(4, isOpen ? 170 : 110);
      Serial.println(isOpen ? "[GRIP_OPEN]" : "[GRIP_CLOSE]");
    }
    lastState = currentState;
  }
  
  if(currentState != lastState){
    lastTime = millis();
  }
}
