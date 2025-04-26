// Arduino双向通信示例代码
void setup() {
    Serial.begin(9600);  // 设置串口波特率
    Serial.println("Arduino双向通信已启动！");
}

void loop() {
    if (Serial.available() > 0) {  // 检查是否有数据从串口接收
        String receivedData = Serial.readStringUntil('\n');  // 读取一行数据
        receivedData.trim();  // 去除多余的空格或换行符

        // 打印接收到的数据
        Serial.print("接收到的数据: ");
        Serial.println(receivedData);

        // 处理数据（这里简单地回传接收到的数据）
        Serial.print("回传确认: ");
        Serial.println(receivedData);
    }
}