import serial
import time

# 配置串口参数
port = 'COM7'  # 请根据实际情况修改串口号
baud_rate = 9600  # 与 Arduino 中设置的波特率一致

# 初始化串口
try:
    ser = serial.Serial(port, baud_rate, timeout=1)
    print(f"成功连接到串口 {port}")
except serial.SerialException as e:
    print(f"串口连接失败: {e}")
    exit()

# 发送角度值给 Arduino
def send_angle(angle):
    if 0 <= angle <= 180:  # 舵机角度范围为 0 到 180 度
        ser.write(str(angle).encode())  # 将角度值转换为字符串并发送
        print(f"发送角度: {angle} 度")
    else:
        print("角度超出范围，请输入 0 到 180 之间的值。")

# 主程序
if __name__ == "__main__":
    while True:
        try:
            angle = int(input("请输入舵机角度 (0-180): "))
            send_angle(angle)
        except ValueError:
            print("请输入有效的整数角度！")
        time.sleep(0.1)  # 简单的延时，避免过快发送数据