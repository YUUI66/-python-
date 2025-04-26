import serial
import time

def main():
    # 配置串口
    port = 'COM3'  # 替换为你的Arduino连接端口
    baud_rate = 9600  # 与Arduino代码中的波特率一致
    timeout = 1  # 设置超时时间

    try:
        # 打开串口
        ser = serial.Serial(port, baud_rate, timeout=timeout)
        print(f"连接到串口 {port}，波特率 {baud_rate}")

        # 等待Arduino初始化
        time.sleep(2)

        # 发送数据
        data_to_send = "Hello, Arduino!\n"  # 添加换行符表示结束
        print(f"发送数据: {data_to_send.strip()}")
        ser.write(data_to_send.encode())  # 将字符串编码为字节并发送

        # 接收回传数据
        response = ser.readline().decode().strip()  # 读取一行并解码为字符串
        print(f"接收到的回传数据: {response}")

    except serial.SerialException as e:
        print(f"串口连接错误: {e}")
    finally:
        # 关闭串口
        if ser.is_open:
            ser.close()
            print("串口已关闭")

if __name__ == "__main__":
    main()