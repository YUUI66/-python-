import numpy as np
from scipy.optimize import minimize
import serial
import time

# ------------------------- 正向运动学工具函数 -------------------------
def dh_matrix(theta, alpha, a, d):
    """生成改进DH参数对应的齐次变换矩阵"""
    ct = np.cos(theta)
    st = np.sin(theta)
    ca = np.cos(alpha)
    sa = np.sin(alpha)
    return np.array([
        [ct, -st, 0, a],
        [st * ca, ct * ca, -sa, -sa * d],
        [st * sa, ct * sa, ca, ca * d],
        [0, 0, 0, 1]
    ])

def fkine(dh_params, q):
    """计算正向运动学"""
    T = np.eye(4)
    for i in range(len(dh_params)):
        theta, alpha, a, d = dh_params[i]
        theta_val = theta + q[i]
        Ti = dh_matrix(theta_val, alpha, a, d)
        T = T @ Ti
    return T

# ------------------------- 逆运动学核心算法 -------------------------
def ikunc(dh_params, T_target, q_guess, base_offset, tool_offset, max_iter=100, tol=1e-6):
    """数值逆运动学优化求解"""
    reach = sum([abs(a) for _, _, a, _ in dh_params]) + sum([abs(d) for _, _, _, d in dh_params])
    omega = np.diag([1, 1, 1, 3 / reach])  # 权重矩阵

    def objective(q):
        T_current = base_offset @ fkine(dh_params, q) @ tool_offset
        error_matrix = np.linalg.inv(T_target) @ T_current - np.eye(4)
        weighted_error = error_matrix @ omega
        return np.sum(weighted_error ** 2)

    # 关节角度限制
    bounds = [
        (np.deg2rad(-90), np.deg2rad(90)),
        (np.deg2rad(0), np.deg2rad(180)),
        (np.deg2rad(-180), np.deg2rad(0)),
        (np.deg2rad(-90), np.deg2rad(90))
    ]

    result = minimize(objective, q_guess,
                     method='SLSQP',
                     bounds=bounds,
                     options={'maxiter': max_iter, 'ftol': tol})

    if not result.success:
        print(f"优化警告: {result.message}, 最终误差: {result.fun:.6f}")
    return result.x

# ------------------------- 串口通信函数 -------------------------
def wait_completion(ser, timeout=30):
    """等待下位机运动完成"""
    start = time.time()
    while time.time() - start < timeout:
        if ser.in_waiting > 0:
            response = ser.readline().decode().strip()
            print(f"[下位机响应] {response}")
            if "[MOVE_FINISH]" in response:
                return True
            elif "ERR" in response:
                print(f"错误: {response}")
                return False

# ------------------------- 主程序 -------------------------
if __name__ == "__main__":
    ser = None
    try:
        # 初始化串口连接
        ser = serial.Serial(port='COM13', baudrate=9600, timeout=2)
        print(f"成功连接串口: {ser.port}")
        ser.flushInput()

        # 输入目标点坐标
        print("请输入起始点和目标点的坐标（单位：米）")
        points = []

        # 输入起始点及生成抬笔点
        print("=== 起始点（笔筒底部）===")
        x1 = float(input("x1: "))
        y1 = float(input("y1: "))
        z1 = float(input("z1: "))
        points.append((x1, y1, z1))           # 原始点
        points.append((x1, y1, z1+0.08))    # 抬笔点

        # 输入目标点及生成放笔点
        print("\n=== 目标点（笔筒顶部）===")
        x3 = float(input("x3: "))
        y3 = float(input("y3: "))
        points.append((x3, y3, z1+0.09))    # 目标点
        points.append((x3, y3, z1+0.04))    # 放笔点

        # 询问障碍物输入
        obs_input = input("\n是否需要输入障碍物坐标？(y/n): ").lower().strip()
        if obs_input == 'y':
            print("\n=== 障碍物坐标输入 ===")
            obs_x = float(input("障碍物X坐标: "))
            obs_y = float(input("障碍物Y坐标: "))

            # 生成绕行路径点（保持抬笔高度）
            z_lift = points[1][2]  # 使用抬笔点的高度
            detour_points = [
                (obs_x, obs_y - 0.06, z_lift),  # 左侧绕行点
                (obs_x + 0.06, obs_y, z_lift),  # 前方绕行点
                (obs_x, obs_y + 0.06, z_lift)   # 右侧绕行点
            ]

            # 在抬笔点和目标点之间插入绕行点
            points.insert(2, detour_points[0])
            points.insert(3, detour_points[1])
            points.insert(4, detour_points[2])

        # 机械臂参数配置
        dh_params = [
            [0, 0, 0, 0],
            [0, np.pi/2, 0, 0],
            [np.pi - np.arctan(35.5/82.7), 0, 0.09, 0],
            [np.arctan(35.5/82.7), 0, 0.09, 0]
        ]
        base_offset = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0],
            [0, 0, 1, 0.08],
            [0, 0, 0, 1]
        ])
        tool_offset = np.array([
            [1, 0, 0, 0.011],
            [0, 1, 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])

        # 生成控制指令（改进：使用前一个解作为初始猜测）
        commands = []
        q_prev = np.zeros(4)  # 初始猜测设为全零
        for idx, (x, y, z) in enumerate(points):
            # 计算目标位姿
            theta_1 = np.arctan2(-y, -x)
            Tc4 = np.array([
                [-np.cos(theta_1), -np.sin(theta_1), 0, 0.011],
                [0, 0, -1, 0],
                [np.sin(theta_1), -np.cos(theta_1), 0, 0],
                [0, 0, 0, 1]
            ])
            Tcw = np.array([[1, 0, 0, x], [0, 1, 0, y], [0, 0, 1, z], [0, 0, 0, 1]])

            T40 = np.linalg.inv(base_offset) @ Tcw @ np.linalg.inv(Tc4)
            T5w = base_offset @ T40 @ tool_offset

            # 逆运动学求解（使用前一个解作为初始猜测）
            q_sol = ikunc(dh_params, T5w, q_prev, base_offset, tool_offset)
            q_prev = q_sol  # 保存当前解供下一个点使用

            # 转换为舵机角度
            converted = [
                np.degrees(q_sol[0]) + 95,    # 关节1
                np.degrees(q_sol[1]),          # 关节2
                np.degrees(q_sol[2]) + 180,    # 关节3
                -np.degrees(q_sol[3]) + 90     # 关节4
            ]

            # 角度校验
            for i, angle in enumerate(converted):
                if not (0 <= angle <= 180):
                    print(f"错误：点{idx+1} 关节{i+1}角度超限: {angle:.1f}°")
                    raise ValueError("角度参数错误")

            # 生成指令
            cmd = f"#{converted[0]:.0f},{converted[1]:.0f},{converted[2]:.0f},{converted[3]:.0f}!\n"
            commands.append(cmd)
            print(f"点{idx+1} 控制指令生成: {cmd.strip()}")

        # 分步发送指令
        for i, cmd in enumerate(commands):
            print(f"\n正在发送第{i+1}个点...")
            ser.write(cmd.encode())

            if wait_completion(ser):
                print(f"第{i+1}个点运动完成")
                # 在第一个点完成后添加夹爪操作停顿
                if i == 0:
                    print("请按下夹爪按钮，1秒后继续执行...")
                    time.sleep(1)
                # 在关键路径点添加稳定时间
                if i in [1, len(commands)-2]:  # 抬笔后和最终放笔前
                    print("等待1秒稳定时间...")
                    time.sleep(1)
            else:
                raise RuntimeError("运动执行失败")

        print("\n完整运动流程完成！笔已安全转移至目标笔筒。")

    except Exception as e:
        print(f"发生错误: {str(e)}")
    finally:
        if ser and ser.is_open:
            ser.close()
            print("串口连接已安全关闭")


