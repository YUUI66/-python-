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
        theta_val = theta + q[i]  # 原offset + 关节变量
        Ti = dh_matrix(theta_val, alpha, a, d)
        T = T @ Ti
    return T


# ------------------------- 逆运动学核心算法 -------------------------
def ikunc(dh_params, T_target, q_guess, base_offset, tool_offset, max_iter=100, tol=1e-6):
    """
    数值逆运动学优化求解
    :param dh_params: DH参数列表
    :param T_target: 目标位姿(4x4)
    :param q_guess: 初始关节角度猜测
    :param base_offset: 基坐标系偏移
    :param tool_offset: 工具坐标系偏移
    :return: 优化后的关节角度
    """
    # 计算机械臂总长度(用于omega矩阵)
    reach = sum([abs(a) for _, _, a, _ in dh_params]) + sum([abs(d) for _, _, _, d in dh_params])

    # 构造权重矩阵omega (与MATLAB实现一致)
    omega = np.diag([1, 1, 1, 3 / reach])

    def objective(q):
        """优化目标函数"""
        T_current = base_offset @ fkine(dh_params, q) @ tool_offset
        error_matrix = np.linalg.inv(T_target) @ T_current - np.eye(4)
        weighted_error = error_matrix @ omega
        return np.sum(weighted_error ** 2)  # 计算平方和

    # 角度限制（单位：度）
    bounds = [
        (np.deg2rad(-90), np.deg2rad(90)),  # 关节1：-90到90度
        (np.deg2rad(0), np.deg2rad(180)),  # 关节2：0到180度
        (np.deg2rad(-180), np.deg2rad(0)),  # 关节3：-180到0度
        (np.deg2rad(-90), np.deg2rad(90))  # 关节4：-90到90度
    ]
    result = minimize(objective, q_guess,
                      method='SLSQP',
                      bounds=bounds,
                      options={'maxiter': max_iter, 'ftol': tol})

    if not result.success:
        print(f"优化警告: {result.message}, 最终误差: {result.fun:.6f}")

    return result.x


# ------------------------- 主程序 -------------------------
if __name__ == "__main__":
    # 初始化串口
    ser = None
    try:
        ser = serial.Serial(
            port='COM8',  # 串口号（Windows格式）
            baudrate=9600,  # 波特率
            parity=serial.PARITY_NONE,  # 校验位
            stopbits=serial.STOPBITS_ONE,  # 停止位
            bytesize=serial.EIGHTBITS,  # 数据位
            timeout=1  # 添加读取超时
        )
        print(f"串口已连接: {ser.port}")
    except Exception as e:
        print(f"串口连接失败: {str(e)}")
        exit(1)

    # --- 第一部分：计算T4矩阵（与原MATLAB代码一致） ---
    x = float(input("请输入 x 的值: "))
    y = float(input("请输入 y 的值: "))
    z = float(input("请输入 z 的值: "))
    l, h = 0.011, 0.08

    theta_1 = np.arctan2(-y, -x)

    # 手动构造变换矩阵
    Tc4 = np.array([
        [-np.cos(theta_1), -np.sin(theta_1), 0, l],
        [0, 0, -1, 0],
        [np.sin(theta_1), -np.cos(theta_1), 0, 0],
        [0, 0, 0, 1]
    ])

    Tcw = np.array([  # 平移矩阵
        [1, 0, 0, x],
        [0, 1, 0, y],
        [0, 0, 1, z],
        [0, 0, 0, 1]
    ])

    T0w = np.array([  # 基坐标系偏移
        [1, 0, 0, 0],
        [0, 1, 0, 0],
        [0, 0, 1, h],
        [0, 0, 0, 1]
    ])

    T40 = (np.linalg.inv(T0w) @ Tcw @ np.linalg.inv(Tc4))

    T54 = np.array([  # 工具坐标系偏移
        [1, 0, 0, l],
        [0, 1, 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ])

    T5w = T0w @ T40 @ T54  # 目标位姿

    # --- 第二部分：机器人建模与逆解计算 ---
    dh_params = [
        [0,                              0,          0,     0],  # 关节1 (theta_offset=0)
        [0,                              np.pi / 2,  0,     0],  # 关节2
        [np.pi - np.arctan(35.5 / 82.7), 0,          0.09,  0],  # 关节3
        [np.arctan(35.5 / 82.7),         0,          0.09,  0]  # 关节4
    ]

    q_guess = np.array([0, 0, 0, 0])
    q_sol = ikunc(dh_params, T5w, q_guess, T0w, T54)

    # 验证结果
    T_total = T0w @ fkine(dh_params, q_sol) @ T54
    print("目标位姿 T4:\n", np.round(T5w, 6))
    print("\n实际计算的位姿 T_total:\n", np.round(T_total, 6))
    print("\n关节角度 (角度):\n", np.round(np.degrees(q_sol), 2))



    # 转换为度数并构造传输字符串
    q_sol_deg = np.degrees(q_sol)
    # 对每个关节角度进行变换
    theta1 = q_sol_deg[0] + 95 # 关节1：加上90度
    theta2 = q_sol_deg[1]  # 关节2：保持不变
    theta3 = q_sol_deg[2] + 180  # 关节3：加上180度
    theta4 = -q_sol_deg[3] + 90  # 关节4：取负后再加上90度

    # 构造传输字符串
    data_str = f"{theta1:.2f},{theta2:.2f},{theta3:.2f},{theta4:.2f}\n"
    # 发送数据
    try:
        ser.write(data_str.encode())  # 将字符串编码为字节后发送
        print(f"已发送数据: {data_str}")
    except Exception as e:
        print(f"串口发送失败: {str(e)}")
    finally:
        if ser and ser.is_open:
            ser.close()
            print("串口已关闭")