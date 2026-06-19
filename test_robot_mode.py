import socket
import json
import time

# VR设备的IP地址和新的监听端口
VR_IP_ADDRESS = "192.168.0.78"  # <-- 在这里替换成您VR设备的IP地址
VR_ROBOT_MODE_PORT = 13580  # <-- 这是我们在RobotModeTcpListener中定义的新端口


def send_robot_mode(mode_value):
    """
    通过TCP将机器人模式信息发送到VR端
    """
    # 1. 创建要发送的JSON数据
    payload = {
        "functionName": "robotMode",
        "value": mode_value
    }
    # 将字典转换为JSON格式的字符串
    json_message = json.dumps(payload)

    # 将字符串编码为字节流以便发送
    message_bytes = json_message.encode('utf-8')

    # 2. 创建TCP socket并连接
    # 使用 'with' 语句可以确保socket在使用后被正确关闭
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
            # 设置一个超时，避免在无法连接时无限期等待
            sock.settimeout(5.0)

            # 连接到VR设备上的服务
            print(f"Connecting to {VR_IP_ADDRESS}:{VR_ROBOT_MODE_PORT}...")
            sock.connect((VR_IP_ADDRESS, VR_ROBOT_MODE_PORT))

            # 3. 发送数据
            sock.sendall(message_bytes)
            print(f"Successfully sent robot mode: '{mode_value}'")

    except ConnectionRefusedError:
        print(f"Connection to {VR_IP_ADDRESS}:{VR_ROBOT_MODE_PORT} was refused. "
              "Is the VR app running and is RobotModeTcpListener active?")
    except socket.timeout:
        print(f"Connection to {VR_IP_ADDRESS}:{VR_ROBOT_MODE_PORT} timed out. "
              "Check the IP address and network connectivity.")
    except Exception as e:
        print(f"An error occurred: {e}")


if __name__ == "__main__":
    # 现在不再需要等待VR端先建立连接，因为VR端现在是服务端
    modes_to_send = ["Auto-Navigating", "Manual Control", "Charging"]

    for mode in modes_to_send:
        send_robot_mode(mode)
        # 等待2秒再发送下一个，以便在VR中观察变化
        time.sleep(2)