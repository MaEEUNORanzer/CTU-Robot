import socket
import json
import struct

# 机器人 IP 地址和端口（假设端口号，具体端口请查阅协议文档）
ROBOT_IP = "192.168.192.5"
PORT = 19206  # 控制 API 端口（假设是控制任务链的端口）

# 报文头构建函数
def build_request_header(data_length, api_type):
    """
    构建请求报文头部
    :param data_length: 数据区的长度
    :param api_type: API 类型编号
    :return: 返回请求报文头部字节
    """
    header = bytearray(16)
    
    # 同步头 0x5A
    header[0] = 0x5A
    
    # 协议版本 0x01
    header[1] = 0x01
    
    # 序号 (示例使用 1)
    header[2:4] = struct.pack(">H", 1)
    
    # 数据总长度 (包括头部的 16 字节)
    header[4:8] = struct.pack(">I", data_length)
    
    # 报文类型 (API 类型编号)
    header[8:10] = struct.pack(">H", api_type)
    
    # 保留字段 0x00 填充
    header[10:16] = b'\x00\x00\x00\x00\x00\x00'
    
    return header

# 发送请求的函数
def send_request(api_type, request_data):
    """
    发送请求并返回响应
    :param api_type: API 类型编号
    :param request_data: 请求的 JSON 数据
    :return: 响应数据
    """
    # 序列化 JSON 数据
    json_data = json.dumps(request_data)
    json_bytes = json_data.encode('utf-8')  # 转为字节
    
    # 构建请求报文头
    header = build_request_header(len(json_bytes), api_type)

    # 创建并连接套接字
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.connect((ROBOT_IP, PORT))  # 连接到机器人控制端口
        s.sendall(header + json_bytes)  # 发送报文（头部 + 数据区）

        # 接收响应
        response_header = s.recv(16)  # 读取报文头部
        data_length = struct.unpack(">I", response_header[4:8])[0]  # 获取数据总长度
        response_data = s.recv(data_length)  # 读取响应数据区

        return response_data.decode('utf-8')  # 返回响应数据的 JSON 字符串

# 执行预存任务链 (API 类型 3106)
def execute_task_chain():
    """
    向机器人发送执行预存任务链的请求
    :return: 返回响应数据
    """
    # 构造执行任务链请求的 JSON 数据
    request_data = {
        "name": "task0"  # 任务链名称
    }
    
    # 发送执行预存任务链请求
    response_json = send_request(3106, request_data)  # 3106 为执行任务链的 API 类型编号
    print("Task Execution Response: ", response_json)
    return response_json

if __name__ == "__main__":
    # 第一步：发送执行预存任务链请求
    execute_task_chain()