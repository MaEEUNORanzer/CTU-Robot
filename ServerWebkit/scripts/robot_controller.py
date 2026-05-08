#!/usr/bin/env python3
"""
机器人控制模块 - 简化版
支持：速度、方向（正负）、距离
"""

import socket
import json
import struct
import time

def build_request_header(data_length, api_type):
    """构建请求报文头部"""
    header = bytearray(16)
    header[0] = 0x5A
    header[1] = 0x01
    header[2:4] = struct.pack(">H", 1)
    header[4:8] = struct.pack(">I", data_length)
    header[8:10] = struct.pack(">H", api_type)
    header[10:16] = b'\x00\x00\x00\x00\x00\x00'
    return header

def send_robot_command(ip, port, api_type, request_data, timeout=5.0):
    """发送命令到机器人并获取响应"""
    json_data = json.dumps(request_data)
    json_bytes = json_data.encode('utf-8')
    header = build_request_header(len(json_bytes), api_type)
    
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.settimeout(timeout)
        s.connect((ip, port))
        s.sendall(header + json_bytes)
        
        response_header = s.recv(16)
        if len(response_header) < 16:
            raise Exception("响应头长度不足")
            
        data_length = struct.unpack(">I", response_header[4:8])[0]
        response_data = s.recv(data_length)
        response_json = response_data.decode('utf-8')
        return json.loads(response_json)

def move_robot(speed=0.5, direction=0, distance=0.2, 
               ip="192.168.192.5", port=19206):
    """
    控制机器人移动 - 简化版本
    
    参数:
    - speed: 速度大小 (m/s)，正值，默认0.5
    - direction: 方向，0=前，180=后，90=左，-90=右，默认0
    - distance: 移动距离 (米)，默认0.2
    - ip: 机器人IP地址，默认 192.168.192.5
    - port: 机器人控制端口，默认 19206
    
    返回: 包含响应信息的字典
    """
    import math
    
    # 将方向角度转换为速度分量
    angle_rad = math.radians(direction)
    vx = speed * math.cos(angle_rad)
    vy = speed * math.sin(angle_rad)
    
    # 构造请求数据
    request_data = {
        "dist": distance,
        "vx": vx,
        "vy": vy,
        "mode": 1
    }
    
    try:
        response = send_robot_command(ip, port, 3055, request_data, timeout=5.0)
        return response
    except Exception as e:
        return {"error": str(e), "ret_code": -1}

# 简洁的调用接口
def move_forward(distance=0.5, speed=0.5, ip="192.168.192.5", port=19206):
    """向前移动"""
    return move_robot(speed=speed, direction=0, distance=distance, ip=ip, port=port)

def move_backward(distance=0.5, speed=0.5, ip="192.168.192.5", port=19206):
    """向后移动"""
    return move_robot(speed=speed, direction=180, distance=distance, ip=ip, port=port)

def move_left(distance=0.5, speed=0.5, ip="192.168.192.5", port=19206):
    """向左移动"""
    return move_robot(speed=speed, direction=90, distance=distance, ip=ip, port=port)

def move_right(distance=0.5, speed=0.5, ip="192.168.192.5", port=19206):
    """向右移动"""
    return move_robot(speed=speed, direction=-90, distance=distance, ip=ip, port=port)
