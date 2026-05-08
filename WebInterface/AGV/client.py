#!/usr/bin/env python3
"""
测试TCP客户端脚本
"""

import json
import socket
import time

def test_client():
    """测试客户端"""
    host = "127.0.0.1"
    port = 8888
    
    # 测试邮箱
    test_email = "test@example.com"
    
    # 定义测试请求
    test_requests = [
        {
            "action": "GET_USER_DATA",
            "email": test_email
        },
        {
            "action": "GET_SYSTEM_STATUS",
            "email": test_email
        },
        {
            "action": "GET_AGV_LIST",
            "email": test_email
        },
        {
            "action": "UPDATE_AGV",
            "email": test_email,
            "agv_id": "AGV-01",
            "updates": {"battery": 80, "speed": 1.5}
        },
        {
            "action": "ADD_TASK",
            "email": test_email,
            "task_data": {
                "agv_id": "AGV-04",
                "from": "仓库B",
                "to": "货架A1",
                "status": "pending",
                "progress": 0
            }
        }
    ]
    
    for i, request in enumerate(test_requests, 1):
        print(f"\n🔧 测试请求 {i}: {request['action']}")
        
        try:
            # 创建socket连接
            client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            client_socket.settimeout(5)
            client_socket.connect((host, port))
            
            # 发送请求
            request_str = json.dumps(request, ensure_ascii=False)
            client_socket.send(request_str.encode('utf-8'))
            print(f"📨 发送: {request_str}")
            
            # 接收响应
            response_data = client_socket.recv(4096)
            response = json.loads(response_data.decode('utf-8'))
            print(f"📤 收到: {json.dumps(response, indent=2, ensure_ascii=False)}")
            
            # 关闭连接
            client_socket.close()
            
            # 短暂等待
            time.sleep(0.5)
            
        except Exception as e:
            print(f"❌ 请求失败: {e}")

def test_save_data():
    """测试保存数据"""
    host = "127.0.0.1"
    port = 8888
    email = "user@test.com"
    
    # 构建测试数据
    test_data = {
        "system": {
            "total_agvs": 10,
            "running_agvs": 5,
            "idle_agvs": 3,
            "warning_agvs": 2
        },
        "agvs": [
            {"id": "AGV-01", "status": "running", "battery": 90},
            {"id": "AGV-02", "status": "idle", "battery": 100}
        ],
        "tasks": [
            {"id": "Task-001", "status": "running", "progress": 50}
        ],
        "email": email
    }
    
    request = {
        "action": "SAVE_USER_DATA",
        "email": email,
        "data": test_data
    }
    
    print(f"\n💾 测试保存数据: {email}")
    
    try:
        client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        client_socket.connect((host, port))
        
        request_str = json.dumps(request, ensure_ascii=False)
        client_socket.send(request_str.encode('utf-8'))
        
        response_data = client_socket.recv(4096)
        response = json.loads(response_data.decode('utf-8'))
        print(f"📤 保存结果: {json.dumps(response, indent=2, ensure_ascii=False)}")
        
        # 测试读取
        print(f"\n📖 测试读取数据: {email}")
        read_request = {
            "action": "GET_USER_DATA",
            "email": email
        }
        
        client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        client_socket.connect((host, port))
        
        read_request_str = json.dumps(read_request, ensure_ascii=False)
        client_socket.send(read_request_str.encode('utf-8'))
        
        read_response_data = client_socket.recv(4096)
        read_response = json.loads(read_response_data.decode('utf-8'))
        print(f"📥 读取结果: 数据长度 = {len(str(read_response))} 字符")
        
    except Exception as e:
        print(f"❌ 测试失败: {e}")

if __name__ == "__main__":
    print("=" * 50)
    print("🧪 AGV TCP服务端测试客户端")
    print("=" * 50)
    
    # 先测试保存和读取
    test_save_data()
    
    # 再测试其他API
    # test_client()