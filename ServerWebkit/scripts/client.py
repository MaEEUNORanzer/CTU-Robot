# -- coding: utf-8 --
import socket
import time
import threading

def test_client(host='127.0.0.1', port=8888):
    """测试TCP客户端"""
    try:
        # 连接服务器
        client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        client_socket.connect((host, port))
        client_socket.settimeout(5.0)
        
        print(f"已连接到服务器 {host}:{port}")
        
        def receive_messages():
            """接收服务器消息的线程"""
            while True:
                try:
                    data = client_socket.recv(1024)
                    if data:
                        message = data.decode('utf-8').strip()
                        print(f"服务器: {message}")
                except socket.timeout:
                    continue
                except:
                    break
        
        # 启动接收线程
        recv_thread = threading.Thread(target=receive_messages, daemon=True)
        recv_thread.start()
        
        # 交互循环
        while True:
            print("\n可用命令:")
            print("  1. SCAN - 开始扫描")
            print("  2. STOP - 停止扫描")
            print("  3. STATUS - 查看状态")
            print("  4. EXIT - 退出")
            
            command = input("\n请输入命令: ").strip().upper()
            
            if command in ['SCAN', 'STOP', 'STATUS', 'EXIT']:
                client_socket.sendall(f"{command}\n".encode('utf-8'))
                
                if command == 'EXIT':
                    time.sleep(1)
                    break
            else:
                print("无效命令")
        
        client_socket.close()
        print("客户端已断开")
        
    except Exception as e:
        print(f"客户端错误: {e}")

if __name__ == "__main__":
    import argparse
    
    parser = argparse.ArgumentParser(description='二维码检测TCP客户端测试')
    parser.add_argument('--host', type=str, default='127.0.0.1', help='服务器主机地址 (默认: 127.0.0.1)')
    parser.add_argument('--port', type=int, default=8888, help='服务器端口 (默认: 8888)')
    
    args = parser.parse_args()
    
    test_client(args.host, args.port)

