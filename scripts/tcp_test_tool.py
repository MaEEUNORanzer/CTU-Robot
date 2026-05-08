# -- coding: utf-8 --
import socket
import threading
import time
from rbk import BasicModule, MoveStatus
from rbkSim import SimModule
import json

class Module:
    def __init__(self, r: SimModule, args):
        self.status = MoveStatus.NONE
        self.counter = 0
        self.running = False
        self.server_thread = None
        self.server_socket = None
        
        # 从args获取IP和端口，如果没有则使用默认值
        self.host = args.get('host', '192.168.5.73')
        self.port = args.get('port', 6666)
    
    def handle_client(self, client_socket, address):
        """处理客户端连接"""
        try:
            # 发送hello文本
            hello_message = "hello\n"
            client_socket.sendall(hello_message.encode('utf-8'))
            self.r.setNotice(f"向客户端 {address} 发送: hello")
            
        except Exception as e:
            self.r.setWarning(f"处理客户端 {address} 异常: {e}")
        finally:
            # 关闭连接
            try:
                client_socket.close()
            except:
                pass
            self.r.setNotice(f"客户端 {address} 断开连接")
    
    def tcp_server_worker(self):
        """TCP服务器工作线程"""
        try:
            # 创建TCP socket
            self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            
            # 绑定地址和端口
            self.server_socket.bind((self.host, self.port))
            
            # 开始监听
            self.server_socket.listen(5)
            
            self.running = True
            self.r.setNotice(f"TCP服务器启动成功，监听地址: {self.host}:{self.port}")
            
            # 设置超时以便能够检查运行状态
            self.server_socket.settimeout(1.0)
            
            # 服务器主循环
            while self.running:
                try:
                    # 接受新客户端连接
                    client_socket, address = self.server_socket.accept()
                    self.r.setNotice(f"客户端连接: {address}")
                    
                    # 创建新线程处理客户端
                    client_thread = threading.Thread(
                        target=self.handle_client,
                        args=(client_socket, address)
                    )
                    client_thread.daemon = True
                    client_thread.start()
                    
                except socket.timeout:
                    # 超时，继续循环
                    continue
                except Exception as e:
                    if self.running:
                        self.r.setWarning(f"接受连接异常: {e}")
                    
        except Exception as e:
            self.r.setWarning(f"TCP服务器启动失败: {e}")
            self.running = False
        finally:
            self.stop_server()
    
    def stop_server(self):
        """停止TCP服务器"""
        self.running = False
        if self.server_socket:
            try:
                self.server_socket.close()
            except:
                pass
        self.r.setNotice("TCP服务器已停止")

    def run(self, r: SimModule, args):   
        self.r = r
        self.status = MoveStatus.RUNNING
        self.counter = r.getCount()
        
        # 输出服务器启动信息
        server_info = {
            "host": self.host,
            "port": self.port,
            "status": "starting",
            "counter": self.counter
        }
        
        r.setInfo(json.dumps(server_info))
        r.logInfo(f"启动TCP服务器: {self.host}:{self.port}")
        
        # 启动TCP服务器线程
        self.server_thread = threading.Thread(target=self.tcp_server_worker)
        self.server_thread.daemon = True
        self.server_thread.start()
        
        # 保持主循环运行
        while self.status == MoveStatus.RUNNING:
            # 每秒检查一次服务器状态
            if not self.running and self.server_thread is not None:
                r.setWarning("TCP服务器已停止")
                break
                
            time.sleep(1)
            self.counter = r.getCount()
            
            # 可以在这里添加其他监控逻辑
            if self.counter % 10 == 0:  # 每10个计数周期输出一次状态
                status_info = {
                    "status": "running",
                    "clients_served": "统计信息可以根据需要添加",
                    "counter": self.counter
                }
                r.setNotice(f"TCP服务器运行中: {json.dumps(status_info)}")
        
        return self.status

    def cancel(self, r):
        self.counter = r.getCount()
        r.setNotice(f"取消任务 -- {self.counter}")
        self.stop_server()
        self.status = MoveStatus.NONE
        return self.status

    def suspend(self, r):
        self.counter = r.getCount()
        r.setNotice(f"暂停任务 -- {self.counter}")
        self.stop_server()
        self.status = MoveStatus.SUSPENDED
        return self.status