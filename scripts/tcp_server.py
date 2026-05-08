import sys
# 导入电池基类
import syspy.battery_Can.canpass_base as cb
# 其他工具类,如定时器
import syspy.lib.misc_utility as mu
import syspy.lib.udp_debug as ud
import syspy.lib.char_utility as cu
# TCP服务器相关模块
import socket
import threading
import time

class testCanBattery(cb.canPassBase):

    def __init__(self):
        # 初始化基类,必须做
        super(testCanBattery, self).__init__()
        # 创建一个超时定时器
        self.connect_timeout_t = mu.Timer(2000)
        # 用来表示数据是否已经正确接收
        self.msg_ok = False
        self.tem = []
        self.battery_info = self.createBatteryMessage()
        # TCP服务器配置
        self.host = '192.168.5.73'
        self.port = 6666
        self.server_socket = None
        self.running = False
        self.clients = []
        self.client_lock = threading.Lock()

    def handleData(self, msg):
        self.clearTimeout()
        canframe = self.recCanframe(msg)
        tem = canframe.Data.hex()
        if canframe.ID == 0x112:
            voltage = round(int(tem[0:2] + tem[2:4], 16) * 0.1, 2)
            percentage = round(int(tem[8:10], 16) * 0.004, 2)
            temperature = round(int(tem[10:12], 16)-40, 2)
            current = round(cu.hexStr_to_int(tem[4:6] + tem[6:8], 16) * 0.1, 2)
            if current < 0:
                is_charging = False
            else:
                is_charging = True
            self.battery_info.charge_voltage = voltage
            self.battery_info.charge_current = current
            self.battery_info.percetage = percentage
            self.battery_info.temperature = temperature
            self.battery_info.is_charging = is_charging
            self.publish(self.battery_info)
            self.msg_ok = True

    def judgeMsgok(self):
        if self.msg_ok:
            # 清除超时错误,重置标志位
            self.msg_ok = False
            self.connect_timeout_t.reset()
        else:
            if self.connect_timeout_t.isTimeUp():
                self.setTimeout()

    def start_tcp_server(self):
        """
        启动TCP服务器
        """
        try:
            # 创建TCP socket
            self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            
            # 绑定地址和端口
            self.server_socket.bind((self.host, self.port))
            
            # 开始监听
            self.server_socket.listen(5)
            
            self.running = True
            print(f"[TCP Server] 服务器启动成功")
            print(f"[TCP Server] 监听地址: {self.host}:{self.port}")
            
            # 启动服务器主循环
            while self.running:
                try:
                    # 接受新客户端连接
                    client_socket, address = self.server_socket.accept()
                    print(f"[TCP Server] 客户端连接: {address}")
                    
                    # 发送hello文本
                    hello_message = "hello\n"
                    client_socket.sendall(hello_message.encode('utf-8'))
                    print(f"[TCP Server] 向客户端 {address} 发送: hello")
                    
                    # 立即关闭连接
                    client_socket.close()
                    print(f"[TCP Server] 客户端 {address} 断开连接")
                    
                except Exception as e:
                    if self.running:
                        print(f"[TCP Server] 接受连接异常: {e}")
                    
        except Exception as e:
            print(f"[TCP Server] 服务器启动失败: {e}")
            self.running = False
        finally:
            self.stop_server()

    def stop_server(self):
        """
        停止TCP服务器
        """
        self.running = False
        
        # 关闭服务器socket
        if self.server_socket:
            try:
                self.server_socket.close()
            except:
                pass
        print("[TCP Server] 服务器已停止")

    def loop(self):
        # 需要至少5s来等待底层初始化,否则将会覆盖操作
        mu.sleep_s(5)
        self.attachCanID(2, 1, 0x112, 0, 0, 0)
        self.battery_info = self.createBatteryMessage()
        
        # 启动TCP服务器线程
        server_thread = threading.Thread(target=self.start_tcp_server)
        server_thread.daemon = True
        server_thread.start()
        
        while True:
            self.judgeMsgok()
            mu.sleep_s(2)

if __name__ == '__main__':
    client = testCanBattery()
    client.loop()