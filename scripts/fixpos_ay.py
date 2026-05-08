# -- coding: utf-8 --
from rbk import BasicModule, MoveStatus
from rbkSim import SimModule
import json
import socket
import time
import threading
from queue import Queue

class Module:
    def __init__(self, r: SimModule, args):
        self.status = MoveStatus.NONE
        self.counter = 0
        self.client_socket = None
        self.aligned = False
        # 固定连接参数
        self.host = '192.168.192.61'
        self.port = 8888
        # 线程控制
        self._worker_thread = None
        self._stop_event = threading.Event()
        self._result_queue = Queue()
        
    def _network_worker(self, r):
        """网络通信工作线程"""
        try:
            r.setNotice(f"工作线程: 连接到服务器 {self.host}:{self.port}")
            self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.client_socket.connect((self.host, self.port))
            self.client_socket.settimeout(0.5)  # 设置较短的超时，便于检查停止信号
            
            r.setNotice("工作线程: 发送SCAN命令...")
            self.client_socket.sendall(b"SCAN\n")
            
            # 循环等待对齐信号
            while not self._stop_event.is_set():
                try:
                    data = self.client_socket.recv(1024)
                    if data:
                        message = data.decode('utf-8').strip()
                        r.logInfo(f"服务器响应: {message}")
                        
                        if "COMPLETE" in message.upper():
                            r.setNotice("工作线程: 收到COMPLETE消息")
                            self._result_queue.put(("SUCCESS", "对齐完成"))
                            self.aligned = True
                            break
                except socket.timeout:
                    # 超时是正常的，继续检查停止信号
                    continue
                except Exception as e:
                    r.setWarning(f"工作线程接收数据出错: {e}")
                    self._result_queue.put(("ERROR", str(e)))
                    break
            
        except Exception as e:
            r.setWarning(f"工作线程连接失败: {e}")
            self._result_queue.put(("ERROR", str(e)))
        finally:
            # 清理连接
            if self.client_socket:
                if self.aligned and not self._stop_event.is_set():
                    try:
                        self.client_socket.sendall(b"EXIT\n")
                        time.sleep(0.5)
                    except:
                        pass
                self.client_socket.close()
                self.client_socket = None
    
    def _get_robot_info(self, r):
        """获取机器人当前信息"""
        return {
            "jack": r.jack(),
            "fork": r.fork(),
            "controller": r.controller(),
            "di": r.Di(),
            "do": r.Do(),
            "counter": r.getCount(),
            "odo": r.odo(),
            "loc": r.loc(),
            "sound": r.sound()
        }

    def run(self, r: SimModule, args):
        """主执行逻辑 - 非阻塞版本"""
        self.status = MoveStatus.RUNNING
        self.counter = r.getCount()
        self._stop_event.clear()
        self.aligned = False
        
        # 清空结果队列
        while not self._result_queue.empty():
            self._result_queue.get()
        
        # 获取机器人信息
        robot_info = self._get_robot_info(r)
        r.logInfo(f"开始二维码检测任务，机器人信息: {json.dumps(robot_info)}")
        r.setNotice(f"启动二维码检测工作线程")
        
        # 启动工作线程
        self._worker_thread = threading.Thread(
            target=self._network_worker,
            args=(r,),
            daemon=True
        )
        self._worker_thread.start()
        
        return self.status
    
    def update(self, r: SimModule):
        """更新方法，在主线程中定期调用"""
        if self.status != MoveStatus.RUNNING:
            return self.status
        
        # 检查线程是否存活
        if self._worker_thread and not self._worker_thread.is_alive():
            r.setWarning("工作线程异常退出")
            self.status = MoveStatus.NONE
            return self.status
        
        # 检查结果
        if not self._result_queue.empty():
            result_type, message = self._result_queue.get()
            
            if result_type == "SUCCESS":
                r.setNotice(f"二维码检测完成: {message}")
                
                # 输出对齐完成后的机器人信息
                aligned_info = self._get_robot_info(r)
                r.setNotice(f"对齐完成，当前机器人信息: {json.dumps(aligned_info)}")
                
                self.status = MoveStatus.FINISHED
            else:
                r.setWarning(f"二维码检测失败: {message}")
                self.status = MoveStatus.NONE
        
        return self.status

    def cancel(self, r):
        """取消任务"""
        self.counter = r.getCount()
        r.setNotice(f"取消二维码检测任务 -- 计数器: {self.counter}")
        
        # 通知工作线程停止
        self._stop_event.set()
        
        # 等待线程结束
        if self._worker_thread and self._worker_thread.is_alive():
            self._worker_thread.join(timeout=2.0)
        
        self.status = MoveStatus.NONE
        return self.status

    def suspend(self, r):
        """暂停任务"""
        self.counter = r.getCount()
        r.setNotice(f"暂停二维码检测任务 -- 计数器: {self.counter}")
        
        # 通知工作线程停止
        self._stop_event.set()
        
        # 等待线程结束
        if self._worker_thread and self._worker_thread.is_alive():
            self._worker_thread.join(timeout=2.0)
        
        self.status = MoveStatus.SUSPENDED
        return self.status