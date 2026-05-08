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
        # 服务器消息处理
        self._message_buffer = ""
        # 状态跟踪
        self.task_started = False
        self.task_completed = False
        # 完成标志
        self._has_finished = False
        # 连接尝试次数
        self._connection_attempts = 0
        self._max_attempts = 3
        
    def _network_worker(self, r):
        """网络通信工作线程"""
        try:
            r.setNotice(f"工作线程: 连接到服务器 {self.host}:{self.port}")
            
            # 建立连接
            self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.client_socket.settimeout(10)  # 连接超时
            self.client_socket.connect((self.host, self.port))
            self.client_socket.settimeout(0.5)  # 接收超时
            
            r.setNotice("工作线程: 等待服务器就绪消息...")
            
            # 接收服务器欢迎消息
            try:
                data = self.client_socket.recv(1024)
                if data:
                    message = data.decode('utf-8').strip()
                    r.logInfo(f"服务器欢迎消息: {message}")
                    
                    if "READY" in message.upper():
                        r.setNotice("服务器已就绪，发送START指令...")
                        
                        # 发送START指令
                        self.client_socket.sendall(b"START")
                        self.task_started = True
                        
                        # 循环接收服务器响应
                        while not self._stop_event.is_set():
                            try:
                                data = self.client_socket.recv(1024)
                                if data:
                                    # 处理接收到的数据
                                    messages = self._process_received_data(data.decode('utf-8'))
                                    
                                    for message in messages:
                                        message = message.strip()
                                        if message:
                                            r.logInfo(f"服务器响应: {message}")
                                            
                                            # 根据消息类型处理
                                            if message.startswith("START:"):
                                                r.setNotice("服务器已开始执行二维码检测任务")
                                            elif message.startswith("DETECTED:"):
                                                r.logInfo(f"检测到二维码: {message}")
                                            elif message.startswith("ADJUST:"):
                                                r.setNotice(f"服务器报告: {message}")
                                            elif message.startswith("NO_QR:"):
                                                r.logInfo("未检测到二维码")
                                            elif message.startswith("COMPLETE"):
                                                r.setNotice("任务完成！")
                                                self.task_completed = True
                                                # 任务完成，标记为已完成
                                                self._has_finished = True
                                                self._result_queue.put(("COMPLETE", message))
                                                break
                                            elif message.startswith("ERROR:"):
                                                r.setWarning(f"服务器报告错误: {message}")
                                                self._result_queue.put(("ERROR", message))
                                                break
                                            elif message.startswith("STOPPED:"):
                                                r.setNotice(f"任务被停止: {message}")
                                                self._result_queue.put(("STOPPED", message))
                                                break
                                else:
                                    # 连接被服务器关闭
                                    r.setWarning("服务器关闭了连接")
                                    if not self.task_completed:
                                        self._result_queue.put(("ERROR", "服务器连接断开，未收到COMPLETE"))
                                    break
                                    
                            except socket.timeout:
                                continue
                            except ConnectionResetError:
                                r.setWarning("连接被服务器重置")
                                if not self.task_completed:
                                    self._result_queue.put(("ERROR", "连接被重置，未收到COMPLETE"))
                                break
                            except Exception as e:
                                r.setWarning(f"接收数据时出错: {e}")
                                if not self.task_completed:
                                    self._result_queue.put(("ERROR", str(e)))
                                break
                    else:
                        r.setWarning(f"服务器返回非预期消息: {message}")
                        self._result_queue.put(("ERROR", f"非预期消息: {message}"))
            except socket.timeout:
                r.setWarning("等待服务器就绪消息超时")
                self._result_queue.put(("ERROR", "连接服务器超时"))
            except Exception as e:
                r.setWarning(f"接收欢迎消息时出错: {e}")
                self._result_queue.put(("ERROR", str(e)))
            
        except ConnectionRefusedError:
            r.setWarning(f"无法连接到服务器 {self.host}:{self.port}，请确保服务器已启动")
            self._result_queue.put(("ERROR", "连接被拒绝"))
        except socket.timeout:
            r.setWarning("连接服务器超时")
            self._result_queue.put(("ERROR", "连接超时"))
        except Exception as e:
            r.setWarning(f"连接服务器失败: {e}")
            self._result_queue.put(("ERROR", str(e)))
        finally:
            # 清理连接
            if self.client_socket:
                self.client_socket.close()
                self.client_socket = None
                r.logInfo("工作线程: 已关闭网络连接")
    
    def _process_received_data(self, data):
        """处理接收到的数据，处理可能的分行情况"""
        self._message_buffer += data
        
        # 按换行符分割消息
        lines = self._message_buffer.split('\n')
        
        # 最后一行可能是未完成的消息，保留在缓冲区
        if lines[-1] != '':
            self._message_buffer = lines[-1]
            complete_messages = lines[:-1]
        else:
            self._message_buffer = ""
            complete_messages = lines
        
        return complete_messages
    
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
        """主执行逻辑"""
        # 如果已经完成任务，直接返回FINISHED
        if self._has_finished:
            r.setWarning("二维码检测任务已完成")
            self.status = MoveStatus.FINISHED
            return self.status
            
        # 如果正在运行，不再启动新线程
        if self._worker_thread and self._worker_thread.is_alive():
            r.setNotice("任务已在执行中")
            return self.status
            
        # 重置状态
        self.status = MoveStatus.RUNNING
        self.counter = r.getCount()
        self._stop_event.clear()
        self.aligned = False
        self.task_started = False
        self.task_completed = False
        self._message_buffer = ""
        
        # 清空结果队列
        while not self._result_queue.empty():
            self._result_queue.get()
        
        # 获取机器人信息
        robot_info = self._get_robot_info(r)
        r.logInfo(f"开始二维码检测任务，机器人信息: {json.dumps(robot_info, indent=2)}")
        r.setNotice(f"启动二维码检测工作线程")
        r.setNotice(f"连接服务器: {self.host}:{self.port}")
        
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
        
        # 检查结果队列
        if not self._result_queue.empty():
            result_type, message = self._result_queue.get()
            
            if result_type == "COMPLETE":
                r.setNotice(f"二维码检测任务成功完成！")
                r.setNotice(f"服务器消息: {message}")
                
                # 输出对齐完成后的机器人信息
                aligned_info = self._get_robot_info(r)
                r.logInfo(f"任务完成，当前机器人信息: {json.dumps(aligned_info, indent=2)}")
                
                # 设置FINISHED状态，这会停止run函数被调用
                self.status = MoveStatus.FINISHED
                self.aligned = True
                self._has_finished = True
                
            elif result_type == "STOPPED":
                r.setNotice(f"二维码检测任务被停止")
                r.setNotice(f"服务器消息: {message}")
                self.status = MoveStatus.NONE
                
            else:  # ERROR
                r.setWarning(f"二维码检测任务失败")
                r.setWarning(f"错误信息: {message}")
                self.status = MoveStatus.NONE
        
        # 检查线程是否存活
        if self._worker_thread and not self._worker_thread.is_alive():
            if self.status == MoveStatus.RUNNING and not self.task_completed:
                r.setWarning("工作线程异常退出，但未收到结果")
                self.status = MoveStatus.NONE
        
        return self.status

    def cancel(self, r):
        """取消任务"""
        self.counter = r.getCount()
        r.setNotice(f"取消二维码检测任务 -- 计数器: {self.counter}")
        
        # 通知工作线程停止
        self._stop_event.set()
        
        # 如果连接还在，尝试关闭
        if self.client_socket:
            try:
                self.client_socket.close()
            except:
                pass
        
        # 等待线程结束
        if self._worker_thread and self._worker_thread.is_alive():
            r.setNotice("等待工作线程退出...")
            self._worker_thread.join(timeout=2.0)
        
        if self.task_started and not self.task_completed:
            r.setWarning("任务被用户取消")
        
        # 重置完成标志，允许重新执行
        self._has_finished = False
        self.status = MoveStatus.NONE
        return self.status

    def suspend(self, r):
        """暂停任务"""
        self.counter = r.getCount()
        r.setNotice(f"暂停二维码检测任务 -- 计数器: {self.counter}")
        
        # 注意：新协议不支持暂停，只能取消
        r.setWarning("新协议不支持暂停操作，将转为取消任务")
        
        # 调用取消逻辑
        return self.cancel(r)