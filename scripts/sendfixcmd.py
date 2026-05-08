# -- coding: utf-8 --
from rbk import BasicModule, MoveStatus
from rbkSim import SimModule
import json
import socket
import time

class Module:
    def __init__(self, r: SimModule, args):
        self.status = MoveStatus.NONE
        self.counter = 0
        self._has_finished = False
        
    def run(self, r: SimModule, args):
        """主执行逻辑 - 直接发送START然后结束"""
        if self._has_finished:
            r.setWarning("START指令已发送，任务已完成")
            self.status = MoveStatus.FINISHED
            return self.status
            
        self.status = MoveStatus.RUNNING
        self.counter = r.getCount()
        
        host = '192.168.192.61'
        port = 8888
        
        r.setNotice(f"🚀 连接到服务器 {host}:{port}")
        r.setNotice(f"📤 发送START指令...")
        
        try:
            # 建立连接
            client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            client_socket.settimeout(3)  # 3秒超时
            
            # 连接并发送
            client_socket.connect((host, port))
            client_socket.sendall(b"START")
            client_socket.close()
            
            r.setNotice(f"✅ START指令发送成功！")
            r.setNotice(f"✅ 客户端任务完成")
            r.setNotice(f"✅ 服务器会持续调整直到校准完成")
            r.setNotice(f"✅ 校准完成后会调用callback_task.py")
            
            self.status = MoveStatus.FINISHED
            self._has_finished = True
            
        except ConnectionRefusedError:
            r.setWarning(f"❌ 无法连接到服务器，请确保服务器已启动")
            self.status = MoveStatus.NONE
        except socket.timeout:
            r.setWarning(f"❌ 连接超时")
            self.status = MoveStatus.NONE
        except Exception as e:
            r.setWarning(f"❌ 连接失败: {e}")
            self.status = MoveStatus.NONE
        
        return self.status
    
    def update(self, r: SimModule):
        """更新方法"""
        return self.status  # 什么都不做，直接返回当前状态

    def cancel(self, r):
        """取消任务"""
        self.counter = r.getCount()
        r.setNotice(f"取消二维码检测任务 -- 计数器: {self.counter}")
        self._has_finished = False
        self.status = MoveStatus.NONE
        return self.status

    def suspend(self, r):
        """暂停任务"""
        return self.cancel(r)