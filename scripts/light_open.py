# -- coding: utf-8 --
from rbk import BasicModule, MoveStatus
from rbkSim import SimModule
from dowrite import DOWriter
from diread import DIReader
import json
import time
import threading

class Module:
    def __init__(self, r: SimModule, args):
        self.status = MoveStatus.NONE
        self.counter = 0
        
        # 启动非阻塞的初始化操作
        self._init_thread = None
        self._init_complete = False
        self._init_failed = False

        self.task_complete = False
        
        # 异步执行初始化操作
        self._start_init_async(r)
    
    def _start_init_async(self, r):
        """启动异步初始化操作"""
        def _init_task():
            try:
                # 执行DO操作

                DOWriter.write_once(11, True)

                self.task_complete = True
                
                # 标记初始化完成
                self._init_complete = True
                
            except Exception as e:
                r.setWarning(f"初始化失败: {e}")
                self._init_failed = True
        
        # 创建并启动初始化线程
        self._init_thread = threading.Thread(
            target=_init_task,
            daemon=True
        )
        self._init_thread.start()
        
        r.setNotice("开始异步初始化...")
    
    def _check_init_status(self, r):
        """检查初始化状态"""
        if self._init_failed:
            r.setNotice("初始化失败")
            return False
        elif self._init_complete:
            r.setNotice("初始化完成")
            return True
        elif self._init_thread and not self._init_thread.is_alive():
            # 线程意外结束
            r.setNotice("初始化线程异常结束")
            self._init_failed = True
            return False
        
        return None  # 仍在进行中

    def run(self, r: SimModule, args):  
        if self.task_complete is not True:
            self.status = MoveStatus.RUNNING
        else:
            self.status = MoveStatus.FINISHED
        return self.status
    
    def update(self, r: SimModule):
        """更新方法，在主线程中定期调用"""
        if self.status == MoveStatus.RUNNING:
            # 检查初始化状态
            init_status = self._check_init_status(r)
            
            if init_status is True:
                # 初始化成功完成
                r.setNotice("所有初始化完成，任务执行中")
                # 可以在这里添加其他逻辑
                
            elif init_status is False:
                # 初始化失败
                r.setWarning("初始化失败，停止任务")
                self.status = MoveStatus.NONE
        
        return self.status

    def cancel(self, r):
        self.counter = r.getCount()
        r.setNotice(f"cancel task -- {self.counter}")
        
        # 可以在这里添加取消逻辑，比如等待初始化线程结束
        if self._init_thread and self._init_thread.is_alive():
            # 注意：这里无法直接停止线程，但可以通过标记来让线程自然结束
            pass
            
        self.status = MoveStatus.NONE
        return self.status

    def suspend(self, r):
        self.counter = r.getCount()
        r.setNotice(f"suspend task -- {self.counter}")
        self.status = MoveStatus.SUSPENDED
        return self.status