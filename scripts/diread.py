# -- coding: utf-8 --
"""
DI状态读取模块
用于读取Modbus TCP模块的DI通道状态

作者: 马硕
版本: 2.0.1
优化版本（无dataclasses依赖）
"""

import json
import time
import socket
import struct
import threading
from typing import Tuple, Optional, List, Callable, Dict, Any
from enum import Enum
from rbk import MoveStatus, BasicModule, ParamServer
from rbkSim import SimModule
from syspy.robot import ModuleTool, Motor, MotorType, Robot, GoodsManger


class DIStatus:
    """DI通道状态类（替代dataclass）"""
    
    def __init__(self, channel: int, state: bool, timestamp: float, error: str = ""):
        """
        初始化DI状态
        
        Args:
            channel: 通道编号
            state: 状态 (True=高电平, False=低电平)
            timestamp: 时间戳
            error: 错误信息
        """
        self.channel = channel
        self.state = state
        self.timestamp = timestamp
        self.error = error
    
    def __str__(self) -> str:
        """字符串表示"""
        if self.error:
            return f"DI{self.channel:02d}: 错误 - {self.error}"
        level = "高电平" if self.state else "低电平"
        return f"DI{self.channel:02d}: {level}"
    
    def __repr__(self) -> str:
        """repr表示"""
        return f"DIStatus(channel={self.channel}, state={self.state}, " \
               f"timestamp={self.timestamp}, error={self.error})"
    
    @property
    def is_high(self) -> bool:
        """是否为高电平"""
        return self.state
    
    @property
    def is_low(self) -> bool:
        """是否为低电平"""
        return not self.state


class ModbusException(Enum):
    """Modbus异常代码枚举"""
    ILLEGAL_FUNCTION = 0x01
    ILLEGAL_DATA_ADDRESS = 0x02
    ILLEGAL_DATA_VALUE = 0x03
    SLAVE_DEVICE_FAILURE = 0x04
    ACKNOWLEDGE = 0x05
    SLAVE_DEVICE_BUSY = 0x06
    MEMORY_PARITY_ERROR = 0x08
    GATEWAY_PATH_UNAVAILABLE = 0x0A
    GATEWAY_TARGET_DEVICE_FAILED_TO_RESPOND = 0x0B


class ModbusTCPClient:
    """Modbus TCP客户端"""
    
    MODBUS_PORT = 502
    DEFAULT_TIMEOUT = 3.0
    MAX_RETRIES = 3
    RETRY_DELAY = 0.5
    
    def __init__(self, host: str, timeout: float = DEFAULT_TIMEOUT):
        """
        初始化Modbus TCP客户端
        
        Args:
            host: 模块IP地址
            timeout: 连接超时时间
        """
        self.host = host
        self.timeout = timeout
        self._transaction_id = 0
        self._lock = threading.RLock()
        
    def _next_transaction_id(self) -> int:
        """获取下一个事务ID"""
        with self._lock:
            self._transaction_id = (self._transaction_id + 1) & 0xFFFF
            return self._transaction_id
    
    def _create_connection(self) -> Optional[socket.socket]:
        """创建Socket连接"""
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
            sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            sock.settimeout(self.timeout)
            sock.connect((self.host, self.MODBUS_PORT))
            return sock
        except Exception as e:
            print(f"[ERROR] 创建连接失败: {e}")
            return None
    
    def _close_connection(self, sock: socket.socket) -> None:
        """关闭连接"""
        try:
            sock.shutdown(socket.SHUT_RDWR)
        except:
            pass
        finally:
            try:
                sock.close()
            except:
                pass
    
    def _send_receive(self, sock: socket.socket, request: bytes) -> Optional[bytes]:
        """发送请求并接收响应"""
        try:
            sock.sendall(request)
            # 读取响应头
            header = sock.recv(7)
            if len(header) < 7:
                return None
            
            # 解析响应长度
            _, _, length = struct.unpack('>HHH', header[:6])
            
            # 读取剩余数据
            remaining = length - 1  # 减去单元标识符长度
            if remaining > 0:
                data = sock.recv(remaining)
                return header + data
            return header
            
        except socket.timeout:
            print(f"[ERROR] 通信超时 ({self.timeout}秒)")
            return None
        except Exception as e:
            print(f"[ERROR] 通信错误: {e}")
            return None
    
    def read_discrete_inputs(self, start_address: int, count: int, 
                           unit_id: int = 1) -> Tuple[Optional[List[bool]], str]:
        """
        读取离散输入
        
        Args:
            start_address: 起始地址
            count: 输入数量
            unit_id: 单元标识符
            
        Returns:
            (状态列表, 错误信息)
        """
        if not (1 <= count <= 2000):
            return None, f"输入数量无效: {count} (应在1-2000之间)"
        
        for attempt in range(self.MAX_RETRIES):
            sock = self._create_connection()
            if not sock:
                if attempt < self.MAX_RETRIES - 1:
                    time.sleep(self.RETRY_DELAY)
                    continue
                return None, "连接失败"
            
            try:
                # 构建请求
                transaction_id = self._next_transaction_id()
                header = struct.pack('>HHHB', 
                                   transaction_id,  # 事务标识符
                                   0x0000,          # 协议标识符
                                   0x0006,          # 长度
                                   unit_id)         # 单元标识符
                
                function_code = 0x02
                request_data = struct.pack('>BHH', function_code, start_address, count)
                request = header + request_data
                
                # 发送并接收
                response = self._send_receive(sock, request)
                if not response:
                    if attempt < self.MAX_RETRIES - 1:
                        time.sleep(self.RETRY_DELAY)
                        continue
                    return None, "通信失败"
                
                # 解析响应
                result, error = self._parse_response(response, function_code, count)
                if result is not None:
                    return result, ""
                elif attempt < self.MAX_RETRIES - 1:
                    time.sleep(self.RETRY_DELAY)
                    continue
                else:
                    return None, error
                    
            finally:
                self._close_connection(sock)
        
        return None, "重试次数超限"
    
    def _parse_response(self, response: bytes, expected_function: int, 
                       input_count: int) -> Tuple[Optional[List[bool]], str]:
        """解析响应"""
        if len(response) < 9:
            return None, f"响应数据太短: {len(response)}字节"
        
        # 解析响应头
        resp_header = struct.unpack('>HHHB', response[:7])
        _, _, resp_length, resp_unit_id = resp_header
        
        # 检查功能码
        resp_function_code = response[7]
        if resp_function_code & 0x80:  # 异常响应
            if len(response) >= 9:
                exception_code = response[8]
                exception_name = self._get_exception_name(exception_code)
                return None, f"Modbus异常: {exception_name} (代码: 0x{exception_code:02X})"
            return None, "异常响应不完整"
        
        if resp_function_code != expected_function:
            return None, f"功能码不匹配: 期望0x{expected_function:02X}，收到0x{resp_function_code:02X}"
        
        # 解析响应数据
        byte_count = response[8]
        if len(response) < 9 + byte_count:
            return None, f"响应数据长度不足"
        
        # 提取位数据
        bit_data = response[9:9+byte_count]
        
        # 转换为布尔列表
        states = []
        for i in range(input_count):
            byte_index = i // 8
            bit_position = i % 8
            
            if byte_index >= len(bit_data):
                states.append(False)
            else:
                state = (bit_data[byte_index] >> bit_position) & 0x01 == 0x01
                states.append(state)
        
        return states, ""
    
    def _get_exception_name(self, code: int) -> str:
        """获取异常名称"""
        try:
            return ModbusException(code).name.replace('_', ' ').title()
        except ValueError:
            return f"未知异常 (0x{code:02X})"


class DIReaderService:
    """DI读取服务"""
    
    def __init__(self, module_ip: str = "192.168.192.30", timeout: float = 3.0):
        """
        初始化DI读取服务
        
        Args:
            module_ip: Modbus模块IP地址
            timeout: 超时时间
        """
        self.module_ip = module_ip
        self.timeout = timeout
        self._client = ModbusTCPClient(module_ip, timeout)
        self._monitoring = False
        self._monitor_thread = None
        self._monitor_callbacks = []
        
    def read_channel(self, channel: int) -> DIStatus:
        """
        读取指定DI通道
        
        Args:
            channel: DI通道编号 (0-15)
            
        Returns:
            DIStatus对象
        """
        if not (0 <= channel <= 15):
            return DIStatus(channel, False, time.time(), 
                           f"通道编号无效: {channel} (应在0-15之间)")
        
        start_time = time.time()
        states, error = self._client.read_discrete_inputs(0, 16)
        
        if error:
            return DIStatus(channel, False, time.time(), error)
        
        return DIStatus(channel, states[channel], time.time())
    
    def read_all_channels(self) -> Tuple[List[DIStatus], str]:
        """
        读取所有DI通道
        
        Returns:
            (状态列表, 错误信息)
        """
        start_time = time.time()
        states, error = self._client.read_discrete_inputs(0, 16)
        
        if error:
            return [], error
        
        results = []
        for i, state in enumerate(states):
            results.append(DIStatus(i, state, time.time()))
        
        return results, ""
    
    def start_monitoring(self, channel: int, interval: float = 1.0, 
                        callback: Optional[Callable] = None) -> None:
        """开始监控指定通道"""
        if not (0 <= channel <= 15):
            raise ValueError(f"通道编号无效: {channel}")
        
        if callback:
            self._monitor_callbacks.append(callback)
        
        self._monitoring = True
        self._monitor_thread = threading.Thread(
            target=self._monitor_loop,
            args=(channel, interval),
            name=f"DI_Monitor_{channel}"
        )
        self._monitor_thread.daemon = True
        self._monitor_thread.start()
    
    def stop_monitoring(self) -> None:
        """停止监控"""
        self._monitoring = False
        if self._monitor_thread and self._monitor_thread.is_alive():
            self._monitor_thread.join(timeout=2.0)
    
    def _monitor_loop(self, channel: int, interval: float) -> None:
        """监控循环"""
        last_state = None
        
        while self._monitoring:
            status = self.read_channel(channel)
            
            if status.error:
                print(f"[WARN] 监控通道{channel}失败: {status.error}")
            else:
                # 状态变化检测
                if last_state is not None and last_state != status.state:
                    for callback in self._monitor_callbacks:
                        try:
                            callback(channel, status.state, last_state, status.timestamp)
                        except Exception as e:
                            print(f"[ERROR] 回调函数执行失败: {e}")
                
                last_state = status.state
            
            # 等待下一次读取
            time.sleep(interval)
    
    def close(self) -> None:
        """关闭资源"""
        self.stop_monitoring()


class DIReaderManager:
    """DI读取管理器（简化单例模式）"""
    
    _instance = None
    
    def __new__(cls):
        if cls._instance is None:
            cls._instance = object.__new__(cls)
            cls._instance._services = {}
        return cls._instance
    
    def get_service(self, module_ip: str, timeout: float = 3.0) -> DIReaderService:
        """获取或创建DI读取服务"""
        key = f"{module_ip}:{timeout}"
        if key not in self._services:
            self._services[key] = DIReaderService(module_ip, timeout)
        return self._services[key]
    
    def close_all(self) -> None:
        """关闭所有服务"""
        for key, service in list(self._services.items()):
            try:
                service.close()
            except Exception as e:
                print(f"[ERROR] 关闭服务失败: {e}")
            del self._services[key]


"""
####BEGIN DEFAULT ARGS####
{
    "module_ip": {
        "value": "192.168.192.30",
        "tips": "CK-TP5163模块IP地址",
        "unit": "",
        "type": "string"
    },
    "di_channel": {
        "value": 5,
        "tips": "要读取的DI通道编号 (0-15)",
        "unit": "",
        "type": "int"
    },
    "polling_interval": {
        "value": 0.5,
        "tips": "轮询间隔时间",
        "unit": "秒",
        "type": "float"
    },
    "timeout_seconds": {
        "value": 3.0,
        "tips": "连接超时时间",
        "unit": "秒",
        "type": "float"
    },
    "retry_count": {
        "value": 3,
        "tips": "重试次数",
        "unit": "次",
        "type": "int"
    }
}
####END DEFAULT ARGS####
"""


# 外部脚本可以调用的简化读取函数
def read_di_channel_simple(channel: int, module_ip: str = "192.168.192.30", 
                          timeout: float = 3.0) -> Tuple[bool, str]:
    """
    读取指定DI通道的简化函数（供外部脚本调用）
    
    Args:
        channel: DI通道编号 (0-15)
        module_ip: Modbus模块IP地址
        timeout: 超时时间
        
    Returns:
        bool: DI状态 (True=高电平, False=低电平)
        str: 错误信息，成功则为空字符串
    """
    try:
        manager = DIReaderManager()
        service = manager.get_service(module_ip, timeout)
        status = service.read_channel(channel)
        return status.state, status.error
    except Exception as e:
        print(f"[ERROR] 读取失败: {e}")
        return False, str(e)


class Module(BasicModule):
    """DI状态读取模块"""
    
    VERSION = "2.0.1"
    MODULE_NAME = "DI状态读取模块"
    MAX_CHANNEL = 15
    MIN_CHANNEL = 0
    
    def __init__(self, r: SimModule, args: dict):
        super(Module, self).__init__()
        self.init = True
        self.status = MoveStatus.NONE
        self.r = r
        
        # 模块配置
        self.module_ip = "192.168.192.30"
        self.di_channel = 5
        self.polling_interval = 0.5
        self.timeout_seconds = 3.0
        self.retry_count = 3
        
        # 运行时状态
        self.read_started = False
        self.read_finished = False
        self.last_state = None
        self.read_count = 0
        
        # 统计信息
        self.success_count = 0
        self.fail_count = 0
        self.total_time = 0.0
        
        # DI服务
        self.di_service = None
        
        # 性能监控
        self.start_time = None
        self.last_read_time = None
        
        # 加载参数
        self._load_parameters(args)
    
    def _load_parameters(self, args: dict) -> None:
        """加载并验证参数"""
        if args:
            self.module_ip = args.get("module_ip", self.module_ip)
            self.di_channel = args.get("di_channel", self.di_channel)
            self.polling_interval = max(0.1, args.get("polling_interval", self.polling_interval))
            self.timeout_seconds = max(0.5, args.get("timeout_seconds", self.timeout_seconds))
            self.retry_count = max(1, min(args.get("retry_count", self.retry_count), 10))
    
    def _validate_channel(self) -> Tuple[bool, str]:
        """验证通道号"""
        if not (self.MIN_CHANNEL <= self.di_channel <= self.MAX_CHANNEL):
            return False, f"DI通道编号无效: {self.di_channel} (应在{self.MIN_CHANNEL}-{self.MAX_CHANNEL}之间)"
        return True, ""
    
    def _initialize_module(self) -> None:
        """初始化模块"""
        # 验证通道
        is_valid, error_msg = self._validate_channel()
        if not is_valid:
            self.r.logError(error_msg)
            self.status = MoveStatus.FAILED
            return
        
        # 显示初始化信息
        self._show_banner()
        
        # 初始化DI服务
        manager = DIReaderManager()
        self.di_service = manager.get_service(self.module_ip, self.timeout_seconds)
        
        # 重置状态
        self.read_started = False
        self.read_finished = False
        self.read_count = 0
        self.success_count = 0
        self.fail_count = 0
        self.total_time = 0.0
        self.start_time = time.time()
        
        self.r.setNotice(f"DI{self.di_channel:02d}: 正在初始化...")
        self.init = False
    
    def _show_banner(self) -> None:
        """显示模块信息"""
        self.r.logInfo("=" * 60)
        self.r.logInfo(f"{self.MODULE_NAME} v{self.VERSION}")
        self.r.logInfo("=" * 60)
        self.r.logInfo(f"目标模块: {self.module_ip}")
        self.r.logInfo(f"读取通道: DI{self.di_channel:02d}")
        self.r.logInfo(f"轮询间隔: {self.polling_interval}秒")
        self.r.logInfo(f"超时时间: {self.timeout_seconds}秒")
        self.r.logInfo(f"重试次数: {self.retry_count}次")
        self.r.logInfo("=" * 60)
    
    def _read_di_channel(self) -> Optional[DIStatus]:
        """读取DI通道"""
        if not self.di_service:
            return None
        
        start_time = time.time()
        status = self.di_service.read_channel(self.di_channel)
        elapsed = time.time() - start_time
        
        self.read_count += 1
        self.total_time += elapsed
        
        return status
    
    def _handle_read_result(self, status: DIStatus) -> None:
        """处理读取结果"""
        if status.error:
            # 读取失败
            self.fail_count += 1
            self.r.setNotice(f"DI{self.di_channel:02d}: 读取失败 - {status.error}")
            
            if self.read_count >= self.retry_count:
                self.r.logError(f"DI{self.di_channel:02d}读取失败: {status.error}")
                self._show_statistics()
                self.status = MoveStatus.FAILED
        else:
            # 读取成功
            self.success_count += 1
            
            # 状态变化检测
            state_changed = (self.last_state is None or self.last_state != status.state)
            self.last_state = status.state
            
            # 显示状态
            level = "●高电平" if status.state else "○低电平"
            self.r.setNotice(f"DI{self.di_channel:02d}: {level}")
            
            # 记录状态变化
            if state_changed or self.read_count <= 3:
                level_text = "高电平" if status.state else "低电平"
                response_time = status.timestamp - self.start_time
                self.r.logInfo(f"DI{self.di_channel:02d}: {level_text} (响应时间: {response_time:.3f}秒)")
            
            # 单次读取完成
            self.read_finished = True
            self.r.logInfo(f"DI{self.di_channel:02d}读取完成，状态: {'高电平' if status.state else '低电平'}")
            self._show_statistics()
            self.status = MoveStatus.FINISHED
    
    def _show_statistics(self) -> None:
        """显示统计信息"""
        if self.read_count > 0:
            self.r.logInfo("=" * 40)
            self.r.logInfo("读取统计:")
            self.r.logInfo(f"  总读取次数: {self.read_count}")
            self.r.logInfo(f"  成功次数: {self.success_count}")
            self.r.logInfo(f"  失败次数: {self.fail_count}")
            success_rate = (self.success_count / self.read_count * 100) if self.read_count > 0 else 0
            self.r.logInfo(f"  成功率: {success_rate:.1f}%")
            if self.success_count > 0:
                avg_time = self.total_time / self.success_count
                self.r.logInfo(f"  平均响应时间: {avg_time:.3f}秒")
            self.r.logInfo("=" * 40)
    
    def run(self, r: SimModule, args: dict) -> MoveStatus:
        """主运行函数"""
        if self.status == MoveStatus.FINISHED:
            return self.status
        
        self.status = MoveStatus.RUNNING
        
        # 初始化模块
        if self.init:
            self._load_parameters(args)
            self._initialize_module()
            if self.status == MoveStatus.FAILED:
                return self.status
        
        # 检查是否需要读取
        if self.status == MoveStatus.RUNNING and not self.read_started:
            self.read_started = True
            r.setNotice(f"DI{self.di_channel:02d}: 正在读取...")
        
        # 执行读取操作
        if not self.read_finished and self.read_started:
            self.last_read_time = time.time()
            status = self._read_di_channel()
            
            if status:
                self._handle_read_result(status)
        
        return self.status
    
    def cancel(self, r: SimModule) -> None:
        """取消操作"""
        self.r.logInfo(f"已取消DI{self.di_channel:02d}读取")
        self._show_statistics()
        self.r.setNotice(f"DI{self.di_channel:02d}: 已取消")
        self._reset_state()
    
    def suspend(self, r: SimModule) -> MoveStatus:
        """暂停操作"""
        self.r.logInfo(f"暂停DI{self.di_channel:02d}读取")
        self.status = MoveStatus.SUSPENDED
        self.r.setNotice(f"DI{self.di_channel:02d}: 已暂停")
        return self.status
    
    def _reset_state(self) -> None:
        """重置状态"""
        self.read_started = False
        self.read_finished = False
        self.init = True
        self.status = MoveStatus.NONE
        self.last_state = None
    
    def get_di_state(self) -> Optional[bool]:
        """获取当前DI状态"""
        return self.last_state
    
    def get_statistics(self) -> dict:
        """获取统计信息"""
        success_rate = (self.success_count / self.read_count) if self.read_count > 0 else 0
        avg_response_time = self.total_time / self.success_count if self.success_count > 0 else 0
        
        return {
            "read_count": self.read_count,
            "success_count": self.success_count,
            "fail_count": self.fail_count,
            "total_time": self.total_time,
            "avg_response_time": avg_response_time,
            "success_rate": success_rate
        }


# 外部调用接口
class DIReader:
    """DI读取器（企业级接口）"""
    
    _manager = DIReaderManager()
    
    @staticmethod
    def read_once(channel: int, module_ip: str = "192.168.192.30", 
                  timeout: float = 3.0) -> Tuple[bool, str]:
        """
        单次读取指定DI通道
        
        Args:
            channel: DI通道编号 (0-15)
            module_ip: Modbus模块IP
            timeout: 超时时间
            
        Returns:
            bool: DI状态 (True=高电平, False=低电平)
            str: 错误信息
        """
        return read_di_channel_simple(channel, module_ip, timeout)
    
    @staticmethod
    def read_di5(module_ip: str = "192.168.192.30", 
                 timeout: float = 3.0) -> Tuple[bool, str]:
        """读取DI5通道的快捷方法"""
        return read_di_channel_simple(5, module_ip, timeout)
    
    @staticmethod
    def read_di0(module_ip: str = "192.168.192.30", 
                 timeout: float = 3.0) -> Tuple[bool, str]:
        """读取DI0通道的快捷方法"""
        return read_di_channel_simple(0, module_ip, timeout)
    
    @staticmethod
    def read_di1(module_ip: str = "192.168.192.30", 
                 timeout: float = 3.0) -> Tuple[bool, str]:
        """读取DI1通道的快捷方法"""
        return read_di_channel_simple(1, module_ip, timeout)
    
    @staticmethod
    def read_all(module_ip: str = "192.168.192.30", 
                 timeout: float = 3.0) -> Tuple[Optional[list], str]:
        """
        读取所有DI通道
        
        Returns:
            list: 16个DI通道的状态列表
            str: 错误信息
        """
        try:
            manager = DIReader._manager
            service = manager.get_service(module_ip, timeout)
            results, error = service.read_all_channels()
            if error:
                return None, error
            
            states = [status.state for status in results]
            return states, ""
        except Exception as e:
            print(f"[ERROR] 读取所有通道失败: {e}")
            return None, str(e)
    
    @staticmethod
    def monitor(channel: int, module_ip: str = "192.168.192.30", 
                interval: float = 1.0, max_attempts: int = 0, 
                on_state_change: Optional[Callable] = None, 
                timeout: float = 3.0) -> dict:
        """
        监控指定DI通道状态变化
        
        Args:
            channel: DI通道编号 (0-15)
            module_ip: Modbus模块IP
            interval: 读取间隔
            max_attempts: 最大尝试次数，0表示无限
            on_state_change: 状态变化回调函数
            timeout: 超时时间
        
        Returns:
            dict: 统计信息
        """
        if not (0 <= channel <= 15):
            return {"error": f"通道编号无效: {channel}"}
        
        manager = DIReader._manager
        service = manager.get_service(module_ip, timeout)
        
        start_time = time.time()
        last_state = None
        attempt_count = 0
        success_count = 0
        fail_count = 0
        change_count = 0
        
        try:
            while max_attempts == 0 or attempt_count < max_attempts:
                # 读取DI
                status = service.read_channel(channel)
                attempt_count += 1
                elapsed = time.time() - start_time
                
                if status.error:
                    fail_count += 1
                    print(f"[WARN] 第{attempt_count}次读取失败: {status.error} (耗时: {elapsed:.3f}秒)")
                else:
                    success_count += 1
                    
                    # 检查状态变化
                    if last_state is not None and last_state != status.state:
                        change_count += 1
                        if on_state_change:
                            try:
                                on_state_change(channel, status.state, last_state, status.timestamp)
                            except Exception as e:
                                print(f"[ERROR] 回调函数执行失败: {e}")
                    
                    last_state = status.state
                    level = "高电平" if status.state else "低电平"
                    print(f"[INFO] 第{attempt_count}次: DI{channel:02d} = {level} (耗时: {elapsed:.3f}秒)")
                
                # 等待下一次读取
                if max_attempts == 0 or attempt_count < max_attempts:
                    time.sleep(interval)
                    
        except KeyboardInterrupt:
            print("[INFO] 用户中断监控")
        except Exception as e:
            print(f"[ERROR] 监控异常: {e}")
        
        # 返回统计
        total_time = time.time() - start_time
        success_rate = success_count / attempt_count if attempt_count > 0 else 0
        avg_interval = total_time / attempt_count if attempt_count > 0 else 0
        
        return {
            "channel": channel,
            "module_ip": module_ip,
            "total_time": total_time,
            "attempts": attempt_count,
            "success": success_count,
            "fail": fail_count,
            "changes": change_count,
            "last_state": last_state,
            "success_rate": success_rate,
            "avg_interval": avg_interval
        }
    
    @staticmethod
    def close_all_connections() -> None:
        """关闭所有连接"""
        DIReader._manager.close_all()


# 原版函数的兼容性封装
def _read_di_channel_internal(channel, module_ip, timeout):
    """读取指定DI通道的内部函数（保持与原函数兼容）"""
    return read_di_channel_simple(channel, module_ip, timeout)


if __name__ == '__main__':
    """模块测试"""
    print("=" * 50)
    print("DI状态读取模块测试")
    print("=" * 50)
    
    # 测试基本读取
    test_channels = [0, 5, 10, 15]
    
    for channel in test_channels:
        print(f"\n测试读取DI{channel:02d}:")
        print("-" * 30)
        
        state, error = DIReader.read_once(channel)
        if error:
            print(f"  失败: {error}")
        else:
            level = "高电平" if state else "低电平"
            print(f"  DI{channel:02d}: {level}")
    
    # 读取所有通道
    print("\n读取所有DI通道:")
    all_states, error = DIReader.read_all()
    if error:
        print(f"  失败: {error}")
    else:
        for i, state in enumerate(all_states):
            level = "高" if state else "低"
            if i % 8 == 0:
                print("  ", end="")
            print(f"DI{i:02d}:{level} ", end="")
            if i % 8 == 7:
                print()
    
    # 连续监控
    print("\n连续监控DI3（模拟3次）:")
    
    # 定义回调函数
    def on_di_change(channel, new_state, old_state, timestamp):
        new_level = "高电平" if new_state else "低电平"
        old_level = "高电平" if old_state else "低电平"
        print(f"[回调] DI{channel:02d}状态变化: {old_level} -> {new_level}")
    
    # 开始监控
    stats = DIReader.monitor(
        channel=3,
        interval=1.0,
        max_attempts=3,
        on_state_change=on_di_change
    )
    
    print(f"\n监控统计:")
    print(f"  通道: DI{stats['channel']:02d}")
    print(f"  尝试次数: {stats['attempts']}")
    print(f"  成功次数: {stats['success']}")
    print(f"  失败次数: {stats['fail']}")
    print(f"  状态变化: {stats['changes']}")
    print(f"  成功率: {stats['success_rate']*100:.1f}%")
    
    print("\n" + "=" * 50)
    print("测试完成")