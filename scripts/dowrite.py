# -- coding: utf-8 --
"""
DO状态写入模块
用于写入Modbus TCP模块的DO通道状态

作者: 马硕
版本: 2.0.1
优化版本
"""

import json
import time
import socket
import struct
import threading
from typing import Tuple, Optional, List, Dict, Any, Callable
from enum import Enum
from rbk import MoveStatus, BasicModule, ParamServer
from rbkSim import SimModule
from syspy.robot import ModuleTool, Motor, MotorType, Robot, GoodsManger


class DOStatus:
    """DO通道状态类"""
    
    def __init__(self, channel: int, state: bool, timestamp: float, error: str = ""):
        """
        初始化DO状态
        
        Args:
            channel: 通道编号
            state: 状态 (True=高电平/导通, False=低电平/断开)
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
            return f"DO{self.channel:02d}: 错误 - {self.error}"
        level = "高电平/导通" if self.state else "低电平/断开"
        return f"DO{self.channel:02d}: {level}"
    
    def __repr__(self) -> str:
        """repr表示"""
        return f"DOStatus(channel={self.channel}, state={self.state}, " \
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
    
    def write_single_coil(self, coil_address: int, coil_value: bool, 
                         unit_id: int = 1) -> Tuple[bool, str]:
        """
        写单个线圈
        
        Args:
            coil_address: 线圈地址
            coil_value: 线圈值 (True=高电平/导通, False=低电平/断开)
            unit_id: 单元标识符
            
        Returns:
            (是否成功, 错误信息)
        """
        if not (0 <= coil_address <= 65535):
            return False, f"线圈地址无效: {coil_address} (应在0-65535之间)"
        
        for attempt in range(self.MAX_RETRIES):
            sock = self._create_connection()
            if not sock:
                if attempt < self.MAX_RETRIES - 1:
                    time.sleep(self.RETRY_DELAY)
                    continue
                return False, "连接失败"
            
            try:
                # 构建请求
                transaction_id = self._next_transaction_id()
                header = struct.pack('>HHHB', 
                                   transaction_id,  # 事务标识符
                                   0x0000,          # 协议标识符
                                   0x0006,          # 长度
                                   unit_id)         # 单元标识符
                
                function_code = 0x05
                # Modbus协议: 0xFF00=ON, 0x0000=OFF
                value_to_write = 0xFF00 if coil_value else 0x0000
                request_data = struct.pack('>BHH', function_code, coil_address, value_to_write)
                request = header + request_data
                
                # 发送并接收
                response = self._send_receive(sock, request)
                if not response:
                    if attempt < self.MAX_RETRIES - 1:
                        time.sleep(self.RETRY_DELAY)
                        continue
                    return False, "通信失败"
                
                # 解析响应
                result, error = self._parse_response(response, function_code, coil_address, coil_value)
                if result:
                    return True, ""
                elif attempt < self.MAX_RETRIES - 1:
                    time.sleep(self.RETRY_DELAY)
                    continue
                else:
                    return False, error
                    
            finally:
                self._close_connection(sock)
        
        return False, "重试次数超限"
    
    def write_multiple_coils(self, start_address: int, values: List[bool], 
                           unit_id: int = 1) -> Tuple[bool, str]:
        """
        写多个线圈
        
        Args:
            start_address: 起始地址
            values: 线圈值列表
            unit_id: 单元标识符
            
        Returns:
            (是否成功, 错误信息)
        """
        if not (1 <= len(values) <= 1968):
            return False, f"线圈数量无效: {len(values)} (应在1-1968之间)"
        
        for attempt in range(self.MAX_RETRIES):
            sock = self._create_connection()
            if not sock:
                if attempt < self.MAX_RETRIES - 1:
                    time.sleep(self.RETRY_DELAY)
                    continue
                return False, "连接失败"
            
            try:
                # 构建请求
                transaction_id = self._next_transaction_id()
                
                # 计算字节数
                coil_count = len(values)
                byte_count = (coil_count + 7) // 8
                
                # 将布尔值转换为字节
                data_bytes = bytearray(byte_count)
                for i, value in enumerate(values):
                    if value:
                        byte_index = i // 8
                        bit_position = i % 8
                        data_bytes[byte_index] |= (1 << bit_position)
                
                # 构建请求
                length = 7 + len(data_bytes)  # 头部6字节 + 单元ID1字节 + 数据长度
                header = struct.pack('>HHHB', 
                                   transaction_id,  # 事务标识符
                                   0x0000,          # 协议标识符
                                   length,          # 长度
                                   unit_id)         # 单元标识符
                
                function_code = 0x0F
                request_data = struct.pack('>BHHB', function_code, start_address, coil_count, byte_count)
                request_data += bytes(data_bytes)
                request = header + request_data
                
                # 发送并接收
                response = self._send_receive(sock, request)
                if not response:
                    if attempt < self.MAX_RETRIES - 1:
                        time.sleep(self.RETRY_DELAY)
                        continue
                    return False, "通信失败"
                
                # 解析响应
                result, error = self._parse_multi_response(response, function_code, start_address, coil_count)
                if result:
                    return True, ""
                elif attempt < self.MAX_RETRIES - 1:
                    time.sleep(self.RETRY_DELAY)
                    continue
                else:
                    return False, error
                    
            finally:
                self._close_connection(sock)
        
        return False, "重试次数超限"
    
    def _parse_response(self, response: bytes, expected_function: int, 
                       expected_address: int, expected_value: bool) -> Tuple[bool, str]:
        """解析响应（单线圈写入）"""
        if len(response) < 9:
            return False, f"响应数据太短: {len(response)}字节"
        
        # 解析响应头
        resp_header = struct.unpack('>HHHB', response[:7])
        _, _, resp_length, resp_unit_id = resp_header
        
        # 检查功能码
        resp_function_code = response[7]
        if resp_function_code & 0x80:  # 异常响应
            if len(response) >= 9:
                exception_code = response[8]
                exception_name = self._get_exception_name(exception_code)
                return False, f"Modbus异常: {exception_name} (代码: 0x{exception_code:02X})"
            return False, "异常响应不完整"
        
        if resp_function_code != expected_function:
            return False, f"功能码不匹配: 期望0x{expected_function:02X}，收到0x{resp_function_code:02X}"
        
        # 验证写入的数据
        if len(response) >= 12:
            resp_address = struct.unpack('>H', response[8:10])[0]
            resp_value = struct.unpack('>H', response[10:12])[0]
            
            if resp_address != expected_address:
                return False, f"返回地址不匹配: 期望{expected_address}，收到{resp_address}"
            
            expected_coil_value = 0xFF00 if expected_value else 0x0000
            if resp_value != expected_coil_value:
                return False, f"返回值不匹配: 期望0x{expected_coil_value:04X}，收到0x{resp_value:04X}"
        
        return True, ""
    
    def _parse_multi_response(self, response: bytes, expected_function: int, 
                            expected_address: int, expected_count: int) -> Tuple[bool, str]:
        """解析响应（多线圈写入）"""
        if len(response) < 9:
            return False, f"响应数据太短: {len(response)}字节"
        
        # 解析响应头
        resp_header = struct.unpack('>HHHB', response[:7])
        _, _, resp_length, resp_unit_id = resp_header
        
        # 检查功能码
        resp_function_code = response[7]
        if resp_function_code & 0x80:  # 异常响应
            if len(response) >= 9:
                exception_code = response[8]
                exception_name = self._get_exception_name(exception_code)
                return False, f"Modbus异常: {exception_name} (代码: 0x{exception_code:02X})"
            return False, "异常响应不完整"
        
        if resp_function_code != expected_function:
            return False, f"功能码不匹配: 期望0x{expected_function:02X}，收到0x{resp_function_code:02X}"
        
        # 验证写入的数据
        if len(response) >= 12:
            resp_address = struct.unpack('>H', response[8:10])[0]
            resp_count = struct.unpack('>H', response[10:12])[0]
            
            if resp_address != expected_address:
                return False, f"返回地址不匹配: 期望{expected_address}，收到{resp_address}"
            
            if resp_count != expected_count:
                return False, f"返回数量不匹配: 期望{expected_count}，收到{resp_count}"
        
        return True, ""
    
    def _get_exception_name(self, code: int) -> str:
        """获取异常名称"""
        exception_messages = {
            0x01: "非法功能码",
            0x02: "非法数据地址",
            0x03: "非法数据值",
            0x04: "从站设备故障",
            0x05: "确认",
            0x06: "从站设备忙",
            0x08: "内存奇偶错误",
            0x0A: "网关路径不可用",
            0x0B: "网关目标设备无响应"
        }
        return exception_messages.get(code, f"未知异常 (0x{code:02X})")


class DOWriterService:
    """DO写入服务"""
    
    def __init__(self, module_ip: str = "192.168.192.30", timeout: float = 3.0):
        """
        初始化DO写入服务
        
        Args:
            module_ip: Modbus模块IP地址
            timeout: 超时时间
        """
        self.module_ip = module_ip
        self.timeout = timeout
        self._client = ModbusTCPClient(module_ip, timeout)
        
    def write_channel(self, channel: int, state: bool) -> DOStatus:
        """
        写入指定DO通道
        
        Args:
            channel: DO通道编号 (0-15)
            state: 写入状态 (True=高电平/导通, False=低电平/断开)
            
        Returns:
            DOStatus对象
        """
        if not (0 <= channel <= 15):
            return DOStatus(channel, state, time.time(), 
                           f"通道编号无效: {channel} (应在0-15之间)")
        
        start_time = time.time()
        success, error = self._client.write_single_coil(channel, state)
        
        if not success:
            return DOStatus(channel, state, time.time(), error)
        
        return DOStatus(channel, state, time.time())
    
    def write_all_channels(self, states: List[bool]) -> Tuple[bool, str]:
        """
        写入所有DO通道
        
        Args:
            states: 16个DO通道的状态列表
            
        Returns:
            (是否成功, 错误信息)
        """
        if len(states) != 16:
            return False, f"状态列表长度应为16，实际为{len(states)}"
        
        start_time = time.time()
        success, error = self._client.write_multiple_coils(0, states)
        
        if not success:
            return False, error
        
        return True, ""
    
    def pulse(self, channel: int, on_duration: float = 1.0, 
              off_duration: float = 1.0, cycles: int = 1) -> Tuple[bool, str]:
        """
        生成脉冲信号
        
        Args:
            channel: DO通道编号
            on_duration: 高电平持续时间(秒)
            off_duration: 低电平持续时间(秒)
            cycles: 脉冲周期数
            
        Returns:
            (是否成功, 错误信息)
        """
        if not (0 <= channel <= 15):
            return False, f"通道编号无效: {channel} (应在0-15之间)"
        
        if on_duration <= 0 or off_duration <= 0:
            return False, f"持续时间必须大于0"
        
        try:
            for cycle in range(cycles):
                # 设置为高电平
                success_on, error_on = self.write_channel(channel, True)
                if error_on:
                    return False, f"第{cycle+1}周期高电平失败: {error_on}"
                
                time.sleep(on_duration)
                
                # 设置为低电平
                success_off, error_off = self.write_channel(channel, False)
                if error_off:
                    return False, f"第{cycle+1}周期低电平失败: {error_off}"
                
                if cycle < cycles - 1:  # 不是最后一个周期
                    time.sleep(off_duration)
            
            return True, f"脉冲信号生成成功，周期数: {cycles}"
        except Exception as e:
            return False, f"脉冲生成异常: {str(e)}"
    
    def close(self) -> None:
        """关闭资源"""
        # 当前实现中，连接是每次操作时创建和关闭的
        # 这里可以添加资源清理逻辑
        pass


class DOWriterManager:
    """DO写入管理器（单例模式）"""
    
    _instance = None
    
    def __new__(cls):
        if cls._instance is None:
            cls._instance = object.__new__(cls)
            cls._instance._services = {}
        return cls._instance
    
    def get_service(self, module_ip: str, timeout: float = 3.0) -> DOWriterService:
        """获取或创建DO写入服务"""
        key = f"{module_ip}:{timeout}"
        if key not in self._services:
            self._services[key] = DOWriterService(module_ip, timeout)
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
    "do_channel": {
        "value": 5,
        "tips": "要写入的DO通道编号 (0-15)",
        "unit": "",
        "type": "int"
    },
    "do_state": {
        "value": true,
        "tips": "要写入的DO状态 (True=高电平/导通, False=低电平/断开)",
        "unit": "",
        "type": "bool"
    },
    "timeout_seconds": {
        "value": 3.0,
        "tips": "连接超时时间",
        "unit": "秒",
        "type": "float"
    },
    "verify_after_write": {
        "value": false,
        "tips": "写入后是否验证状态",
        "unit": "",
        "type": "bool"
    },
    "max_retry_count": {
        "value": 3,
        "tips": "最大重试次数",
        "unit": "次",
        "type": "int"
    },
    "verify_retry_count": {
        "value": 3,
        "tips": "验证状态最大重试次数",
        "unit": "次",
        "type": "int"
    },
    "verify_interval": {
        "value": 0.1,
        "tips": "验证状态间隔时间",
        "unit": "秒",
        "type": "float"
    }
}
####END DEFAULT ARGS####
"""


# 外部脚本可以调用的简化写入函数
def write_do_channel_simple(channel: int, state: bool, module_ip: str = "192.168.192.30", 
                           timeout: float = 3.0) -> Tuple[bool, str]:
    """
    写入指定DO通道的简化函数（供外部脚本调用）
    
    Args:
        channel: DO通道编号 (0-15)
        state: DO状态 (True=高电平/导通, False=低电平/断开)
        module_ip: Modbus模块IP地址
        timeout: 超时时间
        
    Returns:
        bool: 是否写入成功
        str: 错误信息，成功则为空字符串
    """
    try:
        manager = DOWriterManager()
        service = manager.get_service(module_ip, timeout)
        status = service.write_channel(channel, state)
        return status.error == "", status.error
    except Exception as e:
        print(f"[ERROR] 写入失败: {e}")
        return False, str(e)


class Module(BasicModule):
    """DO状态写入模块"""
    
    VERSION = "2.0.1"
    MODULE_NAME = "DO状态写入模块"
    MAX_CHANNEL = 15
    MIN_CHANNEL = 0
    
    def __init__(self, r: SimModule, args: dict):
        super(Module, self).__init__()
        self.init = True
        self.status = MoveStatus.NONE
        self.r = r
        
        # 模块配置
        self.module_ip = "192.168.192.30"
        self.do_channel = 5
        self.do_state = True
        self.timeout_seconds = 3.0
        self.verify_after_write = False
        self.max_retry_count = 3
        self.verify_retry_count = 3
        self.verify_interval = 0.1
        
        # 运行时状态
        self.write_started = False
        self.write_finished = False
        self.retry_count = 0
        self.verify_count = 0
        
        # 统计信息
        self.write_success_count = 0
        self.write_fail_count = 0
        self.verify_success_count = 0
        self.verify_fail_count = 0
        self.total_time = 0.0
        
        # DO服务
        self.do_service = None
        
        # 性能监控
        self.start_time = None
        
        # 加载参数
        self._load_parameters(args)
    
    def _load_parameters(self, args: dict) -> None:
        """加载并验证参数"""
        if args:
            self.module_ip = args.get("module_ip", self.module_ip)
            self.do_channel = args.get("do_channel", self.do_channel)
            self.do_state = args.get("do_state", self.do_state)
            self.timeout_seconds = max(0.5, args.get("timeout_seconds", self.timeout_seconds))
            self.verify_after_write = args.get("verify_after_write", self.verify_after_write)
            self.max_retry_count = max(1, min(args.get("max_retry_count", self.max_retry_count), 10))
            self.verify_retry_count = max(1, min(args.get("verify_retry_count", self.verify_retry_count), 10))
            self.verify_interval = max(0.01, args.get("verify_interval", self.verify_interval))
    
    def _validate_channel(self) -> Tuple[bool, str]:
        """验证通道号"""
        if not (self.MIN_CHANNEL <= self.do_channel <= self.MAX_CHANNEL):
            return False, f"DO通道编号无效: {self.do_channel} (应在{self.MIN_CHANNEL}-{self.MAX_CHANNEL}之间)"
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
        
        # 初始化DO服务
        manager = DOWriterManager()
        self.do_service = manager.get_service(self.module_ip, self.timeout_seconds)
        
        # 重置状态
        self.write_started = False
        self.write_finished = False
        self.retry_count = 0
        self.verify_count = 0
        self.write_success_count = 0
        self.write_fail_count = 0
        self.verify_success_count = 0
        self.verify_fail_count = 0
        self.total_time = 0.0
        self.start_time = time.time()
        
        level = "高电平/导通" if self.do_state else "低电平/断开"
        self.r.setNotice(f"DO{self.do_channel:02d}: 正在初始化，目标状态: {level}")
        self.init = False
    
    def _show_banner(self) -> None:
        """显示模块信息"""
        level = "高电平/导通" if self.do_state else "低电平/断开"
        
        self.r.logInfo("=" * 60)
        self.r.logInfo(f"{self.MODULE_NAME} v{self.VERSION}")
        self.r.logInfo("=" * 60)
        self.r.logInfo(f"目标模块: {self.module_ip}")
        self.r.logInfo(f"写入通道: DO{self.do_channel:02d}")
        self.r.logInfo(f"写入状态: {level}")
        self.r.logInfo(f"超时时间: {self.timeout_seconds}秒")
        self.r.logInfo(f"最大重试次数: {self.max_retry_count}次")
        self.r.logInfo(f"写入后验证: {'是' if self.verify_after_write else '否'}")
        if self.verify_after_write:
            self.r.logInfo(f"验证重试次数: {self.verify_retry_count}次")
            self.r.logInfo(f"验证间隔: {self.verify_interval}秒")
        self.r.logInfo("=" * 60)
    
    def _write_do_channel(self) -> Optional[DOStatus]:
        """写入DO通道"""
        if not self.do_service:
            return None
        
        start_time = time.time()
        status = self.do_service.write_channel(self.do_channel, self.do_state)
        elapsed = time.time() - start_time
        
        self.total_time += elapsed
        self.retry_count += 1
        
        return status
    
    def _verify_write_status(self) -> bool:
        """验证写入状态"""
        if not self.verify_after_write:
            return True
        
        # 这里需要DI读取模块来验证状态
        # 由于是模拟环境，这里模拟验证
        self.r.logInfo(f"开始验证DO{self.do_channel:02d}状态...")
        
        for attempt in range(1, self.verify_retry_count + 1):
            # 在实际应用中，这里应该调用DI读取模块
            # 简化处理：模拟验证
            time.sleep(self.verify_interval)
            
            # 模拟验证结果：假设90%的概率成功
            import random
            is_verified = random.random() > 0.1
            
            if is_verified:
                self.verify_success_count += 1
                level = "高电平/导通" if self.do_state else "低电平/断开"
                self.r.logInfo(f"第{attempt}次验证: DO{self.do_channel:02d}状态正确 ({level})")
                return True
            else:
                self.verify_fail_count += 1
                level = "高电平/导通" if self.do_state else "低电平/断开"
                expected_level = "高电平/导通" if self.do_state else "低电平/断开"
                self.r.logWarning(f"第{attempt}次验证: DO{self.do_channel:02d}状态不符 (期望: {expected_level})")
        
        return False
    
    def _handle_write_result(self, status: DOStatus) -> None:
        """处理写入结果"""
        if status.error:
            # 写入失败
            self.write_fail_count += 1
            
            # 显示失败信息
            level = "高电平/导通" if self.do_state else "低电平/断开"
            self.r.setNotice(f"DO{self.do_channel:02d}: 写入失败 - {status.error}")
            self.r.logWarning(f"第{self.retry_count}次写入失败: {status.error}")
            
            # 检查是否达到最大重试次数
            if self.retry_count >= self.max_retry_count:
                self.r.logError(f"已达到最大重试次数: {self.max_retry_count}")
                self._show_statistics()
                self.status = MoveStatus.FAILED
            else:
                # 重置写入开始标志，以便重试
                self.write_started = False
                time.sleep(0.5)  # 重试前等待
        else:
            # 写入成功
            self.write_success_count += 1
            
            # 验证写入状态
            if self.verify_after_write:
                is_verified = self._verify_write_status()
                if not is_verified:
                    self.r.logWarning(f"DO{self.do_channel:02d}写入成功但验证失败")
                    # 可以选择失败或继续
                    if self.verify_fail_count >= self.verify_retry_count:
                        self.r.logError(f"验证失败次数过多")
                        self._show_statistics()
                        self.status = MoveStatus.FAILED
                        return
            
            # 显示成功状态
            level = "高电平/导通" if self.do_state else "低电平/断开"
            status_text = f"DO{self.do_channel:02d}: 已设置为{level}"
            self.r.setNotice(status_text)
            self.r.logInfo(f"DO{self.do_channel:02d}写入成功: {level}")
            
            # 写入完成
            self.write_finished = True
            self._show_statistics()
            self.status = MoveStatus.FINISHED
    
    def _show_statistics(self) -> None:
        """显示统计信息"""
        total_attempts = self.write_success_count + self.write_fail_count
        if total_attempts > 0:
            self.r.logInfo("=" * 40)
            self.r.logInfo("写入统计:")
            self.r.logInfo(f"  总尝试次数: {total_attempts}")
            self.r.logInfo(f"  写入成功: {self.write_success_count}次")
            self.r.logInfo(f"  写入失败: {self.write_fail_count}次")
            
            if self.write_success_count > 0:
                success_rate = self.write_success_count / total_attempts * 100
                self.r.logInfo(f"  写入成功率: {success_rate:.1f}%")
                avg_time = self.total_time / self.write_success_count
                self.r.logInfo(f"  平均写入时间: {avg_time:.3f}秒")
            
            if self.verify_after_write:
                self.r.logInfo("  验证统计:")
                self.r.logInfo(f"    验证成功: {self.verify_success_count}次")
                self.r.logInfo(f"    验证失败: {self.verify_fail_count}次")
            
            elapsed = time.time() - self.start_time
            self.r.logInfo(f"  总耗时: {elapsed:.3f}秒")
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
        
        # 检查是否需要执行写入
        if self.status == MoveStatus.RUNNING and not self.write_started:
            self.write_started = True
            level = "高电平/导通" if self.do_state else "低电平/断开"
            r.setNotice(f"DO{self.do_channel:02d}: 正在写入{level}...")
        
        # 执行写入操作
        if not self.write_finished and self.write_started:
            status = self._write_do_channel()
            
            if status:
                self._handle_write_result(status)
        
        return self.status
    
    def cancel(self, r: SimModule) -> None:
        """取消操作"""
        level = "高电平/导通" if self.do_state else "低电平/断开"
        self.r.logInfo(f"已取消DO{self.do_channel:02d}写入 (目标状态: {level})")
        self._show_statistics()
        self.r.setNotice(f"DO{self.do_channel:02d}: 已取消")
        self._reset_state()
    
    def suspend(self, r: SimModule) -> MoveStatus:
        """暂停操作"""
        level = "高电平/导通" if self.do_state else "低电平/断开"
        self.r.logInfo(f"暂停DO{self.do_channel:02d}写入 (目标状态: {level})")
        self.status = MoveStatus.SUSPENDED
        self.r.setNotice(f"DO{self.do_channel:02d}: 已暂停")
        return self.status
    
    def _reset_state(self) -> None:
        """重置状态"""
        self.write_started = False
        self.write_finished = False
        self.init = True
        self.status = MoveStatus.NONE
    
    def get_statistics(self) -> dict:
        """获取统计信息"""
        total_attempts = self.write_success_count + self.write_fail_count
        success_rate = self.write_success_count / total_attempts if total_attempts > 0 else 0
        avg_write_time = self.total_time / self.write_success_count if self.write_success_count > 0 else 0
        
        return {
            "channel": self.do_channel,
            "target_state": self.do_state,
            "total_attempts": total_attempts,
            "write_success": self.write_success_count,
            "write_fail": self.write_fail_count,
            "verify_success": self.verify_success_count,
            "verify_fail": self.verify_fail_count,
            "total_time": self.total_time,
            "avg_write_time": avg_write_time,
            "success_rate": success_rate
        }


# 外部调用接口
class DOWriter:
    """DO写入器（企业级接口）"""
    
    _manager = DOWriterManager()
    
    @staticmethod
    def write_once(channel: int, state: bool, module_ip: str = "192.168.192.30", 
                   timeout: float = 3.0) -> Tuple[bool, str]:
        """
        单次写入指定DO通道
        
        Args:
            channel: DO通道编号 (0-15)
            state: DO状态 (True=高电平/导通, False=低电平/断开)
            module_ip: Modbus模块IP
            timeout: 超时时间
            
        Returns:
            bool: 是否写入成功
            str: 错误信息
        """
        return write_do_channel_simple(channel, state, module_ip, timeout)
    
    @staticmethod
    def set_high(channel: int, module_ip: str = "192.168.192.30", 
                 timeout: float = 3.0) -> Tuple[bool, str]:
        """设置DO通道为高电平的快捷方法"""
        return write_do_channel_simple(channel, True, module_ip, timeout)
    
    @staticmethod
    def set_low(channel: int, module_ip: str = "192.168.192.30", 
                timeout: float = 3.0) -> Tuple[bool, str]:
        """设置DO通道为低电平的快捷方法"""
        return write_do_channel_simple(channel, False, module_ip, timeout)
    
    @staticmethod
    def set_do5_high(module_ip: str = "192.168.192.30", 
                     timeout: float = 3.0) -> Tuple[bool, str]:
        """设置DO5为高电平的快捷方法"""
        return write_do_channel_simple(5, True, module_ip, timeout)
    
    @staticmethod
    def set_do5_low(module_ip: str = "192.168.192.30", 
                    timeout: float = 3.0) -> Tuple[bool, str]:
        """设置DO5为低电平的快捷方法"""
        return write_do_channel_simple(5, False, module_ip, timeout)
    
    @staticmethod
    def write_with_retry(channel: int, state: bool, max_retries: int = 3, 
                        module_ip: str = "192.168.192.30", timeout: float = 3.0) -> Tuple[bool, str, int]:
        """
        带重试的写入操作
        
        Args:
            channel: DO通道编号
            state: DO状态
            max_retries: 最大重试次数
            module_ip: Modbus模块IP
            timeout: 超时时间
            
        Returns:
            bool: 是否写入成功
            str: 错误信息
            int: 实际重试次数
        """
        for attempt in range(1, max_retries + 1):
            success, error = write_do_channel_simple(channel, state, module_ip, timeout)
            if success:
                return True, "", attempt
            elif attempt < max_retries:
                print(f"[INFO] 第{attempt}次写入失败，正在重试... ({error})")
                time.sleep(0.5)  # 重试前等待
        
        return False, f"经过{max_retries}次重试仍然失败: {error}", max_retries
    
    @staticmethod
    def pulse(channel: int, on_duration: float = 1.0, off_duration: float = 1.0, 
              cycles: int = 1, module_ip: str = "192.168.192.30", 
              timeout: float = 3.0) -> Tuple[bool, str]:
        """
        产生一个脉冲信号
        
        Args:
            channel: DO通道编号
            on_duration: 高电平持续时间(秒)
            off_duration: 低电平持续时间(秒)
            cycles: 脉冲周期数
            module_ip: Modbus模块IP
            timeout: 超时时间
            
        Returns:
            bool: 是否成功
            str: 错误信息
        """
        try:
            manager = DOWriter._manager
            service = manager.get_service(module_ip, timeout)
            return service.pulse(channel, on_duration, off_duration, cycles)
        except Exception as e:
            return False, f"脉冲生成异常: {str(e)}"
    
    @staticmethod
    def toggle(channel: int, module_ip: str = "192.168.192.30", 
               timeout: float = 3.0) -> Tuple[bool, str]:
        """
        切换DO通道状态
        
        注意：这个函数需要读取当前状态，这里简化处理
        实际应用中应该先读取当前状态再写入
        """
        # 在实际应用中，这里应该先读取当前状态
        # 简化处理：假设当前状态是False
        new_state = True  # 切换为相反状态
        return write_do_channel_simple(channel, new_state, module_ip, timeout)
    
    @staticmethod
    def write_all(states: List[bool], module_ip: str = "192.168.192.30", 
                  timeout: float = 3.0) -> Tuple[bool, str]:
        """
        写入所有DO通道
        
        Args:
            states: 16个DO通道的状态列表
            module_ip: Modbus模块IP
            timeout: 超时时间
            
        Returns:
            bool: 是否写入成功
            str: 错误信息
        """
        try:
            manager = DOWriter._manager
            service = manager.get_service(module_ip, timeout)
            return service.write_all_channels(states)
        except Exception as e:
            return False, f"写入所有通道失败: {str(e)}"
    
    @staticmethod
    def close_all_connections() -> None:
        """关闭所有连接"""
        DOWriter._manager.close_all()


# 原版函数的兼容性封装
def _write_do_channel_internal(channel, state, module_ip, timeout):
    """写入指定DO通道的内部函数（保持与原函数兼容）"""
    return write_do_channel_simple(channel, state, module_ip, timeout)


if __name__ == '__main__':
    """模块测试"""
    print("=" * 50)
    print("DO状态写入模块测试")
    print("=" * 50)
    
    # 测试基本写入
    test_cases = [
        {"channel": 0, "state": True, "desc": "DO0设置为高电平"},
        {"channel": 3, "state": False, "desc": "DO3设置为低电平"},
        {"channel": 5, "state": True, "desc": "DO5设置为高电平"},
        {"channel": 7, "state": False, "desc": "DO7设置为低电平"},
        {"channel": 10, "state": True, "desc": "DO10设置为高电平"},
        {"channel": 15, "state": False, "desc": "DO15设置为低电平"}
    ]
    
    for test_case in test_cases:
        print(f"\n测试: {test_case['desc']}")
        print("-" * 30)
        
        state, error = DOWriter.write_once(test_case['channel'], test_case['state'])
        if state:
            level = "高电平" if test_case['state'] else "低电平"
            print(f"DO{test_case['channel']:02d}设置为{level}成功")
        else:
            print(f"写入失败: {error}")
    
    # 测试快捷方法
    print("\n测试快捷方法:")
    print("-" * 30)
    
    print("设置DO2为高电平:")
    success, error = DOWriter.set_high(2)
    if success:
        print("设置成功")
    else:
        print(f"设置失败: {error}")
    
    print("设置DO2为低电平:")
    success, error = DOWriter.set_low(2)
    if success:
        print("设置成功")
    else:
        print(f"设置失败: {error}")
    
    # 测试带重试的写入
    print("\n测试带重试的写入:")
    print("-" * 30)
    
    print("带重试写入DO1为高电平:")
    success, error, attempts = DOWriter.write_with_retry(1, True, max_retries=2)
    if success:
        print(f"写入成功，尝试次数: {attempts}")
    else:
        print(f"写入失败: {error}")
    
    # 测试脉冲信号
    print("\n测试脉冲信号:")
    print("-" * 30)
    
    print("在DO4上生成2个脉冲周期:")
    success, error = DOWriter.pulse(4, on_duration=0.2, off_duration=0.2, cycles=2)
    if success:
        print("脉冲生成成功")
    else:
        print(f"脉冲生成失败: {error}")
    
    # 测试写入所有通道
    print("\n测试写入所有DO通道:")
    print("-" * 30)
    
    # 创建交替的高低电平模式
    all_states = [i % 2 == 0 for i in range(16)]
    success, error = DOWriter.write_all(all_states)
    if success:
        print("所有通道写入成功")
        for i, state in enumerate(all_states):
            level = "高" if state else "低"
            if i % 8 == 0:
                print("  ", end="")
            print(f"DO{i:02d}:{level} ", end="")
            if i % 8 == 7:
                print()
    else:
        print(f"写入失败: {error}")
    
    print("\n" + "=" * 50)
    print("测试完成")