# # -- coding: utf-8 --
# import json
# import time
# import socket
# import struct
# from rbk import MoveStatus, BasicModule, ParamServer
# from rbkSim import SimModule
# from syspy.robot import ModuleTool, Robot, GoodsManger

# """
# ####BEGIN DEFAULT ARGS####
# {
#     "module_ip": {
#         "value": "192.168.192.30",
#         "tips": "CK-TP5163模块IP地址",
#         "unit": "",
#         "type": "string"
#     },
#     "modbus_port": {
#         "value": 502,
#         "tips": "Modbus TCP端口",
#         "unit": "",
#         "type": "int"
#     },
#     "unit_id": {
#         "value": 1,
#         "tips": "Modbus从站地址",
#         "unit": "",
#         "type": "int"
#     },
#     "timeout_seconds": {
#         "value": 5.0,
#         "tips": "连接超时时间",
#         "unit": "秒",
#         "type": "float"
#     },
#     "di_channel": {
#         "value": 0,
#         "tips": "要读取的DI通道编号 (0-15)",
#         "unit": "",
#         "type": "int"
#     },
#     "polling_interval": {
#         "value": 0.5,
#         "tips": "轮询间隔时间",
#         "unit": "秒",
#         "type": "float"
#     },
#     "max_read_attempts": {
#         "value": 3,
#         "tips": "最大读取尝试次数",
#         "unit": "次",
#         "type": "int"
#     }
# }
# ####END DEFAULT ARGS####
# """

# def read_single_di(module_ip, port, unit_id, di_channel, timeout=5.0):
#     """
#     读取单个DI通道的状态
    
#     Args:
#         module_ip: Modbus模块IP地址
#         port: Modbus端口
#         unit_id: 从站地址
#         di_channel: DI通道编号 (0-15)
#         timeout: 超时时间
    
#     Returns:
#         bool: DI电平状态 (True=高电平/导通, False=低电平/断开)
#         str: 错误信息，成功则为空字符串
#     """
#     if not (0 <= di_channel <= 15):
#         return False, f"DI通道编号无效: {di_channel} (应在0-15之间)"
    
#     client = None
#     try:
#         # 创建socket连接
#         client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
#         client.settimeout(timeout)
        
#         # 连接到Modbus模块
#         client.connect((module_ip, port))
        
#         # 构建Modbus TCP读取离散输入请求
#         # 功能码: 0x02 (读离散输入)
#         # 起始地址: 0
#         # 输入数量: 16
#         transaction_id = 1
#         protocol_id = 0x0000
#         length = 0x0006
#         unit_id_byte = unit_id
        
#         # Modbus TCP头部
#         header = struct.pack('>HHHB', 
#                             transaction_id,    # 事务标识符
#                             protocol_id,       # 协议标识符
#                             length,            # 长度
#                             unit_id_byte)      # 单元标识符
        
#         # 功能码和数据
#         function_code = 0x02
#         start_address = 0
#         input_count = 16
        
#         request_data = struct.pack('>BHH', function_code, start_address, input_count)
#         request = header + request_data
        
#         # 发送请求
#         client.sendall(request)
        
#         # 接收响应
#         response = client.recv(1024)
        
#         if len(response) < 9:
#             return False, f"响应数据太短: {len(response)}字节"
        
#         # 解析响应头
#         resp_header = struct.unpack('>HHHB', response[:7])
#         resp_transaction_id, resp_protocol_id, resp_length, resp_unit_id = resp_header
        
#         # 检查单元ID
#         if resp_unit_id != unit_id:
#             return False, f"单元ID不匹配: 期望{unit_id}，收到{resp_unit_id}"
        
#         # 检查功能码
#         resp_function_code = response[7]
#         if resp_function_code & 0x80:  # 异常响应
#             if len(response) >= 9:
#                 exception_code = response[8]
#                 return False, f"Modbus异常 (代码: 0x{exception_code:02X})"
#             else:
#                 return False, "异常响应不完整"
        
#         if resp_function_code != 0x02:
#             return False, f"功能码不匹配: 期望0x02，收到0x{resp_function_code:02X}"
        
#         # 解析响应数据
#         if len(response) < 9:
#             return False, "响应数据不完整"
        
#         byte_count = response[8]
#         if len(response) < 9 + byte_count:
#             return False, f"响应数据长度不足: 期望{9+byte_count}字节，收到{len(response)}字节"
        
#         # 提取位数据
#         bit_data = response[9:9+byte_count]
        
#         # 计算目标通道所在的字节和位位置
#         byte_index = di_channel // 8
#         bit_position = di_channel % 8
        
#         if byte_index >= len(bit_data):
#             return False, f"响应数据中不包含通道{di_channel}"
        
#         # 读取指定通道的状态
#         channel_byte = bit_data[byte_index]
#         channel_state = (channel_byte >> bit_position) & 0x01 == 0x01
        
#         return channel_state, ""
        
#     except socket.timeout:
#         return False, f"连接或读取超时 ({timeout}秒)"
#     except ConnectionRefusedError:
#         return False, f"连接被拒绝"
#     except socket.gaierror:
#         return False, f"无法解析主机名: {module_ip}"
#     except Exception as e:
#         return False, f"读取失败: {str(e)}"
#     finally:
#         if client:
#             try:
#                 client.close()
#             except:
#                 pass


# class Module(BasicModule):
#     """CK-TP5163 单通道DI状态读取模块
#     读取指定DI通道的电平状态，并通过r.setNotice显示结果
#     """
    
#     def __init__(self, r: SimModule, args):
#         super(Module, self).__init__()
#         self.init = True
#         self.status = MoveStatus.NONE
#         self.r = r
        
#         # 参数初始化
#         self.module_ip = "192.168.192.30"
#         self.modbus_port = 502
#         self.unit_id = 1
#         self.timeout_seconds = 5.0
#         self.di_channel = 0
#         self.polling_interval = 0.5
#         self.max_read_attempts = 3
        
#         # 控制状态
#         self.read_attempts = 0
#         self.last_read_time = 0
#         self.last_di_state = None
#         self.connection_ok = False
        
#         # 统计信息
#         self.success_count = 0
#         self.fail_count = 0

#     def run(self, r: SimModule, args):
#         """主函数，每个运行周期都会执行"""
#         if self.status == MoveStatus.FINISHED:
#             return self.status
        
#         if self.status == MoveStatus.SUSPENDED:
#             return self.status
        
#         # 初始化参数
#         if self.init:
#             self.module_ip = args.get("module_ip", "192.168.192.30")
#             self.modbus_port = args.get("modbus_port", 502)
#             self.unit_id = args.get("unit_id", 1)
#             self.timeout_seconds = args.get("timeout_seconds", 5.0)
#             self.di_channel = args.get("di_channel", 0)
#             self.polling_interval = args.get("polling_interval", 0.5)
#             self.max_read_attempts = args.get("max_read_attempts", 3)
            
#             # 验证参数
#             if not (0 <= self.di_channel <= 15):
#                 r.logError(f"DI通道编号无效: {self.di_channel} (应在0-15之间)")
#                 self.status = MoveStatus.FAILED
#                 return self.status
            
#             # 显示初始化信息
#             r.logInfo("=" * 50)
#             r.logInfo("CK-TP5163 单通道DI状态读取")
#             r.logInfo("=" * 50)
#             r.logInfo(f"目标模块: {self.module_ip}:{self.modbus_port}")
#             r.logInfo(f"从站地址: {self.unit_id}")
#             r.logInfo(f"读取通道: DI{self.di_channel:02d}")
#             r.logInfo(f"轮询间隔: {self.polling_interval}秒")
#             r.logInfo(f"最大尝试次数: {self.max_read_attempts}次")
#             r.logInfo("=" * 50)
            
#             self.read_attempts = 0
#             self.success_count = 0
#             self.fail_count = 0
#             self.connection_ok = False
#             self.status = MoveStatus.RUNNING
#             self.init = False
            
#             r.setNotice(f"DI{self.di_channel:02d}: 正在初始化...")
        
#         # 检查是否需要读取
#         current_time = time.time()
#         if current_time - self.last_read_time < self.polling_interval:
#             return self.status
        
#         # 更新读取时间
#         self.last_read_time = current_time
#         self.read_attempts += 1
        
#         # 读取DI状态
#         di_state, error_msg = read_single_di(
#             module_ip=self.module_ip,
#             port=self.modbus_port,
#             unit_id=self.unit_id,
#             di_channel=self.di_channel,
#             timeout=self.timeout_seconds
#         )
        
#         if error_msg:
#             # 读取失败
#             self.fail_count += 1
#             self.connection_ok = False
            
#             # 显示错误信息
#             status_text = f"DI{self.di_channel:02d}: 读取失败 - {error_msg}"
#             r.setNotice(status_text)
            
#             # 记录错误日志（避免频繁记录）
#             if self.read_attempts <= 3 or self.read_attempts % 10 == 0:
#                 r.logWarning(f"第{self.read_attempts}次读取失败: {error_msg}")
            
#             # 检查是否达到最大尝试次数
#             if self.max_read_attempts > 0 and self.read_attempts >= self.max_read_attempts:
#                 r.logError(f"已达到最大读取尝试次数: {self.max_read_attempts}")
#                 r.logError(f"成功: {self.success_count}次, 失败: {self.fail_count}次")
#                 self.status = MoveStatus.FAILED
#                 r.setNotice(f"DI{self.di_channel:02d}: 读取失败，已停止")
#         else:
#             # 读取成功
#             self.success_count += 1
#             self.connection_ok = True
            
#             # 状态变化检测
#             state_changed = (self.last_di_state is None or self.last_di_state != di_state)
#             self.last_di_state = di_state
            
#             # 显示状态
#             status_text = f"DI{self.di_channel:02d}: {'●高电平' if di_state else '○低电平'}"
#             r.setNotice(status_text)
            
#             # 记录状态变化
#             if state_changed or self.read_attempts <= 3 or self.read_attempts % 20 == 0:
#                 if di_state:
#                     r.logInfo(f"DI{self.di_channel:02d}: 高电平/导通")
#                 else:
#                     r.logInfo(f"DI{self.di_channel:02d}: 低电平/断开")
            
#             # 显示连接状态
#             if self.read_attempts <= 3 or self.read_attempts % 30 == 0:
#                 r.logInfo(f"连接正常，已读取{self.success_count}次")
        
#         return self.status

#     def cancel(self, r):
#         """取消操作"""
#         r.logInfo("取消DI状态读取")
        
#         # 显示统计信息
#         total_attempts = self.success_count + self.fail_count
#         if total_attempts > 0:
#             success_rate = (self.success_count / total_attempts) * 100
#             r.logInfo(f"统计: 尝试{total_attempts}次, 成功{self.success_count}次, 失败{self.fail_count}次")
#             r.logInfo(f"成功率: {success_rate:.1f}%")
        
#         r.setNotice(f"DI{self.di_channel:02d}: 已取消读取")
        
#         # 重置状态
#         self.init = True
#         self.status = MoveStatus.NONE
        
#         return self.status

#     def suspend(self, r):
#         """暂停读取"""
#         r.logInfo("暂停DI状态读取")
#         self.status = MoveStatus.SUSPENDED
#         r.setNotice(f"DI{self.di_channel:02d}: 已暂停")
#         return self.status
    
#     def resume(self, r, args=None):
#         """恢复读取"""
#         r.logInfo("恢复DI状态读取")
#         self.status = MoveStatus.RUNNING
#         r.setNotice(f"DI{self.di_channel:02d}: 正在读取...")
#         return self.status
    
#     def get_current_state(self):
#         """获取当前DI状态"""
#         return {
#             "channel": self.di_channel,
#             "state": self.last_di_state,
#             "connected": self.connection_ok,
#             "success_count": self.success_count,
#             "fail_count": self.fail_count,
#             "read_attempts": self.read_attempts
#         }


# # 测试函数
# def test_read_single_di():
#     """测试读取单个DI通道"""
#     print("测试读取单个DI通道")
#     print("=" * 50)
    
#     # 测试参数
#     test_params = {
#         "module_ip": "192.168.192.30",
#         "port": 502,
#         "unit_id": 1,
#         "timeout": 3.0
#     }
    
#     # 测试不同通道
#     for channel in [0, 1, 7, 15]:
#         print(f"\n测试通道 DI{channel:02d}...")
        
#         # 模拟读取
#         if channel % 2 == 0:  # 偶数通道模拟高电平
#             di_state, error_msg = True, ""
#         else:  # 奇数通道模拟低电平
#             di_state, error_msg = False, ""
        
#         # 模拟连接失败
#         if channel == 7:
#             di_state, error_msg = False, "连接超时"
        
#         if error_msg:
#             print(f"  ✗ 读取失败: {error_msg}")
#         else:
#             status = "●高电平" if di_state else "○低电平"
#             print(f"  ✓ {status}")
    
#     print("\n" + "=" * 50)
#     print("测试完成")


# if __name__ == '__main__':
#     import rbkSim
    
#     # 测试代码
#     print("DI单通道状态读取模块测试")
#     print("=" * 50)

#     # 创建模拟环境
#     class MockR:
#         def logInfo(self, msg):
#             print(f"[INFO] {msg}")
        
#         def logError(self, msg):
#             print(f"[ERROR] {msg}")
        
#         def logWarning(self, msg):
#             print(f"[WARN] {msg}")
        
#         def setNotice(self, msg):
#             print(f"[NOTICE] {msg}")
    
#     r = MockR()
    
#     # 测试参数
#     test_args = {
#         "module_ip": "192.168.192.30",
#         "modbus_port": 502,
#         "unit_id": 1,
#         "timeout_seconds": 3.0,
#         "di_channel": 0,
#         "polling_interval": 1.0,
#         "max_read_attempts": 5
#     }
    
#     # 创建模块实例
#     m = Module(r, None)
    
#     # 模拟运行
#     print("\n模拟模块运行:")
#     print("-" * 30)
    
#     # 模拟几次运行周期
#     for i in range(10):
#         print(f"\n运行周期 {i+1}:")
        
#         # 运行模块
#         result = m.run(r, test_args)
        
#         # 获取当前状态
#         state = m.get_current_state()
#         print(f"  状态: {m.status}")
#         print(f"  DI状态: {state['state']}")
#         print(f"  连接: {state['connected']}")
        
#         if result in [MoveStatus.FINISHED, MoveStatus.FAILED]:
#             print(f"模块结束: {result}")
#             break
    
#     print("\n" + "=" * 50)
#     print("模块测试完成")
    
#     # 单独测试读取函数
#     test_read_single_di()

# -- coding: utf-8 --
import json
import time
import socket
import struct
from rbk import MoveStatus, BasicModule, ParamServer
from rbkSim import SimModule
from syspy.robot import ModuleTool, Robot, GoodsManger

"""
####BEGIN DEFAULT ARGS####
{
    "module_ip": {
        "value": "192.168.192.30",
        "tips": "CK-TP5163模块IP地址",
        "unit": "",
        "type": "string"
    },
    "modbus_port": {
        "value": 502,
        "tips": "Modbus TCP端口",
        "unit": "",
        "type": "int"
    },
    "unit_id": {
        "value": 1,
        "tips": "Modbus从站地址",
        "unit": "",
        "type": "int"
    },
    "timeout_seconds": {
        "value": 5.0,
        "tips": "连接超时时间",
        "unit": "秒",
        "type": "float"
    },
    "operation_mode": {
        "value": "read_di",
        "tips": "操作模式: read_di(读DI), write_do(写DO), read_write(读写)",
        "unit": "",
        "type": "string"
    },
    "di_channel": {
        "value": 0,
        "tips": "要读取的DI通道编号 (0-15)",
        "unit": "",
        "type": "int"
    },
    "do_channel": {
        "value": 0,
        "tips": "要写入的DO通道编号 (0-15)",
        "unit": "",
        "type": "int"
    },
    "do_state": {
        "value": false,
        "tips": "要写入的DO状态 (True=高电平/导通, False=低电平/断开)",
        "unit": "",
        "type": "bool"
    },
    "polling_interval": {
        "value": 0.5,
        "tips": "轮询间隔时间",
        "unit": "秒",
        "type": "float"
    },
    "max_read_attempts": {
        "value": 3,
        "tips": "最大读取尝试次数",
        "unit": "次",
        "type": "int"
    },
    "debug_mode": {
        "value": false,
        "tips": "调试模式，显示详细日志",
        "unit": "",
        "type": "bool"
    }
}
####END DEFAULT ARGS####
"""

class ModbusClient:
    """Modbus TCP客户端，支持读写功能"""
    
    def __init__(self, ip, port=502, unit_id=1, timeout=5.0):
        self.ip = ip
        self.port = port
        self.unit_id = unit_id
        self.timeout = timeout
        self.socket = None
        self.transaction_id = 0
        self.connected = False
    
    def connect(self):
        """建立连接"""
        try:
            if self.socket:
                self.close()
            
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.settimeout(self.timeout)
            self.socket.connect((self.ip, self.port))
            self.connected = True
            return True
        except Exception as e:
            self.connected = False
            raise Exception(f"连接失败: {e}")
    
    def close(self):
        """关闭连接"""
        if self.socket:
            try:
                self.socket.close()
            except:
                pass
        self.socket = None
        self.connected = False
    
    def _next_transaction_id(self):
        """获取下一个事务ID"""
        self.transaction_id = (self.transaction_id + 1) % 65536
        return self.transaction_id
    
    def _build_request(self, function_code, data):
        """构建Modbus请求"""
        transaction_id = self._next_transaction_id()
        protocol_id = 0x0000
        length = len(data) + 1
        
        header = struct.pack('>HHHB', 
                           transaction_id,
                           protocol_id,
                           length,
                           self.unit_id)
        
        body = struct.pack('B', function_code) + data
        return header + body
    
    def _parse_response(self, response, expected_function_code):
        """解析Modbus响应"""
        if len(response) < 9:
            raise Exception(f"响应太短: {len(response)}字节")
        
        # 解析头部
        header = struct.unpack('>HHHB', response[:7])
        _, _, _, resp_unit_id = header
        
        if resp_unit_id != self.unit_id:
            raise Exception(f"单元ID不匹配: 期望{self.unit_id}，收到{resp_unit_id}")
        
        # 检查功能码
        function_code = response[7]
        if function_code & 0x80:  # 异常响应
            if len(response) >= 9:
                exception_code = response[8]
                exceptions = {
                    0x01: "非法功能码",
                    0x02: "非法数据地址",
                    0x03: "非法数据值",
                    0x04: "从站设备故障",
                }
                msg = exceptions.get(exception_code, f"未知异常: 0x{exception_code:02X}")
                raise Exception(f"Modbus异常: {msg}")
            else:
                raise Exception("异常响应不完整")
        
        if function_code != expected_function_code:
            raise Exception(f"功能码不匹配: 期望0x{expected_function_code:02X}，收到0x{function_code:02X}")
        
        return response[8:]
    
    def read_di(self, channel):
        """读取单个DI通道状态
        
        Args:
            channel: DI通道编号 (0-15)
            
        Returns:
            bool: DI状态 (True=高电平, False=低电平)
            str: 错误信息，成功则为空
        """
        if not (0 <= channel <= 15):
            return False, f"DI通道编号无效: {channel}"
        
        if not self.connected:
            return False, "连接未建立"
        
        try:
            # 构建读取离散输入请求
            data = struct.pack('>HH', 0, 16)  # 从地址0开始读16个
            request = self._build_request(0x02, data)
            
            # 发送请求
            self.socket.sendall(request)
            
            # 接收响应
            response = self.socket.recv(1024)
            
            if not response:
                return False, "未收到响应"
            
            # 解析响应
            response_data = self._parse_response(response, 0x02)
            
            if len(response_data) < 1:
                return False, "响应数据不完整"
            
            byte_count = response_data[0]
            bit_data = response_data[1:1+byte_count]
            
            if byte_index := channel // 8 >= len(bit_data):
                return False, f"响应数据中不包含通道{channel}"
            
            # 计算通道状态
            byte_index = channel // 8
            bit_position = channel % 8
            channel_byte = bit_data[byte_index]
            channel_state = (channel_byte >> bit_position) & 0x01 == 0x01
            
            return channel_state, ""
            
        except Exception as e:
            return False, f"读取失败: {str(e)}"
    
    def write_do(self, channel, state):
        """写入单个DO通道状态
        
        Args:
            channel: DO通道编号 (0-15)
            state: DO状态 (True=高电平/导通, False=低电平/断开)
            
        Returns:
            bool: 是否成功
            str: 错误信息，成功则为空
        """
        if not (0 <= channel <= 15):
            return False, f"DO通道编号无效: {channel}"
        
        if not self.connected:
            return False, "连接未建立"
        
        try:
            # 计算线圈地址（假设DO从地址0开始）
            address = channel
            value = 0xFF00 if state else 0x0000
            
            # 构建写单个线圈请求
            data = struct.pack('>HH', address, value)
            request = self._build_request(0x05, data)
            
            # 发送请求
            self.socket.sendall(request)
            
            # 接收响应
            response = self.socket.recv(1024)
            
            if not response:
                return False, "未收到响应"
            
            # 解析响应
            self._parse_response(response, 0x05)
            
            return True, ""
            
        except Exception as e:
            return False, f"写入失败: {str(e)}"
    
    def read_all_di(self):
        """读取所有16个DI通道状态
        
        Returns:
            list: 16个DI通道的状态列表
            str: 错误信息，成功则为空
        """
        if not self.connected:
            return None, "连接未建立"
        
        try:
            # 构建读取离散输入请求
            data = struct.pack('>HH', 0, 16)
            request = self._build_request(0x02, data)
            
            # 发送请求
            self.socket.sendall(request)
            
            # 接收响应
            response = self.socket.recv(1024)
            
            if not response:
                return None, "未收到响应"
            
            # 解析响应
            response_data = self._parse_response(response, 0x02)
            
            if len(response_data) < 1:
                return None, "响应数据不完整"
            
            byte_count = response_data[0]
            bit_data = response_data[1:1+byte_count]
            
            # 解析所有位
            di_states = []
            for byte in bit_data:
                for bit in range(8):
                    di_states.append((byte >> bit) & 0x01 == 0x01)
            
            return di_states[:16], ""
            
        except Exception as e:
            return None, f"读取失败: {str(e)}"


# 全局Modbus客户端实例（供外部调用）
_global_modbus_client = None

def get_modbus_client(ip="192.168.192.30", port=502, unit_id=1, timeout=5.0):
    """获取全局Modbus客户端实例（供其他脚本调用）
    
    Args:
        ip: Modbus模块IP
        port: Modbus端口
        unit_id: 从站地址
        timeout: 超时时间
        
    Returns:
        ModbusClient: Modbus客户端实例
    """
    global _global_modbus_client
    
    if _global_modbus_client is None:
        _global_modbus_client = ModbusClient(ip, port, unit_id, timeout)
    
    return _global_modbus_client

def read_di_channel(channel, ip="192.168.192.30", port=502, unit_id=1, timeout=5.0):
    """读取单个DI通道（供其他脚本调用）
    
    Args:
        channel: DI通道编号 (0-15)
        ip: Modbus模块IP
        port: Modbus端口
        unit_id: 从站地址
        timeout: 超时时间
        
    Returns:
        bool: DI状态 (True=高电平, False=低电平)
        str: 错误信息
    """
    client = get_modbus_client(ip, port, unit_id, timeout)
    
    try:
        if not client.connected:
            client.connect()
        
        return client.read_di(channel)
    except Exception as e:
        return False, f"读取异常: {str(e)}"

def write_do_channel(channel, state, ip="192.168.192.30", port=502, unit_id=1, timeout=5.0):
    """写入单个DO通道（供其他脚本调用）
    
    Args:
        channel: DO通道编号 (0-15)
        state: DO状态 (True=高电平, False=低电平)
        ip: Modbus模块IP
        port: Modbus端口
        unit_id: 从站地址
        timeout: 超时时间
        
    Returns:
        bool: 是否成功
        str: 错误信息
    """
    client = get_modbus_client(ip, port, unit_id, timeout)
    
    try:
        if not client.connected:
            client.connect()
        
        return client.write_do(channel, state)
    except Exception as e:
        return False, f"写入异常: {str(e)}"

def read_all_di_channels(ip="192.168.192.30", port=502, unit_id=1, timeout=5.0):
    """读取所有DI通道（供其他脚本调用）
    
    Args:
        ip: Modbus模块IP
        port: Modbus端口
        unit_id: 从站地址
        timeout: 超时时间
        
    Returns:
        list: 16个DI通道状态列表
        str: 错误信息
    """
    client = get_modbus_client(ip, port, unit_id, timeout)
    
    try:
        if not client.connected:
            client.connect()
        
        return client.read_all_di()
    except Exception as e:
        return None, f"读取异常: {str(e)}"

def close_modbus_connection():
    """关闭Modbus连接（供其他脚本调用）"""
    global _global_modbus_client
    if _global_modbus_client:
        _global_modbus_client.close()
        _global_modbus_client = None


class Module(BasicModule):
    """CK-TP5163 DI/DO读写控制模块
    支持读取DI和写入DO，可通过r.setNotice显示状态
    """
    
    def __init__(self, r: SimModule, args):
        super(Module, self).__init__()
        self.init = True
        self.status = MoveStatus.NONE
        self.r = r
        
        # 参数初始化
        self.module_ip = "192.168.192.30"
        self.modbus_port = 502
        self.unit_id = 1
        self.timeout_seconds = 5.0
        self.operation_mode = "read_di"  # read_di, write_do, read_write
        self.di_channel = 0
        self.do_channel = 0
        self.do_state = False
        self.polling_interval = 0.5
        self.max_read_attempts = 3
        self.debug_mode = False
        
        # 控制状态
        self.modbus_client = None
        self.read_attempts = 0
        self.last_operation_time = 0
        self.last_di_state = None
        self.last_do_state = None
        self.connected = False
        
        # 统计信息
        self.success_count = 0
        self.fail_count = 0
        
        # 运行状态
        self.is_initial_connect = True

    def _connect_modbus(self):
        """连接Modbus设备"""
        try:
            if self.modbus_client:
                self.modbus_client.close()
            
            self.modbus_client = ModbusClient(
                ip=self.module_ip,
                port=self.modbus_port,
                unit_id=self.unit_id,
                timeout=self.timeout_seconds
            )
            
            self.modbus_client.connect()
            self.connected = True
            self.is_initial_connect = False
            
            if self.debug_mode:
                self.r.logInfo(f"已连接到 {self.module_ip}:{self.modbus_port}")
            
            return True
            
        except Exception as e:
            self.connected = False
            if self.debug_mode or self.is_initial_connect:
                self.r.logWarning(f"连接失败: {e}")
            return False

    def _perform_read_di(self):
        """执行读取DI操作"""
        if not self.connected or not self.modbus_client:
            if not self._connect_modbus():
                return False, "连接失败"
        
        try:
            di_state, error_msg = self.modbus_client.read_di(self.di_channel)
            
            if error_msg:
                self.connected = False
                return False, error_msg
            
            # 状态变化检测
            state_changed = (self.last_di_state is None or self.last_di_state != di_state)
            self.last_di_state = di_state
            
            # 显示状态
            status_text = f"DI{self.di_channel:02d}: {'●高电平' if di_state else '○低电平'}"
            self.r.setNotice(status_text)
            
            # 记录状态变化
            if state_changed or (self.debug_mode and self.read_attempts % 10 == 0):
                level = "高电平" if di_state else "低电平"
                self.r.logInfo(f"DI{self.di_channel:02d}: {level}")
            
            return True, ""
            
        except Exception as e:
            self.connected = False
            return False, f"读取异常: {str(e)}"

    def _perform_write_do(self):
        """执行写入DO操作"""
        if not self.connected or not self.modbus_client:
            if not self._connect_modbus():
                return False, "连接失败"
        
        try:
            success, error_msg = self.modbus_client.write_do(self.do_channel, self.do_state)
            
            if error_msg:
                self.connected = False
                return False, error_msg
            
            # 状态变化检测
            state_changed = (self.last_do_state is None or self.last_do_state != self.do_state)
            self.last_do_state = self.do_state
            
            # 显示状态
            action = "设置" if success else "设置失败"
            level = "高电平" if self.do_state else "低电平"
            status_text = f"DO{self.do_channel:02d}: {action} {level}"
            self.r.setNotice(status_text)
            
            if success:
                if state_changed or self.debug_mode:
                    self.r.logInfo(f"DO{self.do_channel:02d}: 已设置为{level}")
                return True, ""
            else:
                return False, "写入失败但无错误信息"
            
        except Exception as e:
            self.connected = False
            return False, f"写入异常: {str(e)}"

    def _perform_read_write(self):
        """执行读写操作（先读后写）"""
        # 读取DI
        read_success, read_error = self._perform_read_di()
        
        if not read_success:
            return False, f"读取失败: {read_error}"
        
        # 写入DO
        write_success, write_error = self._perform_write_do()
        
        if not write_success:
            return False, f"写入失败: {write_error}"
        
        # 组合状态显示
        di_level = "高" if self.last_di_state else "低"
        do_level = "高" if self.do_state else "低"
        status_text = f"DI{self.di_channel:02d}:{di_level} | DO{self.do_channel:02d}:{do_level}"
        self.r.setNotice(status_text)
        
        return True, ""

    def run(self, r: SimModule, args):
        """主运行函数"""
        if self.status == MoveStatus.FINISHED:
            return self.status
        
        if self.status == MoveStatus.SUSPENDED:
            return self.status
        
        # 初始化参数
        if self.init:
            self.module_ip = args.get("module_ip", "192.168.192.30")
            self.modbus_port = args.get("modbus_port", 502)
            self.unit_id = args.get("unit_id", 1)
            self.timeout_seconds = args.get("timeout_seconds", 5.0)
            self.operation_mode = args.get("operation_mode", "read_di")
            self.di_channel = args.get("di_channel", 0)
            self.do_channel = args.get("do_channel", 0)
            self.do_state = args.get("do_state", False)
            self.polling_interval = args.get("polling_interval", 0.5)
            self.max_read_attempts = args.get("max_read_attempts", 3)
            self.debug_mode = args.get("debug_mode", False)
            
            # 参数验证
            if not (0 <= self.di_channel <= 15):
                r.logError(f"DI通道编号无效: {self.di_channel}")
                self.status = MoveStatus.FAILED
                return self.status
            
            if not (0 <= self.do_channel <= 15):
                r.logError(f"DO通道编号无效: {self.do_channel}")
                self.status = MoveStatus.FAILED
                return self.status
            
            if self.operation_mode not in ["read_di", "write_do", "read_write"]:
                r.logError(f"操作模式无效: {self.operation_mode}")
                self.status = MoveStatus.FAILED
                return self.status
            
            # 显示配置信息
            r.logInfo("=" * 50)
            r.logInfo("CK-TP5163 DI/DO读写控制模块")
            r.logInfo("=" * 50)
            r.logInfo(f"目标模块: {self.module_ip}:{self.modbus_port}")
            r.logInfo(f"从站地址: {self.unit_id}")
            r.logInfo(f"操作模式: {self.operation_mode}")
            
            if self.operation_mode in ["read_di", "read_write"]:
                r.logInfo(f"读取通道: DI{self.di_channel:02d}")
            
            if self.operation_mode in ["write_do", "read_write"]:
                level = "高电平" if self.do_state else "低电平"
                r.logInfo(f"写入通道: DO{self.do_channel:02d} -> {level}")
            
            r.logInfo(f"轮询间隔: {self.polling_interval}秒")
            r.logInfo(f"最大尝试次数: {self.max_read_attempts}次")
            r.logInfo("=" * 50)
            
            # 初始化状态
            self.read_attempts = 0
            self.success_count = 0
            self.fail_count = 0
            self.connected = False
            self.is_initial_connect = True
            self.status = MoveStatus.RUNNING
            self.init = False
            
            # 设置初始通知
            if self.operation_mode == "read_di":
                r.setNotice(f"DI{self.di_channel:02d}: 正在初始化...")
            elif self.operation_mode == "write_do":
                level = "高电平" if self.do_state else "低电平"
                r.setNotice(f"DO{self.do_channel:02d}: 准备设置为{level}")
            else:  # read_write
                r.setNotice(f"DI/DO: 正在初始化...")
        
        # 检查是否需要执行操作
        current_time = time.time()
        if current_time - self.last_operation_time < self.polling_interval:
            return self.status
        
        # 更新操作时间
        self.last_operation_time = current_time
        self.read_attempts += 1
        
        # 根据操作模式执行相应操作
        success = False
        error_msg = ""
        
        if self.operation_mode == "read_di":
            success, error_msg = self._perform_read_di()
        elif self.operation_mode == "write_do":
            success, error_msg = self._perform_write_do()
        elif self.operation_mode == "read_write":
            success, error_msg = self._perform_read_write()
        
        # 更新统计
        if success:
            self.success_count += 1
        else:
            self.fail_count += 1
            
            # 显示错误
            if self.debug_mode or self.read_attempts <= 3 or self.read_attempts % 5 == 0:
                prefix = ""
                if self.operation_mode == "read_di":
                    prefix = f"DI{self.di_channel:02d}: "
                elif self.operation_mode == "write_do":
                    prefix = f"DO{self.do_channel:02d}: "
                
                r.setNotice(f"{prefix}错误 - {error_msg}")
                r.logWarning(f"第{self.read_attempts}次操作失败: {error_msg}")
            
            # 检查是否达到最大尝试次数
            if self.max_read_attempts > 0 and self.read_attempts >= self.max_read_attempts:
                r.logError(f"已达到最大尝试次数: {self.max_read_attempts}")
                r.logError(f"统计: 成功{self.success_count}次, 失败{self.fail_count}次")
                self.status = MoveStatus.FAILED
                
                if self.operation_mode == "read_di":
                    r.setNotice(f"DI{self.di_channel:02d}: 读取失败，已停止")
                elif self.operation_mode == "write_do":
                    r.setNotice(f"DO{self.do_channel:02d}: 写入失败，已停止")
                else:
                    r.setNotice("DI/DO: 操作失败，已停止")
        
        return self.status

    def cancel(self, r):
        """取消操作"""
        r.logInfo("取消DI/DO读写操作")
        
        # 关闭连接
        if self.modbus_client:
            self.modbus_client.close()
        
        # 显示统计
        total_attempts = self.success_count + self.fail_count
        if total_attempts > 0:
            success_rate = (self.success_count / total_attempts) * 100 if total_attempts > 0 else 0
            r.logInfo(f"统计: 尝试{total_attempts}次, 成功{self.success_count}次, 失败{self.fail_count}次")
            r.logInfo(f"成功率: {success_rate:.1f}%")
        
        # 设置通知
        if self.operation_mode == "read_di":
            r.setNotice(f"DI{self.di_channel:02d}: 已取消")
        elif self.operation_mode == "write_do":
            r.setNotice(f"DO{self.do_channel:02d}: 已取消")
        else:
            r.setNotice("DI/DO: 已取消")
        
        # 重置状态
        self.init = True
        self.status = MoveStatus.NONE
        
        return self.status

    def suspend(self, r):
        """暂停操作"""
        r.logInfo("暂停DI/DO读写操作")
        self.status = MoveStatus.SUSPENDED
        
        if self.operation_mode == "read_di":
            r.setNotice(f"DI{self.di_channel:02d}: 已暂停")
        elif self.operation_mode == "write_do":
            r.setNotice(f"DO{self.do_channel:02d}: 已暂停")
        else:
            r.setNotice("DI/DO: 已暂停")
        
        return self.status
    
    def resume(self, r, args=None):
        """恢复操作"""
        r.logInfo("恢复DI/DO读写操作")
        self.status = MoveStatus.RUNNING
        
        if self.operation_mode == "read_di":
            r.setNotice(f"DI{self.di_channel:02d}: 正在读取...")
        elif self.operation_mode == "write_do":
            level = "高电平" if self.do_state else "低电平"
            r.setNotice(f"DO{self.do_channel:02d}: 正在设置为{level}")
        else:
            r.setNotice("DI/DO: 正在操作...")
        
        return self.status
    
    def get_status(self):
        """获取当前状态"""
        return {
            "operation_mode": self.operation_mode,
            "di_channel": self.di_channel,
            "di_state": self.last_di_state,
            "do_channel": self.do_channel,
            "do_state": self.last_do_state,
            "connected": self.connected,
            "success_count": self.success_count,
            "fail_count": self.fail_count,
            "read_attempts": self.read_attempts
        }


# ==================== 供其他脚本调用的接口函数 ====================

def set_di_monitor(channel, ip="192.168.192.30", callback=None):
    """在其他脚本中设置DI监控
    
    Args:
        channel: DI通道编号
        ip: Modbus模块IP
        callback: 状态变化回调函数，接收两个参数(channel, state)
    
    Returns:
        bool: 是否成功启动监控
    """
    # 这里可以启动一个后台线程来监控DI状态
    # 简化实现：只提供接口，实际需要在调用脚本中实现
    return True

def set_do_output(channel, state, ip="192.168.192.30"):
    """在其他脚本中设置DO输出
    
    Args:
        channel: DO通道编号
        state: DO状态
        ip: Modbus模块IP
    
    Returns:
        bool: 是否成功
        str: 错误信息
    """
    return write_do_channel(channel, state, ip)

def get_di_input(channel, ip="192.168.192.30"):
    """在其他脚本中获取DI输入
    
    Args:
        channel: DI通道编号
        ip: Modbus模块IP
    
    Returns:
        bool: DI状态
        str: 错误信息
    """
    return read_di_channel(channel, ip)

def get_all_di_inputs(ip="192.168.192.30"):
    """在其他脚本中获取所有DI输入
    
    Args:
        ip: Modbus模块IP
    
    Returns:
        list: 16个DI通道状态
        str: 错误信息
    """
    return read_all_di_channels(ip)


if __name__ == '__main__':
    import rbkSim
    
    # 测试代码
    print("CK-TP5163 DI/DO读写模块测试")
    print("=" * 50)
    
    # 创建模拟环境
    class MockR:
        def logInfo(self, msg):
            print(f"[INFO] {msg}")
        
        def logError(self, msg):
            print(f"[ERROR] {msg}")
        
        def logWarning(self, msg):
            print(f"[WARN] {msg}")
        
        def setNotice(self, msg):
            print(f"[NOTICE] {msg}")
    
    r = MockR()
    
    # 测试1: 读取DI
    print("\n测试1: 读取DI通道")
    print("-" * 30)
    
    test_args1 = {
        "module_ip": "192.168.192.30",
        "modbus_port": 502,
        "unit_id": 1,
        "operation_mode": "read_di",
        "di_channel": 3,
        "polling_interval": 1.0,
        "max_read_attempts": 3,
        "debug_mode": True
    }
    
    m1 = Module(r, None)
    
    for i in range(5):
        print(f"\n运行周期 {i+1}:")
        result = m1.run(r, test_args1)
        state = m1.get_status()
        print(f"  DI状态: {state['di_state']}")
        
        if result in [MoveStatus.FINISHED, MoveStatus.FAILED]:
            break
    
    # 测试2: 写入DO
    print("\n\n测试2: 写入DO通道")
    print("-" * 30)
    
    test_args2 = {
        "module_ip": "192.168.192.30",
        "modbus_port": 502,
        "unit_id": 1,
        "operation_mode": "write_do",
        "do_channel": 5,
        "do_state": True,
        "polling_interval": 2.0,
        "max_read_attempts": 2
    }
    
    m2 = Module(r, None)
    
    for i in range(3):
        print(f"\n运行周期 {i+1}:")
        result = m2.run(r, test_args2)
        state = m2.get_status()
        print(f"  DO状态: {state['do_state']}")
        
        if result in [MoveStatus.FINISHED, MoveStatus.FAILED]:
            break
    
    # 测试3: 外部调用接口
    print("\n\n测试3: 外部调用接口")
    print("-" * 30)
    
    # 模拟外部调用
    print("调用 set_do_output(2, True):")
    success, error = set_do_output(2, True)
    print(f"  结果: {success}, 错误: {error}")
    
    print("\n调用 get_di_input(3):")
    state, error = get_di_input(3)
    print(f"  结果: {state}, 错误: {error}")
    
    print("\n" + "=" * 50)
    print("测试完成")