#!/usr/bin/env python3
"""
极简版AGV系统TCP服务端 - 调试版
功能：按邮箱存储JSON数据，提供简单API
"""

import json
import os
import socket
import threading
import hashlib
from datetime import datetime
import traceback  # 添加traceback模块
from config import *

class AGVDataManager:
    """按邮箱管理AGV数据的极简管理器"""
    
    def __init__(self):
        # 确保数据目录存在
        if not os.path.exists(DATA_DIR):
            os.makedirs(DATA_DIR)
            print(f"🔧 DEBUG: 创建数据目录: {DATA_DIR}")
        else:
            print(f"🔧 DEBUG: 数据目录已存在: {DATA_DIR}")
            
        # 列出data目录中的文件
        print(f"🔧 DEBUG: data目录内容: {os.listdir(DATA_DIR) if os.path.exists(DATA_DIR) else '目录不存在'}")
    
    def _get_user_filename(self, email: str) -> str:
        """根据邮箱生成文件名"""
        # 使用邮箱的MD5哈希作为文件名
        email_clean = email.lower().strip()
        email_hash = hashlib.md5(email_clean.encode()).hexdigest()
        filename = os.path.join(DATA_DIR, f"{email_hash}.json")
        
        print(f"🔧 DEBUG: 邮箱 '{email}' -> 文件名: {filename}")
        return filename
    
    def get_user_data(self, email: str) -> dict:
        """获取用户的所有数据"""
        print(f"🔧 DEBUG: get_user_data 开始, 邮箱: {email}")
        filename = self._get_user_filename(email)
        
        print(f"🔧 DEBUG: 检查文件是否存在: {filename}")
        if not os.path.exists(filename):
            print(f"🔧 DEBUG: 文件不存在, 返回默认数据")
            default_data = DEFAULT_DATA.copy()
            default_data["email"] = email
            default_data["created_at"] = datetime.now().isoformat()
            default_data["updated_at"] = datetime.now().isoformat()
            print(f"🔧 DEBUG: 默认数据keys: {default_data.keys()}")
            return default_data
        
        try:
            print(f"🔧 DEBUG: 读取文件: {filename}")
            with open(filename, 'r', encoding=ENCODING) as f:
                data = json.load(f)
                data["updated_at"] = datetime.now().isoformat()
            
            print(f"🔧 DEBUG: 读取成功, 数据keys: {data.keys()}")
            if "agvs" in data:
                print(f"🔧 DEBUG: agvs数量: {len(data['agvs'])}")
            if "tasks" in data:
                print(f"🔧 DEBUG: tasks数量: {len(data['tasks'])}")
                
            return data
        except Exception as e:
            print(f"❌ 读取数据失败: {e}")
            print(f"🔧 DEBUG: 错误详情: {traceback.format_exc()}")
            return {"error": "读取数据失败"}
    
    def save_user_data(self, email: str, data: dict) -> bool:
        """保存用户的所有数据"""
        print(f"🔧 DEBUG: save_user_data 开始, 邮箱: {email}")
        filename = self._get_user_filename(email)
        
        try:
            # 确保包含必要字段
            if "email" not in data:
                data["email"] = email
            data["updated_at"] = datetime.now().isoformat()
            if "created_at" not in data:
                data["created_at"] = datetime.now().isoformat()
            
            print(f"🔧 DEBUG: 保存到文件: {filename}")
            with open(filename, 'w', encoding=ENCODING) as f:
                json.dump(data, f, ensure_ascii=False, indent=2)
            
            print(f"✅ DEBUG: 保存成功")
            return True
        except Exception as e:
            print(f"❌ 保存数据失败: {e}")
            print(f"🔧 DEBUG: 错误详情: {traceback.format_exc()}")
            return False
    
    def update_agv(self, email: str, agv_id: str, updates: dict) -> bool:
        """更新单个AGV状态"""
        print(f"🔧 DEBUG: update_agv 开始, 邮箱: {email}, AGV: {agv_id}")
        data = self.get_user_data(email)
        
        for agv in data.get("agvs", []):
            if agv["id"] == agv_id:
                agv.update(updates)
                agv["last_updated"] = datetime.now().isoformat()
                return self.save_user_data(email, data)
        
        print(f"🔧 DEBUG: 未找到AGV: {agv_id}")
        return False
    
    def add_task(self, email: str, task_data: dict) -> bool:
        """添加新任务"""
        print(f"🔧 DEBUG: add_task 开始, 邮箱: {email}")
        data = self.get_user_data(email)
        
        # 生成任务ID
        task_count = len(data.get("tasks", []))
        task_id = f"Task-{task_count + 1:03d}"
        task_data["id"] = task_id
        task_data["created_at"] = datetime.now().isoformat()
        
        if "tasks" not in data:
            data["tasks"] = []
        data["tasks"].append(task_data)
        
        print(f"🔧 DEBUG: 添加任务: {task_id}")
        return self.save_user_data(email, data)
    
    def update_task(self, email: str, task_id: str, updates: dict) -> bool:
        """更新任务状态"""
        print(f"🔧 DEBUG: update_task 开始, 邮箱: {email}, 任务: {task_id}")
        data = self.get_user_data(email)
        
        for task in data.get("tasks", []):
            if task["id"] == task_id:
                task.update(updates)
                task["updated_at"] = datetime.now().isoformat()
                return self.save_user_data(email, data)
        
        print(f"🔧 DEBUG: 未找到任务: {task_id}")
        return False
    
    def get_system_status(self, email: str) -> dict:
        """获取系统状态"""
        print(f"🔧 DEBUG: get_system_status 开始, 邮箱: {email}")
        data = self.get_user_data(email)
        return {
            "success": True,
            "data": data.get("system", {}),
            "timestamp": datetime.now().isoformat()
        }
    
    def get_agv_list(self, email: str) -> dict:
        """获取AGV列表"""
        print(f"🔧 DEBUG: get_agv_list 开始, 邮箱: {email}")
        data = self.get_user_data(email)
        
        agvs = data.get("agvs", [])
        print(f"🔧 DEBUG: 返回AGV列表, 数量: {len(agvs)}")
        
        return {
            "success": True,
            "data": agvs,
            "count": len(agvs),
            "timestamp": datetime.now().isoformat()
        }
    
    def get_tasks(self, email: str) -> dict:
        """获取任务列表"""
        print(f"🔧 DEBUG: get_tasks 开始, 邮箱: {email}")
        data = self.get_user_data(email)
        
        tasks = data.get("tasks", [])
        print(f"🔧 DEBUG: 返回任务列表, 数量: {len(tasks)}")
        
        return {
            "success": True,
            "data": tasks,
            "count": len(tasks),
            "timestamp": datetime.now().isoformat()
        }
    
    def get_history(self, email: str) -> dict:
        """获取历史记录"""
        print(f"🔧 DEBUG: get_history 开始, 邮箱: {email}")
        data = self.get_user_data(email)
        
        history = data.get("history", [])
        print(f"🔧 DEBUG: 返回历史记录, 数量: {len(history)}")
        
        return {
            "success": True,
            "data": history,
            "count": len(history),
            "timestamp": datetime.now().isoformat()
        }

class AGVTCPServer:
    """极简TCP服务器"""
    
    def __init__(self, host=SERVER_HOST, port=SERVER_PORT):
        self.host = host
        self.port = port
        self.server_socket = None
        self.running = False
        self.data_manager = AGVDataManager()
    
    def start(self):
        """启动服务器"""
        try:
            self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.server_socket.bind((self.host, self.port))
            self.server_socket.listen(5)
            self.running = True
            
            print(f"🚀 AGV TCP服务器已启动")
            print(f"📡 监听地址: {self.host}:{self.port}")
            print(f"📁 数据目录: {DATA_DIR}")
            print(f"🔧 支持的API: {', '.join(SUPPORTED_APIS)}")
            print(f"🔧 默认数据keys: {DEFAULT_DATA.keys() if isinstance(DEFAULT_DATA, dict) else 'DEFAULT_DATA不是字典'}")
            print("=" * 50)
            
            while self.running:
                try:
                    client_socket, client_address = self.server_socket.accept()
                    print(f"📞 新连接: {client_address}")
                    
                    # 为每个客户端创建新线程
                    client_thread = threading.Thread(
                        target=self.handle_client,
                        args=(client_socket, client_address)
                    )
                    client_thread.daemon = True
                    client_thread.start()
                    
                except KeyboardInterrupt:
                    print("\n🛑 收到停止信号")
                    self.stop()
                except Exception as e:
                    print(f"❌ 接受连接错误: {e}")
                    
        except Exception as e:
            print(f"❌ 服务器启动失败: {e}")
            print(f"🔧 DEBUG: 错误详情: {traceback.format_exc()}")
    
    def handle_client(self, client_socket, client_address):
        """处理客户端连接"""
        print(f"🔧 DEBUG: handle_client 开始, 客户端: {client_address}")
        
        try:
            while True:
                # 接收数据
                request_data = client_socket.recv(BUFFER_SIZE)
                if not request_data:
                    print(f"📤 客户端断开: {client_address}")
                    break
                
                # 解析请求
                request_str = request_data.decode(ENCODING).strip()
                print(f"📨 收到请求: {request_str[:200]}...")
                
                # 处理请求
                response = self.process_request(request_str)
                
                # 发送响应
                response_str = json.dumps(response, ensure_ascii=False)
                client_socket.send(response_str.encode(ENCODING))
                print(f"📤 发送响应: {response_str[:200]}...")
                
        except Exception as e:
            print(f"❌ 处理客户端错误: {e}")
            print(f"🔧 DEBUG: 错误详情: {traceback.format_exc()}")
        finally:
            client_socket.close()
            print(f"🔧 DEBUG: 关闭客户端连接: {client_address}")
    
    def process_request(self, request_str: str) -> dict:
        """处理请求并返回响应"""
        print(f"🔧 DEBUG: process_request 开始")
        print(f"🔧 DEBUG: 原始请求: {request_str}")
        
        try:
            # 解析请求JSON
            request = json.loads(request_str)
            print(f"🔧 DEBUG: 解析后的请求: {request}")
            
            # 检查必需字段
            if "action" not in request or "email" not in request:
                print(f"❌ DEBUG: 缺少必要字段, request keys: {request.keys()}")
                return {
                    "success": False,
                    "error": "缺少必要字段: action 和 email",
                    "timestamp": datetime.now().isoformat()
                }
            
            action = request["action"]
            email = request["email"]
            
            print(f"🔧 DEBUG: 处理请求: {action} for {email}")
            
            # 处理不同的action
            if action == "GET_USER_DATA":
                data = self.data_manager.get_user_data(email)
                response = {
                    "success": True,
                    "action": action,
                    "email": email,
                    "data": data,
                    "timestamp": datetime.now().isoformat()
                }
                print(f"🔧 DEBUG: GET_USER_DATA 响应, 数据keys: {data.keys() if isinstance(data, dict) else '不是字典'}")
                return response
            
            elif action == "SAVE_USER_DATA":
                if "data" not in request:
                    print(f"❌ DEBUG: 缺少data字段")
                    return {"success": False, "error": "缺少data字段"}
                
                success = self.data_manager.save_user_data(email, request["data"])
                return {
                    "success": success,
                    "action": action,
                    "email": email,
                    "message": "保存成功" if success else "保存失败",
                    "timestamp": datetime.now().isoformat()
                }
            
            elif action == "GET_SYSTEM_STATUS":
                return self.data_manager.get_system_status(email)
            
            elif action == "GET_AGV_LIST":
                return self.data_manager.get_agv_list(email)
            
            elif action == "GET_TASKS":
                return self.data_manager.get_tasks(email)
            
            elif action == "GET_HISTORY":
                return self.data_manager.get_history(email)
            
            elif action == "UPDATE_AGV":
                if "agv_id" not in request or "updates" not in request:
                    print(f"❌ DEBUG: 缺少agv_id或updates字段")
                    return {"success": False, "error": "缺少agv_id或updates字段"}
                
                success = self.data_manager.update_agv(email, request["agv_id"], request["updates"])
                return {
                    "success": success,
                    "action": action,
                    "agv_id": request["agv_id"],
                    "message": "更新成功" if success else "更新失败",
                    "timestamp": datetime.now().isoformat()
                }
            
            elif action == "ADD_TASK":
                if "task_data" not in request:
                    print(f"❌ DEBUG: 缺少task_data字段")
                    return {"success": False, "error": "缺少task_data字段"}
                
                success = self.data_manager.add_task(email, request["task_data"])
                return {
                    "success": success,
                    "action": action,
                    "message": "添加成功" if success else "添加失败",
                    "timestamp": datetime.now().isoformat()
                }
            
            elif action == "UPDATE_TASK":
                if "task_id" not in request or "updates" not in request:
                    print(f"❌ DEBUG: 缺少task_id或updates字段")
                    return {"success": False, "error": "缺少task_id或updates字段"}
                
                success = self.data_manager.update_task(email, request["task_id"], request["updates"])
                return {
                    "success": success,
                    "action": action,
                    "task_id": request["task_id"],
                    "message": "更新成功" if success else "更新失败",
                    "timestamp": datetime.now().isoformat()
                }
            
            else:
                print(f"❌ DEBUG: 不支持的操作: {action}")
                return {
                    "success": False,
                    "error": f"不支持的操作: {action}",
                    "supported_actions": SUPPORTED_APIS,
                    "timestamp": datetime.now().isoformat()
                }
                
        except json.JSONDecodeError as e:
            print(f"❌ DEBUG: JSON解析错误: {e}")
            return {
                "success": False,
                "error": f"无效的JSON格式: {str(e)}",
                "timestamp": datetime.now().isoformat()
            }
        except Exception as e:
            print(f"❌ DEBUG: 处理请求失败: {e}")
            print(f"🔧 DEBUG: 错误详情: {traceback.format_exc()}")
            return {
                "success": False,
                "error": f"处理请求失败: {str(e)}",
                "timestamp": datetime.now().isoformat()
            }
    
    def stop(self):
        """停止服务器"""
        self.running = False
        if self.server_socket:
            self.server_socket.close()
        print("🛑 服务器已停止")

def main():
    """主函数"""
    print("=" * 50)
    print("🎯 AGV仓储机器人调度系统 - TCP服务端 (调试版)")
    print("=" * 50)
    
    # 检查config.py配置
    print(f"🔧 DEBUG: 检查配置...")
    print(f"🔧 DEBUG: SERVER_HOST: {SERVER_HOST}")
    print(f"🔧 DEBUG: SERVER_PORT: {SERVER_PORT}")
    print(f"🔧 DEBUG: DATA_DIR: {DATA_DIR}")
    print(f"🔧 DEBUG: ENCODING: {ENCODING}")
    print(f"🔧 DEBUG: BUFFER_SIZE: {BUFFER_SIZE}")
    
    if isinstance(DEFAULT_DATA, dict):
        print(f"🔧 DEBUG: DEFAULT_DATA 类型: dict, keys: {list(DEFAULT_DATA.keys())}")
    else:
        print(f"❌ DEBUG: DEFAULT_DATA 不是字典, 类型: {type(DEFAULT_DATA)}, 值: {DEFAULT_DATA}")
    
    server = AGVTCPServer()
    
    try:
        server.start()
    except KeyboardInterrupt:
        print("\n🛑 手动停止服务器")
        server.stop()
    except Exception as e:
        print(f"❌ 服务器运行错误: {e}")
        print(f"🔧 DEBUG: 错误详情: {traceback.format_exc()}")
        server.stop()

if __name__ == "__main__":
    main()