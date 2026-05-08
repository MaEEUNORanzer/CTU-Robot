"""
极简版AGV系统TCP服务端配置
"""

# 服务器配置
SERVER_HOST = "127.0.0.1"
SERVER_PORT = 8888
BUFFER_SIZE = 4096
ENCODING = "utf-8"

# 数据目录
DATA_DIR = "data"

# 支持的API接口
SUPPORTED_APIS = [
    "GET_SYSTEM_STATUS",     # 获取系统状态
    "GET_AGV_LIST",         # 获取AGV列表
    "GET_TASKS",            # 获取任务列表
    "GET_HISTORY",          # 获取历史记录
    "UPDATE_AGV",           # 更新AGV状态
    "ADD_TASK",             # 添加任务
    "UPDATE_TASK",          # 更新任务
    "GET_USER_DATA",        # 获取用户所有数据
    "SAVE_USER_DATA",       # 保存用户所有数据
]

# 默认数据模板
DEFAULT_DATA = {
    "system": {
        "total_agvs": 15,
        "running_agvs": 8,
        "idle_agvs": 5,
        "warning_agvs": 2,
        "components": [
            {"name": "调度中心", "status": "online", "cpu": 42, "memory": 65, "queue": 8, "response_time": "≤50ms"},
            {"name": "通信网关", "status": "online", "connections": 15, "latency": "≤20ms", "bandwidth": 32},
            {"name": "数据存储", "status": "online", "storage": 78, "io_latency": "≤5ms", "queries": 124},
            {"name": "地图服务", "status": "warning", "last_update": "2小时前", "version": "2.3.1"}
        ]
    },
    "agvs": [
        {"id": "AGV-01", "model": "X1-2000", "status": "running", "battery": 85, "speed": 1.2, "tasks_completed": 142},
        {"id": "AGV-02", "model": "X1-2000", "status": "pending", "battery": 92, "speed": 0, "tasks_completed": 118},
        {"id": "AGV-03", "model": "X2-3000", "status": "completed", "battery": 45, "speed": 0, "tasks_completed": 203}
    ],
    "tasks": [
        {"id": "Task-001", "agv_id": "AGV-01", "from": "仓库A", "to": "货架B3", "status": "running", "progress": 65, "start_time": "12:34", "eta": "5min"},
        {"id": "Task-002", "agv_id": "AGV-02", "from": "货架C2", "to": "装载区", "status": "pending", "progress": 0, "start_time": "13:15", "eta": "12min"},
        {"id": "Task-003", "agv_id": "AGV-03", "from": "接收区", "to": "仓库D", "status": "completed", "progress": 100, "start_time": "11:50", "duration": "8min"}
    ],
    "history": [
        {"event": "任务完成", "agv_id": "AGV-01", "from_to": "仓库A → 货架B3", "status": "completed", "time": "12:34"},
        {"event": "任务开始", "agv_id": "AGV-02", "from_to": "货架C2 → 装载区", "status": "running", "time": "13:15"},
        {"event": "任务失败", "agv_id": "AGV-03", "from_to": "接收区 → 仓库D", "status": "failed", "time": "11:50"}
    ],
    "notifications": [
        {"type": "info", "message": "AGV-05电池电量低于20%", "time": "5分钟前"},
        {"type": "warning", "message": "货架B3区域拥堵", "time": "15分钟前"},
        {"type": "success", "message": "任务#004已完成", "time": "30分钟前"}
    ]
}