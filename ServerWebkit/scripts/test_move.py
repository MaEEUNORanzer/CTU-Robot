#!/usr/bin/env python3
"""
测试脚本 - 导入并使用简化版机器人控制模块
"""

# 导入机器人控制模块
import robot_controller as rc
import time

# 简单的测试
if __name__ == "__main__":
    # 向前移动1米
    # print("向前移动0.3米...")
    # result = rc.move_forward(distance=0.3, speed=0.3)
    # print(f"结果: {result}")
    
    # time.sleep(5)  # 等待1秒
    
    # 向后移动0.5米
    print("\n向后移动0.3米...")
    result = rc.move_backward(distance=0.03, speed=0.3)
    print(f"结果: {result}")