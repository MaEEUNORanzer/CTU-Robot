# -- coding: utf-8 --
"""
####BEGIN DEFAULT ARGS####
{
    "stretch_length": {
        "value": 1.0,
        "tips": "货叉伸缩长度",
        "type": "float",
        "unit": "米",
        "min_value": 0,
        "max_value": 3.0
    }
}
####END DEFAULT ARGS####
"""

from rbk import BasicModule, MoveStatus
from rbkSim import SimModule
from syspy.robot import ModuleTool, Motor, MotorType, Robot, GoodsManger
import json
import time


class Module(BasicModule):
    def __init__(self, r: SimModule, args):
        super(Module, self).__init__()
        self.status = MoveStatus.NONE
        self.init = True
        self.stretch_length = 1.0
        self.robotc = None
        self.rotate_motor = None
        
    def run(self, r: SimModule, args):
        """主函数，每个运行周期都会执行run函数

        Args:
            r (SimModule): 是MoveFactory的类，包含了基本的电机控制，状态查询，消息查询的功能
            args (dict): 输入参数，是个json类

        Returns:
            MoveStatus: 返回运行状态，用于表明脚本的运行状态
        """
        if self.status == MoveStatus.FINISHED or self.status == MoveStatus.FAILED:
            return self.status
            
        self.status = MoveStatus.RUNNING
        
        # 初始化
        if self.init:
            # 获取参数
            self.stretch_length = args.get("stretch_length", 1.0)
            
            # 创建控制对象
            self.robotc = Robot(r)
            
            # 使能电机
            r.enableMotor("huocha2")
            self.rotate_motor = Motor(r, MotorType.LINEAR_MOTOR, "huocha2", -1)
            
            r.setNotice("控制一次标志")
            r.logInfo(f"huocha2伸缩控制脚本初始化，目标长度: {self.stretch_length}米")
            self.init = False
        
        # 执行伸缩控制
        if self.robotc.stretch(self.rotate_motor, self.stretch_length, 0.5):
            r.logInfo(f"伸缩完成，到达 {self.stretch_length}米")
            self.status = MoveStatus.FINISHED
        elif self.rotate_motor.status == MoveStatus.FAILED:
            r.logError("伸缩控制失败")
            self.status = MoveStatus.FAILED
            
        return self.status

    def cancel(self, r):
        """取消操作"""
        r.resetMotor("huocha2")
        r.logInfo("已取消伸缩控制")
        
        # 重置状态
        self.init = True
        self.status = MoveStatus.NONE
        return self.status

    def suspend(self, r):
        """暂停操作"""
        r.logInfo("暂停伸缩控制")
        self.status = MoveStatus.SUSPENDED
        return self.status


if __name__ == '__main__':
    import rbkSim
    r = rbkSim.SimModule()
    
    # # 测试1：使用默认参数
    # m = Module(r, None)
    # print("测试默认参数:", m.run(r, {}))
    
    # 测试2：使用自定义参数
    data = {"stretch_length": 1.5}
    m2 = Module(r, data)
    print("测试自定义参数:", m2.run(r, data))