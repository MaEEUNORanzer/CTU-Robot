# -- coding: utf-8 --
import json
import time
from rbk import MoveStatus, BasicModule, ParamServer
from rbkSim import SimModule
from syspy.robot import ModuleTool, Motor, MotorType, Robot, GoodsManger
"""
####BEGIN DEFAULT ARGS####
{
    "target_height": {
        "value": 0.2,
        "tips": "目标升降高度",
        "unit": "m",
        "type": "float"
    },
    "lift_speed": {
        "value": 1.0,
        "tips": "升降速度",
        "unit": "m/s",
        "type": "float"
    },
    "motor_name": {
        "value": "up",
        "tips": "升降电机名称",
        "unit": "",
        "type": "string"
    }
}
####END DEFAULT ARGS####
"""

class Module(BasicModule):
    """升降控制模块，控制机器人升降机构到达指定高度
    """
    def __init__(self, r:SimModule, args):
        super(Module, self).__init__()
        self.init = True
        self.status = MoveStatus.NONE
        self.r = r
        
        # 参数初始化
        self.target_height = 0.9
        self.lift_speed = 1.0
        self.motor_name = "up"
        
        # 控制状态
        self.lift_started = False
        self.lift_finished = False
        
        # 对象初始化
        self.robotc = None
        self.lift_motor = None

    def get_current_height(self):
        """获取当前升降高度（米）"""
        pos = ModuleTool.get_motor_pos(self.r, self.motor_name)
        if pos == -1:
            self.r.logError("无法获取升降电机位置")
            return None
        return pos

    def run(self, r:SimModule, args):
        """主函数，每个运行周期都会执行run函数

        Args:
            r (SimModule): 是MoveFactory的类，包含了基本的电机控制，状态查询，消息查询的功能
            args ([type]): 输入参数，是个json类

        Returns:
            [type]: 返回运行状态，MoveStatus,用于表明脚本的运行状态
        """
        if self.status == MoveStatus.FINISHED:
            return self.status
        
        self.status = MoveStatus.RUNNING
        
        # 初始化参数和对象
        if self.init:
            self.target_height = args.get("target_height", 0.2)
            self.lift_speed = args.get("lift_speed", 1.0)
            self.motor_name = args.get("motor_name", "up")
            
            # 创建机器人控制对象
            self.robotc = Robot(r)
            
            # 使能电机
            # r.enableMotor(self.motor_name)
            # r.enableMotor(self.motor_name)
            # r.enableMotor(self.motor_name)
            # r.enableMotor(self.motor_name)
            # r.enableMotor(self.motor_name)
            # r.enableMotor(self.motor_name)
            # r.enableMotor(self.motor_name)
            # r.enableMotor(self.motor_name)
            
            # 创建升降电机对象
            self.lift_motor = Motor(r, MotorType.LINEAR_MOTOR, self.motor_name, -1)
            
            # 获取当前位置
            cur_height = self.get_current_height()
            if cur_height is not None:
                r.logInfo(f"升降机构当前高度: {cur_height:.3f}米")
            
            r.logInfo(f"开始升降到 {self.target_height} 米")
            r.setNotice("升降控制脚本初始化完成")
            
            self.lift_started = False
            self.lift_finished = False
            self.init = False
        
        # 第一次进入，初始化状态
        if self.status == MoveStatus.RUNNING and not self.lift_started:
            self.lift_started = True
            # 重置电机状态
            self.lift_motor.status = MoveStatus.NONE
        
        # 执行升降控制
        if not self.lift_finished and self.lift_started:
            if self.robotc.lift(self.lift_motor, self.target_height, self.lift_speed):
                # 升降完成
                self.lift_finished = True
                final_height = self.get_current_height()
                if final_height is not None:
                    r.logInfo(f"升降完成，最终高度: {final_height:.3f}米")
                self.status = MoveStatus.FINISHED
            elif self.lift_motor.status == MoveStatus.FAILED:
                # 升降失败
                r.logError("升降控制失败")
                self.status = MoveStatus.FAILED
        
        return self.status

    def cancel(self, r):
        """取消升降控制"""
        r.resetMotor(self.motor_name)
        r.logInfo("已取消升降控制")
        
        # 重置所有状态
        if self.lift_motor:
            self.lift_motor.status = MoveStatus.NONE
        self.lift_started = False
        self.lift_finished = False
        self.init = True
        self.status = MoveStatus.NONE

    def suspend(self, r):
        """暂停"""
        r.logInfo("暂停升降控制")
        self.status = MoveStatus.SUSPENDED
        return self.status


if __name__ == '__main__':
    import rbkSim
    r = rbkSim.SimModule()
    m = Module(r, None)
    data = {"target_height": 0.2, "lift_speed": 1.0, "motor_name": "up"}
    print(m.run(r, data))