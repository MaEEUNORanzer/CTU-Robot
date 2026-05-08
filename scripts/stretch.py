# -- coding: utf-8 --
import json
from rbk import MoveStatus, BasicModule, ParamServer
from rbkSim import SimModule
from syspy.robot import ModuleTool, Motor, MotorType, Robot, GoodsManger
"""
####BEGIN DEFAULT ARGS####
{
    "target_length": {
        "value": 0.0,
        "tips": "目标伸缩长度/位置",
        "unit": "m",
        "type": "float"
    },
    "stretch_speed": {
        "value": 1.0,
        "tips": "伸缩速度",
        "unit": "m/s",
        "type": "float"
    },
    "motor_name": {
        "value": "huocha2",
        "tips": "伸缩电机名称",
        "unit": "",
        "type": "string"
    }
}
####END DEFAULT ARGS####
"""

class Module(BasicModule):
    """伸缩控制模块，控制机器人伸缩机构到达指定位置
    """
    def __init__(self, r:SimModule, args):
        super(Module, self).__init__()
        self.init = True
        self.status = MoveStatus.NONE
        self.r = r
        
        # 参数初始化
        self.target_length = 0.0
        self.stretch_speed = 1.0
        self.motor_name = "huocha2"
        
        # 控制状态
        self.stretch_started = False
        self.stretch_finished = False
        
        # 对象初始化
        self.robotc = None
        self.stretch_motor = None

    def get_current_length(self):
        """获取当前伸缩长度/位置（米）"""
        pos = ModuleTool.get_motor_pos(self.r, self.motor_name)
        if pos == -1:
            self.r.logError("无法获取伸缩电机位置")
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
            self.target_length = args.get("target_length", 0.0)
            self.stretch_speed = args.get("stretch_speed", 1.0)
            self.motor_name = args.get("motor_name", "huocha2")
            
            # 创建机器人控制对象
            self.robotc = Robot(r)
            
            # 使能电机
            # r.enableMotor(self.motor_name)
            # r.enableMotor(self.motor_name)
            # r.enableMotor(self.motor_name)
            # r.enableMotor(self.motor_name)
            # r.enableMotor(self.motor_name)
            
            # 创建伸缩电机对象
            # 如果是伸缩机构，通常使用直线电机
            self.stretch_motor = Motor(r, MotorType.LINEAR_MOTOR, self.motor_name, -1)
            
            # 获取当前位置
            cur_length = self.get_current_length()
            if cur_length is not None:
                r.logInfo(f"伸缩机构当前位置: {cur_length:.3f}米")
            
            r.logInfo(f"开始伸缩到 {self.target_length} 米，速度: {self.stretch_speed}")
            
            r.setNotice("伸缩控制脚本初始化完成")
            
            self.stretch_started = False
            self.stretch_finished = False
            self.init = False
        
        # 第一次进入，初始化状态
        if self.status == MoveStatus.RUNNING and not self.stretch_started:
            self.stretch_started = True
            # 重置电机状态
            self.stretch_motor.status = MoveStatus.NONE
        
        # 执行伸缩控制
        if not self.stretch_finished and self.stretch_started:
            if self.robotc.stretch(self.stretch_motor, self.target_length, self.stretch_speed):
                # 伸缩完成
                self.stretch_finished = True
                final_length = self.get_current_length()
                if final_length is not None:
                    r.logInfo(f"伸缩完成，最终位置: {final_length:.3f}米")
                self.status = MoveStatus.FINISHED
            elif self.stretch_motor.status == MoveStatus.FAILED:
                # 伸缩失败
                r.logError("伸缩控制失败")
                self.status = MoveStatus.FAILED
        
        return self.status

    def cancel(self, r):
        """取消伸缩控制"""
        r.resetMotor(self.motor_name)
        r.logInfo("已取消伸缩控制")
        
        # 重置所有状态
        if self.stretch_motor:
            self.stretch_motor.status = MoveStatus.NONE
        self.stretch_started = False
        self.stretch_finished = False
        self.init = True
        self.status = MoveStatus.NONE
        return self.status

    def suspend(self, r):
        """暂停伸缩控制"""
        r.logInfo("暂停伸缩控制")
        self.status = MoveStatus.SUSPENDED
        return self.status


if __name__ == '__main__':
    import rbkSim
    r = rbkSim.SimModule()
    m = Module(r, None)
    data = {"target_length": 0.5, "stretch_speed": 1.0, "motor_name": "huocha2"}
    print(m.run(r, data))