# -- coding: utf-8 --
import json
import math
from rbk import MoveStatus, BasicModule, ParamServer
from rbkSim import SimModule
from syspy.robot import ModuleTool, Motor, MotorType, Robot, GoodsManger
"""
####BEGIN DEFAULT ARGS####
{
    "target_angle": {
        "value": 0.0,
        "tips": "目标旋转角度",
        "unit": "degree",
        "type": "float"
    },
    "rot_speed": {
        "value": 1.5,
        "tips": "旋转速度",
        "unit": "rad/s",
        "type": "float"
    },
    "motor_name": {
        "value": "huocha1",
        "tips": "旋转电机名称",
        "unit": "",
        "type": "string"
    }
}
####END DEFAULT ARGS####
"""

class Module(BasicModule):
    """旋转控制模块，控制机器人旋转机构到达指定角度
    """
    def __init__(self, r:SimModule, args):
        super(Module, self).__init__()
        self.init = True
        self.status = MoveStatus.NONE
        self.r = r
        
        # 参数初始化
        self.target_angle = 0.0
        self.rot_speed = 2.0
        self.motor_name = "huocha1"
        
        # 控制状态
        self.rot_started = False
        self.rot_finished = False
        
        # 对象初始化
        self.robotc = None
        self.rot_motor = None

    def deg_to_rad(self, deg):
        """角度转弧度"""
        return deg * math.pi / 180
    
    def get_current_angle(self):
        """获取当前旋转角度（弧度）"""
        pos = ModuleTool.get_motor_pos(self.r, self.motor_name)
        if pos == -1:
            self.r.logError("无法获取旋转电机位置")
            return None
        return pos
    
    def rad_to_deg(self, rad):
        """弧度转角度"""
        return rad * 180 / math.pi

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
            self.target_angle = args.get("target_angle", 0.0)
            self.rot_speed = args.get("rot_speed", 1.5)
            self.motor_name = args.get("motor_name", "huocha1")
            
            # 创建机器人控制对象
            self.robotc = Robot(r)
            
            # # 使能电机

            # r.enableMotor(self.motor_name)
            # r.enableMotor(self.motor_name)
            # r.enableMotor(self.motor_name)
            # r.enableMotor(self.motor_name)
            # r.enableMotor(self.motor_name)
            # r.enableMotor(self.motor_name)
            # r.enableMotor(self.motor_name)
            
            # 创建旋转电机对象
            # 注意：这里使用了旋转电机类型（如果实际是旋转运动的话）
            # 如果是直线电机请保持 MotorType.LINEAR_MOTOR
            self.rot_motor = Motor(r, MotorType.LINEAR_MOTOR, self.motor_name, -1)
            
            # 获取当前位置
            cur_angle = self.get_current_angle()
            if cur_angle is not None:
                cur_deg = self.rad_to_deg(cur_angle)
                r.logInfo(f"旋转机构当前角度: {cur_deg:.1f}度 ({cur_angle:.3f}弧度)")
            
            target_rad = self.deg_to_rad(self.target_angle)
            r.logInfo(f"开始旋转到 {self.target_angle} 度 ({target_rad:.3f}弧度)")
            
            r.setNotice("旋转控制脚本初始化完成")
            
            self.rot_started = False
            self.rot_finished = False
            self.init = False
        
        # 第一次进入，初始化状态
        if self.status == MoveStatus.RUNNING and not self.rot_started:
            self.rot_started = True
            # 重置电机状态
            self.rot_motor.status = MoveStatus.NONE
        
        # 执行旋转控制
        if not self.rot_finished and self.rot_started:
            # 转换目标角度为弧度
            target_rad = self.deg_to_rad(self.target_angle)
            
            if self.robotc.rotate(self.rot_motor, target_rad, self.rot_speed):
                # 旋转完成
                self.rot_finished = True
                final_angle = self.get_current_angle()
                if final_angle is not None:
                    final_deg = self.rad_to_deg(final_angle)
                    r.logInfo(f"旋转完成，最终角度: {final_deg:.1f}度 ({final_angle:.3f}弧度)")
                self.status = MoveStatus.FINISHED
            elif self.rot_motor.status == MoveStatus.FAILED:
                # 旋转失败
                r.logError("旋转控制失败")
                self.status = MoveStatus.FAILED
        
        return self.status

    def cancel(self, r):
        """取消旋转控制"""
        r.resetMotor(self.motor_name)
        r.logInfo("已取消旋转控制")
        
        # 重置所有状态
        if self.rot_motor:
            self.rot_motor.status = MoveStatus.NONE
        self.rot_started = False
        self.rot_finished = False
        self.init = True
        self.status = MoveStatus.NONE
        return self.status

    def suspend(self, r):
        """暂停旋转控制"""
        r.logInfo("暂停旋转控制")
        self.status = MoveStatus.SUSPENDED
        return self.status


if __name__ == '__main__':
    import rbkSim
    r = rbkSim.SimModule()
    m = Module(r, None)
    data = {"target_angle": 90.0, "rot_speed": 1.5, "motor_name": "huocha1"}
    print(m.run(r, data))