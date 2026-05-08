
# 已废弃 已废弃 已废弃 已废弃 已废弃 已废弃
# 已废弃 已废弃 已废弃 已废弃 已废弃 已废弃
# 已废弃 已废弃 已废弃 已废弃 已废弃 已废弃

# # # -- coding: utf-8 --
# from rbk import BasicModule, MoveStatus
# from rbkSim import SimModule
# from syspy.robot import ModuleTool, Motor, MotorType, Robot, GoodsManger
# import json


# class Module:
#     def __init__(self, r: SimModule, args):
#         self.status = MoveStatus.NONE

#         self.robotc = Robot(r)

#         r.stopMotor()

#         r.resetMotor("left")
#         r.resetMotor("right")
#         r.resetMotor("up")
#         r.resetMotor("huocha1")
#         r.resetMotor("huocha2")

#         r.setMotorCalib("up")

#         r.enableMotor("left")
#         r.enableMotor("right")
#         r.enableMotor("up")
#         r.enableMotor("huocha1")
#         r.enableMotor("huocha2")

#         r.setNotice("顶升电机复位完成")
#         r.setNotice("旋转电机复位完成")

#     # r 是 rbkSim 模块中 SimModule 类的实例对象， args 是脚本输入参数
#     def run(self, r: SimModule, args):   
#         self.status = MoveStatus.RUNNING
#         self.counter = r.getCount()

#         return self.status

#     def cancel(self, r):
#         self.counter = r.getCount()
#         # 重置标志位
#         self.motor_control_executed = False
#         self.motor_control_finished = False
#         self.status = MoveStatus.NONE

#     def suspend(self, r):
#         self.counter = r.getCount()
#         self.status = MoveStatus.SUSPENDED