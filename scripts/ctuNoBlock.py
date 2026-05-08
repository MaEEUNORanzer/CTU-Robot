# -*- coding: utf-8 -*-
# @Time : 2022/10/18
# @Author : huang, zhong
# @Version : 2.2.6
# @Support : rbk  3.3.5.68 +
# @Update : 增加取货扫码核对功能

import json
import sys
import time

sys.path.append("../syspy")
from rbkSim import SimModule
from rbk import MoveStatus, BasicModule, normalize_theta, ParamServer
from robot import NetHandle
from pickingRobot import PickRobot
import pickingRobot
import math
import goPath as goPath
import requests

SCRIPT_VERSION = "V2.2.6 - 20221018"
"""
####BEGIN DEFAULT ARGS####
{
    "lift": {
        "value": 0,
        "tips": "pos",
        "type": "int",
        "max_value":1850,
        "min_value":380,
        "unit": "mm"
    },
    "recBoxLift":{
        "value": 0,
        "tips": "卸货时，识别料箱时，货叉的高度",
        "type": "int",
        "max_value":1850,
        "min_value":380,
        "unit": "mm"        
    },
    "rotate": {
        "value": 0,
        "tips": "theta",
        "type": "double",
        "unit": "rad"
    },    
    "stretch": {
        "value": 0,
        "tips": "pos",
        "type": "int",
        "unit": "mm"
    }, 
    "finger": {
        "value": 0,
        "tips": "1: open, 0: close",
        "max_value":1,
        "min_value":0,
        "type": "int"
    }, 
    "visionType": {
        "value": "shelf",
        "default_value":[
        "shelf","box", "reset"
        ],
        "tips": "tips",
        "type": "complex"
    },
    "visionBinType":{
        "value": "code",
        "default_value":[
            "code","markerless", "barcode"
        ],
        "tips":"货物识别类型",
        "type": "complex"
    },
    "binModel":{
        "value":"plasticbox",
        "default_value":[
            "carton","plasticbox"
        ],
        "tips":"料箱种类，当visionBinType为markerless时需要填写这项",
        "type": "complex"        
    },
    "chassisLedFront": {
        "value": 0,
        "tips": "1: open, 0: close",
        "type": "int"
    }, 
    "chassisLedBack": {
        "value": 0,
        "tips": "1: open, 0: close",
        "type": "int"
    }, 
    "buzzer": {
        "value": 0,
        "tips": "0: off 1: fast_blink 2: slow_blink 3: hold",
        "type": "int"
    }, 
    "headLedRed": {
        "value": 0,
        "tips": "1: open, 0: close",
        "type": "int"
    }, 
    "headLedYellow": {
        "value": 0,
        "tips": "1: open, 0: close",
        "type": "int"
    }, 
    "headLedGreen": {
        "value": 0,
        "tips": "1: open, 0: close",
        "type": "int"
    }, 
    "headLedFreq": {
        "value": 0,
        "tips": "0: off 1: fast_blink 2: slow_blink 3: hold",
        "type": "int"
    },
    "operation":{
        "value": "wait",
        "default_value":[
        "load","unload","change","zero","wait","getMarkerPos","put"
        ],
        "tips": "tips",
        "type": "complex"        
    },
    "selfPosition":{
        "value": 0,
        "tips": "pos",
        "type": "int"
    },
    "changePosition0":{
        "value": 0,
        "tips": "pos",
        "type": "int"
    },
    "putPosition":{
        "value": 0,
        "tips": "pos",
        "type": "int"
    },    
    "changePosition1":{
        "value": 0,
        "tips": "pos",
        "type": "int"
    },
    "recAdjust":{
        "value":1,
        "tips":"",
        "type":"int"
    },
    "useLoc":{
        "value":1,
        "tips":"",
        "type":"int",
        "max_value":1,
        "min_value":1
    },
    "unloadHeight":{
        "value": 0,
        "tips": "rec_offz_shelf",
        "type": "double",
        "unit": "mm"
    },
    "loadHeight":{
        "value": 0,
        "tips": "rec_offz_box",
        "type": "double",
        "unit": "mm"
    },
    "barcodeHeight":{
        "value": 0,
        "tips": "一维码高度",
        "type": "double"
    },
    "goodsId": {
        "value": "",
        "type": "string"
    },
    "postAddr":{
        "value":"http://ip:port/api/returnBoxComplete",
        "type":"string"
    },
    "postData":{
        "value":{},
        "type":"string"
    },
    "callTerminalURL": {
        "value":"http://ip:8088/callTerminal/getTerminalStatus",
        "type":"string"
    },
    "callTerminalData": {
        "value":{
            "reach": {
                "address": 905,
                "functionCode": 6,
                "id": "TK02",
                "type": "writeAddr",
                "value": 1
            },
            "action":{
                "address": 803,
                "functionCode": 3,
                "id": "TK02",
                "type": "readAddr"
            },
            "finish":{
                "address": 906,
                "functionCode": 6,
                "id": "TK02",
                "type": "writeAddr",
                "value": 1
            },
            "reset":[
                {
                    "address": 905,
                    "functionCode": 6,
                    "id": "TK02",
                    "type": "writeAddr",
                    "value": 0
                },
                {
                    "address": 906,
                    "functionCode": 6,
                    "id": "TK02",
                    "type": "writeAddr",
                    "value": 0
                }
            ]
        },
        "type":"json"
    }
}
####END DEFAULT ARGS####
"""


def matrixDot(a, b, m, l, n):
    """ c = a * b

    Args:
        a ([type]): 2d matrix m * l
        b ([type]): 2d matrix l * n
        m ([type]): row
        l ([type]): col
        n ([type]): row

    Returns:
        [type]: 2d matrix
    """
    out = [[0] * n for i in range(m)]
    for i in range(m):
        for j in range(n):
            v = 0
            for k in range(l):
                v = v + a[i][k] * b[k][j]
            out[i][j] = v
    return out


def matrixReshape(a, m, n):
    """[summary]

    Args:
        a ([type]): input matrix
        m ([type]): row num
        n ([type]): col num

    Returns:
        [type]: 2d matrix
    """
    out = [[0] * n for i in range(m)]
    mid = []
    for i in range(len(a)):
        for j in range(len(a[i])):
            mid.append(a[i][j])
    for i in range(m):
        for j in range(n):
            out[i][j] = mid[i * n + j]
    return out


def TMatrixReverse(a):
    """transition matrix reverse

    Args:
        a ([type]): 2d 4 * 4 transition matrix

    Returns:
        [type]: 2d  4 * 4 transitionmatrix
    """
    rMatrix = [[0] * 3 for i in range(3)]
    for i in range(3):
        for j in range(3):
            rMatrix[i][j] = a[j][i]
    org_t = [[a[i][3]] for i in range(3)]
    tMatrix = [[0] for i in range(3)]
    tMatrix = matrixDot(rMatrix, org_t, 3, 3, 1)
    for i in range(3):
        tMatrix[i][0] = -tMatrix[i][0]
    out = [[0] * 4 for i in range(4)]
    for i in range(3):
        for j in range(3):
            out[i][j] = rMatrix[i][j]
    for i in range(3):
        out[i][3] = tMatrix[i][0]
    out[3][3] = 1
    return out


def getYPRZYX(p):
    yaw = math.atan2(p[4], p[0])
    pitch = math.atan2(-p[8], math.sqrt(p[9] * p[9] + p[10] * p[10]))
    roll = math.atan2(p[9], p[10])
    dx = p[3]
    dy = p[7]
    dz = p[11]
    return yaw, pitch, roll, dz, dy, dx


def getYPRZYX_code(p, theta):
    # 货叉相对于agv
    Tagv2f = [[math.cos(theta), -math.sin(theta), 0., 0.],
              [math.sin(theta), math.cos(theta), 0., 0.],
              [0., 0., 1., 0.],
              [0., 0., 0., 1.]]
    # 相机相对于货叉
    Tf2c = [[0., 0., 1., 0.262],
            [0., -1., 0., 0.],
            [1., 0., 0., -0.029],
            [0., 0., 0., 1.]]
    # 料箱相对于二维码
    Tm2box = [[0., -1., 0., 0.],
              [0., 0., 1., 0.],
              [-1., 0., 0., 0.],
              [0., 0., 0., 1.]]
    # 二维码相对于相机
    Tc2m_now = matrixReshape(p, 4, 4)
    # 二维码相对于货叉
    Tf2m_now = matrixDot(Tf2c, Tc2m_now, 4, 4, 4)
    # newp = matrixReshape(Tf2m_now, 1, 16)[0]
    # yaw, pitch, roll, dz, dy, dx = getYPRZYX(newp)
    # 二维码相对于小车
    Tagv2m_now = matrixDot(Tagv2f, Tf2m_now, 4, 4, 4)
    # 料箱相对于小车
    Tagv2box = matrixDot(Tagv2m_now, Tm2box, 4, 4, 4)
    # 料箱相对于货叉
    Tf2box = matrixDot(Tf2m_now, Tm2box, 4, 4, 4)
    # 料箱的 [1，0，0]
    boxX = [[1.], [0.], [0.], [1.]]
    # 料箱的 [1,0,0] 在 小车 下的坐标
    boxX2agv = matrixDot(Tagv2box, boxX, 4, 4, 1)

    # 
    angle_agv = math.atan2(boxX2agv[1][0] - Tagv2box[1][3], boxX2agv[0][0] - Tagv2box[0][3])
    dx = Tagv2box[0][3]
    dy = Tagv2box[1][3]
    dz = Tagv2box[2][3]

    # pf2box， 料箱相对于最终货叉坐标系中的位置
    pf2box_ideal = [[Tf2box[0][3]], [0], [Tf2box[2][3]], [1]]
    print(pf2box_ideal)
    # 最终货叉相对于小车坐标下的位姿
    Tagv2f_ideal = [[math.cos(angle_agv), -math.sin(angle_agv), 0., 0.],
                    [math.sin(angle_agv), math.cos(angle_agv), 0., 0.],
                    [0., 0., 1., 0.],
                    [0., 0., 0., 1.]]
    
    # 料箱相对于最终小车的位姿
    pagv2box_ideal = matrixDot(Tagv2f_ideal, pf2box_ideal, 4, 4, 1)
    dist = Tagv2box[0][3] - pagv2box_ideal[0][0]
    return angle_agv, dx, dy, dz, dist, Tagv2box


def getMarkerInWorld(loc, Tagv2m):
    Tw2agv = [[math.cos(loc[2]), -math.sin(loc[2]), 0., loc[0]],
              [math.sin(loc[2]), math.cos(loc[2]), 0., loc[1]],
              [0., 0., 1., 0.],
              [0., 0., 0., 1.]]
    Tw2m = matrixDot(Tw2agv, Tagv2m, 4, 4, 4)
    return Tw2m


def getYPRZYX_markerless(p, theta):
    Tagv2f = [[math.cos(theta), -math.sin(theta), 0., 0.],
              [math.sin(theta), math.cos(theta), 0., 0.],
              [0., 0., 1., 0.],
              [0., 0., 0., 1.]]
    Tf2ifm = [[1., 0., 0., -0.281],
              [0., 1., 0., 0.],
              [0., 0., 1., 0.218],
              [0., 0., 0., 1.]]
    Tifm2box = matrixReshape(p, 4, 4)
    Tf2box = matrixDot(Tf2ifm, Tifm2box, 4, 4, 4)
    # newp = matrixReshape(Tf2box, 1,16)[0]
    # yaw, pitch, roll, dz, dy, dx = getYPRZYX(newp)
    Tagv2box = matrixDot(Tagv2f, Tf2box, 4, 4, 4)
    boxX = [[1.], [0.], [0.], [1.]]
    boxX2agv = matrixDot(Tagv2box, boxX, 4, 4, 1)
    angle_agv = math.atan2(boxX2agv[1][0] - Tagv2box[1][3], boxX2agv[0][0] - Tagv2box[0][3])
    dx = Tagv2box[0][3]
    dy = Tagv2box[1][3]
    dz = Tagv2box[2][3]
    # pf2box， box在理想货叉坐标系中的位置
    pf2box_ideal = [[Tf2box[0][3]], [0], [Tf2box[2][3]], [1]]
    Tagv2f_ideal = [[math.cos(angle_agv), -math.sin(angle_agv), 0., 0.],
                    [math.sin(angle_agv), math.cos(angle_agv), 0., 0.],
                    [0., 0., 1., 0.],
                    [0., 0., 0., 1.]]
    pagv2box_ideal = matrixDot(Tagv2f_ideal, pf2box_ideal, 4, 4, 1)
    dist = Tagv2box[0][3] - pagv2box_ideal[0][0]
    return angle_agv, dx, dy, dz, dist, Tagv2box


class Module(BasicModule):
    def __init__(self, r: SimModule, args):
        super(Module, self).__init__()
        self.status = MoveStatus.RUNNING
        p = ParamServer(__file__)
        ip = p.loadParam("ip", type="str", default="192.168.192.20", comment="ip addr")
        port = p.loadParam("port", type="int", default=4172, maxValue=999999, minValue=0, comment="port")
        self.start_connect_time = time.time()
        self.max_connect_time = p.loadParam("max_connect_time", type="int", default=30, maxValue=999999, minValue=0,
                                            comment="连接等待最长时间s")
        self.h = PickRobot(ip, port)
        self.lift_reach_dist = p.loadParam("lift_reach_dist", type="float", default=0.5, maxValue=10.0, minValue=0.0,
                                           unit="mm", comment="升降机构到位精度")
        self.rotate_reach_angle = p.loadParam("rotate_reach_angle", type="float", default=0.01, maxValue=10.0,
                                              minValue=0.0, unit="rad", comment="旋转到位精度")
        self.stretch_reach_dist = p.loadParam("stretch_reach_dist", type="float", default=0.5, maxValue=10.0,
                                              minValue=0.0, unit="mm", comment="伸缩到位精度")
        self.maxStretchDist = p.loadParam("max_stretch_dist", type="float", default=920.0, maxValue=10000.0,
                                          minValue=0.0, unit="mm", comment="伸缩臂最大伸出长度")
        self.init = True
        self.task = dict()
        self.low = dict({0: 740, 1: 1130, 2: 1520, 3: 1910, 4: 2300})  # mm
        # 此处在背篓取货时需要略低于背篓的高度，此处所更改的数值为默认值，需要在"ctuNoBlock.json"文件里修改才是最终执行的高度
        self.low[0] = p.loadParam("low0", type="float", default=400.0, maxValue=10000.0, minValue=0.0, unit="mm",
                                  comment="取货时，第0层高度")
        self.low[1] = p.loadParam("low1", type="float", default=845.0, maxValue=10000.0, minValue=0.0, unit="mm",
                                  comment="取货时，第1层高度")
        self.low[2] = p.loadParam("low2", type="float", default=1295.0, maxValue=10000.0, minValue=0.0, unit="mm",
                                  comment="取货时，第2层高度")
        self.low[3] = p.loadParam("low3", type="float", default=1745.0, maxValue=10000.0, minValue=0.0, unit="mm",
                                  comment="取货时，第3层高度")
        self.low[4] = p.loadParam("low4", type="float", default=2195.0, maxValue=10000.0, minValue=0.0, unit="mm",
                                  comment="取货时，第4层高度")
        self.low[5] = p.loadParam("low5", type="float", default=2645.0, maxValue=10000.0, minValue=0.0, unit="mm",
                                  comment="取货时，第5层高度")
        self.low[999] = p.loadParam("low999", type="float", default=0.0, maxValue=10000.0, minValue=0.0, unit="mm",
                                    comment="抓斗取货")
        self.high = dict({0: 407, 1: 917, 2: 1427, 3: 1937, 4: 2447, 5: 2957})  # mm
        # 此处在背篓放货时需要略高于背娄的高度，此处所更改的数值为默认值，需要在"ctuNoBlock.json"文件里修改才是最终执行的高度
        self.high[0] = p.loadParam("high0", type="float", default=405.0, maxValue=10000.0, minValue=0.0, unit="mm",
                                   comment="放货时，第0层高度")
        self.high[1] = p.loadParam("high1", type="float", default=855.0, maxValue=10000.0, minValue=0.0, unit="mm",
                                   comment="放货时，第1层高度")
        self.high[2] = p.loadParam("high2", type="float", default=1305.0, maxValue=10000.0, minValue=0.0, unit="mm",
                                   comment="放货时，第2层高度")
        self.high[3] = p.loadParam("high3", type="float", default=1755.0, maxValue=10000.0, minValue=0.0, unit="mm",
                                   comment="放货时，第3层高度")
        self.high[4] = p.loadParam("high4", type="float", default=2205.0, maxValue=10000.0, minValue=0.0, unit="mm",
                                   comment="放货时，第4层高度")
        self.high[5] = p.loadParam("high5", type="float", default=2655.0, maxValue=10000.0, minValue=0.0, unit="mm",
                                   comment="放货时，第5层高度")
        self.high[999] = p.loadParam("high999", type="float", default=0.0, maxValue=10000.0, minValue=0.0, unit="mm",
                                     comment="抓斗放货")
        # 此处修改的是默认值，最终执行请在“ctuNoBlock.json"里进行更改
        self.stretchDist = p.loadParam("stretchDist", type="float", default=752, maxValue=10000.0, minValue=0.0,
                                       unit="mm", comment="放在自己货架上，伸缩臂伸出长度")
        self.rec_offz_box = p.loadParam("rec_offz_box", type="float", default=-85.0, maxValue=1000.0, minValue=-1000.0,
                                        unit="mm", comment="识别货物后，抓货物时高度的调整距离，根据料箱二维码高度调整")
        self.rec_offz_shelf = p.loadParam("rec_offz_shelf", type="float", default=50.0, maxValue=1000.0,
                                          minValue=-1000.0, unit="mm", comment="识别货架后，放货物时高度的调整距离，根据货架二维码高度调整")
        self.fork_up_limit = p.loadParam("fokr_up_limit", type="int", default=4, maxValue=100, minValue=-1, unit="",
                                         comment="货叉上限位DI")
        self.fork_down_limit = p.loadParam("fork_down_limit", type="int", default=2, maxValue=100, minValue=-1, unit="",
                                           comment="货叉下限位DI")
        self.minLiftHeight = p.loadParam("min_fork_height", type="float", default=400.0, maxValue=10000.0, minValue=0.0,
                                         unit="mm", comment="货叉最低高度")
        self.maxLiftHeight = p.loadParam("max_fork_height", type="float", default=1940.0, maxValue=10000.0,
                                         minValue=0.0, unit="mm", comment="货叉最大高度")
        self.fork_limit = p.loadParam("fork_limit", type="int", default=1, maxValue=100, minValue=-1, unit="",
                                      comment="货叉机械限位限位DI")
        self.loadOffset = p.loadParam("loadOffset", type="float", default=0.0, maxValue=500.0, minValue=-500.0,
                                      unit="mm", comment="load货物时，货叉额外伸出的距离")
        self.has_fork_sensor = p.loadParam("forkSensor", type="int", default=0, comment="货叉是否有货物检测传感器，1为有，0为无")
        self.has_tray_sensor = p.loadParam("traySensor", type="int", default=0, comment="背篓是否有货物检测传感器，1为有，0为无")
        self.trays_num = p.loadParam("traysNum", type="int", default=3, comment="背篓层数")
        self.auto_switch_mode = p.loadParam("AutoSwitchMode", type="int", default=1, comment="是否自动切换到机构模式，1为是，0为否")

        r.logInfo(f"init args: {args}")
        self.stretch_status = MoveStatus.NONE
        self.lift_status = MoveStatus.NONE
        self.rotate_status = MoveStatus.NONE
        self.finger_status = MoveStatus.NONE
        self.vision_status = MoveStatus.NONE
        self.indicator_status = MoveStatus.NONE
        self.operation_status = MoveStatus.NONE
        self.getMarkerPos_status = 0  # 0识别获得了位姿， 1获得id, 2保持结果TotTime
        self.getMarkerPos_data = dict()
        self.getMarkerPosTotTime = 5.0
        self.getMarkerStartTime = time.time()
        self.task_list = []
        self.task_id = 0
        self.state = dict()
        self.fork_detect = self.init_forksDetects(r, 2)
        self.tray_detect = self.init_trays(r)
        self.goods_id = ""
        self.tray_floor = None
        self.goPath = goPath.Module(r, args)
        self.waitVision = waitVision()
        self.waitVision.status = MoveStatus.NONE
        self.unloadHeight = self.rec_offz_shelf
        self.loadHeight = self.rec_offz_box
        self.h.connect()
        self.postdata = None
        self.call_terminal = None
        self.barcode_height = None

    def run(self, r: SimModule, args):

        if r.errorExits(52111):
            self.status = MoveStatus.FAILED
            return self.status.value
        self.status = MoveStatus.RUNNING
        if self.init:
            self.init = False
            self.task = args
            self.barcode_height = args.get("barcodeHeight", None)
            if "unloadHeight" in self.task:
                self.unloadHeight = self.task["unloadHeight"]
            if "loadHeight" in self.task:
                self.loadHeight = self.task["loadHeight"]

            if "callTerminalURL" in self.task and "callTerminalData" in self.task:
                addr = self.task.get("callTerminalURL", None)
                data = self.task.get("callTerminalData", None)
                self.call_terminal = CallTerminal(addr, data)
                r.logInfo(f"addr: {addr}, data: {data}")

            if "postAddr" in self.task and type(self.task["postAddr"]) is str and self.task["postAddr"] != "":
                addr = self.task["postAddr"]
                data = self.task.get("postData", "")
                self.postdata = PostData(addr, data)
                r.logDebug("init post {}, {}".format(addr, data))
                self.postdata.reset()

            # 更新goodsId
            try:
                if "goodsId" in args:
                    self.goods_id = args["goodsId"]
                self.get_goodsId(r)
            except Exception as e:
                r.setPickRobotWarning(55801, f"Please update rbk version: {e}")

        if not self.h.isconnect:
            self.state["init"] = self.h.initDevice(r)
            self.state["warning"] = "ctu is connecting!!!!"
            str_state = json.dumps(self.state)
            r.setInfo(str_state)
            r.logDebug(str_state)
            dtime = time.time() - self.start_connect_time
            # if dtime > self.max_connect_time:
            #     r.setPickRobotWarning(55802, "ctu connect is overtime: {}s".format(self.max_connect_time))
            if dtime > 60:
                r.setPickRobotError(53839, f"ctu connect is overtime: 60s")
                self.status = MoveStatus.FAILED
        if self.status is not MoveStatus.FINISHED:
            self.state = self.h.getReport(r)
            self.state["goodsId"] = self.goods_id
            try:
                rbk_version = r.robokitVersion()
                self.state['rbk version'] = rbk_version
                # 检测初始模式，若为任务模式，则将其转变为机构模式
                if "mode" in self.state and bool(self.auto_switch_mode):
                    r.clearWarning(55803)
                    if self.state.get("mode", 1) == 0:
                        self.h.switch_mode(r, 1)
                        if self.h.switch_mode_res['status'] != pickingRobot.Action.FINISHED:
                            return self.status
                elif "mode" not in self.state and bool(self.auto_switch_mode):
                    r.setPickRobotWarning(55803, "mode checking, ctu is connecting! ")
                    return self.status
            except Exception as e:
                r.setPickRobotWarning(55804, f"please update rbk version: {e}")
            if "connect_error" in self.state:
                dtime = time.time() - self.start_connect_time
                if dtime > self.max_connect_time:
                    r.setPickRobotError(53837, "command response time out: {}s".format(self.max_connect_time))
                    self.status = MoveStatus.FAILED
                    str_state = json.dumps(self.state)
                    r.setInfo(str_state)
                    r.logDebug(str_state)
                    r.logInfo(str_state)
                    return self.status
            else:
                self.start_connect_time = time.time()
            self.state["task"] = self.task
            r.logDebug("first in script {}".format(self.task.get("_script_first_run_", False)))
            if self.task.get("_script_first_run_", False) is True:
                self.vision_status = MoveStatus.FINISHED
                self.zero(r)
            elif "operation" in self.task and self.task["operation"] == "load":
                if "lift" in self.task and "rotate" in self.task and "stretch" in self.task:
                    self.load(r)
                else:
                    r.setPickRobotError(53800, "Task is wrong : {}".format(json.dumps(self.task)))
                    self.operation_status = MoveStatus.FAILED
            elif "operation" in self.task and self.task["operation"] == "unload":
                if "lift" in self.task and "rotate" in self.task and "stretch" in self.task:
                    self.unload(r)
                else:
                    r.setPickRobotError(53800, "Task is wrong : {}".format(json.dumps(self.task)))
                    self.operation_status = MoveStatus.FAILED
            elif "operation" in self.task and self.task["operation"] == "change":
                if "changePosition0" in self.task and "changePosition1" in self.task:
                    self.vision_status = MoveStatus.FINISHED
                    self.changePos(r)
                else:
                    r.setPickRobotError(53800, "task is wrong : {}".format(json.dumps(self.task)))
                    self.operation_status = MoveStatus.FAILED
            elif "operation" in self.task and self.task["operation"] == "put":
                if "putPosition" in self.task:
                    self.vision_status = MoveStatus.FINISHED
                    self.putPos(r)
                else:
                    r.setPickRobotError(53800, "Task is wrong : {}".format(json.dumps(self.task)))
                    self.operation_status = MoveStatus.FAILED
            elif "operation" in self.task and self.task["operation"] == "zero":
                self.vision_status = MoveStatus.FINISHED
                self.zero(r)
            elif "operation" in self.task and self.task["operation"] == "getMarkerPos":
                self.stretch_status = MoveStatus.FINISHED
                self.lift_status = MoveStatus.FINISHED
                self.rotate_status = MoveStatus.FINISHED
                self.finger_status = MoveStatus.FINISHED
                self.indicator_status = MoveStatus.FINISHED
                self.getMarkerPos(r)
            elif "recAdjust" in self.task:
                self.stretch_status = MoveStatus.FINISHED
                self.finger_status = MoveStatus.FINISHED
                self.indicator_status = MoveStatus.FINISHED
                if "visionType" in self.task:
                    if "visionBinType" not in self.task:
                        self.task["visionBinType"] = "code"
                    self.rec(r)
                else:
                    r.setPickRobotError(53801, "Rec task is wrong : {}".format(json.dumps(self.task)))
                    self.operation_status = MoveStatus.FAILED
            else:
                self.operation_status = MoveStatus.FINISHED
                if "lift" in self.task:
                    self.lift(r, self.task["lift"])
                else:
                    self.lift_status = MoveStatus.FINISHED
                if "rotate" in self.task:
                    self.rotate(r, self.task["rotate"])
                else:
                    self.rotate_status = MoveStatus.FINISHED
                if "stretch" in self.task:
                    self.stretch(r, self.task["stretch"])
                else:
                    self.stretch_status = MoveStatus.FINISHED
                if "finger" in self.task:
                    self.finger(r, self.task["finger"])
                else:
                    self.finger_status = MoveStatus.FINISHED
                if "visionType" in self.task and self.lift_status is MoveStatus.FINISHED and self.rotate_status is MoveStatus.FINISHED:
                    self.vision(r, self.task["visionType"], self.task.get("visionBinType", "code"),
                                self.task.get("binModel", "plasticbox"))
                else:
                    self.vision_status = MoveStatus.FINISHED

            # 指示灯
            chassisLedFront, chassisLedBack, buzzer, headLedYellow, headLedRed, headLedGreen, headLedFreq = None, None, None, None, None, None, None
            if "chassisLedFront" in self.task:
                chassisLedFront = self.task["chassisLedFront"]
            if "chassisLedBack" in self.task:
                chassisLedBack = self.task["chassisLedBack"]
            if "buzzer" in self.task:
                buzzer = self.task["buzzer"]
            if "headLedRed" in self.task:
                headLedRed = self.task["headLedRed"]
            if "headLedYellow" in self.task:
                headLedYellow = self.task["headLedYellow"]
            if "headLedGreen" in self.task:
                headLedGreen = self.task["headLedGreen"]
            if "headLedFreq" in self.task:
                headLedFreq = self.task["headLedFreq"]
            if chassisLedFront is not None \
                    or chassisLedBack is not None \
                    or buzzer is not None \
                    or headLedYellow is not None \
                    or headLedRed is not None \
                    or headLedGreen is not None \
                    or headLedFreq is not None:
                self.indicator(r, chassisLedFront, chassisLedBack, buzzer, headLedYellow, headLedRed, headLedGreen,
                               headLedFreq)
            else:
                self.indicator_status = MoveStatus.FINISHED
            if self.lift_status == MoveStatus.FAILED or \
                    self.rotate_status == MoveStatus.FAILED or \
                    self.stretch_status == MoveStatus.FAILED or \
                    self.finger_status == MoveStatus.FAILED or \
                    self.indicator_status == MoveStatus.FAILED or \
                    self.vision_status == MoveStatus.FAILED or \
                    self.operation_status == MoveStatus.FAILED:
                self.status = MoveStatus.FAILED
                self.stop(r)
            elif self.lift_status == MoveStatus.FINISHED and \
                    self.rotate_status == MoveStatus.FINISHED and \
                    self.stretch_status == MoveStatus.FINISHED and \
                    self.finger_status == MoveStatus.FINISHED and \
                    self.indicator_status == MoveStatus.FINISHED and \
                    self.vision_status == MoveStatus.FINISHED and \
                    self.operation_status == MoveStatus.FINISHED:
                self.status = MoveStatus.FINISHED
            else:
                self.status = MoveStatus.RUNNING
        if "_script_first_run_" in self.task and self.task[
            "_script_first_run_"] is True and self.status is MoveStatus.FINISHED:
            self.task["_script_first_run_"] = False
            self.status = MoveStatus.RUNNING
            self.operation_status = MoveStatus.NONE

        if self.status == MoveStatus.FINISHED:
            if self.postdata is not None:
                self.postdata.run(r, self)
                if self.postdata.status == MoveStatus.FINISHED:
                    self.status = MoveStatus.FINISHED
                elif self.postdata.status == MoveStatus.FAILED:
                    self.status = MoveStatus.FAILED
                else:
                    self.status = MoveStatus.RUNNING

        movestate = dict()
        movestate["lift"] = self.lift_status
        movestate["rotate"] = self.rotate_status
        movestate["stretch"] = self.stretch_status
        movestate["finger"] = self.finger_status
        movestate["indicator"] = self.indicator_status
        movestate["vision"] = self.vision_status
        movestate["operation"] = self.operation_status
        movestate["status"] = self.status
        self.state["MoveStatus"] = movestate

        if "res" in self.state:
            try:
                execution_result = self.state['res']['res'].get('executionResult', 0)
                if execution_result != 0:
                    fail_description = self.state['res']['res'].get('failDescription', '')
                    if fail_description != 'detect failed':
                        r.setPickRobotError(53802,
                                            f"Report data has error, fail description: {fail_description}, execution result: {hex(execution_result)}")
                        self.status = MoveStatus.FAILED
            except KeyError as e:
                r.logDebug("KeyError: " + str(e))
            except Exception as e:
                r.logDebug(f"error in report info state,{e}")

        try:
            r.logDebug("[HaiRou][{}|{}|{}|{}|{}|{}|{}|{}|{}|{}|{}|{}|{}]".format(
                self.state["lift"]["position"], self.state["lift"]["state"], self.state["lift"]["speed"],
                self.state["rotate"]["position"], self.state["rotate"]["state"], self.state["rotate"]["speed"],
                self.state["stretch"]["position"], self.state["stretch"]["state"], self.state["stretch"]["speed"],
                self.state["finger"]["leftStatus"], self.state["finger"]["rightStatus"], self.state["finger"]["state"],
                self.state["vision"]["state"]))
        except KeyError as e:
            r.logDebug("KeyError: " + str(e))
        except Exception as e:
            r.logDebug(f"Other error in print report info state: {e}")

        try:
            if "forkDetect" in self.state:
                log_key = "HaiRou_forkDetect"
                log_str = ""
                for fork in self.state["forkDetect"]:
                    tmp_str = "{}|{}|{}|".format(fork["id"], fork["state"], fork["state"])
                    log_str = log_str + tmp_str
                r.logDebug("[{}][{}]".format(log_key, log_str))
        except KeyError as e:
            r.logDebug("KeyError: " + str(e))
        except Exception as e:
            r.logDebug(f"Other error in print report info forkDetect state: {e}")

        try:
            if "trays" in self.state:
                log_key = "HaiRou_trays"
                log_str = ""
                for trays in self.state["trays"]:
                    tmp_str = "{}|{}|{}|".format(trays["id"], trays["state"], trays["state"])
                    log_str = log_str + tmp_str
                r.logDebug("[{}][{}]".format(log_key, log_str))
        except KeyError as e:
            r.logDebug("KeyError: " + str(e))
        except Exception as e:
            r.logDebug(f"Other error in print report info trays state: {e}")

        self.report_info(r)  # 数据上报
        if self.status == MoveStatus.FINISHED:  # 任务完成时，清除料箱车过期报错提示
            self.clear_pickrobot_error(r)
            self.clear_pickrobot_warning(r)
        return self.status

    def init_trays(self, r):
        # 判断 rbk 版本是否支持此脚本
        if "getContainers" in dir(SimModule):
            containers = r.getContainers()
            tray_num = len(containers)
            for c in containers:
                if c.get('container_name', None) == '999':
                    tray_num -= 1
        else:
            r.setPickRobotError(53803, "RBK version mismatch, please update rbk")
            return MoveStatus.FAILED
        trays = list()
        try:
            for c in containers:
                d = dict()
                d["binId"] = '-1'
                d["id"] = int(c['container_name'])
                d["state"] = int(not c['has_goods'])  # 海柔协议中1表示没有，0表示有
                d["type"] = -1
                d['goods'] = c['goods_id']
                trays.append(d)
        except Exception as e:
            r.setPickRobotError(53804, f"Init trays error: {e}")
        return trays

    @staticmethod
    def init_forksDetects(r, forkDetects_num: int):
        forks = list()
        for i in range(forkDetects_num):
            d = dict()
            d["binId"] = '-1'
            d["id"] = i
            d["state"] = 1
            d["type"] = -1
            forks.append(d)
        return forks

    def detect_refresh(self, r):
        if "getGoods" in self.state and self.state["getGoods"]["status"] == MoveStatus.FINISHED:
            if self.has_fork_sensor:  # 如果有货叉传感器，检测货叉是否有货
                if not self.check_fork(r):
                    r.setPickRobotError(53805,
                                        f"Failed to pick up the goods {self.goods_id}! {self.state.get('forkDetect', 'No data')}")
                    self.operation_status = MoveStatus.FAILED
                    return False
            self.fork_detect[0]["state"] = 0
            self.fork_detect[1]["state"] = 0
            r.logInfo(f"getGoods detect_refresh---{self.fork_detect}")
        if "putGoods" in self.state and self.state["putGoods"]["status"] == MoveStatus.FINISHED:
            self.fork_detect[0]["state"] = 1
            self.fork_detect[1]["state"] = 1
            r.logInfo(f"putGoods detect_refresh---{self.fork_detect}")
        self.report_info(r)  # 数据上报

        # 背篓 数据库更新
        for tray in self.tray_detect:
            if tray['state'] == 0:
                r.setContainer(str(tray['id']), tray['goods'], tray['binId'])
            elif tray['state'] == 1:
                r.clearContainer(str(tray['id']))

    def check_trays(self, r, lift_height, opt):
        """ TO.DO: 根据取货高度，计算最优空背篓层数 """
        if opt == 'load':
            for tray in self.tray_detect:
                if (tray['state'] == 1) and (tray['id'] != 999):  # 999 默认表示抓斗
                    return tray['id']
        elif opt == 'unload':
            for tray in self.tray_detect:
                if (self.goods_id == tray['goods']) and (tray['state'] == 0):
                    return tray['id']
        return None

    def report_info(self, r):
        if not self.has_fork_sensor:
            self.state["forkDetect"] = self.fork_detect
        if not self.has_tray_sensor:
            self.state["trays"] = self.tray_detect
        self.state["report_counter"] = r.getCount()
        self.state["script version"] = SCRIPT_VERSION
        data = {
            "pickingRobotInfo": self.state
        }
        str_state = json.dumps(data)
        r.setInfo(str_state)
        r.logDebug("[pickingRobotInfo][{}]".format(str_state))

    def get_goodsId(self, r):
        move_task = r.moveTask()
        for p in move_task['params']:
            if p['key'] == 'goodsId':
                self.goods_id = p['string_value']

    # 更新指定层背篓的状态
    def update_data(self, r, tray_floor, tray_state, goodsId):
        if tray_floor is None:
            return
        for tray in self.tray_detect:
            if tray['id'] == tray_floor:
                tray['state'] = tray_state
                tray['goods'] = goodsId
                tray['binId'] = goodsId

    # 获取指定层数的背篓数据
    def get_tray(self, r, tray_floor):
        for tray in self.tray_detect:
            if tray['id'] == tray_floor:
                return tray

    # 解析货叉传感器数据，判断货叉中是否有货
    def check_fork(self, r):
        fork_detects = self.state.get("forkDetect", None)
        if fork_detects is not None:
            r.logInfo(f"Fork detect: {fork_detects}")
            for detect in fork_detects:
                if detect.get('id', None) == 0 and detect.get('state', None) == 0:  # 判断货叉中部传感器光电有效
                    return True
        return False

    @staticmethod
    def check_goodsId(r, goodsId):
        containers = r.getContainers()
        for c in containers:
            if goodsId == c['goods_id']:
                return True
        return False
        # with shelve.open(os.path.dirname(__file__) + 'db/trays.db') as s:
        #     if key in s:
        #         return s[key]
        #     return None

    def lift(self, r, height, clear_error=False):
        self.lift_status = MoveStatus.RUNNING
        if height < self.minLiftHeight:
            r.logDebug("lift {} is set to {}".format(height, self.minLiftHeight))
            height = self.minLiftHeight
        if height > self.maxLiftHeight:
            r.logDebug("lift {} is set to {}".format(height, self.maxLiftHeight))
            height = self.maxLiftHeight
        #     r.setError(f"lift out of maxLiftHeight")
        #     r.logDebug("lift {} is set to {}".format(height, self.maxLiftHeight))
        #     self.lift_status = MoveStatus.FAILED
        #     self.status = MoveStatus.FAILED
        if "lift" in self.state:
            dis = r.Di()
            upLimit = False
            downLimit = False
            forkLimit = False
            for node in dis['node']:
                if node['id'] == self.fork_up_limit:
                    status = node.get('status', False)
                    if status:
                        upLimit = True
                elif node['id'] == self.fork_down_limit:
                    status = node.get('status', False)
                    if status:
                        downLimit = True
                elif node['id'] == self.fork_limit:
                    status = node.get('status', False)
                    if status:
                        forkLimit = True
            device_state = self.state["lift"]
            if upLimit and device_state["position"] < height:
                r.setPickRobotError(53806, "Fork upLimit DI is true, can not up!")
                self.lift_status = MoveStatus.FAILED
                self.status = MoveStatus.FAILED
            elif downLimit and device_state["position"] > height:
                r.setPickRobotError(53807, "Fork downLimit DI is true, can not down!")
                self.lift_status = MoveStatus.FAILED
                self.status = MoveStatus.FAILED
            elif forkLimit:
                r.setPickRobotError(53808, "Fork limitDi is true, can not  move!")
                self.stop(r)
                self.lift_status = MoveStatus.FAILED
                self.status = MoveStatus.FAILED
            elif "stretch" in self.state and self.state["stretch"]["position"] < 10:
                if "position" in device_state and "state" in device_state:
                    if abs(device_state["position"] - height) < self.lift_reach_dist \
                            and device_state["state"] == pickingRobot.ModuleState.IDLE:
                        self.lift_status = MoveStatus.FINISHED
                        self.h.reset_liftPos()
                        return True
                if "state" in device_state:
                    if device_state["state"] == pickingRobot.ModuleState.INIT:
                        res = self.h.liftReset(r)
                        self.h.reset_liftPos()
                        self.state["res"] = res
                    elif device_state["state"] == pickingRobot.ModuleState.IDLE:
                        res = self.h.liftPos(height, r)
                        self.state["res"] = res
                    elif device_state["state"] == pickingRobot.ModuleState.ERROR:
                        if clear_error:
                            res = self.h.liftReset(r)
                            self.h.reset_liftPos()
                            self.state["res"] = res
                        else:
                            r.setPickRobotError(53809, "Lift has error, please zero the machine!")
                            self.stretch_status = MoveStatus.FAILED
            else:
                r.setPickRobotError(53810, "Stretch pos is not zero, cannot lift.!!! {}".format(
                    self.state["stretch"]["position"]))
                self.lift_status = MoveStatus.FAILED
                self.status = MoveStatus.FAILED
        return False

    def rotate(self, r, theta, clear_error=False):
        self.rotate_status = MoveStatus.RUNNING
        theta = normalize_theta(theta)
        if theta < -3.0:
            theta = 2 * math.pi + theta
        if "rotate" in self.state:
            if "stretch" in self.state and self.state["stretch"]["position"] < 10:
                device_state = self.state["rotate"]
                if "position" in device_state and "state" in device_state:
                    if abs(normalize_theta(device_state["position"] - theta)) < self.rotate_reach_angle \
                            and device_state["state"] == pickingRobot.ModuleState.IDLE:
                        self.rotate_status = MoveStatus.FINISHED
                        self.h.reset_rotateAngle()
                        return True
                if "state" in device_state:
                    if device_state["state"] == pickingRobot.ModuleState.INIT:
                        res = self.h.rotateReset(r)
                        self.h.reset_rotateAngle()
                        self.state["res"] = res
                    elif device_state["state"] == pickingRobot.ModuleState.IDLE:
                        res = self.h.rotateAngle(theta, r)
                        self.state["res"] = res
                    elif device_state["state"] == pickingRobot.ModuleState.ERROR:
                        if clear_error:
                            res = self.h.rotateReset(r)
                            self.h.reset_rotateAngle()
                            self.state["res"] = res
                        else:
                            r.setPickRobotError(53811, "Rotate has error, please zero the machine!")
                            self.stretch_status = MoveStatus.FAILED
            else:
                self.rotate_status = MoveStatus.FAILED
                self.status = MoveStatus.FAILED
                r.setPickRobotError(53812, "Stretch pos is not zero, cannot rotate.!!! {}".format(
                    self.state["stretch"]["position"]))
        return False

    def stretch(self, r, pos, clear_error=False):
        if pos > self.maxStretchDist:
            r.setPickRobotError(53813, "Reach max stretch dist!")
            self.stretch_status = MoveStatus.FAILED
            return False
        self.stretch_status = MoveStatus.RUNNING
        if "stretch" in self.state:
            device_state = self.state["stretch"]
            if "position" in device_state and "state" in device_state:
                if abs(device_state["position"] - pos) < self.stretch_reach_dist \
                        and device_state["state"] == pickingRobot.ModuleState.IDLE:
                    self.stretch_status = MoveStatus.FINISHED
                    self.h.reset_stretchPos()
                    return True
            if "state" in device_state:
                if device_state["state"] == pickingRobot.ModuleState.INIT:
                    res = self.h.stretchReset(r)
                    self.h.reset_stretchPos()
                    self.state["res"] = res
                elif device_state["state"] == pickingRobot.ModuleState.IDLE:
                    res = self.h.stretchPos(pos, r)
                    self.state["res"] = res
                elif device_state["state"] == pickingRobot.ModuleState.ERROR:
                    if clear_error:
                        res = self.h.stretchReset(r)
                        self.h.reset_stretchPos()
                        self.state["res"] = res
                    else:
                        r.setPickRobotError(53814, "Stretch has error. please zero the machine!")
                        self.stretch_status = MoveStatus.FAILED
        return False

    def checkFingerStatus(self, r, state):
        if "finger" in self.state:
            device_state = self.state["finger"]
            if device_state.get("state", None) != pickingRobot.ModuleState.IDLE:
                r.setPickRobotWarning(55806, "Finger status is not idle. Ctu cannot lift or rotate or stretch!")
                self.finger_status = MoveStatus.FAILED
                return False
            # if device_state.get("leftStatus",-1) != device_state.get("rightStatus",-1):
            #     r.setError("Finger left and right status is not same. Ctu cannot  lift or rotate or stretch! {}, {}".format(
            #         device_state.get("leftStatus",-1), device_state.get("rightStatus",-1)
            #     ))
            #     self.finger_status = MoveStatus.FAILED
            #     return False
            # if abs(device_state.get("leftStatus",-1)- state) > 0.1 \
            # or abs(device_state.get("rightStatus",-1) - state) > 0.1:
            #     r.setError(f"Finger left and right status is not right! Ctu cannot lift or rotate or stretch! {state}")
            #     self.finger_status = MoveStatus.FAILED
            #     return False
        else:
            r.setPickRobotError(53815, f"No finger data in message! Ctu cannot lift or rotate or stretch!")
            self.finger_status = MoveStatus.FAILED
            return False

    def finger(self, r, pos, clear_error=False):
        self.finger_status = MoveStatus.RUNNING
        if "finger" in self.state:
            device_state = self.state["finger"]
            if "leftStatus" in device_state and "state" in device_state and device_state[
                "state"] != pickingRobot.ModuleState.ERROR \
                    and device_state["state"] != pickingRobot.ModuleState.INIT and device_state[
                "state"] != pickingRobot.ModuleState.RESET:
                # if abs(pos - 1) < 0.1 and abs(device_state["leftStatus"] - 1) < 0.1 and abs(device_state["rightStatus"] - 1) < 0.1 and device_state["state"] == pickingRobot.ModuleState.IDLE:
                if pos == 1 and device_state["leftStatus"] == 1 and device_state["rightStatus"] == 1 and device_state[
                    "state"] == pickingRobot.ModuleState.IDLE:
                    self.finger_status = MoveStatus.FINISHED
                    self.h.reset_fingerPos()
                    return True
                # elif abs(pos) < 0.1 and abs(device_state["leftStatus"]) < 0.1 and abs(device_state["rightStatus"]) < 0.1 and device_state["state"] == pickingRobot.ModuleState.IDLE:
                elif pos == 0 and device_state["leftStatus"] == 0 and device_state["rightStatus"] == 0 and device_state[
                    "state"] == pickingRobot.ModuleState.IDLE:
                    self.finger_status = MoveStatus.FINISHED
                    self.h.reset_fingerPos()
                    return True
            if "state" in device_state:
                if device_state["state"] == pickingRobot.ModuleState.INIT:
                    res = self.h.fingerReset(r)
                    self.h.reset_fingerPos()
                    self.state["res"] = res
                elif device_state["state"] == pickingRobot.ModuleState.IDLE:
                    res = self.h.fingerPos(pos, r)
                    self.state["res"] = res
                elif device_state["state"] == pickingRobot.ModuleState.ERROR:
                    if clear_error:
                        res = self.h.fingerReset(r)
                        self.h.reset_fingerPos()
                        self.state["res"] = res
                    else:
                        r.setPickRobotError(53816, "Finger has error, please zero the machine!")
                        self.finger_status = MoveStatus.FAILED
        return False

    def record_vision(self, r):
        res = self.h.visionRecord(r)
        self.state["record_vision_res"] = res
        self.h.reset_visionRecord()

    def vision(self, r, vtype, binType, binModel, recgo=False):
        if self.vision_status is not MoveStatus.FINISHED:
            self.vision_status = MoveStatus.RUNNING
            if "vision" in self.state:
                device_state = self.state["vision"]
                if vtype == "reset":
                    res = self.h.visionReset(r)
                    self.state["res"] = res
                    self.vision_status = MoveStatus.FINISHED
                else:
                    if "state" in device_state:
                        if device_state["state"] == pickingRobot.ModuleState.INIT:
                            self.h.visionReset(r)
                            self.h.reset_visionReq()
                            self.waitVision.reset()
                        elif device_state["state"] == pickingRobot.ModuleState.ERROR:
                            self.h.visionReset(r)
                            self.h.reset_visionReq()
                            self.waitVision.reset()
                            self.vision_status = MoveStatus.FAILED
                            if not recgo:
                                r.setPickRobotWarning(55807, "rec no results.")
                        elif device_state["state"] == pickingRobot.ModuleState.IDLE:
                            if r.warningExits(55300):
                                r.clearWarning(55300)
                            if self.waitVision.status == MoveStatus.NONE:
                                self.waitVision.reset()
                            self.waitVision.run(r, self)
                            if self.waitVision.status == MoveStatus.FINISHED:
                                res = dict()
                                if vtype == "shelf":
                                    binType = "code"
                                    res = self.h.visionReq(pickingRobot.TargetType.SHELF.value,
                                                           pickingRobot.BinType.DM_MARKED.value,
                                                           pickingRobot.BinModel.PLASTICBOX,
                                                           r)
                                elif vtype == "box":
                                    if binType == "code":
                                        res = self.h.visionReq(pickingRobot.TargetType.BOX.value,
                                                               pickingRobot.BinType.DM_MARKED.value,
                                                               pickingRobot.BinModel.PLASTICBOX, r)
                                    elif binType == "markerless":
                                        if binModel == "carton":
                                            res = self.h.visionReq(pickingRobot.TargetType.BOX.value,
                                                                   pickingRobot.BinType.MARKERLESS.value,
                                                                   pickingRobot.BinModel.CARTON, r)
                                        else:
                                            res = self.h.visionReq(pickingRobot.TargetType.BOX.value,
                                                                   pickingRobot.BinType.MARKERLESS.value,
                                                                   pickingRobot.BinModel.PLASTICBOX, r)
                                    elif binType == "barcode":
                                        res = self.h.visionReq(pickingRobot.TargetType.BOX.value,
                                                               pickingRobot.BinType.BARCODE.value,
                                                               pickingRobot.BinModel.PLASTICBOX, r)
                                    else:
                                        r.setPickRobotError(53817, "VisionBinType type is wrong: {}".format(binType))
                                        self.vision_status = MoveStatus.FAILED
                                else:
                                    res["error"] = "wrong type {}".format(vtype)
                                    res["flag"] = False
                                    self.vision_status = MoveStatus.FAILED
                                self.state["res"] = res
                                if res.get("status", pickingRobot.Action.INIT) is pickingRobot.Action.FINISHED:
                                    self.h.reset_visionReq()
                                    if binType == "barcode" and "binId" in res['res']:
                                        res = res['res']
                                        self.state["rec_result"] = res
                                        self.vision_status = MoveStatus.FINISHED
                                        self.waitVision.status = MoveStatus.NONE
                                        out1 = dict()
                                        yaw, pitch, roll, dx, dy, dz, dist = 0, 0, 0, 0, 0, 0, 0
                                        out1["yaw"] = yaw * 180.0 / math.pi
                                        out1["pitch"] = pitch * 180.0 / math.pi
                                        out1["roll"] = roll * 180.0 / math.pi
                                        out1["dx"] = dx
                                        out1["dy"] = dy
                                        out1["dz"] = dz
                                        out1["dist"] = dist
                                        res["vout"] = out1
                                        res["targetType"] = vtype
                                        res["binType"] = binType
                                        r.setNotice(json.dumps(res))
                                        return res
                                    elif "positionMatrix" in res['res']:
                                        res = res['res']
                                        self.vision_status = MoveStatus.FINISHED
                                        self.waitVision.status = MoveStatus.NONE
                                        yaw, pitch, roll, dx, dy, dz, dist = 0, 0, 0, 0, 0, 0, 0
                                        positionMatrix = [res["positionMatrix"]]
                                        Tagv2box = []
                                        if vtype == "shelf" or (vtype == "box" and binType == "code"):
                                            pass
                                            yaw, dx, dy, dz, dist, Tagv2box = getYPRZYX_code(positionMatrix,
                                                                                             self.state["rotate"][
                                                                                                 "position"])
                                            # yaw = yaw + math.pi/2.0
                                            # roll = roll - math.pi/2.0
                                        elif vtype == "box" and binType == "markerless":
                                            yaw, dx, dy, dz, dist, Tagv2box = getYPRZYX_markerless(positionMatrix,
                                                                                                   self.state["rotate"][
                                                                                                       "position"])
                                        else:
                                            r.setPickRobotError(53817,
                                                                "VisionBinType Type is wrong: {}".format(binType))
                                            self.vision_status = MoveStatus.FAILED
                                        out1 = dict()
                                        out1["yaw"] = yaw * 180.0 / math.pi
                                        out1["dx"] = dx
                                        out1["dy"] = dy
                                        out1["dz"] = dz
                                        out1["dist"] = dist
                                        out1["Ta2b"] = Tagv2box
                                        res["vout"] = out1
                                        res["targetType"] = vtype
                                        res["binType"] = binType
                                        res["binId"] = ""
                                        if "operation" in self.task and self.task["operation"] == "getMarkerPos":
                                            loc_org = r.loc()
                                            loc = [loc_org["x"], loc_org["y"], loc_org["angle"]]
                                            Tw2box = getMarkerInWorld(loc, Tagv2box)
                                            boxX = [[1], [0], [0], [1]]
                                            boxX2w = matrixDot(Tw2box, boxX, 4, 4, 1)
                                            angle_world = math.atan2(boxX2w[1][0] - Tw2box[1][3],
                                                                     boxX2w[0][0] - Tw2box[0][3])
                                            res["x"] = Tw2box[0][3]
                                            res["y"] = Tw2box[1][3]
                                            res["theta"] = angle_world
                                        r.setNotice(json.dumps(res))
                                        self.state["res"] = res
                                        return res
                                    else:
                                        self.vision_status = MoveStatus.FAILED
                                        if not recgo:
                                            r.setPickRobotError(53818, "Rec no results. {}".format(json.dumps(res)))
        return dict()

    def getMarkerPos(self, r):
        self.operation_status = MoveStatus.RUNNING
        if self.vision_status is not MoveStatus.FINISHED \
                or self.vision_status is not MoveStatus.FAILED:
            if self.getMarkerPos_status is 0:
                res = self.vision(r, "shelf", "code", "plasticbox")
                if self.vision_status is MoveStatus.FINISHED:
                    self.getMarkerPos_data["x"] = res["x"]
                    self.getMarkerPos_data["y"] = res["y"]
                    self.getMarkerPos_data["theta"] = res["theta"]
                    self.getMarkerPos_data["id"] = ""
                    self.getMarkerPos_status = 1
                    self.getMarkerStartTime = time.time()
            elif self.getMarkerPos_status is 1:
                dt = time.time() - self.getMarkerStartTime
                if dt > self.getMarkerPosTotTime:
                    self.getMarkerPos_status = 2
                    self.operation_status = MoveStatus.FINISHED
        self.state["MarkerPos"] = self.getMarkerPos_data

    def indicator(self, r, chassisLedFront=None, chassisLedBack=None, buzzer=None, headLedRed=None, headLedYellow=None,
                  headLedGreen=None, headLedFreq=None):
        res = self.h.indicatorReq(chassisLedFront, chassisLedBack, buzzer, headLedRed, headLedYellow, headLedGreen,
                                  headLedFreq, r)
        self.state["res"] = res
        if res['status'] is not pickingRobot.Action.FINISHED:
            self.indicator_status = MoveStatus.RUNNING
            return False
        else:
            self.indicator_status = MoveStatus.FINISHED
        return True

    def runTakList(self, r):
        if self.task_id < len(self.task_list):
            if self.task_list[self.task_id].status == MoveStatus.NONE:
                self.task_list[self.task_id].reset(self)
            elif self.task_list[self.task_id].status == MoveStatus.FINISHED:
                self.task_id = self.task_id + 1
            elif self.task_list[self.task_id].status == MoveStatus.FAILED:
                self.operation_status = MoveStatus.FAILED
            else:
                self.task_list[self.task_id].run(r, self)
        else:
            self.operation_status = MoveStatus.FINISHED

    def rec(self, r):
        if self.operation_status == MoveStatus.NONE:
            self.operation_status = MoveStatus.RUNNING
            self.task_list = [
                recAdjust(self.task["visionType"], self.task.get("visionBinType", "code"),
                          self.task.get("binModel", "plasticbox"), self.loadHeight, self.unloadHeight)
            ]
            self.task_id = 0
        else:
            self.runTakList(r)
        cur_state = dict()
        cur_state["state"] = self.operation_status
        cur_state["task_id"] = self.task_id
        self.state["rec"] = cur_state

    def load(self, r):
        if self.operation_status != MoveStatus.FINISHED:
            if self.goods_id and self.check_goodsId(r, self.goods_id):  # 检查 goodsId 是否已存在
                r.setPickRobotError(53819, f"This good already exists: {self.goods_id}")
                self.operation_status = MoveStatus.FAILED
                return
            self.tray_floor = self.check_trays(r, self.task["lift"], 'load')  # load时，查询空背篓所在层数
            if "selfPosition" in self.task:  # 脚本参数指定 load 背篓层数
                self.tray_floor = int(self.task["selfPosition"])
                if self.get_tray(r, self.tray_floor)["state"] == 0:
                    r.setPickRobotError(53820, f"This tray is full, can not load,  tray_floor:{self.tray_floor}")
                    self.operation_status = MoveStatus.FAILED
                    return
            r.logInfo(f"load begin---trays:{self.tray_detect}---tray_floor:{self.tray_floor}")
            if self.tray_floor is None:  # 背篓满了
                if self.get_tray(r, 999) and self.get_tray(r, 999)['state'] == 1:  # 配置了抓斗container并且抓斗为空，则抓斗取货
                    self.tray_floor = 999
                else:
                    r.setPickRobotError(53821, f"All trays are full, can not load")
                    self.operation_status = MoveStatus.FAILED
                    return
        if self.operation_status == MoveStatus.NONE:
            self.operation_status = MoveStatus.RUNNING
            if "recAdjust" in self.task:
                if "visionType" in self.task and self.task["visionType"] == "box":  # 识别料箱进行取货
                    self.task_list = [
                        preGoods(self.task["lift"], self.task["rotate"]),
                        recAdjust(self.task["visionType"], self.task.get("visionBinType", "code"),
                                  self.task.get("binModel", "plasticbox"), self.loadHeight, self.unloadHeight),
                        getGoods(self.task["stretch"] + self.loadOffset),
                        prePutGoods(self.high[self.tray_floor], 0, "load"),
                        putGoods(self.stretchDist)
                    ]
                    if self.tray_floor == 999:
                        self.task_list = self.task_list[:3]
                    if self.barcode_height is not None:    # 取货前扫描一维码并核对goodsId
                        self.task_list.insert(0, ScanBarcode(self.barcode_height, self.task["rotate"], self.goods_id))
                    if self.call_terminal is not None:  # 设备交互
                        self.task_list.insert(0, self.call_terminal)
                elif "visionType" in self.task and self.task["visionType"] == "shelf":  # 识别货架二维码进行取货
                    self.task_list = [
                        preGoods(self.task["lift"], self.task["rotate"]),
                        recAdjust(self.task["visionType"], self.task.get('visionBinType', 'code'),
                                  self.task.get("binModel", "plasticbox"), self.loadHeight, self.unloadHeight),
                        getGoods(self.task["stretch"] + self.loadOffset),
                        prePutGoods(self.high[self.tray_floor], 0, "load"),
                        putGoods(self.stretchDist)
                    ]
                    if self.tray_floor == 999:
                        self.task_list = self.task_list[:3]
                    if self.call_terminal is not None:  # 设备交互
                        self.task_list.insert(0, self.call_terminal)
                else:
                    r.setPickRobotError(53822, "Task is wrong in load with recAdjust: {}".format(json.dumps(self.task)))
                    self.operation_status = MoveStatus.FAILED
            else:
                self.vision_status = MoveStatus.FINISHED
                self.task_list = [
                    preGoods(self.task["lift"], self.task["rotate"]),
                    getGoods(self.task["stretch"] + self.loadOffset),
                    prePutGoods(self.high[self.tray_floor], 0, "load"),
                    putGoods(self.stretchDist)
                ]
                if self.tray_floor == 999:
                    self.task_list = self.task_list[:2]
                if self.call_terminal is not None:  # 设备交互
                    self.task_list.insert(0, self.call_terminal)
            self.task_id = 0
        else:
            self.runTakList(r)

        if self.operation_status == MoveStatus.FINISHED:
            self.update_data(r, self.tray_floor, 0, self.goods_id)
            r.logInfo(f"load finish---trays:{self.tray_detect}")
        self.detect_refresh(r)

        cur_state = dict()
        cur_state["state"] = self.operation_status
        cur_state["task_id"] = self.task_id
        self.state["load"] = cur_state

    def unload(self, r):
        if self.operation_status != MoveStatus.FINISHED:
            self.tray_floor = self.check_trays(r, self.task["lift"], 'unload')  # unload时，查询 goodsId 所在背篓层数
            if "selfPosition" in self.task:  # 指定背篓层数 unload
                self.tray_floor = int(self.task["selfPosition"])
                if self.get_tray(r, self.tray_floor)["state"] == 1:
                    r.setPickRobotError(53824, f"This tray is empty, can not unload:  tray:{self.tray_floor}")
                    self.operation_status = MoveStatus.FAILED
                    return
            if self.get_tray(r, 999) and self.get_tray(r, 999)['state'] == 0:  # 如果抓斗有货，则先unload抓斗
                self.tray_floor = 999
            r.logInfo(f"unload begin---trays:{self.tray_detect}---tray_floor:{self.tray_floor}")
            if self.tray_floor is None:
                r.setPickRobotError(53825, f"No such goods found, can not unload, goodsId:{self.goods_id}")
                self.operation_status = MoveStatus.FAILED
                return
            if self.get_tray(r, self.tray_floor).get('goods', '') != self.goods_id:
                r.setPickRobotError(53826, f"Get the wrong goods in {self.tray_floor} floor, goodsId:{self.goods_id}")
                self.operation_status = MoveStatus.FAILED
                return
        if self.operation_status == MoveStatus.NONE:
            self.operation_status = MoveStatus.RUNNING
            if "recAdjust" in self.task:
                if "visionType" in self.task and self.task["visionType"] == "shelf":
                    if "recBoxLift" in self.task:
                        self.task_list = [
                            preGoods(self.low[self.tray_floor], 0),
                            getGoods(self.stretchDist),
                            preRecBox(self.task["recBoxLift"], self.task["rotate"]),
                            recBox(),
                            waitVision(),
                            prePutGoods(self.task["lift"], self.task["rotate"], "unload"),
                            recAdjust(self.task["visionType"], self.task.get("visionBinType", "code"),
                                      self.task.get("binModel", "plasticbox"), self.loadHeight, self.unloadHeight),
                            putGoods(self.task["stretch"])
                        ]
                    else:
                        self.task_list = [
                            preGoods(self.low[self.tray_floor], 0),
                            getGoods(self.stretchDist),
                            prePutGoods(self.task["lift"], self.task["rotate"], "unload"),
                            recBox(),
                            waitVision(),
                            recAdjust(self.task["visionType"], self.task.get("visionBinType", "code"),
                                      self.task.get("binModel", "plasticbox"), self.loadHeight, self.unloadHeight),
                            putGoods(self.task["stretch"])
                        ]
                else:
                    r.setPickRobotError(53823,
                                        "Task is wrong in unload with recAdjust: {}".format(json.dumps(self.task)))
                    self.operation_status = MoveStatus.FAILED
            else:
                self.vision_status = MoveStatus.FINISHED
                self.task_list = [
                    preGoods(self.low[self.tray_floor], 0),
                    getGoods(self.stretchDist),
                    prePutGoods(self.task["lift"], self.task["rotate"], "unload"),
                    putGoods(self.task["stretch"])
                ]
            self.task_id = 0
            if self.tray_floor == 999:  # 抓斗放货
                self.task_list = self.task_list[2:]
            if self.call_terminal is not None:  # 设备交互
                self.task_list.insert(0, self.call_terminal)
        else:
            self.runTakList(r)

        if self.operation_status == MoveStatus.FINISHED:
            self.update_data(r, self.tray_floor, 1, "")
            r.logInfo(f"unload finish---trays:{self.tray_detect}")
        self.detect_refresh(r)

        cur_state = dict()
        cur_state["state"] = self.operation_status
        cur_state["task_id"] = self.task_id
        self.state["unload"] = cur_state

    def changePos(self, r):
        has_err = False
        if self.get_tray(r, 999) and self.get_tray(r, 999)['state'] == 0:  # 抓斗有货
            r.setPickRobotError(53828, f"Fork has goods, cannot change position!")
            has_err = True
        if self.get_tray(r, int(self.task["changePosition0"])).get('state', 1) == 1:  # 初始层无货
            r.setPickRobotError(53829,
                                f"The target tray {int(self.task['changePosition0'])} is null, cannot change position")
            has_err = True
        if self.get_tray(r, int(self.task["changePosition1"])).get('state', 0) == 0:  # 备换层已有货
            r.setPickRobotError(53830,
                                f"The target tray {int(self.task['changePosition1'])} has goods, cannot change position")
            has_err = True
        if has_err:
            self.operation_status = MoveStatus.FAILED
            return
        if self.operation_status == MoveStatus.NONE:
            self.operation_status = MoveStatus.RUNNING
            self.task_list = [
                preGoods(self.low[int(self.task["changePosition0"])], 0),
                getGoods(self.stretchDist),
                prePutGoods(self.high[int(self.task["changePosition1"])], 0, "changePos"),
                putGoods(self.stretchDist)
            ]
            self.task_id = 0
        else:
            self.runTakList(r)

        r.logInfo(f"changePos begin ---{self.tray_detect}")
        if self.operation_status == MoveStatus.FINISHED:
            trays_floor0 = int(self.task["changePosition0"])
            trays_floor1 = int(self.task["changePosition1"])
            try:
                self.update_data(r, trays_floor1, 0, self.get_tray(r, trays_floor0)['goods'])
                self.update_data(r, trays_floor0, 1, "")
            except Exception as e:
                r.setPickRobotError(53831, f"changePosition0: not found goods: {e}")
            r.logInfo(f"changePos finish ---{self.tray_detect}")
        self.detect_refresh(r)

        cur_state = dict()
        cur_state["state"] = self.operation_status
        cur_state["task_id"] = self.task_id
        self.state["change"] = cur_state

    def putPos(self, r):
        if self.operation_status == MoveStatus.NONE:
            self.operation_status = MoveStatus.RUNNING
            self.task_list = [
                putGoodsS2(),
                prePutGoods(self.high[int(self.task["putPosition"])], 0, "putPos"),
                putGoods(self.stretchDist)
            ]
            self.task_id = 0
        else:
            self.runTakList(r)

        r.logInfo(f"putPos  begin---{self.tray_detect}")
        if self.operation_status == MoveStatus.FINISHED:
            tray_floor = int(self.task["putPosition"])
            self.update_data(r, tray_floor, 0, self.goods_id)
            r.logInfo(f"putPos finish---{self.tray_detect}")
        self.detect_refresh(r)

        cur_state = dict()
        cur_state["state"] = self.operation_status
        cur_state["task_id"] = self.task_id
        self.state["change"] = cur_state

    def zero(self, r):
        self.operation_status = MoveStatus.RUNNING
        if self.finger_status is not MoveStatus.FINISHED:
            self.h.visionReset(r)
            self.h.fingerReset(r)
            self.finger(r, 0, True)
        elif self.stretch_status is not MoveStatus.FINISHED:
            self.h.stretchReset(r)
            self.stretch(r, 0, True)
        elif self.lift_status is not MoveStatus.FINISHED:
            self.h.liftReset(r)
            self.lift(r, 385, True)
        elif self.rotate_status is not MoveStatus.FINISHED:
            self.h.rotateReset(r)
            self.rotate(r, 0, True)
        else:
            self.operation_status = MoveStatus.FINISHED
        rotate_state = -1
        lift_state = -1
        stretch_state = -1
        finger_state = -1
        if "rotate" in self.state and "state" in self.state["rotate"]:
            rotate_state = self.state["rotate"]["state"]
        if "lift" in self.state and "state" in self.state["lift"]:
            lift_state = self.state["lift"]["state"]
        if "stretch" in self.state and "state" in self.state["stretch"]:
            stretch_state = self.state["stretch"]["state"]
        if "finger" in self.state and "state" in self.state["finger"]:
            finger_state = self.state["finger"]["state"]
        if rotate_state == pickingRobot.ModuleState.ERROR \
                or lift_state == pickingRobot.ModuleState.ERROR \
                or stretch_state == pickingRobot.ModuleState.ERROR \
                or finger_state == pickingRobot.ModuleState.ERROR:
            if not r.errorExits(53000):
                r.setPickRobotWarning(55808, f"Picking robot is zero calibrating: {r.getCount()}")
        else:
            if r.errorExits(53000):
                r.clearError(53000)
            if r.warningExits(55808):
                r.clearWarning(55808)
        r.logDebug("53000 error : {}".format(r.errorExits(53000)))

    @staticmethod
    def clear_pickrobot_error(r: SimModule):
        """
        清除料箱车报错提示
        :param r:
        """
        for error_code in range(53800, 53900):
            if r.errorExits(error_code):
                r.clearError(error_code)
        pass

    @staticmethod
    def clear_pickrobot_warning(r: SimModule):
        """
        清除料箱车警告提示
        :param r:
        """
        for warning_code in range(55800, 55900):
            if r.warningExits(warning_code):
                r.clearWarning(warning_code)
        pass

    def stop(self, r):
        if self.lift_status is MoveStatus.RUNNING:
            self.h.liftStop(r)
        if self.stretch_status is MoveStatus.RUNNING:
            self.h.stretchStop(r)
        if self.rotate_status is MoveStatus.RUNNING:
            self.h.rotateStop(r)
        if self.finger_status is MoveStatus.RUNNING:
            self.h.fingerStop(r)
        if self.vision_status is MoveStatus.RUNNING:
            self.h.visionStop(r)

    def cancel(self, r: SimModule):
        r.logInfo("script cancel")
        self.stop(r)
        self.h.disconnect()
        self.status = MoveStatus.NONE

    def suspend(self, r: SimModule):
        r.logInfo("script suspended")
        self.stop(r)
        if self.stretch_status is not MoveStatus.FINISHED \
                and self.stretch_status is not MoveStatus.NONE:
            self.stretch_status = MoveStatus.RUNNING
        if self.lift_status is not MoveStatus.FINISHED \
                and self.lift_status is not MoveStatus.NONE:
            self.lift_status = MoveStatus.RUNNING
        if self.rotate_status is not MoveStatus.FINISHED \
                and self.rotate_status is not MoveStatus.NONE:
            self.rotate_status = MoveStatus.RUNNING
        if self.finger_status is not MoveStatus.FINISHED \
                and self.finger_status is not MoveStatus.NONE:
            self.finger_status = MoveStatus.RUNNING
        if self.vision_status is not MoveStatus.FINISHED \
                and self.vision_status is not MoveStatus.NONE:
            self.vision_status = MoveStatus.RUNNING
        if self.operation_status is not MoveStatus.FINISHED \
                and self.operation_status is not MoveStatus.NONE:
            self.operation_status = MoveStatus.RUNNING
        self.h.resetAll()
        self.status = MoveStatus.SUSPENDED
        self.start_connect_time = time.time()
        self.state = self.h.getReport(r)
        movestate = dict()
        movestate["lift"] = self.lift_status
        movestate["rotate"] = self.rotate_status
        movestate["stretch"] = self.stretch_status
        movestate["finger"] = self.finger_status
        movestate["indicator"] = self.indicator_status
        movestate["vision"] = self.vision_status
        movestate["operation"] = self.operation_status
        movestate["status"] = self.status
        self.state["MoveStatus"] = movestate
        str_state = json.dumps(self.state)
        r.setInfo(str_state)
        r.logDebug(str_state)


class recBox:
    def __init__(self):
        self.status = MoveStatus.NONE
        self.visionType = "box"
        self.visionBinType = "code"
        self.binModel = "plasticbox"

    def reset(self, ctu):
        ctu.vision_status = MoveStatus.NONE
        self.status = MoveStatus.RUNNING

    def run(self, r, ctu):
        self.status = MoveStatus.RUNNING
        if ctu.vision_status is not MoveStatus.FINISHED:
            res = ctu.vision(r, self.visionType, self.visionBinType, self.binModel, True)
            if ctu.vision_status == MoveStatus.FAILED:
                r.logInfo(f"vision failed, vision res: {res}")
                self.status = MoveStatus.FINISHED
                ctu.vision_status = MoveStatus.FINISHED
            elif ctu.vision_status == MoveStatus.FINISHED:
                method = "vout"
                dz = res[method]["dz"]
                dx = res[method]["dx"]
                dy = res[method]["dy"]
                dist = res[method]["dist"]
                dtheta = res[method]["yaw"] * math.pi / 180.0
                dist_stretch = abs(dx * math.cos(dtheta) + dy * math.sin(dtheta))
                if dz < -0.2 or dist_stretch > 1.0:
                    self.status = MoveStatus.FINISHED
                    r.logDebug(f"Has shelf but not in here. dz {dz} dist {dist_stretch}")
                else:
                    self.status = MoveStatus.FAILED
                    ctu.vision_status = MoveStatus.FAILED
                    r.setPickRobotError(53832, "The shelf has box. Cannot unload!!")
        cur_state = dict()
        cur_state["status"] = self.status
        ctu.state["recBox"] = cur_state


class recAdjust:
    def __init__(self, visionType, visionBinType, binModel, loadHeight, unloadHeight):
        self.status = MoveStatus.NONE
        self.visionType = visionType
        self.visionBinType = visionBinType
        self.binModel = binModel
        self.dtheta = 0  # 角度方向
        self.dist = 0  # 侧向
        self.dz = 0  # 垂直方向
        p = ParamServer(__file__)
        self.max_rec_times = p.loadParam("max_rec_times", type="int", default=10, maxValue=999999, minValue=0,
                                         comment="最多识别次数")
        self.max_adjust_time = p.loadParam("max_adjust_time", type="int", default=10, maxValue=999999, minValue=0,
                                           comment="最多调整次数")
        self.rec_shelf_shift = p.loadParam("rec_shelf_shift", type="float", default=10.0, maxValue=1000.0,
                                           minValue=-1000.0, unit="mm", comment="识别货架时相机距离二维码的z方向偏差")
        self.rec_count = 0
        self.offz_box = loadHeight
        self.offz_shelf = unloadHeight
        self.lift_pos = 0
        self.rot_theta = 0
        self.go_args = dict()
        self.adjust_count = 0
        self.ok = False
        self.first_adj = True
        self.last_ddtheta = 100

    def reset(self, ctu):
        ctu.vision_status = MoveStatus.NONE
        ctu.rotate_status = MoveStatus.NONE
        ctu.lift_status = MoveStatus.NONE
        self.dtheta = 0
        self.dist = 0
        self.dz = 0
        self.rec_count = 0
        ctu.goPath.reset()
        self.status = MoveStatus.RUNNING
        self.lift_pos = 0
        self.rot_theta = 0
        self.go_args = dict()
        self.adjust_count = 0
        self.ok = False
        self.ok_x = 0.012
        self.ok_theta = 0.013
        self.first_adj = True
        self.last_ddtheta = 100

    def run(self, r, ctu):
        self.status = MoveStatus.RUNNING
        if ctu.vision_status is not MoveStatus.FINISHED:
            if self.rec_count < self.max_rec_times:
                res = ctu.vision(r, self.visionType, self.visionBinType, self.binModel)
                if ctu.vision_status == MoveStatus.FAILED:
                    self.rec_count = self.rec_count + 1
                    ctu.vision_status = MoveStatus.RUNNING
                method = "vout"
                if ctu.vision_status == MoveStatus.FINISHED:
                    dist, dz, dtheta = 0, 0, 0
                    if self.visionBinType == "code":
                        dz = res[method]["dz"]
                        dist = res[method]["dist"]
                        dtheta = res[method]["yaw"] * math.pi / 180.0
                    elif self.visionBinType == "markerless":
                        dz = 0
                        dist = res[method]["dist"]
                        dtheta = res[method]["yaw"] * math.pi / 180.0
                    else:
                        r.setPickRobotError(53833, " VisionBinType Type is wrong: {}".format(self.visionBinType))
                        ctu.vision_status = MoveStatus.FAILED
                        self.status = MoveStatus.FAILED
                    ddtheta = normalize_theta(dtheta - ctu.state["rotate"]["position"])
                    if abs(ddtheta) < 0.14:  # 在最大误差范围内
                        self.dtheta = dtheta
                        self.dz = dz
                        self.dist = dist
                        if self.adjust_count >= self.max_adjust_time:
                            self.status = MoveStatus.FAILED
                            r.setPickRobotError(53834, "RecAdjust fails!!! reach max times.")
                        else:
                            self.go_args["coordinate"] = "robot"
                            self.go_args["x"] = self.dist
                            self.go_args["y"] = 0
                            self.go_args["theta"] = 0
                            self.go_args["reachAngle"] = math.pi
                            if "useLoc" not in ctu.task:
                                self.go_args["useOdo"] = 1
                            self.go_args["reachDist"] = 0.002
                            if self.go_args["x"] < 0:
                                self.go_args["backMode"] = 1
                            self.rot_theta = self.dtheta
                            self.ok_x = 0.012
                            self.ok_theta = 0.013
                            if self.visionBinType == "markerless":
                                self.ok_x = 0.02
                                self.ok_theta = 0.035
                            elif self.visionType == "shelf" and self.adjust_count + 3 > self.max_adjust_time:
                                self.ok_x = 0.02
                                self.ok_theta = 0.035
                            elif self.adjust_count + 3 > self.max_adjust_time:
                                self.ok_x = 0.012
                                self.ok_theta = 0.02
                            r.logDebug("[recAdjust][{}|{}|{}|{}|{}|{}]".format(
                                self.go_args["x"], self.ok_x, ddtheta, self.ok_theta, self.last_ddtheta,
                                self.first_adj))
                            if (abs(ddtheta) < self.ok_theta or abs(ddtheta) > abs(self.last_ddtheta)) \
                                    and abs(self.go_args["x"]) < self.ok_x and not self.first_adj:
                                self.ok = True
                                self.lift_pos = ctu.state["lift"]["position"]
                                if self.visionType == "shelf":
                                    self.lift_pos = self.lift_pos + self.dz * 1000 + self.offz_shelf
                                elif self.visionType == "box" and self.visionBinType == "code":
                                    self.lift_pos = self.lift_pos + self.dz * 1000 + self.offz_box
                                elif self.visionType == "box" and self.visionBinType == "markerless":
                                    ctu.lift_status = MoveStatus.FINISHED
                            else:
                                self.first_adj = False
                                self.lift_pos = ctu.state["lift"]["position"]
                                if self.visionType == "shelf":
                                    self.lift_pos = self.lift_pos + self.dz * 1000 + self.rec_shelf_shift
                                elif self.visionType == "box" and self.visionBinType == "code":
                                    self.lift_pos = self.lift_pos + self.dz * 1000
                                elif self.visionType == "box" and self.visionBinType == "markerless":
                                    ctu.lift_status = MoveStatus.FINISHED
                            self.last_ddtheta = ddtheta
                    else:
                        ctu.record_vision(r)
                        r.setNotice(" yaw is too large: {}".format(res[method]["yaw"]))
                        self.rec_count = self.rec_count + 1
                        ctu.vision_status = MoveStatus.NONE
                        self.status = MoveStatus.RUNNING
            else:
                r.setPickRobotError(53835, "Rec fails!!! reach max times.")
                self.status = MoveStatus.FAILED
        else:
            if self.ok:
                if ctu.lift_status != MoveStatus.FINISHED:
                    ctu.lift(r, self.lift_pos)
                if ctu.lift_status == MoveStatus.FAILED:
                    self.status = MoveStatus.FAILED
                elif ctu.lift_status == MoveStatus.FINISHED:
                    ctu.rotate_status = MoveStatus.FINISHED
                    ctu.goPath.status = MoveStatus.FINISHED
                    self.status = MoveStatus.FINISHED
            else:
                if ctu.lift_status is not MoveStatus.FINISHED:
                    ctu.lift(r, self.lift_pos)
                if ctu.goPath.status != MoveStatus.FINISHED and ctu.goPath.status != MoveStatus.FAILED:
                    if abs(self.go_args["x"]) < 0.003:
                        ctu.goPath.status = MoveStatus.FINISHED
                    else:
                        ctu.goPath.run(r, self.go_args)
                if ctu.rotate_status != MoveStatus.FINISHED:
                    ctu.rotate(r, self.rot_theta)
                if ctu.lift_status == MoveStatus.FAILED \
                        or ctu.goPath.status == MoveStatus.FAILED \
                        or ctu.rotate_status == MoveStatus.FAILED:
                    self.status = MoveStatus.FAILED
                elif ctu.lift_status == MoveStatus.FINISHED \
                        and ctu.goPath.status == MoveStatus.FINISHED \
                        and ctu.rotate_status == MoveStatus.FINISHED:
                    self.adjust_count = self.adjust_count + 1
                    ctu.vision_status = MoveStatus.NONE
                    ctu.rotate_status = MoveStatus.NONE
                    ctu.lift_status = MoveStatus.NONE
                    self.dtheta = 0
                    self.dist = 0
                    self.dz = 0
                    self.rec_count = 0
                    ctu.goPath.reset()
                    self.status = MoveStatus.RUNNING
                    self.lift_pos = 0
                    self.rot_theta = 0
                    self.go_args = dict()
        cur_state = dict()
        cur_state["dz"] = self.dz
        cur_state["dist"] = self.dist
        cur_state["dtheta"] = self.dtheta
        cur_state["goaPathStatus"] = ctu.goPath.status
        cur_state["lift_pos"] = self.lift_pos
        cur_state["go_args"] = self.go_args
        cur_state["rot_theta"] = self.rot_theta
        cur_state["adj_count"] = self.adjust_count
        cur_state["visionType"] = self.visionType
        cur_state["binType"] = self.visionBinType
        cur_state["status"] = self.status
        cur_state["ok"] = self.ok
        cur_state["ok_x"] = self.ok_x
        cur_state["ok_theta"] = self.ok_theta
        ctu.state["recAdjStatus"] = cur_state


class preGoods:
    def __init__(self, liftPos, rotAngle):
        self.status = MoveStatus.NONE
        self.liftPos = liftPos
        self.rotAngle = rotAngle

    def reset(self, ctu):
        ctu.finger_status = MoveStatus.NONE
        ctu.rotate_status = MoveStatus.NONE
        ctu.lift_status = MoveStatus.NONE
        self.status = MoveStatus.RUNNING

    def run(self, r, ctu):
        self.status = MoveStatus.RUNNING
        if ctu.finger_status is not MoveStatus.FINISHED:
            ctu.finger(r, 1)
        elif ctu.lift_status is not MoveStatus.FINISHED:
            ctu.lift(r, self.liftPos)
        elif ctu.rotate_status is not MoveStatus.FINISHED:
            ctu.rotate(r, self.rotAngle)
        if ctu.finger_status is MoveStatus.FINISHED and ctu.lift_status is MoveStatus.FINISHED and ctu.rotate_status is MoveStatus.FINISHED:
            self.status = MoveStatus.FINISHED


class getGoodsS1:
    def __init__(self, stretchDist):
        self.status = MoveStatus.NONE
        self.stretchDist = stretchDist

    def reset(self, ctu):
        ctu.finger_status = MoveStatus.NONE
        ctu.stretch_status = MoveStatus.NONE
        self.status = MoveStatus.RUNNING

    def run(self, r, ctu):
        self.status = MoveStatus.RUNNING
        if ctu.stretch_status is not MoveStatus.FINISHED:
            ctu.stretch(r, self.stretchDist)
            ctu.checkFingerStatus(r, 1)
        elif ctu.finger_status is not MoveStatus.FINISHED:
            ctu.finger(r, 0)
        else:
            self.status = MoveStatus.FINISHED


class getGoodsS2:
    def __init__(self):
        self.status = MoveStatus.NONE

    def reset(self, ctu):
        ctu.stretch_status = MoveStatus.NONE
        self.status = MoveStatus.RUNNING

    def run(self, r, ctu):
        self.status = MoveStatus.RUNNING
        if ctu.stretch_status is not MoveStatus.FINISHED:
            ctu.stretch(r, 0)
            ctu.checkFingerStatus(r, 0)
        else:
            self.status = MoveStatus.FINISHED


class getGoods:
    def __init__(self, stretchDist):
        self.status = MoveStatus.NONE
        self.task_list = [getGoodsS1(stretchDist), getGoodsS2()]
        self.task_id = 0

    def reset(self, ctu):
        for task in self.task_list:
            task.status = MoveStatus.NONE
        self.task_id = 0
        self.status = MoveStatus.RUNNING

    def run(self, r, ctu):
        self.status = MoveStatus.RUNNING
        if self.task_id < len(self.task_list):
            if self.task_list[self.task_id].status == MoveStatus.NONE:
                self.task_list[self.task_id].reset(ctu)
            elif self.task_list[self.task_id].status == MoveStatus.FINISHED:
                self.task_id = self.task_id + 1
            else:
                self.task_list[self.task_id].run(r, ctu)
        else:
            self.status = MoveStatus.FINISHED
        cur_state = dict()
        cur_state["task_id"] = self.task_id
        cur_state["status"] = self.status
        ctu.state["getGoods"] = cur_state


class preRecBox:
    def __init__(self, liftPos, rotAngle):
        self.status = MoveStatus.NONE
        self.liftPos = liftPos
        self.rotAngle = rotAngle

    def reset(self, ctu):
        ctu.rotate_status = MoveStatus.NONE
        ctu.lift_status = MoveStatus.NONE
        self.status = MoveStatus.RUNNING

    def run(self, r, ctu):
        self.status = MoveStatus.RUNNING
        if ctu.lift_status is not MoveStatus.FINISHED:
            ctu.lift(r, self.liftPos)
        if ctu.rotate_status is not MoveStatus.FINISHED:
            ctu.rotate(r, self.rotAngle)
        if ctu.lift_status is MoveStatus.FINISHED and ctu.rotate_status is MoveStatus.FINISHED:
            self.status = MoveStatus.FINISHED


class prePutGoods:
    def __init__(self, liftPos, rotAngle, operation):
        self.status = MoveStatus.NONE
        self.liftPos = liftPos
        self.rotAngle = rotAngle
        self.seperate_height = 2000
        self.operation = operation

    def reset(self, ctu):
        ctu.rotate_status = MoveStatus.NONE
        ctu.lift_status = MoveStatus.NONE
        self.status = MoveStatus.RUNNING

    def run(self, r, ctu: Module):
        device_state = ctu.state["lift"]
        self.status = MoveStatus.RUNNING
        r.logDebug("prePutGoods, {}, {}, {}".format(self.seperate_height, self.operation, device_state["position"]))
        if 0 <= self.seperate_height <= device_state["position"] and self.operation == "load":
            self.status = MoveStatus.RUNNING
            if ctu.lift_status is not MoveStatus.FINISHED:
                ctu.lift(r, self.liftPos)
            if ctu.lift_status is MoveStatus.FINISHED and ctu.rotate_status is not MoveStatus.FINISHED:
                ctu.rotate(r, self.rotAngle)
            if ctu.lift_status is MoveStatus.FINISHED and ctu.rotate_status is MoveStatus.FINISHED:
                self.status = MoveStatus.FINISHED
        elif 0 <= self.seperate_height <= self.liftPos and self.operation == "unload":
            if ctu.rotate_status is not MoveStatus.FINISHED:
                ctu.rotate(r, self.rotAngle)
            if ctu.rotate_status is MoveStatus.FINISHED and ctu.lift_status is not MoveStatus.FINISHED:
                ctu.lift(r, self.liftPos)
            if ctu.lift_status is MoveStatus.FINISHED and ctu.rotate_status is MoveStatus.FINISHED:
                self.status = MoveStatus.FINISHED
        else:
            if ctu.lift_status is not MoveStatus.FINISHED:
                ctu.lift(r, self.liftPos)
            if ctu.rotate_status is not MoveStatus.FINISHED:
                ctu.rotate(r, self.rotAngle)
            if ctu.lift_status is MoveStatus.FINISHED and ctu.rotate_status is MoveStatus.FINISHED:
                self.status = MoveStatus.FINISHED


class putGoodsS1:
    def __init__(self, stretchDist):
        self.status = MoveStatus.NONE
        self.stretchDist = stretchDist

    def reset(self, ctu):
        ctu.finger_status = MoveStatus.NONE
        ctu.stretch_status = MoveStatus.NONE
        self.status = MoveStatus.RUNNING

    def run(self, r, ctu):
        self.status = MoveStatus.RUNNING
        if ctu.stretch_status is not MoveStatus.FINISHED:
            ctu.stretch(r, self.stretchDist)
            ctu.checkFingerStatus(r, 0)
        elif ctu.finger_status is not MoveStatus.FINISHED:
            ctu.finger(r, 1)
        else:
            self.status = MoveStatus.FINISHED


class putGoodsS2:
    def __init__(self):
        self.status = MoveStatus.NONE

    def reset(self, ctu):
        ctu.finger_status = MoveStatus.NONE
        ctu.stretch_status = MoveStatus.NONE
        self.status = MoveStatus.RUNNING

    def run(self, r, ctu):
        self.status = MoveStatus.RUNNING
        if ctu.stretch_status is not MoveStatus.FINISHED:
            ctu.stretch(r, 0)
        elif ctu.finger_status is not MoveStatus.FINISHED:
            ctu.finger(r, 0)
        else:
            self.status = MoveStatus.FINISHED


class putGoods:
    def __init__(self, stretchDist):
        self.status = MoveStatus.NONE
        self.task_list = [putGoodsS1(stretchDist), putGoodsS2()]
        self.task_id = 0

    def reset(self, ctu):
        for task in self.task_list:
            task.status = MoveStatus.NONE
        self.task_id = 0
        self.status = MoveStatus.RUNNING

    def run(self, r, ctu):
        self.status = MoveStatus.RUNNING
        if self.task_id < len(self.task_list):
            if self.task_list[self.task_id].status == MoveStatus.NONE:
                self.task_list[self.task_id].reset(ctu)
            elif self.task_list[self.task_id].status == MoveStatus.FINISHED:
                self.task_id = self.task_id + 1
            else:
                self.task_list[self.task_id].run(r, ctu)
        else:
            self.status = MoveStatus.FINISHED
        cur_state = dict()
        cur_state["task_id"] = self.task_id
        cur_state["status"] = self.status
        ctu.state["putGoods"] = cur_state


class ScanBarcode:
    def __init__(self, barcode_height, rot_angle, goodsId):
        self.status = MoveStatus.NONE
        self.barcode_height = barcode_height
        self.rotate_angle = rot_angle
        self.goodsId = goodsId

    def reset(self, ctu: Module):
        ctu.lift_status = MoveStatus.NONE
        ctu.rotate_status = MoveStatus.NONE
        self.status = MoveStatus.RUNNING

    def run(self, r, ctu: Module):
        self.status = MoveStatus.RUNNING
        res = dict()
        if ctu.lift_status is not MoveStatus.FINISHED:
            ctu.lift(r, self.barcode_height)
        elif ctu.rotate_status is not MoveStatus.FINISHED:
            ctu.rotate(r, self.rotate_angle)
        elif ctu.vision_status is not MoveStatus.FINISHED:
            res = ctu.vision(r, "box", "barcode", "plasticbox")

        if ctu.vision_status is MoveStatus.FINISHED:
            if res.get("binId", "Error") == self.goodsId:
                self.status = MoveStatus.FINISHED
            else:
                r.setPickRobotError(53838,
                                    f"barcode mismatch, scan result: {res.get('binId', res)}, goodsId: {self.goodsId}")
                self.status = MoveStatus.FAILED

        cur_state = dict()
        cur_state["status"] = self.status
        cur_state["goodsId"] = self.goodsId
        cur_state["scanResult"] = res
        ctu.state["ScanBarcode"] = cur_state


class waitVision:
    def __init__(self):
        self.status = MoveStatus.NONE
        self.wtime = 0.5
        self.start_time = time.time()

    def reset(self, ctu=None):
        self.status = MoveStatus.RUNNING
        self.start_time = time.time()

    def run(self, r, ctu):
        if self.status is not MoveStatus.FINISHED:
            t = time.time()
            dt = t - self.start_time
            if dt > self.wtime:
                self.status = MoveStatus.FINISHED
            else:
                self.status = MoveStatus.RUNNING


class PostData:
    def __init__(self, addr, data):
        p = ParamServer(__file__)
        self.post_overtime = p.loadParam("PostOvertime", type="int", default="60", comment="post 超时时间")
        self.addr = addr
        self.data = data
        self.head = {'Content-Type': 'application/json'}
        self.status = MoveStatus.NONE
        self.start_time = time.time()
        self.post_send = False

    def reset(self):
        self.status = MoveStatus.RUNNING

    def run(self, r, ctu):
        err_str = ''
        if self.status is not MoveStatus.FINISHED:
            try:
                if not self.post_send:
                    res = requests.post(self.addr, json=self.data, headers=self.head)
                    err_str = "response | {} | status | {}".format(str(res), self.status)
                    if res.status_code == 200:
                        self.post_send = True
                        self.status = MoveStatus.FINISHED
                    else:
                        self.status = MoveStatus.RUNNING
                    r.logDebug("response|{}|status|{}".format(str(res), self.status))
            except requests.exceptions.ConnectionError as e:
                self.status = MoveStatus.RUNNING
                err_str = "connect to {} is refused".format(self.addr)
                r.logDebug("connect to {} is refused".format(self.addr))
            except Exception as e:
                err_str = str(e)
                self.status = MoveStatus.RUNNING
                r.logDebug(str(e))
            if time.time() - self.start_time > self.post_overtime:
                r.setPickRobotWarning(55809, f"Response time out: {self.post_overtime}s {err_str}")
                self.status = MoveStatus.FAILED


class CallTerminal:
    def __init__(self, addr, data):
        self.status = MoveStatus.NONE
        self.net = NetHandle()
        self.report_info = dict()
        self.addr = addr
        self.data = data
        self.get_url = f"http://{str(self.addr)}/getTerminalStatus"
        self.post_url = f"http://{str(self.addr)}/setRobotStatus"
        self.reach_data = data.get("reach", None)
        self.action_data = data.get("action", None)
        self.task_flag = [False] * 2

    def run(self, r, ctu):
        if self.addr is None or self.data is None:
            r.setPickRobotError(53836, f"data has error. addr: {self.addr}, data: {self.data}")
            self.status = MoveStatus.FAILED
        else:
            self.status = MoveStatus.RUNNING
            if not self.task_flag[0]:
                res = self.net.http_post(r, self.post_url, data=self.reach_data, timeout=(0.1, 0.1))
                if res and res.status_code == 200 and res.json().get('code', 1) == 0:
                    self.task_flag[0] = True
            if self.task_flag[0] and not self.task_flag[1]:
                res = self.net.http_get(r, self.get_url, params=self.action_data)
                if res and res.status_code == 200 and res.json().get('code', 1) == 0:
                    self.task_flag[1] = True
            if all(self.task_flag):
                self.status = MoveStatus.FINISHED

        self.report_info['status'] = self.status
        self.report_info['addr'] = self.addr
        self.report_info['data'] = self.data
        self.report_info['call task'] = self.task_flag
        ctu.state['call terminal'] = self.report_info

    def reset(self, ctu=None):
        self.status = MoveStatus.RUNNING


if __name__ == '__main__':
    r = SimModule()
    m = Module(r, None)
    m.suspend(r)
    data = dict()
    data["headLedFreq"] = dict()
    data["headLedFreq"]["value"] = "1"
    print(m.run(r, data))
    # res = {"errorState": [], "finger": {"leftStatus": 1, "rightStatus": 1, "state": 2}, "lastUpdate": 1616926318459, "lift": {"position": 485.754, "speed": 0.548552, "state": 2}, "msgType": 10, "rotate": {"position": -1.56325, "speed": 0.0, "state": 2}, "seqNum": 406, "stretch": {"position": -0.0, "speed": -1.01203, "state": 2}, "vision": {"state": 2}, "task": {"lift": 400, "operation": "load", "recAdjust": 1, "rotate": -1.57, "selfPosition": 3, "stretch": 860, "visionType": "box", "visionBinType": "code"}, "res": {"status": 0, "seqNum": 22, "res": {"executionResult": 0, "msgType": 255, "positionMatrix": [-0.005162753438078038, 0.9998964839473677, 0.013430091832507686, 0.02913439950958338, 0.9995693010948689, 0.004772144649372367, 0.02895581019694668, -0.010669121054354028, 0.02888872246489524, 0.013573799215306236, -0.9994904670326821, 0.34266477417180574, 0.0, 0.0, 0.0, 1.0], "seqNum": 22, "vout": {"yaw": 1.6554542548874005, "pitch": 0.2958052967453177, "roll": -0.7695209701947723, "dx": 0.6046647741718058, "dy": 0.010669121054354028, "dz": 0.0001343995095833793, "dist": 0.010668817268320767}, "targetType": "box", "binType": "code", "binId": ""}}, "recAdjStatus": {"dz": 0.0001343995095833793, "dist": 0.010668817268320767, "dtheta": 0.028893127363934568, "goaPathStatus": 0, "lift_pos": 485.8883995095834, "go_args": {"coordinate": "robot", "x": 0.010668817268320767, "y": 0, "theta": 0, "reachAngle": 3.141592653589793, "useOdo": 1, "reachDist": 0.002}, "rot_theta": -1.5343568726360655, "adj_count": 8, "visionType": "box", "binType": "code", "status": 1}, "load": {"state": 1, "task_id": 1}, "MoveStatus": {"lift": 0, "rotate": 0, "stretch": 0, "finger": 3, "indicator": 3, "vision": 3, "operation": 1, "status": 1}}
    # res = {"errorState": [], "finger": {"leftStatus": 1, "rightStatus": 1, "state": 2}, "lastUpdate": 1616926319885, "lift": {"position": 485.755, "speed": -0.0, "state": 2}, "msgType": 10, "rotate": {"position": -1.53436, "speed": -0.0167552, "state": 2}, "seqNum": 434, "stretch": {"position": -0.0, "speed": 0.337344, "state": 2}, "vision": {"state": 2}, "task": {"lift": 400, "operation": "load", "recAdjust": 1, "rotate": -1.57, "得分": 3, "stretch": 860, "visionType": "box", "visionBinType": "code"}, "res": {"status": 0, "seqNum": 24, "res": {"executionResult": 0, "msgType": 255, "positionMatrix": [-0.007429246509390142, 0.999766852892183, 0.020274273214173785, 0.029153939962044777, 0.9997064956292621, 0.007893279536360431, -0.022904557075420436, 0.015400308396874191, -0.02305924745005849, 0.02009815902567019, -0.9995320580705833, 0.3421031880788719, 0.0, 0.0, 0.0, 1.0], "seqNum": 24, "vout": {"yaw": -1.3213511442478674, "pitch": 0.4256683857140494, "roll": -1.1617419484584592, "dx": 0.6041031880788719, "dy": -0.015400308396874191, "dz": 0.00015393996204477595, "dist": -0.015390086757584222}, "targetType": "box", "binType": "code", "binId": ""}}, "recAdjStatus": {"dz": 0.00015393996204477595, "dist": -0.015390086757584222, "dtheta": -0.023061928042119817, "goaPathStatus": 0, "lift_pos": 485.90893996204477, "go_args": {"coordinate": "robot", "x": -0.015390086757584222, "y": 0, "theta": 0, "reachAngle": 3.141592653589793, "useOdo": 1, "reachDist": 0.002, "backMode": 1}, "rot_theta": -1.5574219280421198, "adj_count": 9, "visionType": "box", "binType": "code", "status": 1}, "load": {"state": 1, "task_id": 1}, "MoveStatus": {"lift": 0, "rotate": 0, "stretch": 0, "finger": 3, "indicator": 3, "vision": 3, "operation": 1, "status": 1}}
    # res = {"errorState": [], "finger": {"leftStatus": 1, "rightStatus": 1, "state": 2}, "lastUpdate": 1616926321457, "lift": {"position": 485.753, "speed": 0.274276, "state": 2}, "msgType": 10, "rotate": {"position": -1.55743, "speed": 0.00167552, "state": 2}, "seqNum": 465, "stretch": {"position": -0.0, "speed": -0.0, "state": 2}, "vision": {"state": 2}, "task": {"lift": 400, "operation": "load", "recAdjust": 1, "rotate": -1.57, "selfPosition": 3, "stretch": 860, "visionType": "box", "visionBinType": "code"}, "res": {"status": 0, "seqNum": 26, "res": {"executionResult": 0, "msgType": 255, "positionMatrix": [-0.007004054562370299, 0.9999739902400965, -0.0017210644922518769, 0.02916108667094663, 0.9999590953971117, 0.007013779326262126, 0.005710904670219258, -0.01148737215256884, 0.005722827297514875, -0.0016809946048815666, -0.999982211594217, 0.342662612258709, 0.0, 0.0, 0.0, 1.0], "seqNum": 26, "vout": {"yaw": 0.3279036839872235, "pitch": 0.4013060470792056, "roll": 0.09861219918784905, "dx": 0.604662612258709, "dy": 0.01148737215256884, "dz": 0.00016108667094662937, "dist": 0.011486346008404863}, "targetType": "box", "binType": "code", "binId": ""}}, "recAdjStatus": {"dz": 0.00016108667094662937, "dist": 0.011486346008404863, "dtheta": 0.005722998914996058, "goaPathStatus": 0, "lift_pos": 0, "go_args": {}, "rot_theta": 0, "adj_count": 10, "visionType": "box", "binType": "code", "status": 1}, "load": {"state": 1, "task_id": 1}, "MoveStatus": {"lift": 0, "rotate": 0, "stretch": 0, "finger": 3, "indicator": 3, "vision": 3, "operation": 1, "status": 1}}
    # res = {"errorState": [], "finger": {"leftStatus": 1, "rightStatus": 1, "state": 2}, "lastUpdate": 1617774196313, "lift": {"position": 1150.08, "speed": -0.0, "state": 2}, "msgType": 10, "rotate": {"position": 3.13999, "speed": 0.00753982, "state": 2}, "seqNum": 97, "stretch": {"position": 0.00202406, "speed": 0.337344, "state": 2}, "vision": {"state": 2}, "task": {"lift": 1080, "operation": "load", "recAdjust": 1, "rotate": 3.14, "selfPosition": 0, "stretch": 920, "visionType": "box", "visionBinType": "code"}, "res": {"status": 0, "seqNum": 6, "res": {"executionResult": 0, "msgType": 255, "positionMatrix": [-0.009344127654362655, 0.9982044929080618, 0.05916483428979484, 0.02896389952587754, 0.9998971162527275, 0.009971164304003799, -0.010311779279186539, -0.008335600717951527, -0.010883206690082945, 0.059062392608007996, -0.9981949657214058, 0.39888548170899063, 0.0, 0.0, 0.0, 1.0], "seqNum": 6, "vout": {"yaw": -0.6236013469464964, "pitch": 0.5353868690333142, "roll": -3.3920243909026055, "dx": 0.6608854817089906, "dy": 0.008335600717951527, "dz": -3.610047412246076e-05, "dist": -1.3359074694907491e-05}, "targetType": "box", "binType": "code", "binId": ""}}, "recAdjStatus": {"dz": -3.610047412246076e-05, "dist": -1.3359074694907491e-05, "dtheta": -0.01088389672408785, "goaPathStatus": 0, "lift_pos": 1070.0438995258776, "go_args": {"coordinate": "robot", "x": -1.3359074694907491e-05, "y": 0, "theta": 0, "reachAngle": 3.141592653589793, "useOdo": 1, "reachDist": 0.002, "backMode": 1}, "rot_theta": 3.1291061032759124, "adj_count": 1, "visionType": "box", "binType": "code", "status": 1}, "load": {"state": 1, "task_id": 1}, "MoveStatus": {"lift": 0, "rotate": 0, "stretch": 0, "finger": 3, "indicator": 3, "vision": 3, "operation": 1, "status": 1}}
    res = {"errorState": [], "finger": {"leftStatus": 1, "rightStatus": 1, "state": 2}, "lastUpdate": 1617774196313,
           "lift": {"position": 1150.08, "speed": -0.0, "state": 2}, "msgType": 10,
           "rotate": {"position": 3.13999, "speed": 0.00753982, "state": 2}, "seqNum": 97,
           "stretch": {"position": 0.00202406, "speed": 0.337344, "state": 2}, "vision": {"state": 2},
           "task": {"lift": 1080, "operation": "load", "recAdjust": 1, "rotate": 3.14, "selfPosition": 0,
                    "stretch": 920, "visionType": "box", "visionBinType": "code"}, "res": {"status": 0, "seqNum": 6,
                                                                                           "res": {"executionResult": 0,
                                                                                                   "msgType": 255,
                                                                                                   "positionMatrix": [
                                                                                                       -0.009344127654362655,
                                                                                                       0.9982044929080618,
                                                                                                       0.05916483428979484,
                                                                                                       0.02896389952587754,
                                                                                                       0.9998971162527275,
                                                                                                       0.009971164304003799,
                                                                                                       -0.010311779279186539,
                                                                                                       -0.008335600717951527,
                                                                                                       -0.010883206690082945,
                                                                                                       0.059062392608007996,
                                                                                                       -0.9981949657214058,
                                                                                                       0.39888548170899063,
                                                                                                       0.0, 0.0, 0.0,
                                                                                                       1.0],
                                                                                                   "seqNum": 6,
                                                                                                   "vout": {
                                                                                                       "yaw": -0.6236013469464964,
                                                                                                       "pitch": 0.5353868690333142,
                                                                                                       "roll": -3.3920243909026055,
                                                                                                       "dx": 0.6608854817089906,
                                                                                                       "dy": 0.008335600717951527,
                                                                                                       "dz": -3.610047412246076e-05,
                                                                                                       "dist": -1.3359074694907491e-05},
                                                                                                   "targetType": "box",
                                                                                                   "binType": "code",
                                                                                                   "binId": ""}},
           "recAdjStatus": {"dz": -3.610047412246076e-05, "dist": -1.3359074694907491e-05,
                            "dtheta": -0.01088389672408785, "goaPathStatus": 0, "lift_pos": 1070.0438995258776,
                            "go_args": {"coordinate": "robot", "x": -1.3359074694907491e-05, "y": 0, "theta": 0,
                                        "reachAngle": 3.141592653589793, "useOdo": 1, "reachDist": 0.002,
                                        "backMode": 1}, "rot_theta": 3.1291061032759124, "adj_count": 1,
                            "visionType": "box", "binType": "code", "status": 1}, "load": {"state": 1, "task_id": 1},
           "MoveStatus": {"lift": 0, "rotate": 0, "stretch": 0, "finger": 3, "indicator": 3, "vision": 3,
                          "operation": 1, "status": 1}}
    print(f"rotate: {res['rotate']['position'] + res['recAdjStatus']['dtheta']}, dist: {res['recAdjStatus']['dist']}")
    yaw, dx, dy, dz, dist, Tagv2box = getYPRZYX_code([res["res"]["res"]["positionMatrix"]], res["rotate"]["position"])
    print(f"yaw: {yaw}, dx: {dx}, dy: {dy}, dz: {dz}, dist: {dist}")
    loc_org = {"x": -26.89989, "y": -1.768388, "angle": -0.228760}
    loc = [loc_org["x"], loc_org["y"], loc_org["angle"]]
    Tw2box = getMarkerInWorld(loc, Tagv2box)

    print("Tw2box: ", Tw2box)
    print("yaw", yaw)
    boxX = [[1], [0], [0], [1]]
    boxX2agv = matrixDot(Tagv2box, boxX, 4, 4, 1)
    boxX2w = matrixDot(Tw2box, boxX, 4, 4, 1)
    angle_agv = math.atan2(boxX2agv[1][0] - Tagv2box[1][3], boxX2agv[0][0] - Tagv2box[0][3])
    angle_world = math.atan2(boxX2w[1][0] - Tw2box[1][3], boxX2w[0][0] - Tw2box[0][3])
    out = dict()
    out["x"] = Tw2box[0][3]
    out["y"] = Tw2box[1][3]
    out["theta"] = angle_world
    print("box in world: ", out)
    print("box in agv: ", Tagv2box[0][3], Tagv2box[1][3], angle_agv)

    res = {"errorState": [], "finger": {"leftStatus": 0, "rightStatus": 0, "state": 2},
           "forkDetect": [{"binId": "", "id": 1, "state": 1, "type": 1}, {"binId": "", "id": 0, "state": 1, "type": 0}],
           "lastUpdate": 1624259373931, "lift": {"position": 1080.0, "speed": 0.548552, "state": 2}, "msgType": 10,
           "rotate": {"position": 3.14, "speed": 0.00167552, "state": 2}, "seqNum": 0,
           "stretch": {"position": -0.0, "speed": -0.0, "state": 2},
           "trays": [{"binId": "", "id": 1, "state": 1, "type": 0}], "vision": {"state": 2},
           "task": {"lift": 1080, "operation": "unload", "recAdjust": 1, "rotate": 3.14, "selfPosition": 1,
                    "stretch": 920, "visionType": "shelf", "visionBinType": "code"},
           "unload": {"state": 1, "task_id": 0},
           "MoveStatus": {"lift": 0, "rotate": 0, "stretch": 0, "finger": 0, "indicator": 3, "vision": 0,
                          "operation": 1, "status": 1}, "connect_error": "ctu recv error!!!"}
    dz = 0
    dist_stretch = 0
    r.logDebug(f"Has shelf but not in here. dz {dz} dist {dist_stretch}")
    try:
        r.logDebug("[HaiRou][{}|{}|{}|{}|{}|{}|{}|{}|{}|{}|{}|{}|{}]".format(
            res["lift"]["position"], res["lift"]["state"], res["lift"]["speed"],
            res["rotate"]["position"], res["rotate"]["state"], res["rotate"]["speed"],
            res["stretch"]["position"], res["stretch"]["state"], res["stretch"]["speed"],
            res["finger"]["leftStatus"], res["finger"]["rightStatus"], res["finger"]["state"],
            res["vision"]["state"]))
    except KeyError as e:
        r.logDebug("KeyError: " + str(e))
    except Exception as e:
        r.logDebug("Other error in print report info state")

    try:
        if "forkDetect" in res:
            log_key = "HaiRou_forkDetect"
            log_str = ""
            for fork in res["forkDetect"]:
                tmp_str = "{}|{}|{}|".format(fork["id"], fork["state"], fork["state"])
                log_str = log_str + tmp_str
            r.logDebug("[{}][{}]".format(log_key, log_str))
    except KeyError as e:
        r.logDebug("KeyError: " + str(e))
    except Exception as e:
        r.logDebug("Other error in print report info forkDetect state")

    try:
        if "trays" in res:
            log_key = "HaiRou_trays"
            log_str = ""
            for trays in res["trays"]:
                tmp_str = "{}|{}|{}|".format(trays["id"], trays["state"], trays["state"])
                log_str = log_str + tmp_str
            if log_str != "":
                r.logDebug("[{}][{}]".format(log_key, log_str))
    except KeyError as e:
        r.logDebug("KeyError: " + str(e))
    except Exception as e:
        r.logDebug("Other error in print report info trays state")

    print(m.checkFingerStatus(r, 1))
    print("DONE")

    addr = "http://127.0.0.1:8088/api"
    data = json.dumps(
        {
            "vehicles": []
        }
    )
    p = PostData(addr, data)
    p.run(r, m)

    # yaw = (yaw+math.pi/2)/math.pi*180.0
    # pitch = pitch/math.pi*180
    # roll = roll/math.pi *180
    # print(dx, dy, dz, dist, yaw, pitch ,roll)

    # yaw, p, r, z, y, x = getYPRZYX(one)
    # print(yaw/math.pi*180,p/math.pi*180,r/math.pi*180,z,y,x)
