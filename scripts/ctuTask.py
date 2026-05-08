# -*- coding: utf-8 -*-
# @Time : 2021/2/14  10: 20
# @Author : zhong
# @Version : 2.2.0
# @Update: 升级任务模式脚本功能：1. 新增load，unload，change操作选项，2. 新增goodsId系统流程 3. 适配binTask任务 4. 适配库位管理功能
import sys
sys.path.append("../syspy")
import syspy.rbk
import json
import time
import goPath as goPath
from rbkSim import SimModule
from rbk import MoveStatus, BasicModule, ParamServer
from pickingRobot import ModeType, PickRobot, BinOpType, BinType, BinModel, LocationType, Action, StopType, ErrorMessage
"""
####BEGIN DEFAULT ARGS####
{
    "operation": {
        "value": "reset",
        "default_value": ["preaction", "external_opt", "internal_opt", "robot_reset", "param_set", "switch_mode", "task_resume", "task_stop", "load", "unload", "change"],
        "tips": "动作指令",
        "type": "complex"
    },
    "mode": {
        "value": "task",
        "default_value": ["task", "module"],
        "tips": "交互模式",
        "type": "complex"
    },
    "robotId": {
        "value": "",
        "tips": "",
        "type": "string"
    },
    "opType": {
        "value": "inspect",
        "default_value": ["put", "take", "move", "inspect"],
        "tips": "操作类型",
        "type": "complex"
    },
    "binId": {
        "value": "reserve",
        "tips": "预留参数",
        "type": "string"
    },
    "binType": {
        "value": "dm_market",
        "default_value": ["dm_market", "markerless", "barcode"],
        "tips": "货物识别类型",
        "type": "complex"
    },
    "binModel": {
        "value": "carton",
        "default_value": ["carton", "plasticbox"],
        "tips": "料箱种类",
        "type": "complex"
    },
    "srcTray": {
        "value": {
            "id": 0,
            "type": 0 
        },
        "tips": "源托盘, 托盘id, 托盘类型",
        "type": "json"
    },
    "dstTray": {
        "value": {
            "id": 3,
            "type": 1 
        },
        "tips": "目标托盘, 托盘id, 托盘类型",
        "type": "json"
    },
    "targetTray": {
        "value": {
            "id": 0,
            "type": 0 
        },
        "tips": "扫描对象托盘, 托盘id, 托盘类型",
        "type": "json"
    },
    "targetPosition": {
        "value": {
            "x": 0,
            "y": 0,
            "theta": 0
        },
        "tips": "位置定义",
        "type": "json"
    },
    "targetHeight": {
        "value": 0,
        "tips": "库位高度",
        "unit": "",
        "type": "double"
    },
    "locationType": {
        "value": "storage_shelf",
        "default_value": ["storage_shelf", "storage_shelf_deep", "conveyor"],
        "tips": "库位类型",
        "type": "complex"
    },
    "preconditions": {
        "value": {
            "liftPositionMax": 0.0,
            "liftPositionMin": 0.0,
            "forkRotationPositionMax": 0.0,
            "forkRotationPositionMin": 0.0,
            "fingerPosition": 0
        },
        "tips": "预备动作前置目标",
        "type": "json"
    },
    "box_width": {
        "value": 0,
        "tips": "货箱宽度配置值",
        "unit": "m",
        "type": "double"
    },
    "box_height": {
        "value": 0,
        "tips": "货箱高度配置值",
        "unit": "m",
        "type": "double"
    },
    "box_depth": {
        "value": 0,
        "tips": "货箱深度配置值",
        "unit": "m",
        "type": "double"
    },
    "box_tag_height": {
        "value": 0,
        "tips": "货箱底部与货箱码下边沿的高度差",
        "unit": "m",
        "type": "double"
    },
    "box_tag_depth": {
        "value": 0,
        "tips": "货箱码到货箱表面的贴码深度",
        "unit": "m",
        "type": "double"
    },
    "shelf_tag_height": {
        "value": 0,
        "tips": "货架放货平面与货架码上边沿的高度差",
        "unit": "m",
        "type": "double"
    },
    "conveyor_tag_height": {
        "value": 0,
        "tips": "输送线放货平面(滚轮面)与货架码上边沿的高度差",
        "unit": "m",
        "type": "double"
    },
    "gap_between_box": {
        "value": 0,
        "tips": "货架上箱子之间的距离",
        "unit": "m",
        "type": "double"
    },
    "changeFrom": {
        "value": 0,
        "type": "int"
    },
    "changeTo": {
        "value": 1,
        "type": "int"
    },
    "goodsId": {
        "value": "",
        "type": "string"
    },
    "postAddr":{
        "value":"http://ip:port/path",
        "type":"string"
    },
    "postData":{
        "value":{},
        "type":"string"
    },
    "reqType":{
        "value": "GET",
        "default_value": ["GET", "POST"],
        "tips": "请求类型",
        "type": "complex"
    }
}
####END DEFAULT ARGS##### 
"""


class Module(BasicModule):
    def __init__(self, r, args):
        super(Module, self).__init__()
        self.status = MoveStatus.NONE
        p = ParamServer(__file__)
        ip = p.loadParam("ip", type="str", default="192.168.192.20", comment="ip addr")
        port = p.loadParam("port", type="int", default=4172, maxValue=999999, minValue=0, comment="port")
        self.start_connect_time = time.time()
        self.max_connect_time = p.loadParam("max_connect_time", type="int", default=30, maxValue=999999, minValue=0,
                                            comment="链接等待最长时间s")
        self.mode = p.loadParam("mode", type="str", default="task", comment="交互模式")
        self.robotId = p.loadParam("robotId", type="str", default="1", comment="机器人ID")
        self.opType = p.loadParam("opType", type="str", default="inspect", comment="操作类型")
        self.binId = p.loadParam("binId", type="str", default="reserve", comment="预留参数")
        self.binType = p.loadParam("binType", type="str", default="markerless", comment="货物识别类型")
        self.binModel = p.loadParam("binModel", type="str", default="plasticbox", comment="料箱种类")
        self.srcTray = p.loadParam("srcTray", type="str", default='{"id": 0, "type": 0}', comment="源托盘, 托盘id, 托盘类型")
        self.dstTray = p.loadParam("dstTray", type="str", default='{"id": 1, "type": 0}', comment="目标托盘, 托盘id, 托盘类型")
        self.targetTray = p.loadParam("targetTray", type="str", default='{"id": 0, "type": 0}',
                                      comment="扫描对象托盘, 托盘id, 托盘类型")
        self.targetPosition = p.loadParam("targetPosition", type="str",
                                          default='{"x": 71.1, "y": 26.3, "theta": 1.5708}', comment="位置定义")
        self.targetHeight = p.loadParam("targetHeight", type="float", default=0, comment="库位高度")
        self.locationType = p.loadParam("locationType", type="str", default="storage_shelf", comment="库位类型")
        self.preconditions = p.loadParam("preconditions", type="str",
                                         default='{"liftPositionMax": 0.0, "liftPositionMin": 0.0,'
                                                 ' "forkRotationPositionMax": 0.0, "forkRotationPositionMin": 0.0,'
                                                 ' "fingerPosition": 0}', comment="预备动作前置目标")
        self.box_width = p.loadParam("box_width", type="float", default=0, comment="货箱宽度配置值")
        self.box_height = p.loadParam("box_height", type="float", default=0, comment="货箱高度配置值")
        self.box_depth = p.loadParam("box_depth", type="float", default=0, comment="货箱深度配置值")
        self.box_tag_height = p.loadParam("box_tag_height", type="float", default=0, comment="货箱底部与货箱码下边沿的高度差")
        self.box_tag_depth = p.loadParam("box_tag_depth", type="float", default=0, comment="货箱码到货箱表面的贴码深度")
        self.shelf_tag_height = p.loadParam("shelf_tag_height", type="float", default=0, comment="货架放货平面与货架码上边沿的高度差")
        self.conveyor_tag_height = p.loadParam("conveyor_tag_height", type="float", default=0,
                                               comment="输送线放货平面(滚轮面)与货架码上边沿的高度差")
        self.gap_between_box = p.loadParam("gap_between_box", type="float", default=0, comment="货架上箱子之间的距离")
        self.di1 = p.loadParam("di1", type="int", default=2, comment="上限位DI")
        self.di2 = p.loadParam("di2", type="int", default=4, comment="下限位DI")
        self.di3 = p.loadParam("di3", type="int", default=1, comment="升降防坠机械限位DI")
        self.init = True
        self.state = dict()
        self.msg_send = False
        self.src_ok = False
        self.src_send = False
        self.resume_send = False
        self.go_path = goPath.Module(r, dict())
        self.h = PickRobot(ip, port)
        self.h.connect()
        r.logInfo(f"init args: {args}")
        self.goodsId = None

    def run(self, r: SimModule, args):
        # 驱动器连接故障，通信超时
        if r.errorExits(52111):
            return MoveStatus.FAILED

        self.status = MoveStatus.RUNNING
        if self.init:
            self.init = False
            r.logInfo(f"before update args:  {json.dumps(args)}")
            self.update_param(r, args)
            r.logInfo(f"after update args:  {json.dumps(args)}")
            self.clear_errors(r)  # 清除用户自定义错误

        if not self.h.isconnect:
            self.state["init"] = self.h.initDevice(r)
            self.state["warning"] = "ctu is connecting!!!!"
            str_state = json.dumps(self.state)
            r.setInfo(str_state)
            r.logDebug(str_state)
            d_time = time.time() - self.start_connect_time
            if d_time > self.max_connect_time:
                r.setError(f"ctu connect is overtime: {self.max_connect_time}s")
                self.status = MoveStatus.FAILED
            return self.status

        if self.status is not MoveStatus.FINISHED:
            self.state = self.h.getReport(r)
            r.setInfo(json.dumps(self.state))

            if "connect_error" in self.state:
                d_time = time.time() - self.start_connect_time
                if d_time > self.max_connect_time:
                    r.setError(f"ctu connect is overtime: {self.max_connect_time}s")
                    self.status = MoveStatus.FAILED
                    str_state = json.dumps(self.state)
                    r.setInfo(str_state)
                    r.logDebug(str_state)
                    return self.status
            else:
                self.start_connect_time = time.time()

            if self.trigger_di(r):
                # 请求stop指令
                self.h.task_stop(r, StopType.STOP_EMG)
                r.setNotice(json.dumps(self.h.task_stop_res))
                self.state["task_stop_res"] = self.h.task_stop_res
                return MoveStatus.FAILED

            # ======================================动作指令类型========================================================
            # ["switch_mode", "preaction", "robot_reset", "param_set", "internal_opt", "external_opt", "task_resume", "task_stop"]
            if "operation" in args:
                if args['operation'] == 'switch_mode':
                    try:
                        if not self.msg_send:
                            self.h.switch_mode(r, self.mode)
                            self.msg_send = True
                        r.setNotice(json.dumps(self.h.switch_mode_res))
                        self.state['result'] = self.h.switch_mode_res
                        if self.h.switch_mode_res['status'] == Action.FINISHED:
                            self.status = MoveStatus.FINISHED
                    except Exception as e:
                        r.setWarning(f"switch_mode exception: {e}")

                elif args['operation'] == 'preaction':
                    try:
                        if not self.msg_send:
                            self.h.preaction(r, self.robotId, self.preconditions)
                            self.msg_send = True
                        r.setNotice(json.dumps(self.h.preaction_res))
                        self.state['result'] = self.h.preaction_res
                        if self.h.preaction_res['status'] == Action.FINISHED:
                            self.status = MoveStatus.FINISHED
                    except Exception as e:
                        r.setWarning(f"preaction exception: {e}")

                elif args['operation'] == 'robot_reset':
                    r.logInfo(f"robot is resetting...")
                    try:
                        if not self.msg_send:
                            self.h.robot_reset(r)
                            self.msg_send = True
                        r.setNotice(json.dumps(self.h.reset_res))
                        self.state['result'] = self.h.reset_res
                        if self.h.reset_res['status'] == Action.FINISHED:
                            self.status = MoveStatus.FINISHED
                    except Exception as e:
                        r.setWarning(f"robot_reset exception: {e}")
                        self.h.task_resume(r, self.robotId)

                elif args['operation'] == 'param_set':
                    try:
                        if not self.msg_send:
                            self.h.param_set(r, self.robotId, self.box_width, self.box_height, self.box_depth,
                                             self.box_tag_height,
                                             self.box_tag_depth, self.shelf_tag_height, self.conveyor_tag_height,
                                             self.gap_between_box)
                            self.msg_send = True
                        r.setNotice(f"param_set_res: {json.dumps(self.h.param_set_res)}")
                        self.state['result'] = self.h.param_set_res
                        if self.h.param_set_res['status'] == Action.FINISHED:
                            self.status = MoveStatus.FINISHED
                    except Exception as e:
                        r.setWarning(f"param_set exception: {e}")
                        self.h.task_resume(r, self.robotId)

                elif args['operation'] == 'internal_opt':
                    try:
                        self.check_module_state(r)
                        if not self.msg_send:
                            self.h.internal_bin_op(r, self.robotId, self.opType, self.binId, self.binType, self.binModel, self.srcTray, self.dstTray, self.targetTray)
                            self.msg_send = True
                        r.setNotice(json.dumps(self.h.internal_bin_op_res))
                        self.state['result'] = self.h.internal_bin_op_res
                        self.check_execution_result(r, self.h.internal_bin_op_res)
                        if self.h.internal_bin_op_res['status'] == Action.FINISHED:
                            self.status = MoveStatus.FINISHED
                    except Exception as e:
                        r.setWarning(f"internal_opt exception: {e}, {self.state}")
                        # return MoveStatus.FAILED

                elif args['operation'] == 'external_opt':
                    try:
                        self.check_module_state(r)
                        if not self.msg_send:
                            self.h.external_bin_op(r, self.robotId, self.opType, self.binId, self.targetPosition,
                                                   self.targetHeight, self.binType, self.binModel, self.locationType)
                            self.msg_send = True
                        r.setNotice(json.dumps(self.h.external_bin_op_res))
                        self.state['result'] = self.h.external_bin_op_res
                        self.check_execution_result(r, self.h.external_bin_op_res)
                        # 监听位置请求 msgType(200), 机器视觉自动校准取放货物位置
                        if self.h.req_position and not self.src_ok:
                            # r.setWarning(f"req_position: {self.h.req_position}")
                            self.src_ok = self.src_pos(r, self.h.req_position)
                            if self.src_ok:
                                self.h.req_position = dict()
                                self.src_ok = False

                        if self.h.external_bin_op_res['status'] == Action.FINISHED:
                            self.status = MoveStatus.FINISHED
                    except Exception as e:
                        r.setWarning(f"external_opt exception: {e}, {self.state}")
                        # return MoveStatus.FAILED

                elif args['operation'] == 'task_resume':
                    r.setWarning(f"task_resume: task is resuming...")
                    try:
                        if not self.msg_send:
                            self.h.task_resume(r, self.robotId)
                            self.msg_send = True
                        r.setNotice(json.dumps(self.h.resume_res))
                        self.state['result'] = self.h.resume_res
                        if self.h.resume_res['status'] == Action.FINISHED:
                            self.status = MoveStatus.FINISHED
                    except Exception as e:
                        r.setWarning(f"task_resume exception: {e}")

                elif args['operation'] == 'task_stop':
                    r.setWarning(f"task stop")
                    try:
                        if not self.msg_send:
                            self.h.task_stop(r, StopType.STOP_EMG)
                            self.msg_send = True
                        r.setNotice(json.dumps(self.h.task_stop_res))
                        self.state['result'] = self.h.task_stop_res
                        if self.h.task_stop_res['status'] == Action.FINISHED:
                            self.status = MoveStatus.FINISHED
                    except Exception as e:
                        r.setWarning(f"task_stop exception: {e}")
                else:
                    r.setError(f"args error: {args}")
            else:
                r.setError("operation must be checked!")
                return MoveStatus.FAILED
            self.check_execution_result(r, self.state.get('result', {}))
            self.state["status"] = self.status
            r.setInfo(json.dumps(self.state))
            return self.status

    def suspend(self, r: SimModule):
        self.status = MoveStatus.SUSPENDED
        self.state = self.h.getReport(r)
        self.start_connect_time = time.time()
        r.setInfo(json.dumps(self.state))

    def cancel(self, r: SimModule):
        r.logInfo(f"Task cancel")
        self.h.task_stop(r, StopType.STOP_EMG)
        self.h.disconnect()
        self.status = MoveStatus.NONE

    def update_param(self, r, args):

        # ==============================================================================================================
        # params = ['mode', 'robotId', 'opType', 'binId', 'binType', 'binModel', 'srcTray', 'dstTray', 'targetTray',
        #           'targetPosition', 'targetHeight', 'locationType', 'preconditions', 'box_width', 'box_height', 'box_depth',
        #           'box_tag_height', 'box_tag_depth', 'shelf_tag_height', 'conveyor_tag_height', 'gap_between_box']
        # ========================================= 更新参数 ============================================================
        if 'mode' in args:
            self.mode = args['mode']
        if 'robotId' in args:
            self.robotId = args['robotId']
        if 'opType' in args:
            self.opType = args['opType']
        if 'binId' in args:
            self.binId = args['binId']
        if 'binType' in args:
            self.binType = args['binType']
        else:
            self.binType = None
        if 'binModel' in args:
            self.binModel = args['binModel']
        else:
            self.binModel = None
        if 'srcTray' in args:
            args['srcTray']['id'] = int(args['srcTray']['id'])
            args['srcTray']['type'] = int(args['srcTray']['type'])
            self.srcTray = args['srcTray']
        else:
            self.srcTray = json.loads(self.srcTray)

        if 'dstTray' in args:
            args['dstTray']['id'] = int(args['dstTray']['id'])
            args['dstTray']['type'] = int(args['dstTray']['type'])
            self.dstTray = args['dstTray']
        else:
            self.dstTray = json.loads(self.dstTray)
        if 'targetTray' in args:
            # self.targetTray['id'] = int(self.srcTray['id'])
            # self.targetTray['type'] = int(self.srcTray['type'])
            self.targetTray = args['targetTray']
        else:
            self.targetTray = json.loads(self.targetTray)

        if 'targetPosition' in args:
            self.targetPosition = args['targetPosition']
        else:
            self.targetPosition = json.loads(self.targetPosition)
        if 'targetHeight' in args:
            self.targetHeight = args['targetHeight']
        if 'locationType' in args:
            self.locationType = args['locationType']
        else:
            self.locationType = None
        if 'preconditions' in args:
            self.preconditions = args['preconditions']
        else:
            self.preconditions = json.loads(self.preconditions)
        if 'box_width' in args:
            self.box_width = args['box_width']
        if 'box_height' in args:
            self.box_height = args['box_height']
        if 'box_depth' in args:
            self.box_depth = args['box_depth']
        if 'box_tag_height' in args:
            self.box_tag_height = args['box_tag_height']
        if 'box_tag_depth' in args:
            self.box_tag_depth = args['box_tag_depth']
        if 'shelf_tag_height' in args:
            self.shelf_tag_height = args['shelf_tag_height']
        if 'conveyor_tag_height' in args:
            self.conveyor_tag_height = args['conveyor_tag_height']
        if 'gap_between_box' in args:
            self.gap_between_box = args['gap_between_box']

        # =================更新参数数据格式======================
        if self.mode == 'task':
            self.mode = ModeType.TASK
        elif self.mode == 'module':
            self.mode = ModeType.MODULE

        if self.opType == 'put':
            self.opType = BinOpType.PUT
        elif self.opType == 'take':
            self.opType = BinOpType.TAKE
        elif self.opType == 'move':
            self.opType = BinOpType.MOVE
        elif self.opType == 'inspect':
            self.opType = BinOpType.INSPECT

        if self.binType == 'dm_market':
            self.binType = BinType.DM_MARKED
        elif self.binType == 'markerless':
            self.binType = BinType.MARKERLESS
        elif self.binType == 'barcode':
            self.binType = BinType.BARCODE

        if self.binModel == 'carton':
            self.binModel = BinModel.CARTON
        elif self.binModel == 'plasticbox':
            self.binModel = BinModel.PLASTICBOX

        if self.locationType == 'storage_shelf':
            self.locationType = LocationType.STORAGE_SHELF
        elif self.locationType == 'storage_shelf_deep':
            self.locationType = LocationType.STORAGE_SHELF_DEEP
        elif self.locationType == 'conveyor':
            self.locationType = LocationType.CONVEYOR
        r.logInfo("update args: {args}")

    def src_pos(self, r, req_position):
        req_posi = req_position['position']
        req_posi['coordinate'] = 'robot'
        req_posi['reachDist'] = 0.005
        req_posi['useOdo'] = 1
        src_status = self.go_path.status
        if req_posi['x'] < 0:
            req_posi['backMode'] = 1
        if abs(req_posi['x']) < 0.005:                  # 底盘微调精度
            src_status = MoveStatus.FINISHED
        if src_status != MoveStatus.FINISHED and src_status != MoveStatus.FAILED:
            self.go_path.run(r, req_posi)

        if src_status == MoveStatus.FINISHED:
            self.go_path = goPath.Module(r, dict())
            req_posi['status'] = "finish"
            self.h.src_pos_resp(r, req_posi)
            return True
        return False

    def trigger_di(self, r):  # 触发上、下限位DI，机械限位DI
        dis = r.Di()
        nodes = dis.get('node', list())
        for node in nodes:
            if node['id'] == self.di1 and node['status']:
                r.setError(f"触发上限位DI")
                return True
            if node['id'] == self.di2 and node['status']:
                r.setError(f"触发下限位DI")
                return True
            if node['id'] == self.di3 and node['status']:
                r.setError(f"触发机械限位DI")
                return True
        return False

    @staticmethod
    def clear_errors(r):
        """
        清除用户自定义错误
        :param r: 
        :return: 
        """""
        for err_code in range(53900, 53999):
            if r.errorExits(err_code):
                r.clearError(err_code)

    def check_execution_result(self, r, result):
        """
        检测指令执行结果是否有报错
        :param r:
        :param result:
        """
        if "res" in result:
            # result: {"status": 2, "seqNum": 1, "res": {"executionResult": 2147484676, "failDescription": "there is box in fork!", "msgType": 255, "resMessageType": 90, "resOptType": 2, "robotId": "", "seqNum": 1}}
            if "executionResult" in result['res']:
                try:
                    execution_result = int(result['res'].get('executionResult', 0))
                    if execution_result != 0:
                        r.logInfo(f"result: {result}")
                        if "failDescription" in result['res']:
                            error_msg = result['res']['failDescription']
                        else:
                            error_msg = ErrorMessage.ERROR_CODE.get(execution_result, "user defined error")
                        if execution_result in range(2147484672, 2147484672+100):
                            r.setUserError(53900 + (execution_result - int(2147484672)), f"Error Message: {error_msg}")
                        else:
                            r.setError(f"Error Message: {error_msg}")
                        self.status = MoveStatus.FAILED
                except Exception as e:
                    r.setWarning(f"check execution result key error : {e}, res: {result['res']}")  # ERROR_CODE key error

    def check_module_state(self, r):
        if self.state:  # 检查机构状态，并自动进行异常状态恢复
            finger = self.state["finger"] if "finger" in self.state else None    # finger = self.state.get("finger", None)
            rotate = self.state["rotate"] if "rotate" in self.state else None
            stretch = self.state["stretch"] if "stretch" in self.state else None
            lift = self.state["lift"] if "lift" in self.state else None
            if (finger and finger['state'] in [4, 0]) or (rotate and rotate['state'] in [4, 0]) or (
                    stretch and stretch['state'] in [4, 0]) or (lift and lift['state'] in [4, 0]):
                if not self.resume_send:
                    r.setWarning(f"check module state error, task is resuming")
                    self.h.task_resume(r, self.robotId)
                    self.resume_send = True
            else:
                self.resume_send = False
                r.clearWarning(55300)
        pass


if __name__ == '__main__':
    error_msg1 = ErrorMessage.ERROR_CODE.get(2147484672, "user defined error")
    error_msg2 = ErrorMessage.ERROR_CODE.get(2147484672+3, "user defined error")
    error_msg3 = ErrorMessage.ERROR_CODE.get(2147484672+4, "user defined error")
    print(error_msg1)
    print(error_msg2)
    print(error_msg3)
