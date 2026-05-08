# -*- coding: utf-8 -*-
# @Time : 2022/09/06
# @Author : huang, zhong
# @Version : 2.2.5
# @Support : rbk  3.3.5.66 以上版本
# @Update : 增加料箱车专属报错码，统一料箱车报错提示

from enum import IntEnum
import struct
import socket
import json
import crc
import time
import sys
import goPath as goPath
from rbk import MoveStatus, SimModule


class MessageType(IntEnum):
    ROBOT_INIT_REQ = 0

    ROBOT_MODE_REQ = 1  # 模式切换
    ROBOT_RESET_REQ = 3  # 重置机构校零
    ROBOT_PARAM_SET = 4  # 参数配置
    ROBOT_STOP_REQ = 5  # 任务停止
    ROBOT_RESUME_REQ = 6  # 任务恢复

    ROBOT_INFO_REPORT = 10
    ROBOT_LIFT_RESET = 20
    ROBOT_LIFT_REQ = 21
    ROBOT_LIFT_STOP = 22
    ROBOT_ROTATE_RESET = 30
    ROBOT_ROTATE_REQ = 31
    ROBOT_ROTATE_STOP = 32
    ROBOT_STRETCH_RESET = 40
    ROBOT_STRETCH_REQ = 41
    ROBOT_STRETCH_STOP = 42
    ROBOT_FINGER_RESET = 50
    ROBOT_FINGER_REQ = 51
    ROBOT_FINGER_STOP = 52
    ROBOT_VISION_RESET = 60
    ROBOT_VISION_REQ = 61
    ROBOT_VISION_RECORD = 62
    ROBOT_VISION_STOP = 63
    ROBOT_INDICATOR_REQ = 70  # 指示器控制

    ROBOT_INTERNAL_BIN_OP = 80  # 内部取放货
    ROBOT_EXTERNAL_BIN_OP = 90  # 外部取放货
    ROBOT_PREACTION_REQ = 100  # 预备动作

    ROBOT_SRC_POS_REQ = 200  # 导航请求
    ROBOT_SRC_POS_RESP = 201  # 导航反向请求

    ROBOT_COMM_RESP = 255  # 指令反向请求


class StopType(IntEnum):
    STOP_EMG = 10  # 停止机构业务


class ModeType(IntEnum):
    TASK = 0  # 任务模式，该模式下不支持控制机构指令
    MODULE = 1  # 机构控制模式，该模式下不支持执行任务指令


class PositionXYT:
    def __init__(self, x=0, y=0, theta=0):
        self.x = x  # 位置坐标x值，单位:m
        self.y = y  # 位置坐标y值，单位:m
        self.theta = theta  # 位置坐标theta值，单位:rad


class TrayType(IntEnum):
    FORK = 0  # 货叉
    TRAY = 1  # 背篓


class BinOpType(IntEnum):
    PUT = 0  # 放箱，将货叉上的料箱放至货架、输送线上
    TAKE = 2  # 取箱，将货架、输送线上的料箱取至货叉上
    MOVE = 3  # 移箱，机器人内部(货叉、背篓)之间移动
    INSPECT = 4  # 扫描，当前主要是识别一维码条码


class LocationType(IntEnum):
    STORAGE_SHELF = 0  # 存储区货架(浅库位)
    STORAGE_SHELF_DEEP = 1  # 深库位(仅特定机型支持)
    CONVEYOR = 10  # 输送线


class RunMode(IntEnum):
    ROBOT_WORKING = 1
    ROBOT_RECOVER = 2
    ROBOT_DEBUG = 3
    ROBOT_RESERVED = 4


class ModuleState(IntEnum):
    """
    机构状态
    """
    UNDEFINED = -1   # 协议之外的不明状态
    INIT = 0    # 初始状态，不可用，需要发送ROBOT_XXX_RESET命令复位机构
    RESET = 1   # 复位状态，复位成功跳转到IDLE，复位失败跳转到ERROR
    IDLE = 2   # 空闲状态，当前无指令执行中，可发送各种操作命令
    WORKING = 3    # 工作状态，当前正在执行指令，可发送各种操作命令，按顺序执行
    ERROR = 4   # 错误状态，不可用，需要发送ROBOT_XXX_RESET命令复位


class TargetType(IntEnum):   # 相机识别对象
    SHELF = 1
    BOX = 2


class BinType(IntEnum):    # 相机识别对象的类型
    DM_MARKED = 0              # 识别货架/货箱二维码，使用2D相机
    MARKERLESS = 10          # 识别无二维码货箱，使用3D相机
    BARCODE = 20                  # 识别货箱一维码，使用2D相机


class BinModel(IntEnum):       # 料箱类型
    CARTON = 0            # 纸箱
    PLASTICBOX = 1       # 有码料箱


class Action(IntEnum):
    INIT = 0
    RUNNING = 1
    FINISHED = 2


class ErrorMessage:
    ERROR_CODE = dict()
    ERROR_CODE[0] = ["OK", "成功，无错误"]
    ERROR_CODE[0x80000001] = ["E_SEQUENCE", "指令序号错误"]
    ERROR_CODE[0x80000002] = ["E_PARSE", "指令解析错误"]
    ERROR_CODE[0x80000003] = ["E_UNSUPPORT", "不支持的功能"]
    ERROR_CODE[0x80000004] = ["E_PARAMETER", "错误的参数"]
    ERROR_CODE[0x80000005] = ["E_OVERLIMIT", "目标超出限制"]
    ERROR_CODE[0x80000006] = ["E_ABNORMAL", "设备状态异常"]
    ERROR_CODE[0x800000FF] = ["E_UNKNOWN", "未知错误"]
    ERROR_CODE[0x80000200] = ["E_SAFE_MANUAL", "手动保护"]
    ERROR_CODE[0x80000201] = ["E_SAFE_STOPPER", "限位保护"]
    ERROR_CODE[0x80000202] = ["E_SAFE_LOCKING", "限位保护"]
    ERROR_CODE[0x80000203] = ["E_SAFE_LOCKING", "锁定保护"]
    ERROR_CODE[0x80000204] = ["E_SAFE_CONTROL", "失控保护"]
    ERROR_CODE[0x80000300] = ["E_DEV_COM", "设备通讯错误"]
    ERROR_CODE[0x80000301] = ["E_DEV_BUSY", "设备已被占用"]
    ERROR_CODE[0x80000302] = ["E_DEV_SUPPORT", "设备不能支持"]
    ERROR_CODE[0x80000303] = ["E_DEV_FAILURE", "设备功能失效"]
    ERROR_CODE[0x80000304] = ["E_DEV_ABNORMAL", "设备数据异常"]
    ERROR_CODE[0x80000400] = ["E_EXTERNAL_TAKE_NOTFOUND", "目标库位为空"]
    ERROR_CODE[0x80000401] = ["E_EXTERNAL_PUT_OCCUPIED", "目标库位占用"]
    ERROR_CODE[0x80000402] = ["E_INTERNAL_TAKE_NOTFOUND", "标背篓为空"]
    ERROR_CODE[0x80000403] = ["E_INTERNAL_PUT_OCCUPIED", "目标背篓占用"]


def del_dict_item(_dict, key):
    if key in _dict:
        return _dict.pop(key)
    return None


class PickRobot:
    def __init__(self, ip, port):
        self.ip = ip
        self.port = port
        self.tcp_client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.tcp_client.setblocking(False)
        self.max_try_times = 20
        self.seqNum_req = 0
        self.isconnect = False
        self.msg_init = {"msgType": MessageType.ROBOT_INIT_REQ.value,
                         "seqNum": 0,
                         "timeStamp": int(round(time.time() * 1000))}
        self.msg_lift_reset = {"msgType": MessageType.ROBOT_LIFT_RESET.value,
                               "seqNum": 0}
        self.msg_lift_stop = {"msgType": MessageType.ROBOT_LIFT_STOP.value,
                              "seqNum": 0}
        self.msg_lift_req = {"msgType": MessageType.ROBOT_LIFT_REQ.value,
                             "seqNum": 0,
                             "liftPosition": 0.0}
        self.msg_rot_rest = {"msgType": MessageType.ROBOT_ROTATE_RESET.value,
                             "seqNum": 0}
        self.msg_rot_stop = {"msgType": MessageType.ROBOT_ROTATE_STOP.value,
                             "seqNum": 0}
        self.msg_rot_req = {"msgType": MessageType.ROBOT_ROTATE_REQ.value,
                            "seqNum": 0,
                            "rotatePosition": 0.0}
        self.msg_stretch_reset = {"msgType": MessageType.ROBOT_STRETCH_RESET.value,
                                  "seqNum": 0}
        self.msg_stretch_stop = {"msgType": MessageType.ROBOT_STRETCH_STOP.value,
                                 "seqNum": 0}
        self.msg_stretch_req = {"msgType": MessageType.ROBOT_STRETCH_REQ.value,
                                "seqNum": 0,
                                "stretchPosition": 0.0}
        self.msg_finger_reset = {"msgType": MessageType.ROBOT_FINGER_RESET.value,
                                 "seqNum": 0}
        self.msg_finger_stop = {"msgType": MessageType.ROBOT_FINGER_STOP.value,
                                "seqNum": 0}
        self.msg_finger_req = {"msgType": MessageType.ROBOT_FINGER_REQ.value,
                               "seqNum": 0,
                               "position": 0}
        self.msg_vision_reset = {"msgType": MessageType.ROBOT_VISION_RESET.value,
                                 "seqNum": 0}
        self.msg_vision_stop = {"msgType": MessageType.ROBOT_VISION_STOP.value,
                                "seqNum": 0}
        self.msg_vision_req = {"msgType": MessageType.ROBOT_VISION_REQ.value,
                               "seqNum": 0,
                               "targetType": 0,
                               "binType": 0,
                               "binModel": 0}
        self.msg_vision_record = {"msgType": MessageType.ROBOT_VISION_RECORD.value,
                                  "seqNum": 0,
                                  "imageId": "last"}
        self.msg_indicator_req = {"msgType": MessageType.ROBOT_INDICATOR_REQ.value,
                                  "seqNum": 0,
                                  "chassisLedFront": 0,
                                  "chassisLedBack": 0,
                                  "buzzer": 0,
                                  "headLedRed": 0,
                                  "headLedYellow": 0,
                                  "headLedGreen": 0,
                                  "headLedFreq": 0
                                  }

        self.msg_src_pos_req = {"msgType": MessageType.ROBOT_SRC_POS_REQ.value,
                                "modeType": 0,
                                "seqNum": 0}

        self.msg_src_pos_resp = {"msgType": MessageType.ROBOT_SRC_POS_RESP.value,
                                 "modeType": 0,
                                 "seqNum": 0}

        self.msg_mode_req = {"msgType": MessageType.ROBOT_MODE_REQ.value,
                             "modeType": 0,
                             "seqNum": 0}

        self.msg_reset_req = {"msgType": MessageType.ROBOT_RESET_REQ.value,
                              "timeStamp": 0,
                              "seqNum": 0}

        self.msg_resume_req = {"msgType": MessageType.ROBOT_RESUME_REQ.value,
                               "robotId": 0,
                               "seqNum": 0}
        self.msg_param_set = {"msgType": MessageType.ROBOT_PARAM_SET.value,
                              "seqNum": 0,
                              "robotId": 0,
                              "box_width": 0,
                              "box_height": 0,
                              "box_depth": 0,
                              "box_tag_height": 0,
                              "box_tag_depth": 0,
                              "shelf_tag_height": 0,
                              "conveyor_tag_height": 0,
                              "gap_between_box": 0
                              }

        self.msg_internal_bin_op = {
            "msgType": MessageType.ROBOT_INTERNAL_BIN_OP.value,
            "seqNum": 0,
            "robotId": '',
            "opType": 0,  # BinOpType
            "binId": '',
            "binType": 0,
            "binModel": 0,
            "srcTray": {
                "id": 0,
                "type": 0
            },
            "dstTray": {
                "id": 0,
                "type": 0
            },
            "targetTray": {
                "id": 0,
                "type": 0
            }
        }

        self.msg_external_bin_op = {
            "msgType": MessageType.ROBOT_EXTERNAL_BIN_OP.value,
            "seqNum": 0,
            "robotId": '',
            "opType": 0,  # BinOpType
            "binId": '',
            "binType": 0,
            "binModel": 0,
            "locationType": 0,
            "targetPosition": 0,
            "targetHeight": 0
        }

        self.msg_preaction_req = {
            "msgType": MessageType.ROBOT_PREACTION_REQ.value,
            "seqNum": 0,
            "robotId": '',
            "preconditions": {
                "liftPositionMax": 0,
                "liftPositionMin": 0,
                "forkRotationPositionMax": 0,
                "forkRotationPositionMin": 0,
                "fingerPosition": 0
            }
        }
        self.msg_task_stop_req = {
            "msgType": MessageType.ROBOT_STOP_REQ.value,
            "seqNum": 0,
            "stopType": 0,
        }

        self.report = dict()
        self.liftReset_res = dict()
        self.resetAction(self.liftReset_res)
        self.liftPos_res = dict()
        self.resetAction(self.liftPos_res)
        self.rotateReset_res = dict()
        self.resetAction(self.rotateReset_res)
        self.rotateAngle_res = dict()
        self.resetAction(self.rotateAngle_res)
        self.stretchReset_res = dict()
        self.resetAction(self.stretchReset_res)
        self.stretchPos_res = dict()
        self.resetAction(self.stretchPos_res)
        self.fingerReset_res = dict()
        self.resetAction(self.fingerReset_res)
        self.fingerPos_res = dict()
        self.resetAction(self.fingerPos_res)
        self.visionReset_res = dict()
        self.resetAction(self.visionReset_res)
        self.visionReq_res = dict()
        self.resetAction(self.visionReq_res)
        self.visionRecord_res = dict()
        self.resetAction(self.visionRecord_res)
        self.indicatorReq_res = dict()
        self.resetAction(self.indicatorReq_res)

        self.switch_mode_res = dict()
        self.resetAction(self.switch_mode_res)
        self.reset_res = dict()
        self.resetAction(self.reset_res)
        self.resume_res = dict()
        self.resetAction(self.resume_res)
        self.param_set_res = dict()
        self.resetAction(self.param_set_res)
        self.internal_bin_op_res = dict()
        self.resetAction(self.internal_bin_op_res)
        self.external_bin_op_res = dict()
        self.resetAction(self.external_bin_op_res)
        self.preaction_res = dict()
        self.resetAction(self.preaction_res)
        self.src_pos_res = dict()
        self.resetAction(self.src_pos_res)
        self.task_stop_res = dict()
        self.resetAction(self.task_stop_res)

        self.reset_time = 5
        self.rotate_reset_stime = -1
        self.vision_reset_stime = -1
        self.lift_reset_stime = -1
        self.finger_reset_stime = -1
        self.stretch_reset_stime = -1
        self.total_hex = ""

        self.mode = ModeType.TASK  # 默认为任务模式
        self.req_position = dict()  # 反向导航请求
        self.go_path = goPath.Module(SimModule(), dict())
        self.src_send = False

    def resetAction(self, data):
        data['status'] = Action.INIT
        data['seqNum'] = -1
        data['res'] = dict()

    def finishAction(self, data, res):
        if data['status'] is not Action.INIT:
            data['status'] = Action.FINISHED
            data['res'] = res

    def resetAll(self):
        self.liftReset_res['status'] = Action.INIT
        self.liftPos_res['status'] = Action.INIT
        self.rotateReset_res['status'] = Action.INIT
        self.rotateAngle_res['status'] = Action.INIT
        self.stretchReset_res['status'] = Action.INIT
        self.stretchPos_res['status'] = Action.INIT
        self.fingerReset_res['status'] = Action.INIT
        self.fingerPos_res['status'] = Action.INIT
        self.visionReset_res['status'] = Action.INIT
        self.visionReq_res['status'] = Action.INIT
        self.visionRecord_res['status'] = Action.INIT
        self.indicatorReq_res['status'] = Action.INIT

    def reset_liftReset(self):
        self.liftReset_res['status'] = Action.INIT

    def reset_liftPos(self):
        self.liftPos_res['status'] = Action.INIT

    def reset_rotateReset(self):
        self.rotateReset_res['status'] = Action.INIT

    def reset_rotateAngle(self):
        self.rotateAngle_res['status'] = Action.INIT

    def reset_stretchReset(self):
        self.stretchReset_res['status'] = Action.INIT

    def reset_stretchPos(self):
        self.stretchPos_res['status'] = Action.INIT

    def reset_fingerReset(self):
        self.fingerReset_res['status'] = Action.INIT

    def reset_fingerPos(self):
        self.fingerPos_res['status'] = Action.INIT

    def reset_visionReset(self):
        self.visionReset_res['status'] = Action.INIT

    def reset_visionReq(self):
        self.visionReq_res["status"] = Action.INIT

    def reset_visionRecord(self):
        self.visionRecord_res['status'] = Action.INIT

    def reset_indcatorReq(self):
        self.indicatorReq_res['status'] = Action.INIT

    def connect(self):
        try:
            self.tcp_client.connect((self.ip, self.port))
        except BlockingIOError:
            pass

    def disconnect(self):
        self.tcp_client.close()

    def getMsg(self, r):
        try:
            total_data = self.tcp_client.recv(1024)
        except Exception as e:
            if r is not None:
                r.logDebug("ctu recv error!!!")
            self.report["connect_error"] = f"ctu recv error!!!"
            self.total_hex = ""
            return dict()
        else:
            self.total_hex = self.total_hex + total_data.hex()
            while True:
                ind = self.total_hex.find("addecefa")
                # r.logDebug("ind: {}, total_hex {}".format(ind, self.total_hex))
                if ind >= 0 and ind + 32 <= len(self.total_hex):
                    head_hex = self.total_hex[ind:ind + 32]
                    fmt = "@IIII"
                    try:
                        usMagic, usSize, crcBody, crcHead = struct.unpack(fmt, bytes.fromhex(head_hex))
                    except:
                        out = dict()
                        out["error"] = "length head {}".format(len(head_hex))
                        if r is not None: r.logDebug(out["error"])
                        self.total_hex = self.total_hex[ind + 32:]
                        continue
                    last_info = self.total_hex[ind + 32:]
                    # r.logDebug("usSize: {} len_last_info {}".format(usSize, len(last_info)))
                    if usSize * 2 <= len(last_info):
                        body_hex = last_info[0:usSize * 2]
                        self.total_hex = last_info[usSize * 2:]
                        fmt = "@" + str(usSize) + "s"
                        body = struct.unpack(fmt, bytes.fromhex(body_hex))
                        try:
                            out = json.loads(body[0])
                        except:
                            print("loads error!!!", body)
                            if r is not None:
                                r.logDebug("ctu loads error!!! {}".format(body))
                            continue
                        else:
                            self.updateRes(out, r)
                    else:
                        break
                else:
                    break

    def updateRes(self, res_msg, r):
        if 'msgType' in res_msg and "seqNum" in res_msg:
            if res_msg['msgType'] == MessageType.ROBOT_COMM_RESP:
                seqNum = res_msg.get("seqNum")
                if self.liftReset_res["seqNum"] == seqNum:
                    self.finishAction(self.liftReset_res, res_msg)
                elif self.liftPos_res['seqNum'] == seqNum:
                    self.finishAction(self.liftPos_res, res_msg)
                elif self.rotateReset_res['seqNum'] == seqNum:
                    self.finishAction(self.rotateReset_res, res_msg)
                elif self.rotateAngle_res['seqNum'] == seqNum:
                    self.finishAction(self.rotateAngle_res, res_msg)
                elif self.stretchReset_res['seqNum'] == seqNum:
                    self.finishAction(self.stretchReset_res, res_msg)
                elif self.stretchPos_res['seqNum'] == seqNum:
                    self.finishAction(self.stretchPos_res, res_msg)
                elif self.fingerReset_res['seqNum'] == seqNum:
                    self.finishAction(self.fingerReset_res, res_msg)
                elif self.fingerPos_res['seqNum'] == seqNum:
                    self.finishAction(self.fingerPos_res, res_msg)
                elif self.visionReset_res['seqNum'] == seqNum:
                    self.finishAction(self.visionReset_res, res_msg)
                elif self.visionRecord_res['seqNum'] == seqNum:
                    self.finishAction(self.visionRecord_res, res_msg)
                elif self.visionReq_res['seqNum'] == seqNum:
                    self.finishAction(self.visionReq_res, res_msg)
                elif self.indicatorReq_res['seqNum'] == seqNum:
                    self.finishAction(self.indicatorReq_res, res_msg)

                elif self.switch_mode_res['seqNum'] == seqNum:
                    self.finishAction(self.switch_mode_res, res_msg)
                elif self.reset_res['seqNum'] == seqNum:
                    self.finishAction(self.reset_res, res_msg)
                elif self.resume_res['seqNum'] == seqNum:
                    self.finishAction(self.resume_res, res_msg)
                elif self.param_set_res['seqNum'] == seqNum:
                    self.finishAction(self.param_set_res, res_msg)
                elif self.internal_bin_op_res['seqNum'] == seqNum:
                    self.finishAction(self.internal_bin_op_res, res_msg)
                elif self.external_bin_op_res['seqNum'] == seqNum:
                    self.finishAction(self.external_bin_op_res, res_msg)
                elif self.preaction_res['seqNum'] == seqNum:
                    self.finishAction(self.preaction_res, res_msg)
                elif self.src_pos_res['seqNum'] == seqNum:
                    self.finishAction(self.src_pos_res, res_msg)
                elif self.task_stop_res['seqNum'] == seqNum:
                    self.finishAction(self.task_stop_res, res_msg)

            elif res_msg['msgType'] == MessageType.ROBOT_INFO_REPORT:
                self.report = res_msg

            elif res_msg['msgType'] == MessageType.ROBOT_SRC_POS_REQ:
                self.req_position = res_msg

    def initDevice(self, r):
        self.msg_init['timeStamp'] = int(round(time.time() * 1000))
        res = self.sendMessage(self.msg_init, r)
        self.isconnect = res["flag"]
        return res

    def getReport(self, r):
        self.getMsg(r)
        return self.report

    def sendMessage(self, msg, r: SimModule):
        usMagic = 0xFACEDEAD
        str_data = json.dumps(msg, separators=(',', ':'))
        byte_data = str_data.encode()

        usSize = len(byte_data)
        crcBody = crc.crcbytes(byte_data)
        # print(byte_data, byte_data.hex())
        head_fmt = "@III"
        head_bytes = struct.pack(head_fmt, usMagic, usSize, crcBody)
        # print(head_bytes.hex())
        crcHead = crc.crcbytes(head_bytes)

        fmt = "@IIII" + str(usSize) + "s"
        # print(fmt)
        sends = struct.pack(fmt, usMagic, usSize, crcBody, crcHead, byte_data)
        if r is not None:
            r.logDebug(sends.hex())
        res_msg = dict()
        try:
            msg_len = len(sends)
            total_send = 0
            t0 = time.time()
            send_suc = True
            while total_send < msg_len:
                # cur_sent = self.tcp_client.send(sends[total_send:])
                cur_sent = self.tcp_client.send(sends[total_send:], socket.MSG_NOSIGNAL)
                if cur_sent == 0:
                    r.logDebug("socket connection broken. send length is zero")
                    send_suc = False
                    break
                total_send = total_send + cur_sent
                t1 = time.time()
                if (t1 - t0) > 0.1:
                    r.logDebug("socket is too slow. ReTry!!!")
                    send_suc = False
                    break
            res_msg["flag"] = send_suc
        except:
            res_msg["flag"] = False
            res_msg["content"] = str(sys.exc_info()[0])
            r.setPickRobotWarning(55800, "socket connection broken, send message failed!")
        return res_msg

    def switch_mode(self, r, mode):
        msg = self.msg_mode_req
        self.seqNum_req += 1
        msg['seqNum'] = self.seqNum_req
        msg['modeType'] = mode
        self.mode = mode
        self.switch_mode_res['seqNum'] = self.seqNum_req
        self.switch_mode_res["status"] = Action.RUNNING
        self.switch_mode_res['res'] = self.sendMessage(msg, r)
        return self.switch_mode_res

    def robot_reset(self, r):  # 所有机构重置校零
        msg = self.msg_reset_req
        self.seqNum_req += 1
        msg['seqNum'] = self.seqNum_req
        msg['timeStamp'] = int(round(time.time() * 1000))
        self.reset_res['seqNum'] = self.seqNum_req
        self.reset_res["status"] = Action.RUNNING
        self.reset_res['res'] = self.sendMessage(msg, r)
        return self.reset_res

    def task_resume(self, r, robotId):
        msg = self.msg_resume_req
        self.seqNum_req += 1
        msg['seqNum'] = self.seqNum_req
        msg['robotId'] = robotId
        msg['settings'] = [{
            'node': 'system::action',
            'value': 'recovery'
        }]
        self.resume_res['seqNum'] = self.seqNum_req
        self.resume_res["status"] = Action.RUNNING
        self.resume_res['res'] = self.sendMessage(msg, r)
        return self.resume_res

    def param_set(self, r, robotId, box_width, box_height, box_depth, box_tag_height, box_tag_depth, shelf_tag_height,
                  conveyor_tag_height, gap_between_box):
        msg = self.msg_param_set
        self.seqNum_req += 1
        msg['seqNum'] = self.seqNum_req
        msg['robotId'] = robotId
        msg['box_width'] = box_width
        msg['box_height'] = box_height
        msg['box_depth'] = box_depth
        msg['box_tag_height'] = box_tag_height
        msg['box_tag_depth'] = box_tag_depth
        msg['shelf_tag_height'] = shelf_tag_height
        msg['conveyor_tag_height'] = conveyor_tag_height
        msg['gap_between_box'] = gap_between_box
        self.param_set_res['seqNum'] = self.seqNum_req
        self.param_set_res["status"] = Action.RUNNING
        self.param_set_res['res'] = self.sendMessage(msg, r)
        return self.param_set_res

    def internal_bin_op(self, r, robotId, opType, binId='reserve', binType=None, binModel=None, srcTray=None,
                        dstTray=None,
                        targetTray=None):
        msg = self.msg_internal_bin_op
        self.seqNum_req += 1
        msg['seqNum'] = self.seqNum_req
        msg['robotId'] = robotId
        msg['opType'] = opType
        if opType == BinOpType.MOVE:
            msg['binId'] = binId
            if binType is not None:
                msg['binType'] = binType
            else:
                del_dict_item(msg, 'binType')
            if binModel is not None:
                msg['binModel'] = binModel
            else:
                del_dict_item(msg, 'binModel')
            msg['srcTray'] = srcTray
            msg['dstTray'] = dstTray
        else:
            del_dict_item(msg, 'binId')
            del_dict_item(msg, 'binType')
            del_dict_item(msg, 'binModel')
            del_dict_item(msg, 'srcTray')
            del_dict_item(msg, 'dstTray')
        if opType == BinOpType.INSPECT:
            msg['targetTray'] = targetTray
        else:
            del_dict_item(msg, 'targetTray')
        self.internal_bin_op_res['seqNum'] = self.seqNum_req
        self.internal_bin_op_res["status"] = Action.RUNNING
        self.internal_bin_op_res['res'] = self.sendMessage(msg, r)
        return self.internal_bin_op_res

    def external_bin_op(self, r, robotId, opType, binId='reserve', targetPosition=0, targetHeight=0, binType=None,
                        binModel=None, locationType=None):
        msg = self.msg_external_bin_op
        self.seqNum_req += 1
        msg['seqNum'] = self.seqNum_req
        msg['robotId'] = robotId
        msg['opType'] = opType
        msg['binId'] = binId
        msg['targetPosition'] = targetPosition
        msg['targetHeight'] = targetHeight
        if binType is not None:
            msg['binType'] = binType
        else:
            del_dict_item(msg, 'binType')
        if binModel is not None:
            msg['binModel'] = binModel
        else:
            del_dict_item(msg, 'binModel')
        if locationType is not None:
            msg['locationType'] = locationType
        else:
            del_dict_item(msg, 'locationType')
        if opType not in [BinOpType.PUT, BinOpType.TAKE]:
            del_dict_item(msg, 'binId')
            del_dict_item(msg, 'binType')
            del_dict_item(msg, 'binModel')

        self.external_bin_op_res['seqNum'] = self.seqNum_req
        self.external_bin_op_res["status"] = Action.RUNNING
        self.external_bin_op_res['res'] = self.sendMessage(msg, r)

        return self.external_bin_op_res

    def preaction(self, r, robotId, preconditions=None):  # 仅在任务模式下使用
        msg = self.msg_preaction_req
        self.seqNum_req += 1
        msg['seqNum'] = self.seqNum_req
        msg['robotId'] = robotId
        msg['preconditions'] = preconditions  # type: json
        if preconditions is None:
            del_dict_item(msg, 'preconditions')

        self.preaction_res['seqNum'] = self.seqNum_req
        self.preaction_res["status"] = Action.RUNNING
        self.preaction_res['res'] = self.sendMessage(msg, r)
        return self.preaction_res

    def src_pos(self, r):
        req_posi = self.req_position['position']
        req_posi['coordinate'] = 'robot'
        src_status = self.go_path.status
        if req_posi['x'] < 0:
            req_posi['backMode'] = 1
        if abs(req_posi['x']) < 0.003:
            src_status = MoveStatus.FINISHED
        if not self.src_send and src_status != MoveStatus.FINISHED and src_status != MoveStatus.FAILED:
            self.go_path.run(r, req_posi)
            self.src_send = True
        if src_status == MoveStatus.FINISHED:
            self.go_path = goPath.Module(r, dict())
            self.src_send = False
            req_posi['status'] = "finish"
            self.src_pos_resp(r, req_posi)
            return True
        return False

    def src_pos_resp(self, r, position):
        msg = self.msg_src_pos_resp
        self.seqNum_req += 1
        msg['seqNum'] = self.seqNum_req
        msg['position'] = position
        self.msg_src_pos_resp['seqNum'] = self.seqNum_req
        self.msg_src_pos_resp["status"] = Action.RUNNING
        self.msg_src_pos_resp['res'] = self.sendMessage(msg, r)
        return self.msg_src_pos_resp

    def task_stop(self, r, stop_type):
        msg = self.msg_task_stop_req
        self.seqNum_req += 1
        msg['seqNum'] = self.seqNum_req
        msg['stopType'] = stop_type
        self.task_stop_res['seqNum'] = self.seqNum_req
        self.task_stop_res["status"] = Action.RUNNING
        self.task_stop_res['res'] = self.sendMessage(msg, r)
        return self.task_stop_res

    def liftReset(self, r):
        if self.liftReset_res['status'] is Action.INIT:
            self.lift_reset_stime = time.time()
            msg = self.msg_lift_reset
            self.seqNum_req = self.seqNum_req + 1
            msg["seqNum"] = self.seqNum_req
            self.liftReset_res["seqNum"] = self.seqNum_req
            self.liftReset_res["status"] = Action.RUNNING
            self.liftReset_res["res"] = dict()
            self.liftReset_res["res"] = self.sendMessage(msg, r)
            r.logDebug(f"lift reset: {msg}")
        else:
            dt = time.time() - self.lift_reset_stime
            if dt > self.reset_time:
                self.liftReset_res["status"] = Action.INIT
        return self.liftReset_res

    def liftStop(self, r):
        msg = self.msg_lift_stop
        self.seqNum_req = self.seqNum_req + 1
        msg["seqNum"] = self.seqNum_req
        self.sendMessage(msg, r)
        r.logDebug(f"lift stop: {msg}")

    def liftPos(self, height, r):
        if self.liftPos_res['status'] is Action.INIT:
            msg = self.msg_lift_req
            self.seqNum_req = self.seqNum_req + 1
            msg["seqNum"] = self.seqNum_req
            msg["liftPosition"] = height
            self.liftPos_res["seqNum"] = self.seqNum_req
            self.liftPos_res["status"] = Action.RUNNING
            self.liftPos_res["res"] = dict()
            self.liftPos_res["res"] = self.sendMessage(msg, r)
            r.logDebug(f"lift pos: {msg}")
        return self.liftPos_res

    def rotateReset(self, r):
        if self.rotateReset_res['status'] is Action.INIT:
            self.rotate_reset_stime = time.time()
            msg = self.msg_rot_rest
            self.seqNum_req = self.seqNum_req + 1
            msg["seqNum"] = self.seqNum_req
            self.rotateReset_res["seqNum"] = self.seqNum_req
            self.rotateReset_res["status"] = Action.RUNNING
            self.rotateReset_res["res"] = dict()
            self.rotateReset_res["res"] = self.sendMessage(msg, r)
            r.logDebug(f"rotate reset: {msg}")
        else:
            dt = time.time() - self.rotate_reset_stime
            if dt > self.reset_time:
                self.rotateReset_res["status"] = Action.INIT
        return self.rotateReset_res

    def rotateStop(self, r):
        msg = self.msg_rot_stop
        self.seqNum_req = self.seqNum_req + 1
        msg["seqNum"] = self.seqNum_req
        self.sendMessage(msg, r)
        r.logDebug(f"rotate stop: {msg}")

    def rotateAngle(self, theta, r):
        if self.rotateAngle_res['status'] is Action.INIT:
            msg = self.msg_rot_req
            self.seqNum_req = self.seqNum_req + 1
            msg["seqNum"] = self.seqNum_req
            msg["rotatePosition"] = theta
            self.rotateAngle_res["seqNum"] = self.seqNum_req
            self.rotateAngle_res["status"] = Action.RUNNING
            self.rotateAngle_res["res"] = dict()
            self.rotateAngle_res["res"] = self.sendMessage(msg, r)
            r.logDebug(f"rotate angle: {msg}")
        return self.rotateAngle_res

    def stretchReset(self, r):
        if self.stretchReset_res["status"] is Action.INIT:
            self.stretch_reset_stime = time.time()
            msg = self.msg_stretch_reset
            self.seqNum_req = self.seqNum_req + 1
            msg["seqNum"] = self.seqNum_req
            self.stretchReset_res["seqNum"] = self.seqNum_req
            self.stretchReset_res["status"] = Action.RUNNING
            self.stretchReset_res["res"] = dict()
            self.stretchReset_res["res"] = self.sendMessage(msg, r)
            r.logDebug(f"stretch reset: {msg}")
        else:
            dt = time.time() - self.stretch_reset_stime
            if dt > self.reset_time:
                self.stretchReset_res["status"] = Action.INIT
        return self.stretchReset_res

    def stretchStop(self, r):
        msg = self.msg_stretch_stop
        self.seqNum_req = self.seqNum_req + 1
        msg["seqNum"] = self.seqNum_req
        self.sendMessage(msg, r)
        r.logDebug(f"stretch stop: {msg}")

    def stretchPos(self, value, r):
        if self.stretchPos_res["status"] is Action.INIT:
            msg = self.msg_stretch_req
            self.seqNum_req = self.seqNum_req + 1
            msg["seqNum"] = self.seqNum_req
            msg["stretchPosition"] = value
            self.stretchPos_res["seqNum"] = self.seqNum_req
            self.stretchPos_res["status"] = Action.RUNNING
            self.stretchPos_res["res"] = dict()
            self.stretchPos_res["res"] = self.sendMessage(msg, r)
            r.logDebug(f"stretch pos: {msg}")
        return self.stretchPos_res

    def fingerReset(self, r):
        if self.fingerReset_res["status"] is Action.INIT:
            self.finger_reset_stime = time.time()
            msg = self.msg_finger_reset
            self.seqNum_req = self.seqNum_req + 1
            msg["seqNum"] = self.seqNum_req
            self.fingerReset_res["seqNum"] = self.seqNum_req
            self.fingerReset_res["status"] = Action.RUNNING
            self.fingerReset_res["res"] = dict()
            self.fingerReset_res["res"] = self.sendMessage(msg, r)
            r.logDebug(f"finger reset: {msg}")
        else:
            dt = time.time() - self.finger_reset_stime
            if dt > self.reset_time:
                self.fingerReset_res["status"] = Action.INIT
        return self.fingerReset_res

    def fingerStop(self, r):
        msg = self.msg_finger_stop
        self.seqNum_req = self.seqNum_req + 1
        msg["seqNum"] = self.seqNum_req
        self.sendMessage(msg, r)
        r.logDebug(f"finger stop: {msg}")

    def fingerPos(self, value, r):
        if self.fingerPos_res["status"] is Action.INIT:
            msg = self.msg_finger_req
            self.seqNum_req = self.seqNum_req + 1
            msg["seqNum"] = self.seqNum_req
            msg["position"] = value
            self.fingerPos_res["seqNum"] = self.seqNum_req
            self.fingerPos_res["status"] = Action.RUNNING
            self.fingerPos_res["res"] = dict()
            self.fingerPos_res["res"] = self.sendMessage(msg, r)
            r.logDebug(f"finger pos: {msg}")
        return self.fingerPos_res

    def visionReset(self, r):
        if self.visionReset_res["status"] is Action.INIT:
            self.vision_reset_stime = time.time()
            msg = self.msg_vision_reset
            self.seqNum_req = self.seqNum_req + 1
            msg["seqNum"] = self.seqNum_req
            self.visionReset_res["seqNum"] = self.seqNum_req
            self.visionReset_res["status"] = Action.RUNNING
            self.visionReset_res["res"] = dict()
            self.visionReq_res["status"] = Action.INIT
            self.visionReset_res["res"] = self.sendMessage(msg, r)
            r.logDebug(f"vision reset: {msg}")
        else:
            dt = time.time() - self.vision_reset_stime
            if dt > self.reset_time:
                self.visionReset_res["status"] = Action.INIT
        return self.visionReset_res

    def visionStop(self, r):
        msg = self.msg_vision_stop
        self.seqNum_req = self.seqNum_req + 1
        msg["seqNum"] = self.seqNum_req
        self.sendMessage(msg, r)
        r.logDebug(f"vision stop: {msg}")

    def visionReq(self, targetType, binType, binModel, r):
        if self.visionReq_res["status"] is Action.INIT:
            msg = self.msg_vision_req
            self.seqNum_req = self.seqNum_req + 1
            msg["seqNum"] = self.seqNum_req
            msg["targetType"] = targetType
            msg["binType"] = binType
            msg["binModel"] = binModel
            self.visionReq_res["seqNum"] = self.seqNum_req
            self.visionReq_res["status"] = Action.RUNNING
            self.visionReq_res["res"] = dict()
            self.visionReq_res["res"] = self.sendMessage(msg, r)
            r.logDebug(f"vision req: {msg}")
        return self.visionReq_res

    def visionRecord(self, r):
        if self.visionRecord_res["status"] is Action.INIT:
            msg = self.msg_vision_record
            self.seqNum_req = self.seqNum_req + 1
            msg["seqNum"] = self.seqNum_req
            self.visionRecord_res["seqNum"] = self.seqNum_req
            self.visionRecord_res["status"] = Action.RUNNING
            self.visionRecord_res["res"] = dict()
            self.visionRecord_res["res"] = self.sendMessage(msg, r)
        return self.visionRecord_res

    def indicatorReq(self, chassisLedFront=None, chassisLedBack=None, buzzer=None, headLedRed=None, headLedYellow=None,
                     headLedGreen=None, headLedFreq=None, r=None):
        if self.indicatorReq_res["status"] is Action.INIT:
            msg = self.msg_indicator_req
            self.seqNum_req = self.seqNum_req + 1
            msg["seqNum"] = self.seqNum_req
            if chassisLedFront is not None:
                msg["chassisLedFront"] = chassisLedFront
            else:
                del msg["chassisLedFront"]
            if chassisLedBack is not None:
                msg["chassisLedBack"] = chassisLedBack
            else:
                del msg["chassisLedBack"]
            if buzzer is not None:
                msg["buzzer"] = buzzer
            else:
                del msg["buzzer"]
            if headLedRed is not None:
                msg["headLedRed"] = headLedRed
            else:
                del msg["headLedRed"]
            if headLedYellow is not None:
                msg["headLedYellow"] = headLedYellow
            else:
                del msg["headLedYellow"]
            if headLedGreen is not None:
                msg["headLedGreen"] = headLedGreen
            else:
                del msg["headLedGreen"]
            if headLedFreq is not None:
                msg["headLedFreq"] = headLedFreq
            else:
                del msg["headLedFreq"]
            self.indicatorReq_res["seqNum"] = self.seqNum_req
            self.indicatorReq_res["status"] = Action.RUNNING
            self.indicatorReq_res["res"] = dict()
            self.indicatorReq_res["res"] = self.sendMessage(msg, r)
        return self.indicatorReq_res


if __name__ == "__main__":
    h = PickRobot("192.168.192.20", 4172)
    r = None
    h.connect()
    print(h.initDevice(r))
    print(h.isconnect)
    h.indicatorReq(headLedRed=1)
    h.initDevice(r)
    print(h.isconnect)
    state = h.getReport(r)
    print(state)
    h.disconnect()
