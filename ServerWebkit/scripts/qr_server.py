# -- coding: utf-8 --
import sys
import platform
import os
import cv2
import numpy as np
import math
import time
import socket
import threading
import queue
from ctypes import *
from ultralytics import YOLO
import robot_controller as rc

# 兼容不同操作系统加载动态库
currentsystem = platform.system()
if currentsystem == 'Windows':
    sys.path.append(os.path.join(os.getenv('MVCAM_COMMON_RUNENV'), "Samples", "Python", "MvImport"))
else:
    sys.path.append(os.path.join("..", "..", "MvImport"))

from MvCameraControl_class import *

# ================== 用户配置区域 ==================
IMAGE_CENTER_X = 512
IMAGE_CENTER_Y = 640
CENTER_ALIGN_THRESHOLD = 150
PIXELS_PER_CM = 25.6
MODEL_PATH = 'best.pt'
CONFIDENCE_THRESHOLD = 0.1

move_horizontal_cm = 0
# ================================================

class RealTimeCamera:
    """实时相机控制类，确保获取和处理同步"""
    def __init__(self, rotate_90_degrees=True):
        self.cam = None
        self.is_streaming = False
        self.rotate_90_degrees = rotate_90_degrees
        self.original_width = 0
        self.original_height = 0
        self.rotated_width = 0
        self.rotated_height = 0
        self.latest_frame = None
        self.frame_lock = threading.Lock()
        self.frame_count = 0
        self.grabbing_thread = None
        self.stop_event = threading.Event()
        self.frame_ready = threading.Event()
        
    def connect_camera(self):
        """连接第一个可用相机"""
        try:
            MvCamera.MV_CC_Initialize()
            
            deviceList = MV_CC_DEVICE_INFO_LIST()
            tlayerType = (MV_GIGE_DEVICE | MV_USB_DEVICE)
            
            ret = MvCamera.MV_CC_EnumDevices(tlayerType, deviceList)
            if ret != 0 or deviceList.nDeviceNum == 0:
                print("[ERROR] 未找到可用相机设备")
                return False
            
            stDeviceList = cast(deviceList.pDeviceInfo[0], POINTER(MV_CC_DEVICE_INFO)).contents
            
            self.cam = MvCamera()
            ret = self.cam.MV_CC_CreateHandle(stDeviceList)
            if ret != 0:
                print("[ERROR] 创建相机句柄失败")
                return False
                
            ret = self.cam.MV_CC_OpenDevice(MV_ACCESS_Exclusive, 0)
            if ret != 0:
                print("[ERROR] 打开相机设备失败")
                return False
                
            # 设置为连续采集模式
            self.cam.MV_CC_SetEnumValue("TriggerMode", MV_TRIGGER_MODE_OFF)
            self.cam.MV_CC_SetEnumValue("AcquisitionMode", 2)  # 连续采集
            
            print("[INFO] 相机连接成功")
            return True
            
        except Exception as e:
            print(f"[ERROR] 连接相机失败: {e}")
            return False
    
    def _grab_frames(self):
        """持续抓取帧的线程函数"""
        print("[INFO] 开始采集线程")
        
        while not self.stop_event.is_set():
            try:
                stOutFrame = MV_FRAME_OUT()  
                memset(byref(stOutFrame), 0, sizeof(stOutFrame))
                
                # 获取最新帧（不等待）
                ret = self.cam.MV_CC_GetImageBuffer(stOutFrame, 100)
                
                if ret == 0 and stOutFrame.pBufAddr is not None:
                    nWidth = stOutFrame.stFrameInfo.nWidth
                    nHeight = stOutFrame.stFrameInfo.nHeight
                    
                    self.original_width = nWidth
                    self.original_height = nHeight
                    
                    if self.rotate_90_degrees:
                        self.rotated_width = nHeight
                        self.rotated_height = nWidth
                    else:
                        self.rotated_width = nWidth
                        self.rotated_height = nHeight
                    
                    # 计算数据大小
                    if stOutFrame.stFrameInfo.enPixelType == 0x01080001:  # Mono8
                        data_size = nWidth * nHeight
                    elif stOutFrame.stFrameInfo.enPixelType == 0x02180014:  # RGB8_Packed
                        data_size = nWidth * nHeight * 3
                    elif stOutFrame.stFrameInfo.enPixelType == 0x0218000C:  # BGR8_Packed
                        data_size = nWidth * nHeight * 3
                    else:
                        data_size = nWidth * nHeight * 3
                    
                    # 从指针获取数据
                    buffer_ptr = cast(stOutFrame.pBufAddr, POINTER(c_ubyte * data_size))
                    buffer_data = bytearray(buffer_ptr.contents)
                    image_array = np.frombuffer(buffer_data, dtype=np.uint8)
                    
                    # 转换为OpenCV图像
                    if stOutFrame.stFrameInfo.enPixelType == 0x01080001:  # Mono8
                        frame = image_array.reshape((nHeight, nWidth))
                        frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
                    elif stOutFrame.stFrameInfo.enPixelType == 0x02180014:  # RGB8_Packed
                        frame = image_array.reshape((nHeight, nWidth, 3))
                        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                    elif stOutFrame.stFrameInfo.enPixelType == 0x0218000C:  # BGR8_Packed
                        frame = image_array.reshape((nHeight, nWidth, 3))
                    else:
                        frame = image_array.reshape((nHeight, nWidth, 3))
                    
                    # 自动旋转90度
                    if self.rotate_90_degrees:
                        frame = cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
                    
                    # 更新最新帧
                    with self.frame_lock:
                        self.latest_frame = frame.copy()
                        self.frame_count += 1
                        self.frame_ready.set()  # 通知有新帧可用
                    
                    # 释放缓冲区
                    self.cam.MV_CC_FreeImageBuffer(stOutFrame)
                    
                else:
                    # 获取失败，短暂等待
                    time.sleep(0.001)
                    
            except Exception as e:
                print(f"[ERROR] 采集线程异常: {e}")
                time.sleep(0.01)
        
        print("[INFO] 采集线程停止")
    
    def start_streaming(self):
        """开始实时采集"""
        if not self.cam or self.is_streaming:
            return False
        
        try:
            ret = self.cam.MV_CC_StartGrabbing()
            if ret != 0:
                print("[ERROR] 启动采集失败")
                return False
            
            self.is_streaming = True
            self.stop_event.clear()
            self.frame_ready.clear()
            
            # 启动采集线程
            self.grabbing_thread = threading.Thread(target=self._grab_frames, daemon=True)
            self.grabbing_thread.start()
            
            # 等待第一帧
            if self.frame_ready.wait(timeout=1.0):
                print(f"[INFO] 实时采集已启动，帧大小: {self.rotated_width}x{self.rotated_height}")
                return True
            else:
                print("[ERROR] 等待第一帧超时")
                return False
            
        except Exception as e:
            print(f"[ERROR] 启动实时采集失败: {e}")
            return False
    
    def get_latest_frame(self, timeout=0.1):
        """获取最新的帧（同步）"""
        if not self.is_streaming:
            return None
        
        # 等待新帧可用
        if self.frame_ready.wait(timeout=timeout):
            with self.frame_lock:
                frame = self.latest_frame.copy() if self.latest_frame is not None else None
                self.frame_ready.clear()  # 清除事件，等待下一帧
                return frame
        else:
            return None
    
    def clear_buffer(self):
        """清除缓冲的旧帧"""
        with self.frame_lock:
            self.latest_frame = None
            self.frame_ready.clear()
    
    def stop_streaming(self):
        """停止采集"""
        if not self.is_streaming:
            return
        
        self.stop_event.set()
        self.is_streaming = False
        
        if self.grabbing_thread and self.grabbing_thread.is_alive():
            self.grabbing_thread.join(timeout=2.0)
        
        if self.cam:
            try:
                self.cam.MV_CC_StopGrabbing()
                self.cam.MV_CC_CloseDevice()
                self.cam.MV_CC_DestroyHandle()
                MvCamera.MV_CC_Finalize()
            except:
                pass
        
        print("[INFO] 相机采集已停止")
    
    def get_frame_rate(self):
        """获取帧率（每秒）"""
        with self.frame_lock:
            return self.frame_count
    
    def reset_frame_count(self):
        """重置帧计数器"""
        with self.frame_lock:
            self.frame_count = 0


class YOLODetector:
    """基于YOLO的实时二维码检测器"""
    def __init__(self, model_path=MODEL_PATH, conf_threshold=CONFIDENCE_THRESHOLD,
                 center_x=IMAGE_CENTER_X, center_y=IMAGE_CENTER_Y,
                 align_threshold=CENTER_ALIGN_THRESHOLD, pixels_per_cm=PIXELS_PER_CM):
        # 加载YOLO模型
        print(f"[INFO] 正在加载YOLO模型: {model_path}")
        self.model = YOLO(model_path)
        self.conf_threshold = conf_threshold
        self.center_x = center_x
        self.center_y = center_y
        self.align_threshold = align_threshold
        self.pixels_per_cm = pixels_per_cm
        
        # 检测统计
        self.detection_count = 0
        self.last_detection_time = 0
        self.avg_processing_time = 0
        self.processing_count = 0
        
        print(f"[INFO] YOLO检测器初始化完成，中心点: ({center_x}, {center_y})")
    
    def detect_realtime(self, frame):
        """实时检测（同步调用，确保使用最新帧）"""
        if frame is None:
            return []
        
        start_time = time.time()
        
        try:
            # 使用YOLO进行检测
            results = self.model(frame, conf=self.conf_threshold, verbose=False)
            result = results[0]
            
            if result.boxes is None or len(result.boxes) == 0:
                processing_time = (time.time() - start_time) * 1000
                self._update_processing_time(processing_time)
                return []
            
            detections = []
            for i, box in enumerate(result.boxes):
                # 获取边界框坐标
                x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
                confidence = float(box.conf[0])
                
                # 计算中心点
                center_x = (x1 + x2) // 2
                center_y = (y1 + y2) // 2
                width = x2 - x1
                height = y2 - y1
                
                # 计算偏移量
                horizontal_offset = center_x - self.center_x
                vertical_offset = center_y - self.center_y
                horizontal_cm = horizontal_offset / self.pixels_per_cm
                vertical_cm = vertical_offset / self.pixels_per_cm

                print("///////////////////////////////")
                print(horizontal_cm)
                move_horizontal_cm = horizontal_cm
                print("///////////////////////////////")
                
                # 判断是否对齐
                is_aligned = abs(horizontal_offset) <= self.align_threshold
                direction = "右" if horizontal_offset > 0 else "左" if horizontal_offset < 0 else "居中"
                
                detection = {
                    'x1': x1, 'y1': y1, 'x2': x2, 'y2': y2,
                    'center_x': center_x,
                    'center_y': center_y,
                    'width': width,
                    'height': height,
                    'horizontal_offset': horizontal_offset,
                    'vertical_offset': vertical_offset,
                    'horizontal_cm': horizontal_cm,
                    'vertical_cm': vertical_cm,
                    'direction': direction,
                    'is_aligned': is_aligned,
                    'confidence': confidence,
                    'timestamp': time.time(),
                    'detection_time': start_time
                }
                
                detections.append(detection)
            
            self.detection_count += len(detections)
            processing_time = (time.time() - start_time) * 1000
            self._update_processing_time(processing_time)
            
            return detections
            
        except Exception as e:
            print(f"[YOLO] 检测异常: {e}")
            return []
    
    def _update_processing_time(self, processing_time):
        """更新平均处理时间"""
        self.processing_count += 1
        self.avg_processing_time = (self.avg_processing_time * (self.processing_count - 1) + processing_time) / self.processing_count
    
    def get_best_detection(self, detections):
        """获取最佳检测结果"""
        if not detections:
            return None
        
        best_det = None
        best_score = -float('inf')
        
        for det in detections:
            # 评分：置信度 + 距离中心的加权
            distance = abs(det['center_x'] - self.center_x) / self.center_x
            score = det['confidence'] * 0.8 + (1 - distance) * 0.2
            
            if score > best_score:
                best_score = score
                best_det = det
        
        return best_det
    
    def draw_detections(self, frame, detections):
        """在图像上绘制检测结果"""
        if frame is None or not detections:
            return frame
        
        output_frame = frame.copy()
        
        for det in detections:
            # 绘制边界框
            cv2.rectangle(output_frame, (det['x1'], det['y1']), (det['x2'], det['y2']), (0, 255, 0), 2)
            
            # 绘制中心点
            cv2.circle(output_frame, (det['center_x'], det['y1'] - 5), 3, (0, 0, 255), -1)
            cv2.line(output_frame, (det['center_x'], det['y1'] - 10), (det['center_x'], det['y1'] + 10), (0, 0, 255), 1)
            cv2.line(output_frame, (det['center_x'] - 10, det['y1']), (det['center_x'] + 10, det['y1']), (0, 0, 255), 1)
            
            # 绘制文本
            label = f"Conf: {det['confidence']:.2f}"
            offset_label = f"Offset: {det['horizontal_offset']:+d}px ({det['horizontal_cm']:+.2f}cm)"
            
            cv2.putText(output_frame, label, (det['x1'], det['y1']-10), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
            cv2.putText(output_frame, offset_label, (det['x1'], det['y1']-30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
        
        # 绘制图像中心
        cv2.circle(output_frame, (self.center_x, self.center_y), 8, (255, 0, 0), 2)
        cv2.line(output_frame, (self.center_x, 0), (self.center_x, frame.shape[0]), (255, 0, 0), 1)
        cv2.line(output_frame, (0, self.center_y), (frame.shape[1], self.center_y), (255, 0, 0), 1)
        
        # 绘制对齐阈值区域
        cv2.line(output_frame, (self.center_x - self.align_threshold, 0), 
                (self.center_x - self.align_threshold, frame.shape[0]), (0, 255, 255), 1)
        cv2.line(output_frame, (self.center_x + self.align_threshold, 0), 
                (self.center_x + self.align_threshold, frame.shape[0]), (0, 255, 255), 1)
        
        return output_frame
    
    def get_stats(self):
        """获取检测统计"""
        return {
            'total_detections': self.detection_count,
            'avg_processing_time': self.avg_processing_time,
            'total_processed': self.processing_count
        }


class RealTimeQRServer:
    """实时二维码检测服务器"""
    def __init__(self, host='0.0.0.0', port=8888, 
                 center_x=IMAGE_CENTER_X, center_y=IMAGE_CENTER_Y,
                 threshold=CENTER_ALIGN_THRESHOLD, pixels_per_cm=PIXELS_PER_CM,
                 conf_threshold=CONFIDENCE_THRESHOLD):
        self.host = host
        self.port = port
        
        # 服务器状态
        self.server_socket = None
        self.client_socket = None
        self.client_address = None
        self.is_running = False
        self.scanning = False
        
        # 硬件组件
        self.camera = None
        self.detector = None
        
        # 校准参数
        self.center_x = center_x
        self.center_y = center_y
        self.align_threshold = threshold
        self.pixels_per_cm = pixels_per_cm
        self.conf_threshold = conf_threshold
        
        # 校准状态
        self.center_align_count = 0
        self.required_center_count = 5
        self.consecutive_failures = 0
        self.max_failures = 10
        
        # 显示控制
        self.show_video = True
        self.show_stats = True
        self.frame_rate = 0
        self.last_fps_time = time.time()
        self.fps_counter = 0
        
        # AGV移动参数
        self.agv_move_distance = 0.1
        self.agv_move_speed = 0.3
        self.agv_wait_time = 3
        
        print(f"[INFO] 实时QR服务器初始化完成")
        print(f"[INFO] 中心点: ({center_x}, {center_y}), 阈值: {threshold}px")
        print(f"[INFO] 每厘米像素: {pixels_per_cm}, 置信度阈值: {conf_threshold}")
    
    def start(self):
        """启动服务器"""
        try:
            # 创建TCP服务器
            self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.server_socket.bind((self.host, self.port))
            self.server_socket.listen(1)
            self.server_socket.settimeout(1.0)
            
            self.is_running = True
            print(f"[INFO] 实时QR服务器启动在 {self.host}:{self.port}")
            print(f"[INFO] 等待客户端连接...")
            
            # 初始化相机和检测器
            if not self.initialize_hardware():
                print("[ERROR] 硬件初始化失败，服务器停止")
                return False
            
            # 启动主循环
            self.main_loop()
            return True
            
        except Exception as e:
            print(f"[ERROR] 启动服务器失败: {e}")
            return False
        finally:
            self.stop()
    
    def initialize_hardware(self):
        """初始化相机和检测器"""
        try:
            # 初始化相机
            print("[INFO] 正在初始化相机...")
            self.camera = RealTimeCamera(rotate_90_degrees=True)
            if not self.camera.connect_camera():
                print("[ERROR] 无法连接相机")
                return False
            
            if not self.camera.start_streaming():
                print("[ERROR] 无法启动相机采集")
                return False
            
            # 等待第一帧
            time.sleep(0.1)
            test_frame = self.camera.get_latest_frame(timeout=0.5)
            if test_frame is None:
                print("[ERROR] 无法获取相机帧")
                return False
            
            print(f"[INFO] 相机初始化成功，图像尺寸: {test_frame.shape[1]}x{test_frame.shape[0]}")
            
            # 初始化YOLO检测器
            print("[INFO] 正在初始化YOLO检测器...")
            self.detector = YOLODetector(
                conf_threshold=self.conf_threshold,
                center_x=self.center_x,
                center_y=self.center_y,
                align_threshold=self.align_threshold,
                pixels_per_cm=self.pixels_per_cm
            )
            
            # 测试检测
            test_detections = self.detector.detect_realtime(test_frame)
            print(f"[INFO] YOLO检测器初始化成功，测试检测: {len(test_detections)} 个二维码")
            
            return True
            
        except Exception as e:
            print(f"[ERROR] 硬件初始化失败: {e}")
            return False
    
    def main_loop(self):
        """主循环"""
        last_client_check = time.time()
        
        while self.is_running:
            try:
                # 处理客户端连接
                if self.client_socket is None:
                    self.accept_client()
                else:
                    self.handle_client()
                
                # 定期检查客户端连接状态
                if time.time() - last_client_check > 5:
                    last_client_check = time.time()
                    if self.client_socket:
                        try:
                            self.client_socket.send(b'PING\n')
                        except:
                            print("[INFO] 客户端连接已断开")
                            self.disconnect_client()
                
                # 如果没有客户端连接，显示实时预览
                if self.show_video and (self.client_socket is None or not self.scanning):
                    self.display_preview()
                
                # 如果正在扫描，执行扫描任务
                if self.scanning:
                    self.scanning_task()
                
            except KeyboardInterrupt:
                print("\n[INFO] 收到中断信号")
                break
            except Exception as e:
                print(f"[ERROR] 主循环异常: {e}")
                time.sleep(0.1)
    
    def accept_client(self):
        """接受客户端连接"""
        try:
            client_socket, client_address = self.server_socket.accept()
            self.client_socket = client_socket
            self.client_address = client_address
            self.scanning = False
            self.center_align_count = 0
            
            print(f"[INFO] 客户端已连接: {client_address}")
            self.send_to_client("READY: 实时二维码检测服务器已就绪")
            
        except socket.timeout:
            pass
        except Exception as e:
            print(f"[ERROR] 接受客户端连接失败: {e}")
    
    def handle_client(self):
        """处理客户端消息"""
        try:
            self.client_socket.settimeout(0.1)
            
            try:
                data = self.client_socket.recv(1024)
                if data:
                    command = data.decode('utf-8').strip()
                    self.process_command(command)
            except socket.timeout:
                pass
            except ConnectionResetError:
                print(f"[INFO] 客户端断开连接: {self.client_address}")
                self.disconnect_client()
                return
            
        except Exception as e:
            print(f"[ERROR] 处理客户端消息失败: {e}")
            self.disconnect_client()
    
    def process_command(self, command):
        """处理客户端命令"""
        cmd_upper = command.upper()
        
        if cmd_upper == "SCAN":
            if not self.scanning:
                self.scanning = True
                self.center_align_count = 0
                self.consecutive_failures = 0
                print(f"[INFO] 开始实时扫描任务")
                self.send_to_client("START: 开始实时扫描任务")
                
        elif cmd_upper == "STOP":
            if self.scanning:
                self.scanning = False
                print(f"[INFO] 停止扫描任务")
                self.send_to_client("STOP: 扫描任务已停止")
                
        elif cmd_upper == "STATUS":
            status = "SCANNING" if self.scanning else "IDLE"
            fps_info = f", 帧率: {self.frame_rate:.1f}FPS" if self.frame_rate > 0 else ""
            stats = self.detector.get_stats() if self.detector else {}
            stats_info = f", 检测次数: {stats.get('total_detections', 0)}" if stats else ""
            self.send_to_client(f"STATUS: {status}, 对齐次数: {self.center_align_count}{fps_info}{stats_info}")
            
        elif cmd_upper == "CALIBRATE_CENTER":
            # 自动校准中心点
            self.calibrate_center()
            
        elif cmd_upper.startswith("SET_CENTER "):
            try:
                _, params = command.split(" ", 1)
                x, y = map(int, params.split())
                self.center_x = x
                self.center_y = y
                self.detector.center_x = x
                self.detector.center_y = y
                self.send_to_client(f"SET_CENTER: 中心点已设置为 ({x}, {y})")
            except:
                self.send_to_client("ERROR: 格式错误，使用 SET_CENTER x y")
                
        elif cmd_upper.startswith("SET_THRESHOLD "):
            try:
                _, threshold = command.split(" ", 1)
                self.align_threshold = int(threshold)
                self.detector.align_threshold = int(threshold)
                self.send_to_client(f"SET_THRESHOLD: 对齐阈值已设置为 {threshold}px")
            except:
                self.send_to_client("ERROR: 格式错误，使用 SET_THRESHOLD 数值")
                
        elif cmd_upper.startswith("SET_CONF "):
            try:
                _, conf = command.split(" ", 1)
                conf_value = float(conf)
                self.conf_threshold = conf_value
                self.detector.conf_threshold = conf_value
                self.send_to_client(f"SET_CONF: 置信度阈值已设置为 {conf_value}")
            except:
                self.send_to_client("ERROR: 格式错误，使用 SET_CONF 数值")
                
        elif cmd_upper == "PREVIEW_ON":
            self.show_video = True
            self.send_to_client("PREVIEW_ON: 视频预览已开启")
            
        elif cmd_upper == "PREVIEW_OFF":
            self.show_video = False
            self.send_to_client("PREVIEW_OFF: 视频预览已关闭")
            
        elif cmd_upper == "STATS_ON":
            self.show_stats = True
            self.send_to_client("STATS_ON: 统计信息已开启")
            
        elif cmd_upper == "STATS_OFF":
            self.show_stats = False
            self.send_to_client("STATS_OFF: 统计信息已关闭")
            
        elif cmd_upper == "GET_CENTER":
            self.send_to_client(f"CENTER: {self.center_x} {self.center_y}")
            
        elif cmd_upper == "GET_THRESHOLD":
            self.send_to_client(f"THRESHOLD: {self.align_threshold}")
            
        elif cmd_upper == "GET_STATS":
            stats = self.detector.get_stats() if self.detector else {}
            self.send_to_client(f"STATS: 检测次数={stats.get('total_detections', 0)}, 平均处理时间={stats.get('avg_processing_time', 0):.1f}ms")
            
        elif cmd_upper == "EXIT":
            self.send_to_client("BYE: 服务器关闭")
            self.is_running = False
            
        else:
            self.send_to_client(f"ERROR: 未知命令: {command}")
    
    def calibrate_center(self):
        """自动校准中心点"""
        print("[INFO] 开始自动校准中心点...")
        self.send_to_client("CALIBRATING: 正在自动校准中心点")
        
        samples = []
        for _ in range(10):  # 采集10个样本
            frame = self.camera.get_latest_frame(timeout=0.1)
            if frame is not None:
                detections = self.detector.detect_realtime(frame)
                if detections:
                    best = self.detector.get_best_detection(detections)
                    if best:
                        samples.append((best['center_x'], best['center_y']))
            time.sleep(0.05)
        
        if samples:
            avg_x = int(np.mean([s[0] for s in samples]))
            avg_y = int(np.mean([s[1] for s in samples]))
            self.center_x = avg_x
            self.center_y = avg_y
            self.detector.center_x = avg_x
            self.detector.center_y = avg_y
            
            print(f"[INFO] 校准完成: 新中心点 ({avg_x}, {avg_y})")
            self.send_to_client(f"CALIBRATED: 中心点已校准为 ({avg_x}, {avg_y})")
        else:
            print("[WARN] 校准失败: 未检测到二维码")
            self.send_to_client("CALIBRATE_FAILED: 校准失败，未检测到二维码")
    
    def display_preview(self):
        """显示实时预览"""
        try:
            # 获取最新帧
            frame = self.camera.get_latest_frame(timeout=0.05)
            if frame is None:
                return
            
            # 计算帧率
            self.fps_counter += 1
            current_time = time.time()
            if current_time - self.last_fps_time >= 1.0:
                self.frame_rate = self.fps_counter / (current_time - self.last_fps_time)
                self.fps_counter = 0
                self.last_fps_time = current_time
            
            # 检测二维码
            detections = self.detector.detect_realtime(frame)
            frame_with_det = self.detector.draw_detections(frame, detections)
            
            # 显示统计信息
            if self.show_stats:
                stats = self.detector.get_stats()
                
                # 左上角信息
                info_y = 20
                cv2.putText(frame_with_det, f"FPS: {self.frame_rate:.1f}", (10, info_y), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
                cv2.putText(frame_with_det, f"Center: ({self.center_x}, {self.center_y})", (10, info_y+20), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
                cv2.putText(frame_with_det, f"Threshold: {self.align_threshold}px", (10, info_y+40), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
                cv2.putText(frame_with_det, f"Detections: {len(detections)}", (10, info_y+60), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
                cv2.putText(frame_with_det, f"Process: {stats.get('avg_processing_time', 0):.1f}ms", (10, info_y+80), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
            
            # 右上角状态
            status = "SCANNING" if self.scanning else "IDLE"
            status_color = (0, 255, 0) if not self.scanning else (0, 0, 255)
            cv2.putText(frame_with_det, f"状态: {status}", (frame_with_det.shape[1] - 150, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, status_color, 2)
            
            if self.scanning:
                cv2.putText(frame_with_det, f"对齐: {self.center_align_count}/{self.required_center_count}", 
                           (frame_with_det.shape[1] - 200, 60), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            
            cv2.imshow("实时二维码检测 - 按Q退出", frame_with_det)
            
            if cv2.waitKey(1) & 0xFF == ord('q'):
                self.is_running = False
                
        except Exception as e:
            print(f"[PREVIEW] 预览异常: {e}")
    
    def scanning_task(self):
        """执行扫描任务 - 100%实时"""
        try:
            # 获取最新帧（确保是此时此刻的）
            frame = self.camera.get_latest_frame(timeout=0.05)
            if frame is None:
                self.consecutive_failures += 1
                if self.consecutive_failures >= self.max_failures:
                    print("[ERROR] 连续获取图像失败，停止扫描")
                    self.send_to_client("ERROR: 无法获取图像，停止扫描")
                    self.scanning = False
                time.sleep(0.05)
                return
            
            self.consecutive_failures = 0
            detection_start = time.time()
            
            # 实时检测
            detections = self.detector.detect_realtime(frame)
            detection_time = (time.time() - detection_start) * 1000
            
            if detections:
                # 获取最佳检测结果
                best_det = self.detector.get_best_detection(detections)
                
                if best_det:
                    offset = best_det['horizontal_offset']
                    offset_cm = best_det['horizontal_cm']
                    is_aligned = best_det['is_aligned']
                    direction = best_det['direction']
                    
                    timestamp = time.strftime("%H:%M:%S")
                    print(f"[{timestamp}] QR检测 | 偏移: {offset:+d}px ({offset_cm:+.2f}cm) {direction} | 对齐: {'是' if is_aligned else '否'} | 时间: {detection_time:.1f}ms")
                    
                    self.send_to_client(f"QR_DETECTED: 偏移={offset:+d}px ({offset_cm:+.2f}cm) {direction}")
                    
                    if is_aligned:
                        self.center_align_count += 1
                        self.send_to_client(f"ALIGNED: 第 {self.center_align_count} 次中心对齐")
                        
                        if self.center_align_count >= self.required_center_count:
                            print(f"[SUCCESS] 已达到连续 {self.required_center_count} 次中心对齐，任务完成")
                            self.send_to_client(f"COMPLETE: 已达到连续 {self.required_center_count} 次中心对齐，任务完成")
                            self.scanning = False
                            self.center_align_count = 0
                    else:
                        # 重置中心对齐计数
                        old_count = self.center_align_count
                        self.center_align_count = 0
                        
                        # 根据偏移方向控制AGV
                        if offset > 0:  # 向右偏移
                            adjust_msg = f"向右调整 {-offset_cm:.2f}cm"
                            result = rc.move_backward(distance=abs(move_horizontal_cm/100), speed=self.agv_move_speed)
                        else:  # 向左偏移
                            adjust_msg = f"向左调整 {-offset_cm:.2f}cm"
                            result = rc.move_forward(distance=abs(move_horizontal_cm/100), speed=self.agv_move_speed)
                        
                        print(f"[ADJUST] 偏离中心，{adjust_msg}，结果: {result}")
                        self.send_to_client(f"ADJUST: 偏离中心区域 ({old_count}次对齐被重置)，需要{adjust_msg}")
                        
                        # 等待AGV移动完成
                        time.sleep(self.agv_wait_time)
            else:
                timestamp = time.strftime("%H:%M:%S")
                print(f"[{timestamp}] 未检测到二维码 | 时间: {detection_time:.1f}ms")
                self.center_align_count = 0
            
            time.sleep(3)
            
            # 保持实时性，不额外sleep
            
        except Exception as e:
            print(f"[ERROR] 扫描任务异常: {e}")
            time.sleep(0.1)
    
    def send_to_client(self, message):
        """发送消息到客户端"""
        try:
            if self.client_socket:
                self.client_socket.sendall(f"{message}\n".encode('utf-8'))
        except Exception as e:
            print(f"[ERROR] 发送消息到客户端失败: {e}")
    
    def disconnect_client(self):
        """断开客户端连接"""
        if self.client_socket:
            try:
                self.client_socket.close()
            except:
                pass
            self.client_socket = None
            self.client_address = None
            self.scanning = False
            self.center_align_count = 0
            print("[INFO] 客户端已断开")
    
    def stop(self):
        """停止服务器"""
        self.is_running = False
        self.scanning = False
        
        if self.camera:
            self.camera.stop_streaming()
        
        if self.client_socket:
            self.disconnect_client()
        
        if self.server_socket:
            try:
                self.server_socket.close()
            except:
                pass
        
        cv2.destroyAllWindows()
        print("[INFO] 服务器已停止")


def main():
    """主函数"""
    import argparse
    
    parser = argparse.ArgumentParser(description='实时YOLO二维码检测服务器')
    parser.add_argument('--host', type=str, default='0.0.0.0', help='服务器主机地址 (默认: 0.0.0.0)')
    parser.add_argument('--port', type=int, default=8888, help='服务器端口 (默认: 8888)')
    parser.add_argument('--model', type=str, default=MODEL_PATH, 
                       help=f'YOLO模型路径 (默认: {MODEL_PATH})')
    parser.add_argument('--conf', type=float, default=CONFIDENCE_THRESHOLD,
                       help=f'置信度阈值 (默认: {CONFIDENCE_THRESHOLD})')
    parser.add_argument('--center-x', type=int, default=IMAGE_CENTER_X, 
                       help=f'图像水平中心点 (默认: {IMAGE_CENTER_X})')
    parser.add_argument('--center-y', type=int, default=IMAGE_CENTER_Y,
                       help=f'图像垂直中心点 (默认: {IMAGE_CENTER_Y})')
    parser.add_argument('--threshold', type=int, default=CENTER_ALIGN_THRESHOLD,
                       help=f'中心对齐阈值(像素) (默认: {CENTER_ALIGN_THRESHOLD})')
    parser.add_argument('--pixels-per-cm', type=float, default=PIXELS_PER_CM,
                       help=f'每厘米像素数 (默认: {PIXELS_PER_CM})')
    parser.add_argument('--agv-distance', type=float, default=0.3,
                       help='AGV移动距离(米) (默认: 0.3)')
    parser.add_argument('--agv-speed', type=float, default=0.3,
                       help='AGV移动速度 (默认: 0.3)')
    parser.add_argument('--agv-wait', type=float, default=3.0,
                       help='AGV移动后等待时间(秒) (默认: 3.0)')
    parser.add_argument('--no-preview', action='store_true', help='禁用视频预览')
    parser.add_argument('--no-stats', action='store_true', help='禁用统计信息')
    
    args = parser.parse_args()
    
    print("=" * 60)
    print("实时YOLO二维码检测服务器")
    print("=" * 60)
    print(f"服务器地址: {args.host}:{args.port}")
    print(f"YOLO模型: {args.model}")
    print(f"置信度阈值: {args.conf}")
    print(f"图像中心点: ({args.center_x}, {args.center_y})")
    print(f"对齐阈值: {args.threshold} 像素")
    print(f"像素-厘米转换: {args.pixels_per_cm} 像素/厘米")
    print(f"AGV参数: 距离={args.agv_distance}m, 速度={args.agv_speed}, 等待={args.agv_wait}s")
    print(f"连续对齐次数: 5 次")
    print(f"视频预览: {'启用' if not args.no_preview else '禁用'}")
    print(f"统计信息: {'启用' if not args.no_stats else '禁用'}")
    print("=" * 60)
    print("[提示] 按Q键退出程序")
    print("=" * 60)
    
    # 创建并启动服务器
    server = RealTimeQRServer(
        host=args.host, 
        port=args.port,
        center_x=args.center_x,
        center_y=args.center_y,
        threshold=args.threshold,
        pixels_per_cm=args.pixels_per_cm,
        conf_threshold=args.conf
    )
    
    server.show_video = not args.no_preview
    server.show_stats = not args.no_stats
    server.agv_move_distance = args.agv_distance
    server.agv_move_speed = args.agv_speed
    server.agv_wait_time = args.agv_wait
    
    try:
        server.start()
    except KeyboardInterrupt:
        print("\n[INFO] 检测到Ctrl+C，正在停止服务器...")
    except Exception as e:
        print(f"[ERROR] 服务器运行异常: {e}")
    finally:
        server.stop()
    
    print("[INFO] 程序已退出")


if __name__ == "__main__":
    main()