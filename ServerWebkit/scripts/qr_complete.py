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
import json
from ctypes import *
from pyzbar.pyzbar import decode, ZBarSymbol
import robot_controller as rc

# 兼容不同操作系统加载动态库
currentsystem = platform.system()
if currentsystem == 'Windows':
    sys.path.append(os.path.join(os.getenv('MVCAM_COMMON_RUNENV'), "Samples", "Python", "MvImport"))
else:
    sys.path.append(os.path.join("..", "..", "MvImport"))

from MvCameraControl_class import *

# ================== 用户配置区域 ==================
# 注意：以下中心点坐标是基于旋转90度后的坐标系
IMAGE_CENTER_X = 612  # 旋转后图像水平中心点（原图像高度）
IMAGE_CENTER_Y = 640  # 旋转后图像垂直中心点（原图像宽度）
CENTER_ALIGN_THRESHOLD = 50  # 中心对齐阈值(像素)，在此范围内算中心对齐
PIXELS_PER_CM = 25.6  # 12.8像素代表1厘米

move_horizontal_cm = 0
# ================================================

class SimpleCamera:
    """简化的相机控制类，支持旋转90度采集"""
    def __init__(self, rotate_90_degrees=True):
        self.cam = None
        self.is_streaming = False
        self.rotate_90_degrees = rotate_90_degrees
        self.original_width = 0
        self.original_height = 0
        self.rotated_width = 0
        self.rotated_height = 0
        
    def connect_camera(self):
        """连接第一个可用相机"""
        try:
            # 初始化SDK
            MvCamera.MV_CC_Initialize()
            
            # 枚举设备
            deviceList = MV_CC_DEVICE_INFO_LIST()
            tlayerType = (MV_GIGE_DEVICE | MV_USB_DEVICE)
            
            ret = MvCamera.MV_CC_EnumDevices(tlayerType, deviceList)
            if ret != 0 or deviceList.nDeviceNum == 0:
                return False
            
            # 使用第一个设备
            stDeviceList = cast(deviceList.pDeviceInfo[0], POINTER(MV_CC_DEVICE_INFO)).contents
            
            # 创建相机实例
            self.cam = MvCamera()
            
            # 创建句柄
            ret = self.cam.MV_CC_CreateHandle(stDeviceList)
            if ret != 0:
                return False
                
            # 打开设备
            ret = self.cam.MV_CC_OpenDevice(MV_ACCESS_Exclusive, 0)
            if ret != 0:
                return False
                
            # 设置触发模式为off
            self.cam.MV_CC_SetEnumValue("TriggerMode", MV_TRIGGER_MODE_OFF)
            
            return True
            
        except Exception as e:
            print(f"[ERROR] 连接相机失败: {e}")
            return False
    
    def start_streaming(self):
        """开始采集"""
        if not self.cam or self.is_streaming:
            return False
        
        try:
            ret = self.cam.MV_CC_StartGrabbing()
            if ret == 0:
                self.is_streaming = True
                return True
        except:
            pass
        return False
    
    def get_frame(self):
        """获取一帧图像并自动旋转90度"""
        if not self.is_streaming or not self.cam:
            return None
        
        stOutFrame = MV_FRAME_OUT()  
        memset(byref(stOutFrame), 0, sizeof(stOutFrame))
        
        ret = self.cam.MV_CC_GetImageBuffer(stOutFrame, 1000)
        if ret == 0 and stOutFrame.pBufAddr is not None:
            # 获取图像信息
            nWidth = stOutFrame.stFrameInfo.nWidth
            nHeight = stOutFrame.stFrameInfo.nHeight
            
            # 记录原始尺寸
            self.original_width = nWidth
            self.original_height = nHeight
            
            # 计算旋转后的尺寸
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
                image = image_array.reshape((nHeight, nWidth))
                image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
            elif stOutFrame.stFrameInfo.enPixelType == 0x02180014:  # RGB8_Packed
                image = image_array.reshape((nHeight, nWidth, 3))
                image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
            elif stOutFrame.stFrameInfo.enPixelType == 0x0218000C:  # BGR8_Packed
                image = image_array.reshape((nHeight, nWidth, 3))
            else:
                # 默认按RGB处理
                image = image_array.reshape((nHeight, nWidth, 3))
            
            # 释放缓冲区
            self.cam.MV_CC_FreeImageBuffer(stOutFrame)
            
            # 自动旋转90度
            if self.rotate_90_degrees:
                image = cv2.rotate(image, cv2.ROTATE_90_COUNTERCLOCKWISE)
            
            return image
        return None
    
    def clear_frame_buffer(self):
        """清空帧缓冲区，确保下次获取的是最新帧"""
        if not self.is_streaming or not self.cam:
            return
        
        # 尝试清空几帧旧数据
        for _ in range(5):
            stOutFrame = MV_FRAME_OUT()  
            memset(byref(stOutFrame), 0, sizeof(stOutFrame))
            
            ret = self.cam.MV_CC_GetImageBuffer(stOutFrame, 100)
            if ret == 0 and stOutFrame.pBufAddr is not None:
                self.cam.MV_CC_FreeImageBuffer(stOutFrame)
            else:
                break
    
    def stop_streaming(self):
        """停止采集"""
        if not self.is_streaming or not self.cam:
            return
        
        try:
            self.cam.MV_CC_StopGrabbing()
            self.cam.MV_CC_CloseDevice()
            self.cam.MV_CC_DestroyHandle()
            MvCamera.MV_CC_Finalize()
        except:
            pass
        
        self.is_streaming = False
    
    def get_image_info(self):
        """获取图像信息"""
        return {
            'original_width': self.original_width,
            'original_height': self.original_height,
            'rotated_width': self.rotated_width,
            'rotated_height': self.rotated_height,
            'is_rotated': self.rotate_90_degrees
        }


class QRCodeDetectorOptimized:
    """优化的二维码检测器，基于旋转90度后的坐标系"""

    global move_horizontal_cm

    def __init__(self, center_x=IMAGE_CENTER_X, center_y=IMAGE_CENTER_Y, 
                 threshold=CENTER_ALIGN_THRESHOLD, pixels_per_cm=PIXELS_PER_CM):
        self.center_x = center_x
        self.center_y = center_y
        self.align_threshold = threshold
        self.pixels_per_cm = pixels_per_cm
        self.min_qr_size = 20
        self.confidence_threshold = 0.3
        self.detection_history = []
        self.history_size = 3
        self.enable_preprocessing = True
        self.use_multiple_strategies = True
        
    def preprocess_image(self, image):
        """图像预处理以提高二维码识别率"""
        if image is None:
            return image
        
        # 转换为灰度图
        if len(image.shape) == 3:
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        else:
            gray = image
        
        # 1. 高斯滤波去噪
        gray = cv2.GaussianBlur(gray, (3, 3), 0)
        
        # 2. 直方图均衡化增强对比度
        gray = cv2.equalizeHist(gray)
        
        # 3. 自适应阈值二值化
        gray = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, 
                                    cv2.THRESH_BINARY, 11, 2)
        
        return gray
    
    def detect_qrcodes(self, image):
        """检测二维码并计算中心偏移量 - 基于旋转后的坐标系"""
        if image is None:
            return []
        
        all_detections = []
        
        # 策略1: 使用原始灰度图像
        if len(image.shape) == 3:
            gray_original = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        else:
            gray_original = image
        
        # 策略2: 使用预处理图像
        if self.enable_preprocessing:
            gray_preprocessed = self.preprocess_image(image)
        else:
            gray_preprocessed = None
        
        # 对两种图像版本都进行检测
        image_versions = [gray_original]
        if gray_preprocessed is not None:
            image_versions.append(gray_preprocessed)
        
        for img_version in image_versions:
            detections = self._detect_with_pyzbar(img_version, image)
            all_detections.extend(detections)
            
            # 如果启用了多种策略，还可以尝试OpenCV
            if self.use_multiple_strategies and not detections:
                detections_cv = self._detect_with_opencv(img_version, image)
                all_detections.extend(detections_cv)
        
        # 去除重复检测
        unique_detections = self._remove_duplicates(all_detections)
        
        # 时间滤波
        filtered_detections = self.temporal_filtering(unique_detections)
        
        return filtered_detections
    
    def _detect_with_pyzbar(self, gray_image, original_image):
        """使用pyzbar检测二维码"""
        detections = []

        global move_horizontal_cm
        
        try:
            barcodes = decode(gray_image, symbols=[ZBarSymbol.QRCODE])
            
            for barcode in barcodes:
                (x, y, w, h) = barcode.rect
                
                # 尺寸过滤
                if w < self.min_qr_size or h < self.min_qr_size:
                    continue
                
                # 置信度检查
                if hasattr(barcode, 'quality') and barcode.quality < self.confidence_threshold * 100:
                    continue
                
                # 计算二维码中心
                qr_center_x = x + w // 2
                qr_center_y = y + h // 2
                
                # 计算水平偏移量（像素）: 左侧为负，右侧为正
                horizontal_offset = qr_center_x - self.center_x
                
                # 计算垂直偏移量
                vertical_offset = qr_center_y - self.center_y
                
                # 计算实际偏移距离（厘米）
                horizontal_cm = horizontal_offset / self.pixels_per_cm
                move_horizontal_cm = horizontal_cm
                vertical_cm = vertical_offset / self.pixels_per_cm
                
                # 判断中心是否对齐
                is_aligned = abs(horizontal_offset) <= self.align_threshold
                
                # 判断左右偏移方向
                if horizontal_offset > 0:
                    direction = "右"
                elif horizontal_offset < 0:
                    direction = "左"
                else:
                    direction = "居中"
                
                # 获取二维码的四个角点
                points = []
                for point in barcode.polygon:
                    points.append((point.x, point.y))
                
                detection = {
                    'x': x,
                    'y': y,
                    'width': w,
                    'height': h,
                    'center_x': qr_center_x,
                    'center_y': qr_center_y,
                    'points': points,
                    'horizontal_offset': horizontal_offset,
                    'vertical_offset': vertical_offset,
                    'horizontal_cm': horizontal_cm,
                    'vertical_cm': vertical_cm,
                    'direction': direction,
                    'is_aligned': is_aligned,
                    'confidence': getattr(barcode, 'quality', 100) / 100.0,
                    'data': barcode.data.decode('utf-8') if barcode.data else '',
                    'timestamp': time.time(),
                    'method': 'pyzbar',
                    'image_width': original_image.shape[1],
                    'image_height': original_image.shape[0]
                }
                
                detections.append(detection)
                
        except Exception as e:
            pass
        
        return detections
    
    def _detect_with_opencv(self, image, original_image):
        """使用OpenCV备用检测方法"""
        detections = []
        
        try:
            # 创建QRCode检测器
            qr_detector = cv2.QRCodeDetector()
            
            # 检测和解码
            data, points, _ = qr_detector.detectAndDecode(image)
            
            if points is not None and data:
                points = points[0].astype(int)
                
                # 计算边界框
                x = int(points[:, 0].min())
                y = int(points[:, 1].min())
                w = int(points[:, 0].max() - x)
                h = int(points[:, 1].max() - y)
                
                # 计算中心点
                qr_center_x = x + w // 2
                qr_center_y = y + h // 2
                
                # 计算水平偏移量: 左侧为负，右侧为正
                horizontal_offset = qr_center_x - self.center_x
                
                # 计算垂直偏移量
                vertical_offset = qr_center_y - self.center_y
                
                # 计算实际偏移距离（厘米）
                horizontal_cm = horizontal_offset / self.pixels_per_cm
                vertical_cm = vertical_offset / self.pixels_per_cm
                
                # 判断中心是否对齐
                is_aligned = abs(horizontal_offset) <= self.align_threshold
                
                # 判断方向
                if horizontal_offset > 0:
                    direction = "右"
                elif horizontal_offset < 0:
                    direction = "左"
                else:
                    direction = "居中"
                
                detection = {
                    'x': x,
                    'y': y,
                    'width': w,
                    'height': h,
                    'center_x': qr_center_x,
                    'center_y': qr_center_y,
                    'points': [(int(p[0]), int(p[1])) for p in points],
                    'horizontal_offset': horizontal_offset,
                    'vertical_offset': vertical_offset,
                    'horizontal_cm': horizontal_cm,
                    'vertical_cm': vertical_cm,
                    'direction': direction,
                    'is_aligned': is_aligned,
                    'confidence': 0.3,
                    'data': data,
                    'timestamp': time.time(),
                    'method': 'opencv',
                    'image_width': original_image.shape[1],
                    'image_height': original_image.shape[0]
                }
                
                detections.append(detection)
                
        except Exception as e:
            pass
        
        return detections
    
    def _remove_duplicates(self, detections):
        """去除重复检测"""
        if not detections:
            return []
        
        unique_detections = []
        used_positions = []
        
        for detection in detections:
            center = (detection['center_x'], detection['center_y'])
            
            # 检查是否与已有检测重叠
            is_duplicate = False
            for used_center in used_positions:
                distance = np.sqrt((center[0] - used_center[0]) ** 2 + 
                                 (center[1] - used_center[1]) ** 2)
                if distance < 30:
                    is_duplicate = True
                    break
            
            if not is_duplicate:
                unique_detections.append(detection)
                used_positions.append(center)
        
        return unique_detections
    
    def temporal_filtering(self, detections):
        """简单的时间滤波"""
        current_time = time.time()
        
        # 移除过期历史记录
        self.detection_history = [d for d in self.detection_history 
                                 if current_time - d['timestamp'] < 0.5]
        
        if not detections:
            return []
        
        # 如果有历史记录，检查新检测是否与历史一致
        if self.detection_history:
            filtered = []
            for det in detections:
                for hist in self.detection_history:
                    # 检查中心点距离是否小于阈值
                    center_dist = np.sqrt((det['center_x'] - hist['center_x']) ** 2 + 
                                        (det['center_y'] - hist['center_y']) ** 2)
                    if center_dist < 50:
                        filtered.append(det)
                        break
            detections = filtered
        
        # 更新历史记录
        self.detection_history.extend(detections)
        
        # 限制历史记录大小
        if len(self.detection_history) > self.history_size * 5:
            self.detection_history = self.detection_history[-self.history_size * 5:]
        
        return detections
    
    def set_center(self, center_x, center_y):
        """设置图像中心点"""
        self.center_x = center_x
        self.center_y = center_y
    
    def set_confidence_threshold(self, threshold):
        """设置置信度阈值"""
        self.confidence_threshold = max(0.0, min(1.0, threshold))
    
    def set_alignment_threshold(self, threshold):
        """设置对齐阈值"""
        self.align_threshold = max(1, threshold)
    
    def set_pixels_per_cm(self, pixels_per_cm):
        """设置每厘米像素数"""
        self.pixels_per_cm = max(0.1, pixels_per_cm)
    
    def set_preprocessing(self, enable):
        """启用/禁用图像预处理"""
        self.enable_preprocessing = enable
    
    def set_min_qr_size(self, size):
        """设置最小二维码尺寸"""
        self.min_qr_size = max(1, size)


class QRTcpServer:
    """二维码检测TCP服务器 - 支持单次启动指令"""

    global move_horizontal_cm

    def __init__(self, host='0.0.0.0', port=8888, 
                 center_x=IMAGE_CENTER_X, center_y=IMAGE_CENTER_Y,
                 threshold=CENTER_ALIGN_THRESHOLD, pixels_per_cm=PIXELS_PER_CM):
        self.host = host
        self.port = port
        self.center_x = center_x
        self.center_y = center_y
        self.align_threshold = threshold
        self.pixels_per_cm = pixels_per_cm
        self.server_socket = None
        self.client_socket = None
        self.client_address = None
        self.is_running = False
        self.scanning = False
        self.camera = None
        self.detector = None
        self.center_align_count = 0
        self.required_center_count = 3
        self.task_completed = False
        self.task_thread = None
        
    def start(self):
        """启动TCP服务器"""
        try:
            # 创建TCP套接字
            self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.server_socket.bind((self.host, self.port))
            self.server_socket.listen(1)
            self.server_socket.settimeout(1.0)  # 设置超时以便能够检查is_running
            
            self.is_running = True
            print(f"[INFO] TCP服务器启动在 {self.host}:{self.port}")
            print(f"[INFO] 等待客户端连接和指令...")
            
            # 初始化相机
            if not self.initialize_camera():
                print("[ERROR] 相机初始化失败，服务器停止")
                return
            
            # 启动主循环
            self.main_loop()
            
        except Exception as e:
            print(f"[ERROR] 启动TCP服务器失败: {e}")
        finally:
            self.stop()
    
    def initialize_camera(self):
        """初始化相机和检测器"""
        try:
            self.camera = SimpleCamera(rotate_90_degrees=True)
            if not self.camera.connect_camera():
                print("[ERROR] 无法连接相机")
                return False
            
            if not self.camera.start_streaming():
                print("[ERROR] 无法启动相机采集")
                return False
            
            # 初始化检测器
            self.detector = QRCodeDetectorOptimized(
                center_x=self.center_x,
                center_y=self.center_y,
                threshold=self.align_threshold,
                pixels_per_cm=self.pixels_per_cm
            )
            
            print("[INFO] 相机初始化成功")
            return True
            
        except Exception as e:
            print(f"[ERROR] 初始化相机失败: {e}")
            return False
    
    def main_loop(self):
        """主循环"""
        while self.is_running:
            try:
                # 等待客户端连接
                if self.client_socket is None:
                    self.accept_client()
                else:
                    # 处理客户端消息
                    self.handle_client()
                    
            except socket.timeout:
                continue
            except Exception as e:
                print(f"[ERROR] 主循环异常: {e}")
                time.sleep(1)
    
    def accept_client(self):
        """接受客户端连接"""
        try:
            client_socket, client_address = self.server_socket.accept()
            self.client_socket = client_socket
            self.client_address = client_address
            self.scanning = False
            self.task_completed = False
            self.center_align_count = 0
            
            print(f"[INFO] 客户端已连接: {client_address}")
            
            # 重置客户端socket的超时时间
            self.client_socket.settimeout(None)  # 阻塞模式
            self.send_to_client("READY: 二维码检测服务器已就绪，发送START开始执行任务")
            
        except socket.timeout:
            pass
        except Exception as e:
            print(f"[ERROR] 接受客户端连接失败: {e}")
    
    def handle_client(self):
        """处理客户端消息"""
        try:
            # 接收客户端指令
            data = self.client_socket.recv(1024)
            if not data:
                print(f"[INFO] 客户端断开连接: {self.client_address}")
                self.disconnect_client()
                return
                
            command = data.decode('utf-8').strip().upper()
            print(f"[INFO] 收到客户端指令: {command}")
            
            if command == "START":
                if not self.task_completed and not self.scanning:
                    # 启动任务执行
                    self.scanning = True
                    self.center_align_count = 0
                    print(f"[INFO] 开始扫描任务")
                    self.send_to_client("START: 开始扫描任务")
                    
                    # 执行扫描任务直到完成
                    self.execute_scanning_task()
                    
                elif self.task_completed:
                    self.send_to_client("ERROR: 任务已完成，请重新连接以开始新任务")
                else:
                    self.send_to_client("ERROR: 任务已在执行中")
            else:
                self.send_to_client(f"ERROR: 未知命令: {command}")
                    
        except ConnectionResetError:
            print(f"[INFO] 客户端断开连接: {self.client_address}")
            self.disconnect_client()
        except Exception as e:
            print(f"[ERROR] 处理客户端消息失败: {e}")
            self.disconnect_client()
    
    # 修改 execute_scanning_task 方法
    def execute_scanning_task(self):
        """执行扫描任务直到完成"""
        try:
            while self.scanning and not self.task_completed:
                # 清空帧缓冲区
                self.camera.clear_frame_buffer()
                
                # 获取最新帧
                image = self.camera.get_frame()
                
                if image is None:
                    print("[WARN] 获取图像失败")
                    time.sleep(0.1)
                    continue
                
                # 检测二维码
                detection_start = time.time()
                detections = self.detector.detect_qrcodes(image)
                detection_time = (time.time() - detection_start) * 1000
                
                timestamp = time.strftime("%Y-%m-%d %H:%M:%S")
                
                if detections:
                    # 获取第一个检测结果
                    detection = detections[0]
                    offset = detection['horizontal_offset']
                    offset_cm = detection['horizontal_cm']
                    is_aligned = detection['is_aligned']
                    direction = detection['direction']
                    
                    # 输出检测信息
                    print(f"[{timestamp}] 检测到二维码 | 偏移: {offset:+d}px ({offset_cm:+.2f}cm) {direction} | 对齐: {'是' if is_aligned else '否'} | 处理时间: {detection_time:.1f}ms")
                    
                    # 发送到客户端
                    self.send_to_client(f"DETECTED: 偏移={offset:+d}px ({offset_cm:+.2f}cm) {direction}, 对齐={'是' if is_aligned else '否'}")
                    
                    # 处理对齐状态
                    if is_aligned:
                        self.center_align_count += 1
                        print(f"[INFO] 第 {self.center_align_count} 次中心对齐")
                        
                        if self.center_align_count >= self.required_center_count:
                            print(f"[INFO] 已达到连续 {self.required_center_count} 次中心对齐，扫描任务完成")
                            self.send_to_client(f"COMPLETE")
                            # 修改：发送完成消息后等待5秒再断开连接
                            time.sleep(5)
                            self.scanning = False
                            self.task_completed = True
                            break
                    else:
                        # 重置中心对齐计数
                        self.center_align_count = 0
    
                        dis = abs((offset_cm/100))
                        print("/"*10)
                        print(dis)
                        print("/"*10)
                        
                        # 判断偏离方向
                        if offset > 0:
                            adjust_msg = "向右调整"
                            result = rc.move_backward(distance=dis, speed=0.3)
                            print(f"结果: {result}")
                        else:
                            adjust_msg = "向左调整"
                            result = rc.move_forward(distance=dis, speed=0.3)
                            print(f"结果: {result}")
                        
                        print(f"[WARN] 偏离中心区域，正在命令AGV调整位置 ({adjust_msg})")
                        self.send_to_client(f"ADJUST: 偏离中心区域，需要{adjust_msg}")
                        
                        # 等待AGV移动完成
                        time.sleep(5)
                else:
                    print(f"[{timestamp}] 未检测到二维码 | 处理时间: {detection_time:.1f}ms")
                    # 重置中心对齐计数
                    self.center_align_count = 0
                    self.send_to_client("NO_QR: 未检测到二维码")
                
                # 等待0.1秒
                time.sleep(0.1)
                
        except Exception as e:
            print(f"[ERROR] 扫描任务异常: {e}")
            self.send_to_client(f"ERROR: 扫描任务异常: {str(e)}")
            self.scanning = False
        
        if not self.task_completed and not self.scanning:
            self.send_to_client("STOPPED: 扫描任务被停止")
    
    def send_to_client(self, message):
        """发送消息到客户端"""
        try:
            if self.client_socket:
                # 添加换行符以确保客户端能够正确读取
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
            self.task_completed = False
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
        
        print("[INFO] 服务器已停止")


# 客户端代码示例
class SimpleClient:
    """简单的客户端实现"""
    def __init__(self, host='127.0.0.1', port=8888):
        self.host = host
        self.port = port
        self.socket = None
        
    def connect_and_run(self):
        """连接到服务器并执行任务"""
        try:
            # 连接到服务器
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.connect((self.host, self.port))
            self.socket.settimeout(None)  # 阻塞模式，无限期等待
            print(f"[CLIENT] 已连接到服务器 {self.host}:{self.port}")
            
            # 接收欢迎消息
            response = self.socket.recv(1024).decode('utf-8').strip()
            print(f"[SERVER] {response}")
            
            # 发送启动命令
            print("[CLIENT] 发送启动指令: START")
            self.socket.sendall(b"START")
            
            # 持续接收服务器响应
            while True:
                try:
                    response = self.socket.recv(1024).decode('utf-8').strip()
                    if not response:
                        print("[CLIENT] 服务器断开连接")
                        break
                    
                    print(f"[SERVER] {response}")
                    
                    # 检查是否收到完成消息
                    if response.startswith("COMPLETE:"):
                        print("[CLIENT] 任务已完成，准备断开连接")
                        break
                        
                except socket.timeout:
                    continue
                except ConnectionResetError:
                    print("[CLIENT] 连接被服务器重置")
                    break
            
        except KeyboardInterrupt:
            print("\n[CLIENT] 用户中断")
        except Exception as e:
            print(f"[CLIENT] 错误: {e}")
        finally:
            if self.socket:
                self.socket.close()
                print("[CLIENT] 已断开连接")


def main():
    """主函数"""
    import argparse
    
    parser = argparse.ArgumentParser(description='二维码检测TCP服务器')
    parser.add_argument('--host', type=str, default='0.0.0.0', help='服务器主机地址 (默认: 0.0.0.0)')
    parser.add_argument('--port', type=int, default=8888, help='服务器端口 (默认: 8888)')
    parser.add_argument('--center-x', type=int, default=IMAGE_CENTER_X, 
                       help=f'图像水平中心点 (默认: {IMAGE_CENTER_X})')
    parser.add_argument('--center-y', type=int, default=IMAGE_CENTER_Y,
                       help=f'图像垂直中心点 (默认: {IMAGE_CENTER_Y})')
    parser.add_argument('--threshold', type=int, default=CENTER_ALIGN_THRESHOLD,
                       help=f'中心对齐阈值(像素) (默认: {CENTER_ALIGN_THRESHOLD})')
    parser.add_argument('--pixels-per-cm', type=float, default=PIXELS_PER_CM,
                       help=f'每厘米像素数 (默认: {PIXELS_PER_CM})')
    parser.add_argument('--client', action='store_true', help='以客户端模式运行')
    parser.add_argument('--client-host', type=str, default='127.0.0.1', help='客户端连接的主机地址')
    
    args = parser.parse_args()
    
    if args.client:
        # 客户端模式
        print("=" * 60)
        print("二维码检测客户端")
        print("=" * 60)
        print(f"服务器地址: {args.client_host}:{args.port}")
        print("客户端将发送START指令并等待服务器响应")
        print("收到COMPLETE消息后自动断开连接")
        print("=" * 60)
        
        client = SimpleClient(host=args.client_host, port=args.port)
        client.connect_and_run()
    else:
        # 服务器模式
        print("=" * 60)
        print("二维码检测TCP服务器")
        print("=" * 60)
        print(f"服务器地址: {args.host}:{args.port}")
        print(f"图像中心点: ({args.center_x}, {args.center_y})")
        print(f"对齐阈值: {args.threshold} 像素")
        print(f"像素-厘米转换: {args.pixels_per_cm} 像素/厘米")
        print(f"连续对齐次数: 5 次")
        print(f"扫描间隔: 0.1 秒")
        print("=" * 60)
        print("运行方式:")
        print("1. 先启动服务器: python server.py")
        print("2. 再启动客户端: python server.py --client")
        print("或使用单独的客户端代码连接")
        print("=" * 60)
        
        # 创建并启动服务器
        server = QRTcpServer(
            host=args.host, 
            port=args.port,
            center_x=args.center_x,
            center_y=args.center_y,
            threshold=args.threshold,
            pixels_per_cm=args.pixels_per_cm
        )
        
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