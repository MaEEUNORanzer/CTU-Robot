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
import subprocess
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


# 在文件开头修改全局变量声明
PREVIEW_ENABLED = True  # 是否启用画面预览
PREVIEW_WINDOW_NAME = "Camera Preview"  # 预览窗口名称
PREVIEW_SCALE = 0.5  # 预览缩放比例

# ================== 用户配置区域 ==================
# 注意：相机是横着安装的，需要旋转90度
# 旋转后的目标分辨率：1024×720（横屏）
# 这意味着原始采集应该是：720×1024（竖屏）
ROTATED_WIDTH = 1024   # 旋转后的宽度
ROTATED_HEIGHT = 720   # 旋转后的高度
RAW_WIDTH = ROTATED_HEIGHT  # 原始采集宽度 = 旋转后的高度
RAW_HEIGHT = ROTATED_WIDTH  # 原始采集高度 = 旋转后的宽度

# 中心点基于旋转后的坐标系
IMAGE_CENTER_X = ROTATED_WIDTH // 2 + 100
IMAGE_CENTER_Y = ROTATED_HEIGHT // 2  # 360
CENTER_ALIGN_THRESHOLD = 70           # 中心对齐阈值(像素)
PIXELS_PER_CM = 20.0                  # 20像素代表1厘米

move_horizontal_cm = 0
# ================================================

class SimpleCamera:
    """相机控制类，支持旋转90度采集，直接设置相机原始分辨率"""
    
    def __init__(self, rotate_90_degrees=True, target_raw_width=RAW_WIDTH, target_raw_height=RAW_HEIGHT):
        self.cam = None
        self.is_streaming = False
        self.rotate_90_degrees = rotate_90_degrees
        # 目标原始分辨率（旋转前）
        self.target_raw_width = target_raw_width
        self.target_raw_height = target_raw_height
        # 实际采集分辨率
        self.actual_raw_width = 0
        self.actual_raw_height = 0
        # 旋转后的分辨率
        self.rotated_width = 0
        self.rotated_height = 0
        # 相机能力参数
        self.width_range = None
        self.height_range = None
        self.pixel_format = None
        
    def connect_camera(self):
        """连接相机并设置分辨率"""
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
            
            # 设置分辨率
            if not self.set_camera_resolution():
                print("[WARN] 无法设置目标分辨率，将使用相机默认分辨率")
            
            # 设置触发模式为off
            self.cam.MV_CC_SetEnumValue("TriggerMode", MV_TRIGGER_MODE_OFF)
            
            return True
            
        except Exception as e:
            print(f"[ERROR] 连接相机失败: {e}")
            return False
    
    def get_camera_capabilities(self):
        """获取相机的参数范围"""
        try:
            # 获取宽度范围
            stWidth = MVCC_INTVALUE()
            ret = self.cam.MV_CC_GetIntValue("Width", stWidth)
            if ret == 0:
                self.width_range = (stWidth.nMin, stWidth.nMax, stWidth.nInc)
                print(f"[INFO] 相机宽度范围: {stWidth.nMin} - {stWidth.nMax}, 步进: {stWidth.nInc}")
            else:
                self.width_range = None
                print("[WARN] 无法获取宽度范围")
                
            # 获取高度范围
            stHeight = MVCC_INTVALUE()
            ret = self.cam.MV_CC_GetIntValue("Height", stHeight)
            if ret == 0:
                self.height_range = (stHeight.nMin, stHeight.nMax, stHeight.nInc)
                print(f"[INFO] 相机高度范围: {stHeight.nMin} - {stHeight.nMax}, 步进: {stHeight.nInc}")
            else:
                self.height_range = None
                print("[WARN] 无法获取高度范围")
                
        except Exception as e:
            print(f"[WARN] 获取相机能力失败: {e}")
            self.width_range = None
            self.height_range = None
    
    def set_camera_resolution(self):
        """设置相机的分辨率"""
        try:
            # 先获取相机能力
            self.get_camera_capabilities()
            
            # 设置宽度
            if self.target_raw_width and self.width_range:
                min_w, max_w, inc_w = self.width_range
                # 调整到最接近的可用值
                adjusted_width = self.adjust_to_inc(self.target_raw_width, min_w, max_w, inc_w)
                
                ret = self.cam.MV_CC_SetIntValue("Width", adjusted_width)
                if ret == 0:
                    self.actual_raw_width = adjusted_width
                    if adjusted_width != self.target_raw_width:
                        print(f"[INFO] 宽度调整: {self.target_raw_width} -> {adjusted_width}")
                else:
                    print(f"[WARN] 设置宽度失败: {ret}")
            
            # 设置高度
            if self.target_raw_height and self.height_range:
                min_h, max_h, inc_h = self.height_range
                # 调整到最接近的可用值
                adjusted_height = self.adjust_to_inc(self.target_raw_height, min_h, max_h, inc_h)
                
                ret = self.cam.MV_CC_SetIntValue("Height", adjusted_height)
                if ret == 0:
                    self.actual_raw_height = adjusted_height
                    if adjusted_height != self.target_raw_height:
                        print(f"[INFO] 高度调整: {self.target_raw_height} -> {adjusted_height}")
                else:
                    print(f"[WARN] 设置高度失败: {ret}")
            
            # 计算旋转后的尺寸
            if self.rotate_90_degrees:
                self.rotated_width = self.actual_raw_height
                self.rotated_height = self.actual_raw_width
            else:
                self.rotated_width = self.actual_raw_width
                self.rotated_height = self.actual_raw_height
            
            print(f"[INFO] 相机原始分辨率: {self.actual_raw_width} x {self.actual_raw_height}")
            print(f"[INFO] 旋转后分辨率: {self.rotated_width} x {self.rotated_height}")
            
            return True
            
        except Exception as e:
            print(f"[ERROR] 设置分辨率失败: {e}")
            return False
    
    def adjust_to_inc(self, value, min_val, max_val, inc):
        """调整值为最接近的步进倍数"""
        if inc <= 0 or value < min_val or value > max_val:
            return min_val
        
        # 计算最接近的步进倍数
        value = max(min_val, min(value, max_val))
        remainder = (value - min_val) % inc
        
        if remainder > inc / 2:
            adjusted = value + (inc - remainder)
        else:
            adjusted = value - remainder
        
        # 确保在范围内
        return max(min_val, min(adjusted, max_val))
    
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
    
    def get_frame(self, show_info=True):
        """获取一帧图像并自动旋转90度"""
        if not self.is_streaming or not self.cam:
            return None
        
        stOutFrame = MV_FRAME_OUT()  
        memset(byref(stOutFrame), 0, sizeof(stOutFrame))
        
        ret = self.cam.MV_CC_GetImageBuffer(stOutFrame, 1000)
        if ret == 0 and stOutFrame.pBufAddr is not None:
            # 获取当前图像信息
            nWidth = stOutFrame.stFrameInfo.nWidth
            nHeight = stOutFrame.stFrameInfo.nHeight
            
            # 记录实际采集尺寸
            self.actual_raw_width = nWidth
            self.actual_raw_height = nHeight
            
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
            
            # 添加图像信息标注
            if show_info:
                h, w = image.shape[:2]
                # 显示分辨率
                cv2.putText(image, f"{w}x{h}", (10, 30), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                # 显示时间戳
                timestamp = time.strftime("%H:%M:%S")
                cv2.putText(image, timestamp, (10, 60), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                # 显示旋转状态
                cv2.putText(image, f"Rotated: {self.rotate_90_degrees}", (10, 90), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            
            return image
        return None
    
    def clear_frame_buffer(self):
        """清空帧缓冲区"""
        if not self.is_streaming or not self.cam:
            return
        
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
            'raw_width': self.actual_raw_width,
            'raw_height': self.actual_raw_height,
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
        self.confidence_threshold = 0.2
        self.detection_history = []
        self.history_size = 3
        self.enable_preprocessing = True
        self.use_multiple_strategies = True
        
    def preprocess_image(self, image):
        if image is None:
            return image

        # 1. 缩小（提速 + 抗噪）
        h, w = image.shape[:2]
        image = cv2.resize(image, (int(w * 0.6), int(h * 0.6)))

        # 2. 灰度
        if len(image.shape) == 3:
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        else:
            gray = image.copy()

        # 3. 亮度 & 对比度（暗光核心）
        gray = cv2.convertScaleAbs(gray, alpha=2.0, beta=100)

        # gray = cv2.convertScaleAbs(gray, alpha=1.0, beta=-100)

        # 4. CLAHE（暗部拉爆）
        clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8))
        gray = clahe.apply(gray)

        # 5. 锐化（让轮廓立起来）
        kernel = np.array([
            [0, -1, 0],
            [-1, 5.5, -1],
            [0, -1, 0]
        ])
        gray = cv2.filter2D(gray, -1, kernel)

        # 6. Otsu 二值化（最快最稳）
        _, gray = cv2.threshold(
            gray, 0, 255,
            cv2.THRESH_BINARY + cv2.THRESH_OTSU
        )

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
                
                # 计算实际偏移距离（厘米）- 调整比例扩大校准范围
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
    """二维码检测TCP服务器 - 支持单次启动指令，校准完成后调用回调脚本"""

    global move_horizontal_cm

    def __init__(self, host='0.0.0.0', port=8888, 
                 center_x=IMAGE_CENTER_X, center_y=IMAGE_CENTER_Y,
                 threshold=CENTER_ALIGN_THRESHOLD, pixels_per_cm=PIXELS_PER_CM,
                 callback_script='callback_task.py',
                 target_raw_width=RAW_WIDTH, target_raw_height=RAW_HEIGHT):
        self.host = host
        self.port = port
        self.center_x = center_x
        self.center_y = center_y
        self.align_threshold = threshold
        self.pixels_per_cm = pixels_per_cm
        self.callback_script = callback_script
        self.target_raw_width = target_raw_width
        self.target_raw_height = target_raw_height
        self.server_socket = None
        self.client_socket = None
        self.client_address = None
        self.is_running = False
        self.scanning = False
        self.camera = None
        self.detector = None
        self.center_align_count = 0
        self.required_center_count = 5
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
            print(f"[INFO] 回调脚本路径: {self.callback_script}")
            print(f"[INFO] 画面预览: {'启用' if PREVIEW_ENABLED else '禁用'}")
            if PREVIEW_ENABLED:
                print(f"[INFO] 按ESC键可退出预览窗口")
            
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
            # 计算原始采集分辨率（旋转前）
            print(f"[INFO] 目标旋转后分辨率: {ROTATED_WIDTH}x{ROTATED_HEIGHT}")
            print(f"[INFO] 原始采集分辨率: {self.target_raw_width}x{self.target_raw_height}")
            
            self.camera = SimpleCamera(
                rotate_90_degrees=True,
                target_raw_width=self.target_raw_width,
                target_raw_height=self.target_raw_height
            )
            
            if not self.camera.connect_camera():
                print("[ERROR] 无法连接相机")
                return False
            
            if not self.camera.start_streaming():
                print("[ERROR] 无法启动相机采集")
                return False
            
            # 获取实际分辨率并更新中心点
            camera_info = self.camera.get_image_info()
            actual_rotated_width = camera_info['rotated_width']
            actual_rotated_height = camera_info['rotated_height']
            
            print(f"[INFO] 相机实际旋转后分辨率: {actual_rotated_width}x{actual_rotated_height}")
            
            # 如果实际分辨率与目标不同，调整中心点
            if actual_rotated_width != ROTATED_WIDTH or actual_rotated_height != ROTATED_HEIGHT:
                print(f"[WARN] 旋转后分辨率 {actual_rotated_width}x{actual_rotated_height} 与目标 {ROTATED_WIDTH}x{ROTATED_HEIGHT} 不同")
                self.center_x = actual_rotated_width // 2
                self.center_y = actual_rotated_height // 2
                print(f"[INFO] 调整中心点: ({self.center_x}, {self.center_y})")
            
            # 初始化检测器
            self.detector = QRCodeDetectorOptimized(
                center_x=self.center_x,
                center_y=self.center_y,
                threshold=self.align_threshold,
                pixels_per_cm=self.pixels_per_cm
            )
            
            print("[INFO] 相机初始化成功")
            print(f"[INFO] 实际旋转后尺寸: {actual_rotated_width} x {actual_rotated_height}")
            print(f"[INFO] 图像中心点: ({self.center_x}, {self.center_y})")
            print(f"[INFO] 对齐阈值: {self.align_threshold} 像素")
            print(f"[INFO] 像素厘米比例: {self.pixels_per_cm} 像素/厘米")
            return True
            
        except Exception as e:
            print(f"[ERROR] 初始化相机失败: {e}")
            return False

    def run_callback_script(self):
        """执行回调脚本"""
        try:
            print(f"[INFO] 正在执行回调脚本: {self.callback_script}")
            
            if os.path.exists(self.callback_script):
                result = subprocess.run(
                    ["python3", self.callback_script],
                    capture_output=True,
                    text=True,
                    timeout=1024
                )
                
                print(f"[INFO] 回调脚本返回码: {result.returncode}")
                print(f"[INFO] 回调脚本输出: {result.stdout}")
                if result.stderr:
                    print(f"[WARN] 回调脚本错误: {result.stderr}")
                    
                if result.returncode == 0:
                    print(f"[INFO] 回调脚本执行成功")
                    return True
                else:
                    print(f"[ERROR] 回调脚本执行失败，返回码: {result.returncode}")
                    return False
            else:
                print(f"[ERROR] 回调脚本不存在: {self.callback_script}")
                return False
                
        except subprocess.TimeoutExpired:
            print(f"[ERROR] 回调脚本执行超时")
            return False
        except Exception as e:
            print(f"[ERROR] 运行回调脚本失败: {e}")
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
                    self.send_to_client("INFO: 客户端可断开连接，服务器将持续调整直到完成")
                    
                    # 获取当前相机信息
                    camera_info = self.camera.get_image_info()
                    rotated_width = camera_info['rotated_width']
                    rotated_height = camera_info['rotated_height']
                    
                    self.send_to_client(f"INFO: 图像尺寸: {rotated_width}x{rotated_height}")
                    self.send_to_client(f"INFO: 中心点: ({self.center_x}, {self.center_y})")
                    self.send_to_client(f"INFO: 对齐阈值: {self.align_threshold} 像素")
                    
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
    
    def execute_scanning_task(self):
        """执行扫描任务直到完成"""
        # 创建预览窗口
        if PREVIEW_ENABLED:
            camera_info = self.camera.get_image_info()
            rotated_width = camera_info['rotated_width']
            rotated_height = camera_info['rotated_height']
            
            cv2.namedWindow(PREVIEW_WINDOW_NAME, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(PREVIEW_WINDOW_NAME, 
                            int(rotated_width * PREVIEW_SCALE), 
                            int(rotated_height * PREVIEW_SCALE))
            
            print(f"[INFO] 预览窗口尺寸: {int(rotated_width * PREVIEW_SCALE)}x{int(rotated_height * PREVIEW_SCALE)}")
        
        try:
            while self.scanning and not self.task_completed:
                # 清空帧缓冲区
                self.camera.clear_frame_buffer()
                
                # 获取最新帧
                image = self.camera.get_frame(show_info=True)
                
                if image is None:
                    print("[WARN] 获取图像失败")
                    time.sleep(0.1)
                    continue
                
                # 显示预览
                if PREVIEW_ENABLED:
                    preview_image = image.copy()
                    
                    # 绘制中心十字线
                    h, w = preview_image.shape[:2]
                    center_x, center_y = self.center_x, self.center_y
                    
                    # 绘制中心十字线
                    cv2.line(preview_image, (center_x, 0), (center_x, h), (0, 255, 0), 1)
                    cv2.line(preview_image, (0, center_y), (w, center_y), (0, 255, 0), 1)
                    
                    # 绘制对齐阈值区域
                    cv2.rectangle(preview_image, 
                                (center_x - self.align_threshold, center_y - self.align_threshold),
                                (center_x + self.align_threshold, center_y + self.align_threshold),
                                (0, 255, 255), 1)
                    
                    # 绘制中心点
                    cv2.circle(preview_image, (center_x, center_y), 5, (0, 0, 255), -1)
                    
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
                        qr_center_x = detection['center_x']
                        qr_center_y = detection['center_y']
                        
                        # 在预览图像上绘制二维码位置
                        x, y, w_rect, h_rect = detection['x'], detection['y'], detection['width'], detection['height']
                        
                        # 绘制二维码边界框
                        color = (0, 255, 0) if is_aligned else (0, 0, 255)
                        cv2.rectangle(preview_image, (x, y), (x + w_rect, y + h_rect), color, 2)
                        
                        # 绘制二维码中心点
                        cv2.circle(preview_image, (qr_center_x, qr_center_y), 3, (255, 0, 0), -1)
                        
                        # 绘制从中心到二维码中心的连线
                        cv2.line(preview_image, (center_x, center_y), (qr_center_x, qr_center_y), (255, 0, 0), 2)
                        
                        # 在图像上显示偏移信息
                        cv2.putText(preview_image, f"Offset: {offset:+d}px ({offset_cm:+.1f}cm)", 
                                  (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
                        cv2.putText(preview_image, f"Direction: {direction}", 
                                  (10, 150), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
                        cv2.putText(preview_image, f"Aligned: {'Yes' if is_aligned else 'No'}", 
                                  (10, 180), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0) if is_aligned else (0, 0, 255), 2)
                        cv2.putText(preview_image, f"Align Count: {self.center_align_count}/{self.required_center_count}", 
                                  (10, 210), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                        
                        # 输出检测信息
                        print(f"[{timestamp}] 检测到二维码 | 偏移: {offset:+d}px ({offset_cm:+.2f}cm) {direction} | 对齐: {'是' if is_aligned else '否'} | 处理时间: {detection_time:.1f}ms")
                        
                        # 处理对齐状态
                        if is_aligned:
                            self.center_align_count += 1
                            print(f"[INFO] 第 {self.center_align_count} 次中心对齐")
                            
                            if self.center_align_count >= self.required_center_count:
                                print(f"[INFO] 已达到连续 {self.required_center_count} 次中心对齐，校准完成")
                                
                                # 调用回调脚本
                                print(f"[INFO] 开始执行回调脚本: {self.callback_script}")
                                callback_result = self.run_callback_script()
                                
                                if callback_result:
                                    print(f"[INFO] 回调脚本执行成功")
                                    self.send_to_client(f"COMPLETE: 校准完成，回调脚本执行成功")
                                else:
                                    print(f"[WARN] 回调脚本执行失败，但校准已完成")
                                    self.send_to_client(f"COMPLETE: 校准完成，但回调脚本执行失败")
                                
                                # 等待5秒后断开连接
                                time.sleep(5)
                                self.scanning = False
                                self.task_completed = True
                                break
                        else:
                            # 重置中心对齐计数
                            self.center_align_count = 0
        
                            dis = abs((offset_cm/100/2))
                            print("/"*10)
                            print(f"偏移距离: {dis:.4f} 米")
                            print(f"原始偏移: {offset_cm:.2f} cm")
                            print("/"*10)
                            
                            # 判断偏离方向
                            if offset > 0:
                                adjust_msg = "向右调整"
                                # result = rc.move_backward(distance=(dis-0.03), speed=0.3)
                                result = rc.move_backward(distance=dis, speed=0.3)
                                print(f"AGV调整结果: {result}")
                            else:
                                adjust_msg = "向左调整"
                                # result = rc.move_forward(distance=(dis+0.06), speed=0.3)
                                result = rc.move_forward(distance=dis, speed=0.3)
                                print(f"AGV调整结果: {result}")
                            
                            print(f"[WARN] 偏离中心区域，正在命令AGV调整位置 ({adjust_msg})")
                            
                            # 等待AGV移动完成
                            time.sleep(6)
                    else:
                        # 没有检测到二维码时显示提示
                        cv2.putText(preview_image, "No QR Code Detected", 
                                  (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                        print(f"[{timestamp}] 未检测到二维码 | 处理时间: {detection_time:.1f}ms")
                        # 重置中心对齐计数
                        self.center_align_count = 0
                    
                    # 显示FPS信息
                    current_time = time.time()
                    fps = 1 / (current_time - detection_start + 0.001)
                    cv2.putText(preview_image, f"FPS: {int(fps)}", 
                              (w - 150, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
                    
                    # 显示预览图像
                    cv2.imshow(PREVIEW_WINDOW_NAME, preview_image)
                    
                    # 处理键盘事件（按ESC退出预览）
                    key = cv2.waitKey(1) & 0xFF
                    if key == 27:  # ESC键
                        print("[INFO] 用户按ESC键，停止预览")
                        cv2.destroyAllWindows()
                        self.scanning = False
                        break
                else:
                    # 如果不启用预览，只进行检测
                    detection_start = time.time()
                    detections = self.detector.detect_qrcodes(image)
                    detection_time = (time.time() - detection_start) * 1000
                    
                    timestamp = time.strftime("%Y-%m-%d %H:%M:%S")
                    
                    if detections:
                        detection = detections[0]
                        offset = detection['horizontal_offset']
                        offset_cm = detection['horizontal_cm']
                        is_aligned = detection['is_aligned']
                        direction = detection['direction']
                        
                        print(f"[{timestamp}] 检测到二维码 | 偏移: {offset:+d}px ({offset_cm:+.2f}cm) {direction} | 对齐: {'是' if is_aligned else '否'} | 处理时间: {detection_time:.1f}ms")
                        
                        if is_aligned:
                            self.center_align_count += 1
                            print(f"[INFO] 第 {self.center_align_count} 次中心对齐")
                            
                            if self.center_align_count >= self.required_center_count:
                                print(f"[INFO] 已达到连续 {self.required_center_count} 次中心对齐，校准完成")
                                
                                callback_result = self.run_callback_script()
                                
                                if callback_result:
                                    self.send_to_client(f"COMPLETE: 校准完成，回调脚本执行成功")
                                else:
                                    self.send_to_client(f"COMPLETE: 校准完成，但回调脚本执行失败")
                                
                                time.sleep(5)
                                self.scanning = False
                                self.task_completed = True
                                break
                        else:
                            self.center_align_count = 0

                            dis = abs((offset_cm/100))
                            
                            # 判断偏离方向
                            if offset > 0:
                                adjust_msg = "向右调整"
                                result = rc.move_backward(distance=(dis-0.03), speed=0.3)
                                print(f"AGV调整结果: {result}")
                            else:
                                adjust_msg = "向左调整"
                                result = rc.move_forward(distance=(dis+0.06), speed=0.3)
                                print(f"AGV调整结果: {result}")
                            
                            time.sleep(6)
                    else:
                        print(f"[{timestamp}] 未检测到二维码 | 处理时间: {detection_time:.1f}ms")
                        self.center_align_count = 0
                
                # 等待0.1秒
                time.sleep(0.1)
                
        except Exception as e:
            print(f"[ERROR] 扫描任务异常: {e}")
            self.send_to_client(f"ERROR: 扫描任务异常: {str(e)}")
            self.scanning = False
        
        finally:
            # 关闭预览窗口
            if PREVIEW_ENABLED:
                cv2.destroyAllWindows()
    
        if not self.task_completed and not self.scanning:
            self.send_to_client("STOPPED: 扫描任务被停止")
    
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
            self.task_completed = False
            print("[INFO] 客户端已断开")
    
    def stop(self):
        """停止服务器"""
        self.is_running = False
        self.scanning = False
        
        # 关闭预览窗口
        if PREVIEW_ENABLED:
            cv2.destroyAllWindows()
        
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
    
    # 声明使用全局变量
    global PREVIEW_ENABLED, PREVIEW_WINDOW_NAME, PREVIEW_SCALE
    
    parser = argparse.ArgumentParser(description='二维码检测TCP服务器')
    parser.add_argument('--host', type=str, default='0.0.0.0', help='服务器主机地址 (默认: 0.0.0.0)')
    parser.add_argument('--port', type=int, default=8888, help='服务器端口 (默认: 8888)')
    parser.add_argument('--rotated-width', type=int, default=ROTATED_WIDTH, 
                       help=f'旋转后图像宽度 (默认: {ROTATED_WIDTH})')
    parser.add_argument('--rotated-height', type=int, default=ROTATED_HEIGHT,
                       help=f'旋转后图像高度 (默认: {ROTATED_HEIGHT})')
    parser.add_argument('--center-x', type=int, default=None, 
                       help=f'图像水平中心点 (默认: rotated-width/2)')
    parser.add_argument('--center-y', type=int, default=None,
                       help=f'图像垂直中心点 (默认: rotated-height/2)')
    parser.add_argument('--threshold', type=int, default=CENTER_ALIGN_THRESHOLD,
                       help=f'中心对齐阈值(像素) (默认: {CENTER_ALIGN_THRESHOLD})')
    parser.add_argument('--pixels-per-cm', type=float, default=PIXELS_PER_CM,
                       help=f'每厘米像素数 (默认: {PIXELS_PER_CM})')
    parser.add_argument('--callback-script', type=str, default='callback_task.py',
                       help=f'校准完成后的回调脚本 (默认: callback_task.py)')
    parser.add_argument('--client', action='store_true', help='以客户端模式运行')
    parser.add_argument('--client-host', type=str, default='127.0.0.1', help='客户端连接的主机地址')
    parser.add_argument('--preview', action='store_true', help='启用画面预览')
    parser.add_argument('--no-preview', action='store_false', dest='preview', help='禁用画面预览')
    parser.set_defaults(preview=PREVIEW_ENABLED)
    
    args = parser.parse_args()
    
    # 计算中心点
    center_x = args.center_x if args.center_x is not None else (args.rotated_width // 2 + 100)
    center_y = args.center_y if args.center_y is not None else args.rotated_height // 2
    
    # 计算原始采集分辨率
    raw_width = args.rotated_height  # 原始宽度 = 旋转后的高度
    raw_height = args.rotated_width  # 原始高度 = 旋转后的宽度
    
    # 更新预览设置
    PREVIEW_ENABLED = args.preview
    
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
        print(f"目标旋转后分辨率: {args.rotated_width} x {args.rotated_height}")
        print(f"原始采集分辨率: {raw_width} x {raw_height}")
        print(f"图像中心点: ({center_x}, {center_y})")
        print(f"对齐阈值: {args.threshold} 像素")
        print(f"像素-厘米转换: {args.pixels_per_cm} 像素/厘米")
        print(f"连续对齐次数: 5 次")
        print(f"回调脚本: {args.callback_script}")
        print(f"画面预览: {'启用' if PREVIEW_ENABLED else '禁用'}")
        print("=" * 60)
        print("工作流程:")
        print("1. 客户端连接并发送START指令")
        print("2. 客户端可立即断开连接")
        print("3. 服务器持续调整AGV位置直到校准完成")
        print("4. 校准完成后调用回调脚本")
        print("=" * 60)
        
        # 创建并启动服务器
        server = QRTcpServer(
            host=args.host, 
            port=args.port,
            center_x=center_x,
            center_y=center_y,
            threshold=args.threshold,
            pixels_per_cm=args.pixels_per_cm,
            callback_script=args.callback_script,
            target_raw_width=raw_width,
            target_raw_height=raw_height
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