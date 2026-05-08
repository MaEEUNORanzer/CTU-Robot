# -- coding: utf-8 --
import sys
import platform
import os
import cv2
import numpy as np
import math
import time
import threading
import queue
from ctypes import *
from pyzbar.pyzbar import decode, ZBarSymbol
import tkinter as tk
from tkinter import ttk, messagebox, scrolledtext
from PIL import Image, ImageTk

# 兼容不同操作系统加载动态库
currentsystem = platform.system()
if currentsystem == 'Windows':
    sys.path.append(os.path.join(os.getenv('MVCAM_COMMON_RUNENV'), "Samples", "Python", "MvImport"))
else:
    sys.path.append(os.path.join("..", "..", "MvImport"))

from MvCameraControl_class import *

# ================== 用户配置区域 ==================
# 注意：以下中心点坐标是基于旋转90度后的坐标系
IMAGE_CENTER_X = 512  # 旋转后图像水平中心点（原图像高度）
IMAGE_CENTER_Y = 640  # 旋转后图像垂直中心点（原图像宽度）
CENTER_ALIGN_THRESHOLD = 100  # 中心对齐阈值(像素)，在此范围内算中心对齐
PIXELS_PER_CM = 12.8  # 每厘米对应的像素数，可配置
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
            print(f"连接相机失败: {e}")
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
                    'confidence': 0.5,
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


class QRCodeGUI:
    """二维码检测GUI应用程序，基于旋转90度的坐标系"""
    def __init__(self):
        # 初始化相机，自动旋转90度
        self.camera = SimpleCamera(rotate_90_degrees=True)
        self.detector = None
        self.running = False
        self.thread = None
        self.image_queue = queue.Queue(maxsize=1)
        self.detection_interval = 3.0
        self.cycle_count = 0
        self.detection_count = 0
        self.start_time = 0
        self.last_detection_time = 0
        self.last_frame = None
        
        # 创建主窗口
        self.root = tk.Tk()
        self.root.title("DEBUG")
        self.root.geometry("1200x800")
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
        
        # 设置样式
        style = ttk.Style()
        style.theme_use('clam')
        
        # 创建左侧控制面板
        self.create_control_panel()
        
        # 创建右侧显示区域
        self.create_display_area()
        
    def create_control_panel(self):
        """创建左侧控制面板"""
        control_frame = ttk.Frame(self.root, padding="10")
        control_frame.grid(row=0, column=0, sticky="nsew", padx=5, pady=5)
        
        # 标题
        title_label = ttk.Label(control_frame, text="DEBUG", font=("Arial", 16, "bold"))
        title_label.grid(row=0, column=0, pady=(0, 20), sticky="w")
        
        # 坐标系说明
        info_frame = ttk.LabelFrame(control_frame, text="坐标系说明", padding="10")
        info_frame.grid(row=1, column=0, sticky="ew", pady=(0, 10))
        
        info_text = "坐标系已旋转90度：\n"
        info_text += "相机与目标相距必须控制在70CM左右\n"
        info_text += "- 相机采集时自动旋转90度\n"
        info_text += "- 所有坐标基于旋转后的坐标系\n"
        info_text += "- 中心点对应旋转后的图像中心"
        
        ttk.Label(info_frame, text=info_text, font=("Arial", 9)).grid(row=0, column=0, sticky="w", pady=5)
        
        # 中心点设置
        center_frame = ttk.LabelFrame(control_frame, text="中心点设置 (旋转后坐标系)", padding="10")
        center_frame.grid(row=2, column=0, sticky="ew", pady=(0, 10))
        
        ttk.Label(center_frame, text="水平中心点 (X):").grid(row=0, column=0, sticky="w", pady=5)
        self.center_x_var = tk.StringVar(value=str(IMAGE_CENTER_X))
        ttk.Entry(center_frame, textvariable=self.center_x_var, width=10).grid(row=0, column=1, sticky="w", pady=5, padx=(5, 0))
        
        ttk.Label(center_frame, text="垂直中心点 (Y):").grid(row=1, column=0, sticky="w", pady=5)
        self.center_y_var = tk.StringVar(value=str(IMAGE_CENTER_Y))
        ttk.Entry(center_frame, textvariable=self.center_y_var, width=10).grid(row=1, column=1, sticky="w", pady=5, padx=(5, 0))
        
        ttk.Label(center_frame, text="对齐阈值 (像素):").grid(row=2, column=0, sticky="w", pady=5)
        self.threshold_var = tk.StringVar(value=str(CENTER_ALIGN_THRESHOLD))
        ttk.Entry(center_frame, textvariable=self.threshold_var, width=10).grid(row=2, column=1, sticky="w", pady=5, padx=(5, 0))
        
        # 像素-厘米转换设置
        conversion_frame = ttk.LabelFrame(control_frame, text="像素-厘米转换", padding="10")
        conversion_frame.grid(row=3, column=0, sticky="ew", pady=(0, 10))
        
        ttk.Label(conversion_frame, text="像素/厘米:").grid(row=0, column=0, sticky="w", pady=5)
        self.pixels_per_cm_var = tk.StringVar(value=str(PIXELS_PER_CM))
        ttk.Entry(conversion_frame, textvariable=self.pixels_per_cm_var, width=10).grid(row=0, column=1, sticky="w", pady=5, padx=(5, 0))
        
        # 检测设置
        detect_frame = ttk.LabelFrame(control_frame, text="检测设置", padding="10")
        detect_frame.grid(row=4, column=0, sticky="ew", pady=(0, 10))
        
        ttk.Label(detect_frame, text="检测间隔 (秒):").grid(row=0, column=0, sticky="w", pady=5)
        self.interval_var = tk.StringVar(value="3.0")
        ttk.Entry(detect_frame, textvariable=self.interval_var, width=10).grid(row=0, column=1, sticky="w", pady=5, padx=(5, 0))
        
        ttk.Label(detect_frame, text="最小二维码尺寸:").grid(row=1, column=0, sticky="w", pady=5)
        self.min_size_var = tk.StringVar(value="20")
        ttk.Entry(detect_frame, textvariable=self.min_size_var, width=10).grid(row=1, column=1, sticky="w", pady=5, padx=(5, 0))
        
        ttk.Label(detect_frame, text="置信度阈值:").grid(row=2, column=0, sticky="w", pady=5)
        self.confidence_var = tk.StringVar(value="0.3")
        ttk.Entry(detect_frame, textvariable=self.confidence_var, width=10).grid(row=2, column=1, sticky="w", pady=5, padx=(5, 0))
        
        # 功能开关
        switch_frame = ttk.LabelFrame(control_frame, text="功能开关", padding="10")
        switch_frame.grid(row=5, column=0, sticky="ew", pady=(0, 10))
        
        self.preprocess_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(switch_frame, text="启用图像预处理", variable=self.preprocess_var).grid(row=0, column=0, sticky="w", pady=5)
        
        self.multistrategy_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(switch_frame, text="启用多种检测策略", variable=self.multistrategy_var).grid(row=1, column=0, sticky="w", pady=5)
        
        # 控制按钮
        button_frame = ttk.Frame(control_frame)
        button_frame.grid(row=6, column=0, sticky="ew", pady=(0, 20))
        
        self.start_button = ttk.Button(button_frame, text="开始检测", command=self.start_detection, width=12)
        self.start_button.grid(row=0, column=0, pady=5, padx=(0, 5))
        
        self.stop_button = ttk.Button(button_frame, text="停止检测", command=self.stop_detection, state="disabled", width=12)
        self.stop_button.grid(row=0, column=1, pady=5)
        
        self.snapshot_button = ttk.Button(button_frame, text="保存截图", command=self.save_snapshot, width=12)
        self.snapshot_button.grid(row=1, column=0, pady=5, padx=(0, 5))
        
        # 状态信息
        status_frame = ttk.LabelFrame(control_frame, text="系统状态", padding="10")
        status_frame.grid(row=7, column=0, sticky="ew", pady=(0, 10))
        
        self.status_label = ttk.Label(status_frame, text="准备就绪", font=("Arial", 10))
        self.status_label.grid(row=0, column=0, sticky="w", pady=2)
        
        self.camera_status_label = ttk.Label(status_frame, text="相机: 未连接", font=("Arial", 9))
        self.camera_status_label.grid(row=1, column=0, sticky="w", pady=2)
        
        self.cycle_label = ttk.Label(status_frame, text="检测周期: 0", font=("Arial", 9))
        self.cycle_label.grid(row=2, column=0, sticky="w", pady=2)
        
        self.detection_label = ttk.Label(status_frame, text="检测次数: 0", font=("Arial", 9))
        self.detection_label.grid(row=3, column=0, sticky="w", pady=2)
        
        self.align_status_label = ttk.Label(status_frame, text="对齐状态: 未知", font=("Arial", 9))
        self.align_status_label.grid(row=4, column=0, sticky="w", pady=2)
        
        # 图像信息
        image_frame = ttk.LabelFrame(control_frame, text="图像信息", padding="10")
        image_frame.grid(row=8, column=0, sticky="ew", pady=(0, 10))
        
        self.image_info_label = ttk.Label(image_frame, text="尺寸: 未获取", font=("Arial", 9))
        self.image_info_label.grid(row=0, column=0, sticky="w", pady=2)
        
        # 统计信息
        stats_frame = ttk.LabelFrame(control_frame, text="运行统计", padding="10")
        stats_frame.grid(row=9, column=0, sticky="ew")
        
        self.time_label = ttk.Label(stats_frame, text="运行时间: 0s", font=("Arial", 9))
        self.time_label.grid(row=0, column=0, sticky="w", pady=2)
        
        self.rate_label = ttk.Label(stats_frame, text="检测率: 0%", font=("Arial", 9))
        self.rate_label.grid(row=1, column=0, sticky="w", pady=2)
        
        self.fps_label = ttk.Label(stats_frame, text="FPS: 0.0", font=("Arial", 9))
        self.fps_label.grid(row=2, column=0, sticky="w", pady=2)
        
    def create_display_area(self):
        """创建右侧显示区域"""
        display_frame = ttk.Frame(self.root, padding="10")
        display_frame.grid(row=0, column=1, sticky="nsew", padx=5, pady=5)
        
        # 视频显示区域
        video_frame = ttk.LabelFrame(display_frame, text="实时视频流 (已旋转90度)", padding="5")
        video_frame.grid(row=0, column=0, sticky="nsew", pady=(0, 10))
        
        self.video_label = ttk.Label(video_frame, text="等待视频流...", relief="sunken")
        self.video_label.grid(row=0, column=0, padx=5, pady=5, sticky="nsew")
        
        # 结果显示区域
        result_frame = ttk.LabelFrame(display_frame, text="检测结果 (基于旋转坐标系)", padding="10")
        result_frame.grid(row=1, column=0, sticky="ew")
        
        # 创建一个文本显示框
        self.result_text = scrolledtext.ScrolledText(result_frame, height=10, width=80, font=("Courier", 9))
        self.result_text.grid(row=0, column=0, sticky="nsew")
        
        # 配置网格权重
        self.root.grid_columnconfigure(1, weight=1)
        self.root.grid_rowconfigure(0, weight=1)
        display_frame.grid_columnconfigure(0, weight=1)
        display_frame.grid_rowconfigure(0, weight=3)
        display_frame.grid_rowconfigure(1, weight=1)
        
    def start_detection(self):
        """开始检测"""
        try:
            # 获取配置参数
            center_x = int(self.center_x_var.get())
            center_y = int(self.center_y_var.get())
            align_threshold = int(self.threshold_var.get())
            pixels_per_cm = float(self.pixels_per_cm_var.get())
            detection_interval = float(self.interval_var.get())
            min_size = int(self.min_size_var.get())
            confidence_threshold = float(self.confidence_var.get())
            
            # 初始化检测器
            self.detector = QRCodeDetectorOptimized(center_x, center_y, align_threshold, pixels_per_cm)
            self.detector.set_confidence_threshold(confidence_threshold)
            self.detector.set_min_qr_size(min_size)
            self.detector.set_preprocessing(self.preprocess_var.get())
            self.detector.use_multiple_strategies = self.multistrategy_var.get()
            
            # 连接相机
            if not self.camera.connect_camera():
                messagebox.showerror("错误", "无法连接相机，请检查相机连接")
                return
                
            if not self.camera.start_streaming():
                messagebox.showerror("错误", "无法启动相机采集")
                return
            
            # 更新状态
            self.status_label.config(text="✓ 相机已连接")
            self.camera_status_label.config(text="相机: 运行中")
            
            # 重置计数
            self.cycle_count = 0
            self.detection_count = 0
            self.start_time = time.time()
            self.last_detection_time = 0
            
            # 启用/禁用按钮
            self.start_button.config(state="disabled")
            self.stop_button.config(state="normal")
            self.snapshot_button.config(state="normal")
            
            # 启动检测线程
            self.running = True
            self.thread = threading.Thread(target=self.detection_loop, daemon=True)
            self.thread.start()
            
        except ValueError as e:
            messagebox.showerror("输入错误", "请输入有效的数值")
        except Exception as e:
            messagebox.showerror("启动错误", f"启动失败: {str(e)}")
    
    def stop_detection(self):
        """停止检测"""
        self.running = False
        if self.thread:
            self.thread.join(timeout=2.0)
        
        self.camera.stop_streaming()
        
        # 更新状态
        self.status_label.config(text="检测已停止")
        self.camera_status_label.config(text="相机: 已停止")
        
        # 启用/禁用按钮
        self.start_button.config(state="normal")
        self.stop_button.config(state="disabled")
        self.snapshot_button.config(state="disabled")
        
        # 显示统计信息
        self.update_statistics()
    
    def detection_loop(self):
        """检测循环"""
        detection_interval = float(self.interval_var.get())
        
        while self.running:
            cycle_start_time = time.time()
            self.cycle_count += 1
            
            # 清空缓冲区
            self.camera.clear_frame_buffer()
            
            # 获取最新帧
            image = self.camera.get_frame()
            
            if image is None:
                self.update_result_text(f"[{time.strftime('%H:%M:%S')}] 获取图像失败\n")
                time.sleep(detection_interval)
                continue
            
            # 记录当前帧
            self.last_frame = image.copy()
            
            # 更新图像信息
            self.update_image_info()
            
            # 检测二维码
            detection_start = time.time()
            detections = self.detector.detect_qrcodes(image)
            detection_time = (time.time() - detection_start) * 1000
            
            # 处理检测结果
            if detections:
                self.detection_count += 1
                self.last_detection_time = time.time()
                
                # 获取第一个检测结果
                detection = detections[0]
                offset = detection['horizontal_offset']
                offset_cm = detection['horizontal_cm']
                is_aligned = detection['is_aligned']
                direction = detection['direction']
                
                # 判断对齐状态
                if is_aligned:
                    status = "✓ 中心已对齐"
                    self.align_status_label.config(text=f"对齐状态: 已对齐", foreground="green")
                else:
                    if offset > 0:
                        status = "← 需向左移动"
                    else:
                        status = "→ 需向右移动"
                    self.align_status_label.config(text=f"对齐状态: 未对齐", foreground="red")
                
                # 构建结果文本
                result_text = f"\n[{time.strftime('%Y-%m-%d %H:%M:%S')}] 检测周期 {self.cycle_count}\n"
                result_text += f"检测到二维码 (旋转坐标系)\n"
                result_text += f"二维码中心: ({detection['center_x']}, {detection['center_y']})\n"
                result_text += f"水平偏移: {offset:+d} 像素 ({direction})\n"
                result_text += f"水平偏移: {offset_cm:+.2f} 厘米\n"
                result_text += f"垂直偏移: {detection['vertical_offset']:+d} 像素\n"
                result_text += f"垂直偏移: {detection['vertical_cm']:+.2f} 厘米\n"
                result_text += f"对齐阈值: ±{self.detector.align_threshold} 像素\n"
                result_text += f"对齐状态: {status}\n"
                result_text += f"像素-厘米转换: {self.detector.pixels_per_cm:.1f} 像素/厘米\n"
                result_text += f"处理时间: {detection_time:.1f}ms\n"
                
                if detection['data']:
                    data_preview = detection['data'][:50] + "..." if len(detection['data']) > 50 else detection['data']
                    result_text += f"二维码数据: {data_preview}\n"
                
                self.update_result_text(result_text)
                
                # 绘制检测结果
                display_image = self.draw_detection_result(image.copy(), detection)
            else:
                self.update_result_text(f"[{time.strftime('%Y-%m-%d %H:%M:%S')}] 检测周期 {self.cycle_count} - 未检测到二维码\n")
                self.align_status_label.config(text="对齐状态: 未检测到二维码", foreground="gray")
                display_image = self.draw_center_lines(image.copy())
            
            # 更新视频显示
            self.update_video_display(display_image)
            
            # 更新状态信息
            self.update_status_display(detection_time)
            
            # 计算本次周期耗时
            cycle_time = time.time() - cycle_start_time
            
            # 等待下一次检测
            if cycle_time < detection_interval:
                time.sleep(detection_interval - cycle_time)
            else:
                self.update_result_text(f"警告: 检测周期耗时{cycle_time:.1f}秒，超过设定的{detection_interval}秒间隔\n")
    
    def draw_center_lines(self, image):
        """绘制中心线"""
        if image is None:
            return image
        
        height, width = image.shape[:2]
        center_x = int(self.center_x_var.get())
        center_y = int(self.center_y_var.get())
        threshold = int(self.threshold_var.get())
        
        # 绘制中心十字线
        cv2.line(image, (center_x, 0), (center_x, height), (0, 255, 255), 1, cv2.LINE_AA)
        cv2.line(image, (0, center_y), (width, center_y), (0, 255, 255), 1, cv2.LINE_AA)
        
        # 绘制中心点
        cv2.circle(image, (center_x, center_y), 3, (0, 255, 255), -1)
        
        # 绘制对齐阈值区域
        left_bound = center_x - threshold
        right_bound = center_x + threshold
        cv2.line(image, (left_bound, center_y-10), (left_bound, center_y+10), (0, 255, 0), 2)
        cv2.line(image, (right_bound, center_y-10), (right_bound, center_y+10), (0, 255, 0), 2)
        
        # 绘制像素-厘米转换信息
        if self.detector:
            pixels_per_cm = self.detector.pixels_per_cm
            cv2.putText(image, f"1cm = {pixels_per_cm:.1f}px", (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        # 绘制坐标系信息
        cv2.putText(image, "坐标系: 已旋转90度", (10, height-20),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        return image
    
    def draw_detection_result(self, image, detection):
        """绘制检测结果"""
        if image is None:
            return image
        
        image = self.draw_center_lines(image)
        
        # 绘制二维码边界框
        if 'points' in detection and len(detection['points']) >= 4:
            points = np.array(detection['points'], np.int32)
            
            # 根据对齐状态选择颜色
            if detection['is_aligned']:
                color = (0, 255, 0)  # 绿色
            else:
                if detection['horizontal_offset'] > 0:
                    color = (0, 165, 255)  # 橙色
                else:
                    color = (255, 0, 0)  # 红色
            
            cv2.polylines(image, [points], True, color, 2)
            
            # 绘制二维码中心点
            cv2.circle(image, (detection['center_x'], detection['center_y']), 4, color, -1)
            
            # 绘制从图像中心到二维码中心的连线
            center_x = int(self.center_x_var.get())
            center_y = int(self.center_y_var.get())
            cv2.line(image, (center_x, center_y), (detection['center_x'], detection['center_y']), (255, 255, 255), 1, cv2.LINE_AA)
            
            # 标注偏移量
            offset = detection['horizontal_offset']
            offset_cm = detection['horizontal_cm']
            cv2.putText(image, f"偏移: {offset:+d}px ({offset_cm:+.2f}cm)", 
                       (detection['center_x']-60, detection['center_y']-20),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        return image
    
    def update_video_display(self, image):
        """更新视频显示"""
        if image is None:
            return
        
        # 调整图像大小以适应显示
        height, width = image.shape[:2]
        max_width = 800
        max_height = 500
        
        if width > max_width or height > max_height:
            scale = min(max_width/width, max_height/height)
            new_width = int(width * scale)
            new_height = int(height * scale)
            image = cv2.resize(image, (new_width, new_height))
        
        # 转换颜色空间
        image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        image_pil = Image.fromarray(image_rgb)
        image_tk = ImageTk.PhotoImage(image_pil)
        
        # 更新显示
        self.video_label.config(image=image_tk)
        self.video_label.image = image_tk
    
    def update_image_info(self):
        """更新图像信息"""
        info = self.camera.get_image_info()
        if info and info['original_width'] > 0:
            text = f"原始尺寸: {info['original_width']}x{info['original_height']}\n"
            text += f"旋转尺寸: {info['rotated_width']}x{info['rotated_height']}"
            self.image_info_label.config(text=text)
    
    def update_result_text(self, text):
        """更新结果文本"""
        self.root.after(0, self._update_result_text, text)
    
    def _update_result_text(self, text):
        """在主线程中更新结果文本"""
        self.result_text.insert("end", text)
        self.result_text.see("end")
    
    def update_status_display(self, detection_time):
        """更新状态显示"""
        self.root.after(0, self._update_status_display, detection_time)
    
    def _update_status_display(self, detection_time):
        """在主线程中更新状态显示"""
        # 更新基本状态
        self.cycle_label.config(text=f"检测周期: {self.cycle_count}")
        self.detection_label.config(text=f"检测次数: {self.detection_count}")
        
        # 更新运行时间
        elapsed_time = time.time() - self.start_time
        self.time_label.config(text=f"运行时间: {elapsed_time:.1f}s")
        
        # 更新FPS
        if elapsed_time > 0 and self.cycle_count > 0:
            fps = self.cycle_count / elapsed_time
            self.fps_label.config(text=f"FPS: {fps:.1f}")
        
        # 更新检测率
        if self.cycle_count > 0:
            detection_rate = (self.detection_count / self.cycle_count) * 100
            self.rate_label.config(text=f"检测率: {detection_rate:.1f}%")
    
    def save_snapshot(self):
        """保存当前帧截图"""
        if self.last_frame is None:
            messagebox.showwarning("警告", "没有可保存的图像")
            return
        
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        filename = f"qr_snapshot_rotated_{timestamp}.jpg"
        
        try:
            cv2.imwrite(filename, self.last_frame)
            messagebox.showinfo("成功", f"截图已保存为: {filename}")
        except Exception as e:
            messagebox.showerror("错误", f"保存截图失败: {str(e)}")
    
    def update_statistics(self):
        """更新统计信息"""
        elapsed_time = time.time() - self.start_time
        
        stats_text = f"\n{'='*60}\n"
        stats_text += "程序运行统计信息 (旋转坐标系):\n"
        stats_text += f"{'='*60}\n"
        stats_text += f"总运行时间: {elapsed_time:.1f}秒\n"
        stats_text += f"总检测周期: {self.cycle_count}\n"
        stats_text += f"检测到二维码次数: {self.detection_count}\n"
        
        if self.cycle_count > 0:
            detection_rate = (self.detection_count / self.cycle_count) * 100
            stats_text += f"二维码检测率: {detection_rate:.1f}%\n"
            stats_text += f"平均周期时间: {elapsed_time/self.cycle_count:.1f}秒\n"
        
        center_x = int(self.center_x_var.get())
        center_y = int(self.center_y_var.get())
        threshold = int(self.threshold_var.get())
        if self.detector:
            pixels_per_cm = self.detector.pixels_per_cm
            stats_text += f"像素-厘米转换: {pixels_per_cm:.1f} 像素/厘米\n"
        
        stats_text += f"最终中心点: ({center_x}, {center_y})\n"
        stats_text += f"最终对齐阈值: {threshold}像素\n"
        
        # 显示图像信息
        info = self.camera.get_image_info()
        if info and info['original_width'] > 0:
            stats_text += f"原始图像尺寸: {info['original_width']}x{info['original_height']}\n"
            stats_text += f"处理后图像尺寸: {info['rotated_width']}x{info['rotated_height']}\n"
        
        stats_text += f"{'='*60}\n"
        
        self.update_result_text(stats_text)
    
    def on_closing(self):
        """窗口关闭事件处理"""
        if self.running:
            self.stop_detection()
        self.root.destroy()
    
    def run(self):
        """运行GUI应用程序"""
        # 配置网格权重
        self.root.grid_columnconfigure(0, weight=0)
        self.root.grid_columnconfigure(1, weight=1)
        self.root.grid_rowconfigure(0, weight=1)
        
        # 运行主循环
        self.root.mainloop()


def main():
    """主函数"""
    app = QRCodeGUI()
    app.run()


if __name__ == "__main__":
    main()
