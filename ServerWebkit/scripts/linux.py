# -- coding: utf-8 --
"""
海康工业相机二维码扫描与偏差测量系统
功能：图像旋转校正、二维码识别、水平偏差测量、阈值判断
"""

import sys
import platform
import threading
import os
import numpy as np
import time
import json
import math
from datetime import datetime
from ctypes import *
import cv2
from collections import deque
import argparse

# ==================== 1. 路径配置 ====================
currentsystem = platform.system()
if currentsystem == 'Windows':
    sys.path.append(os.path.join(os.getenv('MVCAM_COMMON_RUNENV'), "Samples", "Python", "MvImport"))
else:
    mvs_base = "/opt/MVS"
    sys.path.append(os.path.join(mvs_base, "Samples", "aarch64", "Python", "MvImport"))
    lib_path = os.path.join(mvs_base, "lib", "aarch64")
    if os.path.exists(lib_path):
        os.environ['LD_LIBRARY_PATH'] = lib_path + ":" + os.environ.get('LD_LIBRARY_PATH', '')

# 尝试导入二维码识别库
try:
    from pyzbar import pyzbar
    HAS_PYZBAR = True
except ImportError:
    print("警告: 未找到 pyzbar 库，尝试使用 OpenCV 内置二维码检测")
    try:
        qr_detector = cv2.QRCodeDetector()
        HAS_PYZBAR = False
        HAS_CV_QR = True
    except:
        HAS_CV_QR = False
        print("错误: 无法导入二维码检测库")
        sys.exit(1)

from MvCameraControl_class import *

# ==================== 2. 图像旋转与偏差测量类 ====================
class ImageRotationAndAlignment:
    """图像旋转校正与偏差测量类"""
    
    def __init__(self, rotation_angle=-90, center_threshold=300):
        """
        初始化
        
        参数:
            rotation_angle: 旋转角度（度），负数表示逆时针
            center_threshold: 中心阈值（像素），默认300x300区域
        """
        self.rotation_angle = rotation_angle
        self.center_threshold = center_threshold
        self.center_region = None
        self.image_center = None
        self.rotation_matrix = None
        self.need_rotation = abs(rotation_angle) > 0.1
        
        print(f"图像旋转设置: 逆时针旋转 {-rotation_angle} 度")
        print(f"中心阈值区域: {center_threshold}x{center_threshold} 像素")
    
    def rotate_image(self, image):
        """
        旋转图像（逆时针90度）
        
        参数:
            image: 输入图像
            
        返回:
            旋转后的图像
        """
        if image is None or not self.need_rotation:
            return image
        
        try:
            # 获取图像尺寸
            (h, w) = image.shape[:2]
            
            # 计算旋转中心
            center = (w // 2, h // 2)
            
            # 创建旋转矩阵
            self.rotation_matrix = cv2.getRotationMatrix2D(center, self.rotation_angle, 1.0)
            
            # 计算旋转后的图像边界
            cos = np.abs(self.rotation_matrix[0, 0])
            sin = np.abs(self.rotation_matrix[0, 1])
            
            new_w = int((h * sin) + (w * cos))
            new_h = int((h * cos) + (w * sin))
            
            # 调整旋转矩阵的平移分量
            self.rotation_matrix[0, 2] += (new_w / 2) - center[0]
            self.rotation_matrix[1, 2] += (new_h / 2) - center[1]
            
            # 执行旋转
            rotated = cv2.warpAffine(image, self.rotation_matrix, (new_w, new_h))
            
            # 更新图像中心点
            self.image_center = (new_w // 2, new_h // 2)
            
            # 计算中心区域
            half_threshold = self.center_threshold // 2
            x1 = self.image_center[0] - half_threshold
            y1 = self.image_center[1] - half_threshold
            x2 = self.image_center[0] + half_threshold
            y2 = self.image_center[1] + half_threshold
            
            # 确保坐标在图像范围内
            x1 = max(0, x1)
            y1 = max(0, y1)
            x2 = min(new_w, x2)
            y2 = min(new_h, y2)
            
            self.center_region = (x1, y1, x2, y2)
            
            return rotated
            
        except Exception as e:
            print(f"图像旋转失败: {e}")
            return image
    
    def transform_point(self, point):
        """
        将点坐标从原始图像转换到旋转后的图像
        
        参数:
            point: (x, y) 坐标
            
        返回:
            旋转后的坐标
        """
        if not self.need_rotation or self.rotation_matrix is None:
            return point
        
        try:
            x, y = point
            # 应用旋转矩阵变换
            x_new = self.rotation_matrix[0, 0] * x + self.rotation_matrix[0, 1] * y + self.rotation_matrix[0, 2]
            y_new = self.rotation_matrix[1, 0] * x + self.rotation_matrix[1, 1] * y + self.rotation_matrix[1, 2]
            
            return (int(x_new), int(y_new))
        except:
            return point
    
    def calculate_horizontal_deviation(self, qr_center):
        """
        计算水平偏差
        
        参数:
            qr_center: 二维码中心点 (x, y)
            
        返回:
            dict: 包含偏差信息的字典
        """
        if self.image_center is None or qr_center is None:
            return None
        
        try:
            qr_x, qr_y = qr_center
            center_x, center_y = self.image_center
            
            # 计算水平偏差（x方向）
            horizontal_deviation = qr_x - center_x
            
            # 计算垂直偏差（y方向）
            vertical_deviation = qr_y - center_y
            
            # 计算总距离
            total_distance = math.sqrt(horizontal_deviation**2 + vertical_deviation**2)
            
            # 计算角度
            angle = math.degrees(math.atan2(vertical_deviation, horizontal_deviation))
            
            # 判断是否在中心区域内
            in_center_region = self.is_in_center_region(qr_center)
            
            deviation_info = {
                'horizontal': horizontal_deviation,
                'vertical': vertical_deviation,
                'distance': total_distance,
                'angle': angle,
                'in_center_region': in_center_region,
                'qr_center': qr_center,
                'image_center': self.image_center
            }
            
            return deviation_info
            
        except Exception as e:
            print(f"偏差计算失败: {e}")
            return None
    
    def is_in_center_region(self, point):
        """
        判断点是否在中心区域内
        
        参数:
            point: (x, y) 坐标
            
        返回:
            bool: 是否在中心区域内
        """
        if self.center_region is None or point is None:
            return False
        
        x, y = point
        x1, y1, x2, y2 = self.center_region
        
        return x1 <= x <= x2 and y1 <= y <= y2
    
    def get_deviation_status(self, deviation_info):
        """
        获取偏差状态描述
        
        参数:
            deviation_info: 偏差信息字典
            
        返回:
            str: 状态描述
        """
        if deviation_info is None:
            return "无偏差信息"
        
        horizontal = deviation_info['horizontal']
        vertical = deviation_info['vertical']
        
        status_parts = []
        
        if abs(horizontal) < 10 and abs(vertical) < 10:
            status_parts.append("完美居中")
        elif deviation_info['in_center_region']:
            status_parts.append("在中心区域内")
        else:
            if horizontal > 0:
                status_parts.append(f"偏右 {abs(horizontal):.1f}px")
            elif horizontal < 0:
                status_parts.append(f"偏左 {abs(horizontal):.1f}px")
            
            if vertical > 0:
                status_parts.append(f"偏下 {abs(vertical):.1f}px")
            elif vertical < 0:
                status_parts.append(f"偏上 {abs(vertical):.1f}px")
        
        return " | ".join(status_parts) if status_parts else "位置正常"

# ==================== 3. 高精度二维码扫描器 ====================
class HighPrecisionQRScanner:
    """高精度二维码扫描器"""
    
    def __init__(self, config=None):
        self.config = {
            'detection_interval': 0.3,
            'min_qr_size': 30,
            'confidence_threshold': 0.5,
            'save_detections': True,
            'show_debug': False,
        }
        
        if config:
            self.config.update(config)
        
        self.last_detection_time = 0
        self.detection_history = deque(maxlen=100)
        self.qr_statistics = {
            'total_detected': 0,
            'successful_reads': 0,
            'failed_reads': 0,
            'last_detection_time': None
        }
        
        if self.config['save_detections']:
            self.init_save_directories()
    
    def init_save_directories(self):
        self.base_dir = "qr_alignments"
        self.sub_dirs = {
            'aligned': os.path.join(self.base_dir, 'aligned'),
            'deviations': os.path.join(self.base_dir, 'deviations'),
        }
        
        for dir_path in self.sub_dirs.values():
            if not os.path.exists(dir_path):
                os.makedirs(dir_path)
    
    def detect_qrcodes(self, image):
        """检测二维码"""
        if image is None:
            return [], image
        
        current_time = time.time()
        if current_time - self.last_detection_time < self.config['detection_interval']:
            return [], image
        
        self.last_detection_time = current_time
        
        # 转换为灰度图
        if len(image.shape) == 3:
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        else:
            gray = image
        
        all_detections = []
        
        if HAS_PYZBAR:
            try:
                barcodes = pyzbar.decode(gray)
                
                for barcode in barcodes:
                    (x, y, w, h) = barcode.rect
                    
                    if w < self.config['min_qr_size'] or h < self.config['min_qr_size']:
                        continue
                    
                    try:
                        barcode_data = barcode.data.decode("utf-8")
                    except:
                        barcode_data = barcode.data.decode("latin-1", errors='ignore')
                    
                    barcode_type = barcode.type
                    
                    # 计算二维码中心
                    qr_center = (x + w // 2, y + h // 2)
                    
                    detection = {
                        'x': x,
                        'y': y,
                        'width': w,
                        'height': h,
                        'center_x': qr_center[0],
                        'center_y': qr_center[1],
                        'data': barcode_data,
                        'type': barcode_type,
                        'confidence': 0.9,
                        'timestamp': datetime.now().isoformat(),
                    }
                    
                    all_detections.append(detection)
                    
            except Exception as e:
                if self.config['show_debug']:
                    print(f"pyzbar检测失败: {e}")
        
        # 更新统计
        if all_detections:
            self.qr_statistics['total_detected'] += len(all_detections)
            self.qr_statistics['successful_reads'] += sum(1 for d in all_detections if d['data'])
            self.qr_statistics['failed_reads'] += sum(1 for d in all_detections if not d['data'])
            self.qr_statistics['last_detection_time'] = datetime.now().isoformat()
        
        return all_detections, gray

# ==================== 4. 可视化与标注函数 ====================
def draw_alignment_guidelines(image, rotation_system, deviation_info=None):
    """
    绘制对齐参考线和偏差信息
    
    参数:
        image: 输入图像
        rotation_system: 旋转系统对象
        deviation_info: 偏差信息
        
    返回:
        绘制了参考线的图像
    """
    if image is None:
        return image
    
    result = image.copy()
    h, w = result.shape[:2]
    
    # 绘制中心十字线
    if rotation_system.image_center:
        center_x, center_y = rotation_system.image_center
        
        # 水平线
        cv2.line(result, (0, center_y), (w, center_y), (0, 255, 255), 2, cv2.LINE_AA)
        # 垂直线
        cv2.line(result, (center_x, 0), (center_x, h), (0, 255, 255), 2, cv2.LINE_AA)
        
        # 绘制中心点
        cv2.circle(result, (center_x, center_y), 10, (0, 255, 255), -1)
        cv2.circle(result, (center_x, center_y), 15, (0, 255, 255), 2)
        
        # 绘制中心区域（阈值区域）
        if rotation_system.center_region:
            x1, y1, x2, y2 = rotation_system.center_region
            cv2.rectangle(result, (x1, y1), (x2, y2), (255, 255, 0), 2)
            
            # 标注中心区域尺寸
            region_w = x2 - x1
            region_h = y2 - y1
            cv2.putText(result, f"Center: {region_w}x{region_h}", 
                       (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
    
    # 绘制偏差信息
    if deviation_info:
        qr_center = deviation_info['qr_center']
        image_center = deviation_system.image_center
        
        if qr_center and image_center:
            # 绘制从中心到二维码的连线
            cv2.arrowedLine(result, image_center, qr_center, (0, 0, 255), 3, tipLength=0.05)
            
            # 在二维码中心绘制标记
            cv2.circle(result, qr_center, 8, (0, 0, 255), -1)
            cv2.circle(result, qr_center, 12, (0, 0, 255), 2)
            
            # 绘制偏差线
            horizontal = deviation_info['horizontal']
            vertical = deviation_info['vertical']
            
            # 水平偏差线
            if abs(horizontal) > 5:
                y_line = image_center[1]
                cv2.line(result, (image_center[0], y_line), (qr_center[0], y_line), 
                        (255, 0, 0), 2, cv2.LINE_AA)
            
            # 垂直偏差线
            if abs(vertical) > 5:
                x_line = image_center[0]
                cv2.line(result, (x_line, image_center[1]), (x_line, qr_center[1]), 
                        (0, 255, 0), 2, cv2.LINE_AA)
            
            # 标注偏差值
            deviation_text = f"ΔX: {horizontal:+.1f}px, ΔY: {vertical:+.1f}px"
            text_pos = (qr_center[0] + 20, qr_center[1] - 20)
            cv2.putText(result, deviation_text, text_pos,
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
    
    return result

def create_alignment_dashboard(image, scanner, rotation_system, frame_count, fps, detections, deviation_info):
    """
    创建对齐仪表板
    
    参数:
        image: 输入图像
        scanner: 二维码扫描器
        rotation_system: 旋转系统
        frame_count: 帧计数
        fps: 帧率
        detections: 检测结果
        deviation_info: 偏差信息
        
    返回:
        带有仪表板的图像
    """
    if image is None:
        return image
    
    overlay = image.copy()
    h, w = overlay.shape[:2]
    
    # 创建半透明背景
    overlay_bg = overlay.copy()
    cv2.rectangle(overlay_bg, (0, 0), (w, 180), (0, 0, 0), -1)
    overlay = cv2.addWeighted(overlay, 0.7, overlay_bg, 0.3, 0)
    
    # 系统信息
    info_lines = [
        f"二维码对齐测量系统 | 旋转: {-rotation_system.rotation_angle}° | 阈值: {rotation_system.center_threshold}px",
        f"帧率: {fps:.1f} FPS | 帧号: {frame_count} | 尺寸: {w}x{h}",
        f"{'='*60}"
    ]
    
    # 检测状态
    if detections:
        info_lines.append(f"检测到 {len(detections)} 个二维码")
        
        for i, det in enumerate(detections[:2]):  # 显示前2个
            info_lines.append(f"  QR#{i+1}: {det['type']} (置信度: {det['confidence']:.2f})")
            if det['data'] and len(det['data']) < 30:
                info_lines.append(f"    数据: {det['data']}")
    else:
        info_lines.append("未检测到二维码")
    
    # 偏差信息
    if deviation_info:
        status = rotation_system.get_deviation_status(deviation_info)
        info_lines.extend([
            f"{'='*60}",
            f"对齐状态: {status}",
            f"水平偏差: {deviation_info['horizontal']:+.1f} px",
            f"垂直偏差: {deviation_info['vertical']:+.1f} px",
            f"总距离: {deviation_info['distance']:.1f} px",
            f"角度: {deviation_info['angle']:.1f}°"
        ])
        
        # 偏差判断
        if deviation_info['in_center_region']:
            info_lines.append("✓ 二维码在中心阈值区域内")
        else:
            info_lines.append("✗ 二维码超出中心阈值区域")
    
    # 统计信息
    stats = scanner.qr_statistics
    info_lines.extend([
        f"{'='*60}",
        f"检测统计: 总数={stats['total_detected']}, 成功={stats['successful_reads']}, 失败={stats['failed_reads']}"
    ])
    
    # 控制提示
    info_lines.extend([
        f"{'='*60}",
        "控制: [Q]退出 [S]截图 [D]调试 [P]暂停 [R]重置统计 [C]校准中心"
    ])
    
    # 绘制文本
    for i, line in enumerate(info_lines):
        y_pos = 25 + i * 20
        if y_pos < h - 10:
            cv2.putText(overlay, line, (10, y_pos),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    
    return overlay

# ==================== 5. 工作线程 ====================
g_bExit = False
frame_counter = 0
fps = 0
last_time = time.time()
paused = False
last_deviation_print = 0

def work_thread(cam, scanner, rotation_system, show_gui=True):
    """工作线程函数"""
    global g_bExit, frame_counter, fps, last_time, paused, last_deviation_print
    
    stOutFrame = MV_FRAME_OUT()  
    memset(byref(stOutFrame), 0, sizeof(stOutFrame))
    
    # 创建显示窗口
    window_name = "QR Alignment Measurement System"
    if show_gui:
        try:
            cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(window_name, 1024, 768)
        except:
            show_gui = False
            print("警告: 无法创建显示窗口，将仅输出文本信息")
    
    print("\n" + "="*60)
    print("二维码对齐测量系统已启动")
    print("="*60)
    
    while not g_bExit:
        if paused:
            cv2.waitKey(100)
            continue
            
        # 获取图像
        ret = cam.MV_CC_GetImageBuffer(stOutFrame, 1000)
        
        if ret == 0 and stOutFrame.pBufAddr is not None:
            # 帧计数
            frame_counter += 1
            
            # 计算帧率
            current_time = time.time()
            if current_time - last_time >= 1.0:
                fps = frame_counter / (current_time - last_time)
                frame_counter = 0
                last_time = current_time
            
            # 转换图像
            image = convert_to_image(stOutFrame.pBufAddr, stOutFrame.stFrameInfo)
            
            if image is not None:
                # 1. 旋转图像
                rotated_image = rotation_system.rotate_image(image)
                
                # 2. 检测二维码
                detections, enhanced_img = scanner.detect_qrcodes(rotated_image)
                
                # 3. 计算偏差
                deviation_info = None
                if detections:
                    # 取第一个检测到的二维码
                    qr = detections[0]
                    qr_center = (qr['center_x'], qr['center_y'])
                    
                    # 计算偏差
                    deviation_info = rotation_system.calculate_horizontal_deviation(qr_center)
                    
                    # 打印偏差信息（每秒一次）
                    if deviation_info and current_time - last_deviation_print >= 1.0:
                        print_deviation_info(deviation_info, qr)
                        last_deviation_print = current_time
                
                # 4. 显示结果
                if show_gui:
                    # 绘制参考线
                    aligned_image = draw_alignment_guidelines(rotated_image, rotation_system, deviation_info)
                    
                    # 添加仪表板
                    dashboard_image = create_alignment_dashboard(
                        aligned_image, scanner, rotation_system, 
                        frame_counter, fps, detections, deviation_info
                    )
                    
                    # 显示图像
                    cv2.imshow(window_name, dashboard_image)
                    
                    # 键盘控制
                    key = cv2.waitKey(1) & 0xFF
                    handle_keyboard_input(key, scanner)
                    
                    if key == ord('q') or key == 27:
                        g_bExit = True
            
            # 释放缓冲区
            cam.MV_CC_FreeImageBuffer(stOutFrame)
        else:
            if ret != 0:
                print(f"获取图像失败: 0x{ret:x}")
            time.sleep(0.01)
    
    if show_gui:
        cv2.destroyAllWindows()

def print_deviation_info(deviation_info, qr_info):
    """打印偏差信息"""
    if not deviation_info:
        return
    
    horizontal = deviation_info['horizontal']
    vertical = deviation_info['vertical']
    distance = deviation_info['distance']
    angle = deviation_info['angle']
    in_center = deviation_info['in_center_region']
    
    # 偏差状态判断
    status = "CENTER" if in_center else "DEVIATION"
    
    # 打印偏差信息
    timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
    
    print(f"[{timestamp}] QR_ALIGNMENT: {status} "
          f"ΔX={horizontal:+.1f}px ΔY={vertical:+.1f}px "
          f"Dist={distance:.1f}px Angle={angle:.1f}° "
          f"Data={qr_info['data'][:20] if qr_info.get('data') else 'N/A'}")

def handle_keyboard_input(key, scanner):
    """处理键盘输入"""
    global paused, g_bExit
    
    if key == ord('q') or key == 27:
        g_bExit = True
    elif key == ord('s'):
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"alignment_snapshot_{timestamp}.jpg"
        # 注意：这里需要获取当前显示的图像
        print(f"截图功能需要额外实现")
    elif key == ord('d'):
        scanner.config['show_debug'] = not scanner.config['show_debug']
        print(f"调试模式: {'开启' if scanner.config['show_debug'] else '关闭'}")
    elif key == ord('p'):
        paused = not paused
        print(f"程序: {'暂停' if paused else '继续'}")
    elif key == ord('r'):
        scanner.qr_statistics = {
            'total_detected': 0,
            'successful_reads': 0,
            'failed_reads': 0,
            'last_detection_time': None
        }
        print("统计信息已重置")

# ==================== 6. 图像转换函数 ====================
def convert_to_image(pData, stFrameInfo):
    """将相机原始数据转换为OpenCV图像"""
    if pData is None:
        return None
    
    try:
        buffer_ptr = cast(pData, POINTER(c_ubyte * stFrameInfo.nFrameLen))
        buffer_bytes = bytearray(buffer_ptr.contents)
        image_data = np.frombuffer(buffer_bytes, dtype=np.uint8)
        
        if stFrameInfo.enPixelType == PixelType_Gvsp_Mono8:
            image = image_data.reshape(stFrameInfo.nHeight, stFrameInfo.nWidth)
            image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
        elif stFrameInfo.enPixelType == PixelType_Gvsp_BayerRG8:
            image = image_data.reshape(stFrameInfo.nHeight, stFrameInfo.nWidth)
            image = cv2.cvtColor(image, cv2.COLOR_BAYER_RG2BGR)
        elif stFrameInfo.enPixelType == PixelType_Gvsp_RGB8_Packed:
            image = image_data.reshape(stFrameInfo.nHeight, stFrameInfo.nWidth, 3)
            image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        elif stFrameInfo.enPixelType == PixelType_Gvsp_BGR8_Packed:
            image = image_data.reshape(stFrameInfo.nHeight, stFrameInfo.nWidth, 3)
        else:
            try:
                image = image_data.reshape(stFrameInfo.nHeight, stFrameInfo.nWidth, -1)
            except:
                return None
        
        return image
    except Exception as e:
        print(f"图像转换错误: {e}")
        return None

# ==================== 7. 主程序 ====================
def main():
    global g_bExit
    
    # 命令行参数
    parser = argparse.ArgumentParser(description='二维码对齐测量系统')
    parser.add_argument('--no-gui', action='store_true', help='无GUI模式')
    parser.add_argument('--debug', action='store_true', help='调试模式')
    parser.add_argument('--rotation', type=float, default=-90.0, help='旋转角度（逆时针）')
    parser.add_argument('--threshold', type=int, default=300, help='中心阈值尺寸')
    parser.add_argument('--interval', type=float, default=0.3, help='检测间隔(秒)')
    args = parser.parse_args()
    
    print("="*60)
    print("二维码对齐测量系统")
    print(f"系统: {platform.system()} {platform.release()}")
    print(f"架构: {platform.machine()}")
    print("="*60)
    
    # 初始化系统
    rotation_system = ImageRotationAndAlignment(
        rotation_angle=args.rotation,
        center_threshold=args.threshold
    )
    
    scanner_config = {
        'detection_interval': args.interval,
        'show_debug': args.debug,
        'save_detections': True,
    }
    
    scanner = HighPrecisionQRScanner(scanner_config)
    
    cam = None
    try:
        # 初始化SDK
        print("正在初始化海康SDK...")
        MvCamera.MV_CC_Initialize()

        SDKVersion = MvCamera.MV_CC_GetSDKVersion()
        print(f"SDK版本: 0x{SDKVersion:x}")
        
        # 枚举设备
        deviceList = MV_CC_DEVICE_INFO_LIST()
        tlayerType = MV_GIGE_DEVICE | MV_USB_DEVICE
        
        ret = MvCamera.MV_CC_EnumDevices(tlayerType, deviceList)
        if ret != 0:
            print(f"枚举设备失败! 错误码[0x{ret:x}]")
            sys.exit()

        if deviceList.nDeviceNum == 0:
            print("未找到任何设备!")
            sys.exit()

        print(f"找到 {deviceList.nDeviceNum} 个设备")
        print("-"*40)

        # 显示设备列表
        for i in range(deviceList.nDeviceNum):
            mvcc_dev_info = cast(deviceList.pDeviceInfo[i], POINTER(MV_CC_DEVICE_INFO)).contents
            if mvcc_dev_info.nTLayerType == MV_GIGE_DEVICE:
                print(f"[{i}] GigE 网络相机")
                strModeName = decoding_char(mvcc_dev_info.SpecialInfo.stGigEInfo.chModelName)
                print(f"   型号: {strModeName}")
                
                nip1 = ((mvcc_dev_info.SpecialInfo.stGigEInfo.nCurrentIp & 0xff000000) >> 24)
                nip2 = ((mvcc_dev_info.SpecialInfo.stGigEInfo.nCurrentIp & 0x00ff0000) >> 16)
                nip3 = ((mvcc_dev_info.SpecialInfo.stGigEInfo.nCurrentIp & 0x0000ff00) >> 8)
                nip4 = (mvcc_dev_info.SpecialInfo.stGigEInfo.nCurrentIp & 0x000000ff)
                print(f"   IP地址: {nip1}.{nip2}.{nip3}.{nip4}")
            elif mvcc_dev_info.nTLayerType == MV_USB_DEVICE:
                print(f"[{i}] USB 相机")
                strModeName = decoding_char(mvcc_dev_info.SpecialInfo.stUsb3VInfo.chModelName)
                strSerialNumber = decoding_char(mvcc_dev_info.SpecialInfo.stUsb3VInfo.chSerialNumber)
                print(f"   型号: {strModeName}")
                print(f"   序列号: {strSerialNumber}")
            print()

        # 选择设备
        try:
            nConnectionNum = input("请输入要连接的设备编号: ")
        except KeyboardInterrupt:
            print("\n用户中断")
            sys.exit(1)
        
        if int(nConnectionNum) >= deviceList.nDeviceNum:
            print("输入错误! 设备编号无效")
            sys.exit()

        # 创建相机实例
        cam = MvCamera()
        stDeviceList = cast(deviceList.pDeviceInfo[int(nConnectionNum)], POINTER(MV_CC_DEVICE_INFO)).contents

        ret = cam.MV_CC_CreateHandle(stDeviceList)
        if ret != 0:
            raise Exception(f"创建句柄失败! 错误码[0x{ret:x}]")

        ret = cam.MV_CC_OpenDevice(MV_ACCESS_Exclusive, 0)
        if ret != 0:
            raise Exception(f"打开设备失败! 错误码[0x{ret:x}]")
        
        if stDeviceList.nTLayerType == MV_GIGE_DEVICE:
            nPacketSize = cam.MV_CC_GetOptimalPacketSize()
            if int(nPacketSize) > 0:
                ret = cam.MV_CC_SetIntValue("GevSCPSPacketSize", nPacketSize)
        
        ret = cam.MV_CC_SetEnumValue("TriggerMode", MV_TRIGGER_MODE_OFF)
        if ret != 0:
            raise Exception(f"设置触发模式失败! 错误码[0x{ret:x}]")
        
        # 开始取流
        print("\n开始二维码对齐测量...")
        ret = cam.MV_CC_StartGrabbing()
        if ret != 0:
            raise Exception(f"开始取流失败! 错误码[0x{ret:x}]")

        # 启动工作线程
        try:
            hThreadHandle = threading.Thread(
                target=work_thread, 
                args=(cam, scanner, rotation_system, not args.no_gui)
            )
            hThreadHandle.daemon = True
            hThreadHandle.start()
        except Exception as e:
            print(f"启动线程失败: {e}")
            sys.exit(1)
        
        print("\n按 Enter 键停止程序...")
        input()

    except KeyboardInterrupt:
        print("\n程序被用户中断")
    except Exception as e:
        print(f"\n发生错误: {e}")
        import traceback
        traceback.print_exc()
    finally:
        g_bExit = True
        time.sleep(0.5)
        
        if cam is not None:
            try:
                cam.MV_CC_StopGrabbing()
                cam.MV_CC_CloseDevice()
                cam.MV_CC_DestroyHandle()
            except:
                pass
        
        MvCamera.MV_CC_Finalize()
        
        # 输出最终统计
        print("\n" + "="*60)
        print("对齐测量完成统计:")
        stats = scanner.qr_statistics
        print(f"  总检测数: {stats['total_detected']}")
        print(f"  成功读取: {stats['successful_reads']}")
        print(f"  失败读取: {stats['failed_reads']}")
        if stats['last_detection_time']:
            print(f"  最后检测: {stats['last_detection_time'][11:19]}")
        print("="*60)

def decoding_char(ctypes_char_array):
    """解码字符数组"""
    byte_str = memoryview(ctypes_char_array).tobytes()
    null_index = byte_str.find(b'\x00')
    if null_index != -1:
        byte_str = byte_str[:null_index]
    
    for encoding in ['utf-8', 'latin-1', 'gbk']:
        try:
            return byte_str.decode(encoding)
        except UnicodeDecodeError:
            continue
    
    return byte_str.decode('latin-1', errors='replace')

if __name__ == "__main__":
    main()