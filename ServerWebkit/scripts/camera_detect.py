#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
二维码检测TCP服务器 (ARM Linux优化版)
适用于ARM架构的Linux系统
"""

import sys
import os
import cv2
import numpy as np
import time
import socket
import signal
import threading
from pyzbar.pyzbar import decode, ZBarSymbol

# ================== ARM Linux优化配置 ==================
# 注意：以下中心点坐标是基于旋转90度后的坐标系
IMAGE_CENTER_X = 512  # 旋转后图像水平中心点
IMAGE_CENTER_Y = 640  # 旋转后图像垂直中心点
CENTER_ALIGN_THRESHOLD = 100  # 中心对齐阈值(像素)
PIXELS_PER_CM = 12.8  # 12.8像素代表1厘米
# =======================================================

class ARMVideoCapture:
    """ARM Linux优化的视频采集类，兼容多种摄像头"""
    def __init__(self, camera_index=0, rotate_90_degrees=True, resolution=(640, 480)):
        self.camera_index = camera_index
        self.rotate_90_degrees = rotate_90_degrees
        self.resolution = resolution
        self.cap = None
        self.is_streaming = False
        self.rotated_width = 0
        self.rotated_height = 0
        
    def connect_camera(self):
        """连接相机 (ARM Linux兼容版本)"""
        try:
            # 尝试不同的后端
            backends = [
                cv2.CAP_V4L2,    # V4L2 (Linux)
                cv2.CAP_ANY,     # 自动选择
            ]
            
            for backend in backends:
                try:
                    self.cap = cv2.VideoCapture(self.camera_index, backend)
                    if self.cap.isOpened():
                        print(f"[INFO] 使用后端 {backend} 成功打开摄像头")
                        break
                except:
                    continue
            
            if not self.cap or not self.cap.isOpened():
                print("[ERROR] 无法打开摄像头")
                return False
            
            # 设置分辨率
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.resolution[0])
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.resolution[1])
            
            # 设置帧率
            self.cap.set(cv2.CAP_PROP_FPS, 30)
            
            # 获取实际分辨率
            actual_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            actual_height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            
            # 计算旋转后的尺寸
            if self.rotate_90_degrees:
                self.rotated_width = actual_height
                self.rotated_height = actual_width
            else:
                self.rotated_width = actual_width
                self.rotated_height = actual_height
            
            print(f"[INFO] 相机分辨率: {actual_width}x{actual_height}")
            print(f"[INFO] 旋转后分辨率: {self.rotated_width}x{self.rotated_height}")
            
            self.is_streaming = True
            return True
            
        except Exception as e:
            print(f"[ERROR] 连接相机失败: {e}")
            return False
    
    def get_frame(self):
        """获取一帧图像并自动旋转90度"""
        if not self.is_streaming or not self.cap:
            return None
        
        try:
            ret, frame = self.cap.read()
            if not ret or frame is None:
                return None
            
            # 自动旋转90度
            if self.rotate_90_degrees:
                frame = cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
            
            return frame
            
        except Exception as e:
            print(f"[WARN] 获取帧失败: {e}")
            return None
    
    def clear_frame_buffer(self):
        """清空帧缓冲区，确保下次获取的是最新帧"""
        if not self.is_streaming or not self.cap:
            return
        
        # 尝试清空几帧旧数据
        for _ in range(3):
            ret, _ = self.cap.read()
            if not ret:
                break
    
    def stop_streaming(self):
        """停止采集"""
        if self.cap:
            self.cap.release()
        self.is_streaming = False
        print("[INFO] 相机已停止")
    
    def get_image_info(self):
        """获取图像信息"""
        return {
            'rotated_width': self.rotated_width,
            'rotated_height': self.rotated_height,
            'is_rotated': self.rotate_90_degrees
        }


class ARMQRCodeDetector:
    """ARM优化的二维码检测器"""
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
        """ARM优化的图像预处理"""
        if image is None:
            return image
        
        try:
            # 转换为灰度图
            if len(image.shape) == 3:
                gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            else:
                gray = image
            
            # ARM优化：简化预处理步骤以提高性能
            # 1. 高斯滤波去噪
            gray = cv2.GaussianBlur(gray, (3, 3), 0)
            
            # 2. 直方图均衡化增强对比度
            gray = cv2.equalizeHist(gray)
            
            return gray
            
        except Exception as e:
            print(f"[WARN] 图像预处理失败: {e}")
            return image
    
    def detect_qrcodes(self, image):
        """ARM优化的二维码检测"""
        if image is None:
            return []
        
        all_detections = []
        
        try:
            # 转换为灰度图
            if len(image.shape) == 3:
                gray_original = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            else:
                gray_original = image
            
            # ARM优化：简化图像版本以提高性能
            image_versions = [gray_original]
            
            if self.enable_preprocessing:
                gray_preprocessed = self.preprocess_image(image)
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
            
        except Exception as e:
            print(f"[WARN] 二维码检测失败: {e}")
            return []
    
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
                
                # 计算水平偏移量
                horizontal_offset = qr_center_x - self.center_x
                vertical_offset = qr_center_y - self.center_y
                
                # 计算实际偏移距离（厘米）
                horizontal_cm = horizontal_offset / self.pixels_per_cm
                vertical_cm = vertical_offset / self.pixels_per_cm
                
                # 判断中心是否对齐
                is_aligned = abs(horizontal_offset) <= self.align_threshold
                
                # 判断方向
                direction = "右" if horizontal_offset > 0 else "左" if horizontal_offset < 0 else "居中"
                
                detection = {
                    'center_x': qr_center_x,
                    'center_y': qr_center_y,
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
                
                # 计算水平偏移量
                horizontal_offset = qr_center_x - self.center_x
                vertical_offset = qr_center_y - self.center_y
                
                # 计算实际偏移距离（厘米）
                horizontal_cm = horizontal_offset / self.pixels_per_cm
                vertical_cm = vertical_offset / self.pixels_per_cm
                
                # 判断中心是否对齐
                is_aligned = abs(horizontal_offset) <= self.align_threshold
                
                # 判断方向
                direction = "右" if horizontal_offset > 0 else "左" if horizontal_offset < 0 else "居中"
                
                detection = {
                    'center_x': qr_center_x,
                    'center_y': qr_center_y,
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
    
    def set_alignment_threshold(self, threshold):
        """设置对齐阈值"""
        self.align_threshold = max(1, threshold)
    
    def set_pixels_per_cm(self, pixels_per_cm):
        """设置每厘米像素数"""
        self.pixels_per_cm = max(0.1, pixels_per_cm)


class ARMQRTcpServer:
    """ARM Linux优化的二维码检测TCP服务器"""
    def __init__(self, host='0.0.0.0', port=8888, 
                 center_x=IMAGE_CENTER_X, center_y=IMAGE_CENTER_Y,
                 threshold=CENTER_ALIGN_THRESHOLD, pixels_per_cm=PIXELS_PER_CM,
                 camera_index=0):
        self.host = host
        self.port = port
        self.camera_index = camera_index
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
        self.required_center_count = 5
        self.last_scan_time = 0
        self.scan_interval = 0.1  # 0.1秒扫描间隔
        
    def start(self):
        """启动TCP服务器"""
        try:
            # 设置信号处理
            signal.signal(signal.SIGINT, self.signal_handler)
            signal.signal(signal.SIGTERM, self.signal_handler)
            
            # 创建TCP套接字
            self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.server_socket.bind((self.host, self.port))
            self.server_socket.listen(1)
            self.server_socket.settimeout(1.0)
            
            self.is_running = True
            print(f"[INFO] TCP服务器启动在 {self.host}:{self.port}")
            print(f"[INFO] 等待客户端连接...")
            
            # 初始化相机
            if not self.initialize_camera():
                print("[ERROR] 相机初始化失败，服务器停止")
                return
            
            # 启动主循环
            self.main_loop()
            
        except Exception as e:
            print(f"[ERROR] 启动TCP服务器失败: {e}")
        finally:
            self.cleanup()
    
    def signal_handler(self, signum, frame):
        """信号处理器"""
        print(f"\n[INFO] 收到信号 {signum}，正在关闭服务器...")
        self.is_running = False
    
    def initialize_camera(self):
        """初始化相机和检测器"""
        try:
            # 创建相机对象
            self.camera = ARMVideoCapture(
                camera_index=self.camera_index,
                rotate_90_degrees=True,
                resolution=(640, 480)
            )
            
            if not self.camera.connect_camera():
                print("[ERROR] 无法连接相机")
                return False
            
            # 初始化检测器
            self.detector = ARMQRCodeDetector(
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
            self.center_align_count = 0
            
            print(f"[INFO] 客户端已连接: {client_address}")
            self.send_to_client("READY: 二维码检测服务器已就绪")
            self.send_to_client("COMMANDS: SCAN, STOP, STATUS, EXIT")
            
        except socket.timeout:
            pass
        except Exception as e:
            print(f"[ERROR] 接受客户端连接失败: {e}")
    
    def handle_client(self):
        """处理客户端消息"""
        try:
            # 设置非阻塞接收
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
            
            # 如果正在扫描，执行扫描任务
            if self.scanning:
                current_time = time.time()
                if current_time - self.last_scan_time >= self.scan_interval:
                    self.last_scan_time = current_time
                    self.scanning_task()
                
        except Exception as e:
            print(f"[ERROR] 处理客户端消息失败: {e}")
            self.disconnect_client()
    
    def process_command(self, command):
        """处理客户端命令"""
        cmd = command.upper()
        
        if cmd == "SCAN":
            if not self.scanning:
                self.scanning = True
                self.center_align_count = 0
                self.last_scan_time = 0
                print(f"[INFO] 开始扫描任务")
                self.send_to_client("START: 扫描任务开始 (0.1秒间隔)")
        elif cmd == "STOP":
            if self.scanning:
                self.scanning = False
                print(f"[INFO] 停止扫描任务")
                self.send_to_client("STOP: 扫描任务已停止")
        elif cmd == "STATUS":
            status = "SCANNING" if self.scanning else "IDLE"
            align_status = f"{self.center_align_count}/{self.required_center_count}"
            self.send_to_client(f"STATUS: {status}, 对齐次数: {align_status}")
        elif cmd == "EXIT":
            self.send_to_client("BYE: 服务器关闭")
            self.is_running = False
        else:
            self.send_to_client(f"ERROR: 未知命令: {command}")
    
    def scanning_task(self):
        """执行扫描任务"""
        try:
            # 清空帧缓冲区
            self.camera.clear_frame_buffer()
            
            # 获取最新帧
            image = self.camera.get_frame()
            
            if image is None:
                print("[WARN] 获取图像失败")
                return
            
            # 检测二维码
            detection_start = time.time()
            detections = self.detector.detect_qrcodes(image)
            detection_time = (time.time() - detection_start) * 1000
            
            timestamp = time.strftime("%H:%M:%S")
            
            if detections:
                # 获取第一个检测结果
                detection = detections[0]
                offset = detection['horizontal_offset']
                offset_cm = detection['horizontal_cm']
                is_aligned = detection['is_aligned']
                direction = detection['direction']
                
                # 输出检测信息
                align_status = "✓" if is_aligned else "✗"
                print(f"[{timestamp}] 检测到二维码 | 偏移: {offset:+4d}px ({offset_cm:+6.2f}cm) {direction} | 对齐: {align_status} | 时间: {detection_time:4.0f}ms")
                
                # 发送到客户端
                self.send_to_client(f"QR: 偏移={offset:+d}px ({offset_cm:+.2f}cm) {direction}")
                
                # 处理对齐状态
                if is_aligned:
                    self.center_align_count += 1
                    print(f"[INFO] 中心对齐 {self.center_align_count}/{self.required_center_count}")
                    
                    if self.center_align_count >= self.required_center_count:
                        print(f"[SUCCESS] 扫描任务完成！连续对齐{self.required_center_count}次")
                        self.send_to_client(f"SUCCESS: 扫描任务完成，连续对齐{self.required_center_count}次")
                        self.scanning = False
                        self.center_align_count = 0
                else:
                    # 重置中心对齐计数
                    self.center_align_count = 0
                    
                    # 判断偏离方向
                    if offset > 0:
                        adjust_msg = "向右调整"
                    else:
                        adjust_msg = "向左调整"
                    
                    print(f"[ADJUST] 偏离中心区域，正在命令AGV调整位置 ({adjust_msg})")
                    self.send_to_client(f"ADJUST: 需要{adjust_msg}")
                    
                    # 等待3秒
                    time.sleep(3)
                    
            else:
                # 重置中心对齐计数
                self.center_align_count = 0
                
                # 输出未检测到信息
                print(f"[{timestamp}] 未检测到二维码 | 时间: {detection_time:4.0f}ms")
            
        except Exception as e:
            print(f"[ERROR] 扫描任务异常: {e}")
    
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
    
    def cleanup(self):
        """清理资源"""
        self.is_running = False
        self.scanning = False
        
        if self.camera:
            self.camera.stop_streaming()
        
        self.disconnect_client()
        
        if self.server_socket:
            try:
                self.server_socket.close()
            except:
                pass
        
        print("[INFO] 服务器已停止，资源已清理")


def main():
    """主函数"""
    import argparse
    
    parser = argparse.ArgumentParser(description='ARM二维码检测TCP服务器')
    parser.add_argument('--host', type=str, default='0.0.0.0', 
                       help='服务器主机地址 (默认: 0.0.0.0)')
    parser.add_argument('--port', type=int, default=8888, 
                       help='服务器端口 (默认: 8888)')
    parser.add_argument('--camera', type=int, default=0, 
                       help='相机索引 (默认: 0)')
    parser.add_argument('--center-x', type=int, default=IMAGE_CENTER_X, 
                       help=f'图像水平中心点 (默认: {IMAGE_CENTER_X})')
    parser.add_argument('--center-y', type=int, default=IMAGE_CENTER_Y,
                       help=f'图像垂直中心点 (默认: {IMAGE_CENTER_Y})')
    parser.add_argument('--threshold', type=int, default=CENTER_ALIGN_THRESHOLD,
                       help=f'中心对齐阈值(像素) (默认: {CENTER_ALIGN_THRESHOLD})')
    parser.add_argument('--pixels-per-cm', type=float, default=PIXELS_PER_CM,
                       help=f'每厘米像素数 (默认: {PIXELS_PER_CM})')
    
    args = parser.parse_args()
    
    print("=" * 60)
    print("ARM Linux 二维码检测TCP服务器")
    print("=" * 60)
    print(f"服务器地址: {args.host}:{args.port}")
    print(f"相机索引: {args.camera}")
    print(f"图像中心点: ({args.center_x}, {args.center_y})")
    print(f"对齐阈值: {args.threshold} 像素")
    print(f"像素转换: {args.pixels_per_cm} 像素/厘米")
    print(f"连续对齐: {5} 次")
    print(f"扫描间隔: 0.1 秒")
    print("=" * 60)
    
    # 创建并启动服务器
    server = ARMQRTcpServer(
        host=args.host, 
        port=args.port,
        camera_index=args.camera,
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
    
    print("[INFO] 程序已退出")


if __name__ == "__main__":
    main()
