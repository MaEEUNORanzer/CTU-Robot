from ultralytics import YOLO
import cv2
import numpy as np

# 1. 加载你的模型
model = YOLO('best.pt')

def detect_qr_codes(image_path, conf_threshold=0.7, save_result=True):
    """
    检测图片中的二维码位置
    
    Args:
        image_path: 图片路径
        conf_threshold: 置信度阈值（默认0.5）
        save_result: 是否保存带标注的结果图
    """
    # 2. 进行预测
    results = model(image_path, conf=conf_threshold)
    
    # 3. 处理第一个结果（假设只有一张图片）
    result = results[0]
    
    # 4. 获取检测到的二维码数量
    num_qrcodes = len(result.boxes)
    print(f"检测到 {num_qrcodes} 个二维码")
    
    # 5. 获取详细信息
    qr_positions = []
    for i, box in enumerate(result.boxes):
        # 获取坐标 [x1, y1, x2, y2]
        x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
        
        # 获取置信度
        confidence = box.conf[0].item()
        
        # 获取类别
        class_id = int(box.cls[0].item())
        class_name = result.names[class_id]
        
        qr_positions.append({
            'id': i + 1,
            'position': [x1, y1, x2, y2],
            'confidence': round(confidence, 3),
            'class': class_name
        })
        
        print(f"二维码{i+1}: 位置({x1}, {y1}, {x2}, {y2}), 置信度{confidence:.3f}")
    
    # 6. 可视化结果（自动保存）
    if save_result and num_qrcodes > 0:
        # 使用Ultralytics内置的绘图功能
        plotted_img = result.plot()  # 自动绘制框和标签
        output_path = image_path.replace('.jpg', '_detected.jpg').replace('.png', '_detected.png')
        cv2.imwrite(output_path, plotted_img)
        print(f"结果已保存到: {output_path}")
    
    return qr_positions

# 使用示例
if __name__ == "__main__":
    # 检测单张图片
    image_path = "test.jpg"  # 替换为你的图片路径
    qr_boxes = detect_qr_codes(image_path)
