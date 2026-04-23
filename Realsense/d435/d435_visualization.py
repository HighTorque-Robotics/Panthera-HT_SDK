#!/usr/bin/env python3
import pyrealsense2 as rs
import numpy as np
import cv2

# 配置参数
WIDTH, HEIGHT = 640, 480
FPS = 30

# 深度范围（毫米）
MIN_DEPTH_MM = 100
MAX_DEPTH_MM = 3000

def main():
    # 创建管道
    pipeline = rs.pipeline()
    config = rs.config()
    
    # 启用数据流
    config.enable_stream(rs.stream.depth, WIDTH, HEIGHT, rs.format.z16, FPS)
    config.enable_stream(rs.stream.color, WIDTH, HEIGHT, rs.format.bgr8, FPS)
    config.enable_stream(rs.stream.infrared, 1, WIDTH, HEIGHT, rs.format.y8, FPS)  # 左红外
    config.enable_stream(rs.stream.infrared, 2, WIDTH, HEIGHT, rs.format.y8, FPS)  # 右红外
    
    # 启动流
    print("启动RealSense D435摄像头...")
    pipeline.start(config)
    
    # 创建对齐对象（将深度图对齐到彩色图）
    align_to = rs.stream.color
    align = rs.align(align_to)
    
    print("开始可视化 - 按 'q' 退出")
    
    try:
        while True:
            # 等待获取帧
            frames = pipeline.wait_for_frames()
            
            # 对齐深度帧到彩色帧
            aligned_frames = align.process(frames)
            
            # 获取各种帧
            depth_frame = aligned_frames.get_depth_frame()
            color_frame = aligned_frames.get_color_frame()
            ir_frame_left = frames.get_infrared_frame(1)  # 左红外
            ir_frame_right = frames.get_infrared_frame(2)  # 右红外
            
            if not depth_frame or not color_frame or not ir_frame_left or not ir_frame_right:
                continue
            
            # 转换为numpy数组
            depth_image = np.asanyarray(depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())
            ir_image_left = np.asanyarray(ir_frame_left.get_data())
            ir_image_right = np.asanyarray(ir_frame_right.get_data())
            
            # 处理深度图像 - 创建彩色深度图
            depth_clipped = np.clip(depth_image, MIN_DEPTH_MM, MAX_DEPTH_MM)
            depth_normalized = ((depth_clipped - MIN_DEPTH_MM) * 255.0 / 
                              (MAX_DEPTH_MM - MIN_DEPTH_MM)).astype(np.uint8)
            depth_colormap = cv2.applyColorMap(depth_normalized, cv2.COLORMAP_JET)
            
            # 将红外图像转换为3通道以便拼接
            ir_left_3ch = cv2.cvtColor(ir_image_left, cv2.COLOR_GRAY2BGR)
            ir_right_3ch = cv2.cvtColor(ir_image_right, cv2.COLOR_GRAY2BGR)
            
            # 在图像上添加标签
            cv2.putText(color_image, 'RGB', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            cv2.putText(depth_colormap, 'Depth', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            cv2.putText(ir_left_3ch, 'IR Left', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            cv2.putText(ir_right_3ch, 'IR Right', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            
            # 创建拼接图像
            top_row = np.hstack((color_image, depth_colormap))
            bottom_row = np.hstack((ir_left_3ch, ir_right_3ch))
            combined_image = np.vstack((top_row, bottom_row))
            
            # 显示拼接后的图像
            cv2.imshow('RealSense D435 - RGB, Depth, IR Left, IR Right', combined_image)
            
            # 也可以单独显示每个图像
            # cv2.imshow('RGB Image', color_image)
            # cv2.imshow('Depth Image', depth_colormap)
            # cv2.imshow('IR Left', ir_image_left)
            # cv2.imshow('IR Right', ir_image_right)
            
            # 按'q'退出
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
                
    except KeyboardInterrupt:
        print("\n程序被中断")
        
    finally:
        # 清理资源
        pipeline.stop()
        cv2.destroyAllWindows()
        print("程序结束")

if __name__ == "__main__":
    main()