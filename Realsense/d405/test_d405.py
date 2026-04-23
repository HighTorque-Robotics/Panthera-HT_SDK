#!/usr/bin/env python3
"""
Intel RealSense D405相机RGB和深度图显示例程
参考D435例程编写
"""
import pyrealsense2 as rs
import numpy as np
import cv2

# 配置参数
WIDTH, HEIGHT = 640, 480
FPS = 30

# 深度范围（毫米）- D405是近距离相机，范围更小
MIN_DEPTH_MM = 70    # D405最小测量距离约70mm
MAX_DEPTH_MM = 1000  # 设置为1米，适合桌面近距离应用

def main():
    # 创建管道
    pipeline = rs.pipeline()
    config = rs.config()

    # 启用数据流
    config.enable_stream(rs.stream.depth, WIDTH, HEIGHT, rs.format.z16, FPS)
    config.enable_stream(rs.stream.color, WIDTH, HEIGHT, rs.format.bgr8, FPS)

    # 启动流
    print("启动RealSense D405相机...")
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

            if not depth_frame or not color_frame:
                continue

            # 转换为numpy数组
            depth_image = np.asanyarray(depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())

            # 处理深度图像 - 创建彩色深度图
            depth_clipped = np.clip(depth_image, MIN_DEPTH_MM, MAX_DEPTH_MM)
            depth_normalized = ((depth_clipped - MIN_DEPTH_MM) * 255.0 /
                              (MAX_DEPTH_MM - MIN_DEPTH_MM)).astype(np.uint8)
            depth_colormap = cv2.applyColorMap(depth_normalized, cv2.COLORMAP_JET)

            # 在图像上添加标签
            cv2.putText(color_image, 'RGB', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            cv2.putText(depth_colormap, 'Depth', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

            # 获取中心点深度值并显示
            center_x, center_y = WIDTH // 2, HEIGHT // 2
            depth_value = depth_frame.get_distance(center_x, center_y)
            cv2.circle(color_image, (center_x, center_y), 5, (0, 255, 255), -1)
            cv2.putText(color_image, f'{depth_value:.3f}m', (center_x + 10, center_y),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

            # 水平拼接图像
            combined_image = np.hstack((color_image, depth_colormap))

            # 显示拼接后的图像
            cv2.imshow('RealSense D405 - RGB & Depth', combined_image)

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
