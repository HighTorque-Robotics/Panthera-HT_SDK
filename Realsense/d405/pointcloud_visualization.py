#!/usr/bin/env python3
import pyrealsense2 as rs
import numpy as np
import cv2
import open3d as o3d
from threading import Thread
import time

class PointCloudVisualizer:
    def __init__(self):
        # 配置参数
        self.WIDTH, self.HEIGHT = 640, 480
        self.FPS = 30
        
        # 深度范围（米）
        self.MIN_DEPTH = 0.1
        self.MAX_DEPTH = 3.0
        
        # RealSense配置
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        
        # 启用数据流
        self.config.enable_stream(rs.stream.depth, self.WIDTH, self.HEIGHT, rs.format.z16, self.FPS)
        self.config.enable_stream(rs.stream.color, self.WIDTH, self.HEIGHT, rs.format.bgr8, self.FPS)
        
        # 创建对齐对象（将深度图对齐到彩色图）
        self.align_to = rs.stream.color
        self.align = rs.align(self.align_to)
        
        # Open3D可视化器
        self.vis = o3d.visualization.Visualizer()
        self.point_cloud = o3d.geometry.PointCloud()
        self.first_frame = True
        
    def start_camera(self):
        """启动摄像头"""
        print("启动RealSense D435摄像头...")
        profile = self.pipeline.start(self.config)
        
        # 获取相机内参
        color_profile = rs.video_stream_profile(profile.get_stream(rs.stream.color))
        self.intrinsic = color_profile.get_intrinsics()
        print(f"相机内参: fx={self.intrinsic.fx}, fy={self.intrinsic.fy}, cx={self.intrinsic.ppx}, cy={self.intrinsic.ppy}")
        
    def setup_visualizer(self):
        """设置Open3D可视化器"""
        self.vis.create_window("RealSense D435 Point Cloud", width=800, height=600)
        self.vis.add_geometry(self.point_cloud)
        
        # 设置渲染选项
        render_option = self.vis.get_render_option()
        render_option.background_color = [0.05, 0.05, 0.05]
        render_option.point_size = 1.0
        
    def depth_to_pointcloud(self, depth_image, color_image):
        """将深度图和彩色图转换为点云"""
        # 获取图像尺寸
        height, width = depth_image.shape
        
        # 创建网格坐标
        u, v = np.meshgrid(np.arange(width), np.arange(height))
        
        # 获取有效深度点（非零且在范围内）
        depth_mm = depth_image.astype(np.float32)
        depth_m = depth_mm / 1000.0  # 转换为米
        
        # 过滤深度范围
        valid_mask = (depth_m > self.MIN_DEPTH) & (depth_m < self.MAX_DEPTH) & (depth_m > 0)
        
        # 获取有效的像素坐标和深度值
        u_valid = u[valid_mask]
        v_valid = v[valid_mask]
        z_valid = depth_m[valid_mask]
        
        # 使用相机内参将像素坐标转换为3D坐标
        x = (u_valid - self.intrinsic.ppx) * z_valid / self.intrinsic.fx
        y = (v_valid - self.intrinsic.ppy) * z_valid / self.intrinsic.fy
        
        # 组合3D点
        points_3d = np.column_stack((x, y, z_valid))
        
        # 获取对应的颜色信息
        colors = color_image[valid_mask]
        colors_normalized = colors.astype(np.float32) / 255.0
        # OpenCV使用BGR，需要转换为RGB
        colors_rgb = colors_normalized[:, [2, 1, 0]]
        
        return points_3d, colors_rgb
        
    def update_pointcloud(self, points_3d, colors):
        """更新点云数据"""
        self.point_cloud.points = o3d.utility.Vector3dVector(points_3d)
        self.point_cloud.colors = o3d.utility.Vector3dVector(colors)
        
        if self.first_frame:
            self.vis.add_geometry(self.point_cloud)
            self.first_frame = False
        else:
            self.vis.update_geometry(self.point_cloud)
            
    def run(self):
        """主运行循环"""
        self.start_camera()
        self.setup_visualizer()
        
        print("开始点云可视化 - 关闭窗口退出程序")
        print("控制说明:")
        print("  - 鼠标左键拖拽: 旋转视角")
        print("  - 鼠标右键拖拽: 平移视角") 
        print("  - 鼠标滚轮: 缩放")
        
        frame_count = 0
        start_time = time.time()
        
        try:
            while self.vis.poll_events():
                # 获取帧数据
                frames = self.pipeline.wait_for_frames()
                
                # 对齐深度帧到彩色帧
                aligned_frames = self.align.process(frames)
                
                depth_frame = aligned_frames.get_depth_frame()
                color_frame = aligned_frames.get_color_frame()
                
                if not depth_frame or not color_frame:
                    continue
                
                # 转换为numpy数组
                depth_image = np.asanyarray(depth_frame.get_data())
                color_image = np.asanyarray(color_frame.get_data())
                
                # 生成点云
                points_3d, colors = self.depth_to_pointcloud(depth_image, color_image)
                
                if len(points_3d) > 0:
                    # 更新点云
                    self.update_pointcloud(points_3d, colors)
                    
                    # 更新可视化器
                    self.vis.update_renderer()
                    
                    # 显示RGB和深度图（可选）
                    self.show_2d_images(color_image, depth_image)
                    
                # 计算并显示FPS
                frame_count += 1
                if frame_count % 30 == 0:
                    elapsed = time.time() - start_time
                    fps = 30 / elapsed
                    print(f"点云FPS: {fps:.1f}, 点数: {len(points_3d)}")
                    start_time = time.time()
                
                # 检查退出条件
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
                    
        except KeyboardInterrupt:
            print("\n程序被中断")
            
        finally:
            self.cleanup()
            
    def show_2d_images(self, color_image, depth_image):
        """显示2D图像（RGB和深度图）"""
        # 创建深度图的彩色版本用于显示
        depth_colormap = cv2.applyColorMap(
            cv2.convertScaleAbs(depth_image, alpha=0.03), 
            cv2.COLORMAP_JET
        )
        
        # 调整图像大小以便并排显示
        height = 240
        scale = height / color_image.shape[0]
        width = int(color_image.shape[1] * scale)
        
        color_resized = cv2.resize(color_image, (width, height))
        depth_resized = cv2.resize(depth_colormap, (width, height))
        
        # 并排显示
        combined = np.hstack((color_resized, depth_resized))
        
        # 添加标签
        cv2.putText(combined, 'RGB', (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(combined, 'Depth', (width + 10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        cv2.imshow('RGB & Depth', combined)
        
    def cleanup(self):
        """清理资源"""
        self.pipeline.stop()
        cv2.destroyAllWindows()
        self.vis.destroy_window()
        print("程序结束")

def main():
    """简化版本 - 不使用Open3D，仅显示对齐的RGB和深度图"""
    print("RealSense D435 RGB-Depth 对齐可视化程序")
    print("(如果需要3D点云，请安装 open3d: pip install open3d)")
    
    # 配置参数
    WIDTH, HEIGHT = 640, 480
    FPS = 30
    
    # RealSense配置
    pipeline = rs.pipeline()
    config = rs.config()
    
    config.enable_stream(rs.stream.depth, WIDTH, HEIGHT, rs.format.z16, FPS)
    config.enable_stream(rs.stream.color, WIDTH, HEIGHT, rs.format.bgr8, FPS)
    
    # 启动流
    print("启动摄像头...")
    pipeline.start(config)
    
    # 创建对齐对象
    align = rs.align(rs.stream.color)
    
    print("开始显示对齐的RGB和深度图 - 按 'q' 退出")
    
    try:
        while True:
            frames = pipeline.wait_for_frames()
            aligned_frames = align.process(frames)
            
            depth_frame = aligned_frames.get_depth_frame()
            color_frame = aligned_frames.get_color_frame()
            
            if not depth_frame or not color_frame:
                continue
                
            depth_image = np.asanyarray(depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())
            
            # 创建深度图的彩色版本
            depth_colormap = cv2.applyColorMap(
                cv2.convertScaleAbs(depth_image, alpha=0.03),
                cv2.COLORMAP_JET
            )
            
            # 并排显示
            combined = np.hstack((color_image, depth_colormap))
            cv2.putText(combined, 'RGB (Aligned)', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            cv2.putText(combined, 'Depth (Aligned)', (WIDTH + 10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            
            cv2.imshow('RGB-Depth Aligned', combined)
            
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
                
    except KeyboardInterrupt:
        print("\n程序被中断")
        
    finally:
        pipeline.stop()
        cv2.destroyAllWindows()
        print("程序结束")

if __name__ == "__main__":
    # 检查是否安装了Open3D
    try:
        import open3d as o3d
        print("检测到Open3D，启动完整点云可视化...")
        visualizer = PointCloudVisualizer()
        visualizer.run()
    except ImportError:
        print("未找到Open3D库，运行简化版本...")
        main()