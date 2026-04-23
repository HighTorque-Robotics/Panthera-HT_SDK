# Intel RealSense D405 测试程序

这是一个简单的Python程序，用于测试Intel RealSense D405相机并显示RGB和深度图像。

## 依赖安装

```bash
pip install -r requirements.txt
```

## 运行程序

```bash
python test_d405.py
```

或者：

```bash
chmod +x test_d405.py
./test_d405.py
```

## 功能特性

- 同时显示RGB图像和深度图像（水平拼接）
- 深度图使用JET彩色映射进行可视化
- 在RGB图像上显示中心点的深度值
- 深度帧和RGB帧对齐，确保像素对应

## 操作说明

- 程序启动后会打开一个窗口显示RGB和深度图像
- 按 `q` 键退出程序

## 注意事项

- 确保D405相机已正确连接到电脑
- 确保已安装Intel RealSense SDK
- 如果遇到权限问题，可能需要添加udev规则
