# 高擎电机控制SDK
高擎机器人电机控制SDK，提供C++控制接口，用于控制高擎关节电机。

## 一、介绍

本项目是一个用于控制高擎机器人电机的SDK，通过USB虚拟串口完成到CANFD协议的通信转发板与电机通信。本SDK提供了完整的C++库，可以方便地集成到各种机器人控制系统中实现电机控制。

### 主要特性

* 支持多种电机型号（4438、5046、5047、6056等系列）；
* 多种控制模式：位置控制、速度控制、力矩控制、混合控制等；
* 支持多个CAN总线板卡和多个电机，目前硬件支持7路CAN总线，每条总线可以挂30个电机；
* 提供LCM（Lightweight Communications and Marshalling）消息发布；
* 实时电机状态反馈；

## 二、安装依赖

### 一键安装
在 install_scripts 目录下运行
```
chmod 777 Interface_cpp_setup.sh
./Interface_cpp_setup.sh
```
若不使用一键安装，则按下面步骤手动进行

### 依赖库

* CMake >= 3.0.2
* C++11 编译器
* libserialport（串口通信）
* yaml-cpp（YAML配置文件解析）
* lcm
* serial_cmake


1. 安装串口依赖
```
sudo apt-get install libserialport-dev
```
2. 安装yaml解析器
```
sudo apt-get install libyaml-cpp-dev
```

## 三、编译

1. 获取代码
```
git clone http://git.clicki.cn/livelybot/hightorque_robot.git
```

2. 编译
```
cd hightorque_robot
mkdir build
cd build
cmake ..
make -j8
```

3. 安装
```
sudo make install
```

4. 补充
当外部项目调用当前库的时候，可能会出现找不到liblivelybot_serial.so.*的情况，这个时候需要申明安装位置
在当前命令行执行：
```
export LD_LIBRARY_PATH="/usr/local/lib:$LD_LIBRARY_PATH"
```
或者在`~/.bashrc`中添加如下行
```
......
export LD_LIBRARY_PATH="/usr/local/lib:$LD_LIBRARY_PATH"
```
并执行`source ~/.bashrc`使修改生效。