# 环境

1. OpenCV
2. fmt  (格式化字符串) [fmt](https://fmt.dev/latest/index.html)
3. Eigen3 (矩阵) 
4. ceres (优化库，安装较麻烦，会遇到BUG)
5. MVSDK (相机驱动)


打算按模块编程和使用多线程
```bash
├── CMakeLists.txt
├── Configs
├── devices
├── main.cpp
├── modules
├── myThreads.h
├── README.md
├── test
└── utils
```

## main

因为经常 ctrl + c 打断代码,可能有的文件没保存,对于这种情况添加了信号处理,就是当 ctrl + c 后,会改变主循环的控制量,跳出循环,再进行一些类的析构函数.

## devices

### camera

迈德威视相机

### serial

串口

## modules

### detect_armour

传统视觉检测装甲版（阈值 + 找轮廓）

### kalman_filter

拓展卡尔曼滤波（参考上交开源）



## test 

一些功能的简单测试代码
