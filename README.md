打算按模块编程和使用多线程
```bash
├── build			-- build
├── CMakeLists.txt	-- CMake
├── configs			-- 配置文件存放处
├── devices			-- 处理设备的代码，如相机、串口
├── main.cpp		-- 主文件入口
├── README.md		-- README.md
└── utils			-- 存放一些工具类
```

## main

因为经常 ctrl + c 打断代码,可能有的文件没保存,对于这种情况添加了信号处理,就是当 ctrl + c 后,会改变主循环的控制量,跳出循环,再进行一些类的析构函数.



## utils

1. record -- 录制
2. FPS -- 计算FPS，显示到图片上
3. logger -- 日志输出类
4. config -- 配置参数类, 打算删除