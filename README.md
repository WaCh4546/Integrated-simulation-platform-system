# Integrated-simulation-platform-system

本项目内容为设计一套飞行模拟仿真系统，在操作输入和飞控律控制下，驱动和控制一套飞机缩比模型在一定范围内模拟真实飞机的空中飞行，从而为空中加油测试提供平台支持。

![仿真平台搭建.png](https://github.com/WaCh4546/Integrated-simulation-platform-system/blob/main/%E4%BB%BF%E7%9C%9F%E5%B9%B3%E5%8F%B0%E6%90%AD%E5%BB%BA.png?raw=true)

整个系统由两台工业机器人（加油机、受油机）、上位机、双目视觉部分、第七轴等组成。该系统中，可以通过摇杆、油门杆等实现飞机操控或者通过双目视觉定位飞机受油孔进而控制自动加油。



飞机受油口、伞套的自动识别均采用yolov4网络

「加油伞套训练集」 https://www.aliyundrive.com/s/bsdg7AqxnaN 提取码: x0z6
「受油口训练集」 https://www.aliyundrive.com/s/gVt3xaTP67X 提取码: 58xu

「标注程序」https://www.aliyundrive.com/s/3fMTjtvo18Y 提取码: tk33
