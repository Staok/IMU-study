# IMU-study

本项目是“瞰百易”计划的一部分

![0](README.assets/0.jpg)

对常见IMU芯片的原理、驱动和数据融合算法整理，以区分某度、某坛上面碎片化严重到影响入坑的乱象。

仓库内容比较大 约 400MB。

注：角加速度、角速度的融合算法中，卡尔曼滤波（参考网上）、类一阶滤波（一阶滤波算法改的）和清华滤波（本科智能车竞赛常用的）这三种算法的具体 C 语言实现的代码，已经写在了我的另一个项目 MCU Framework 的 IMU Device 中，[链接](https://github.com/Staok/stm32_framework/tree/master/STM32F4DSP_HAL_freeRTOS_Framework/DEVICES/IMU)。

目前目录的树形图（欢迎提交 PR 补充）：

```
├─0 加速度计、陀螺仪介绍
├─1 IMU的惯导、数据融合文档
│  ├─卡尔曼滤波
│  ├─四元数-欧拉角
│  └─数据融合、姿态解算
├─2 MPU系列传感器资料
│  ├─MPU-6050中文资料、测试程序和硬件资料
│  ├─MPU-6500官网资料、测试程序和硬件资料
│  ├─MPU-9250官网资料、测试程序和硬件资料
│  ├─MPU-9255官网资料、测试程序和硬件资料
│  └─MPU_DMP官网库和说明
├─3 其他各种IMU
│  ├─ADXL345 三轴加速度计
│  ├─ENC03 模拟 单轴陀螺仪
│  ├─L3G4200 三轴陀螺仪
│  ├─MXC400xXC 廉价 三轴加速度计
│  ├─ST的
│  ├─TDK-InvenSense Motion Sensor
│  ├─【BOSCH 博世】
│  ├─【MMA系列】
│  └─【TDK InvenSense】
├─4 电子罗盘 磁力计
│  ├─AK8975 三轴磁力计 0.3uT
│  ├─HMC5883L 三轴磁力计
│  ├─HMC5983 三轴磁力计 带温补
│  └─IST8310 三轴磁力计 0.3uT
├─5 气压计
│  ├─BMP085 I2C 分辨率为0.06hPa（0.5米）
│  ├─BMP280 I2C SPI 分辨率为0.12hPa（1米）
│  └─MS5611 I2C SPI 分辨率 10cm
├─README.assets
├─传感器的校准
└─综合性软件例程
    └─HMC5983 和 MPU6500 和 MS5611 的驱动程序
```

