# WK_STEREO

一个基于ESP32CAM的简易空间定位系统
使用对极几何方法实现对于双目相机相对位姿的校准
使用三角化实现对于空间单点的定位
相机内参校准的精度和对极几何校准的精度直接决定了最终三角化的精度

### 前置工作

1.编译烧录ESP32CAM程序，修改自身的参数，见[LGQWakkk/ESP32CAM_UDP_Video: ESP32CAM UDP 传输JPEG流 Python OpenCV上位机显示 (github.com)](https://github.com/LGQWakkk/ESP32CAM_UDP_Video)

2.相机内参校准：可以使用各种方式获取相机的内参，随后将两个相机的内参输入到calibration.py模块中的内参部分。

### GUI基本使用方法

运行main.py

初始界面如下：

![image-20240912110457399]([WK_STEREO/readme_image/img1.png at main · LGQWakkk/WK_STEREO (github.com)](https://github.com/LGQWakkk/WK_STEREO/blob/main/readme_image/img1.png))

### 初始设置

首先设置UDP监听IP地址以及端口

IP一般就是电脑无线网卡此时的IP地址 

监听的端口取决于ESP32程序中发送的端口

我这里将CAM1的端口设置为6666 CAM2设置为6667

随后开启ESP32并等待其自动连接网络并开始发送图像

随后点击“Start Listening”开始监听，正常情况下会开始显示ESP32发送来的图像。

同时会显示检测的点的状态

注意：只有当每个相机都稳定的检测到仅仅是一个点的时候才作为检测有效，否则无法正常运行，所以请确保环境中光源的稳定性与纯净性，确保检测稳定只有一个点。

### 相机外参校准

![image-20240912110957571]([WK_STEREO/readme_image/img1.png at main · LGQWakkk/WK_STEREO (github.com)](https://github.com/LGQWakkk/WK_STEREO/blob/main/readme_image/img2.png))

PS:Start Auto Calibration现在不能使用(施工ing...)

校准的基本原理就是，根据几个两个相机的共视点，可以根据对极几何求解出本质矩阵，随后可以根据相机内参分解出两个相机的相对Rt(相对旋转以及相对位移)，获取位姿之后就可以实现对于点的三角化，从而实现定位。

校准步骤：

1.将红外信标移动到某一个位置并**保持静止**

2.当两个相机都是检测成功，点击"Capture Sample"进行单次采样，若"Valid Samples"增加，则表示采集成功。

3.移动红外信标到另一个位置，并**保持静止**

4.重复进行2 3步，至少采集10个点以上(其实更少也是可以，只要不报错即可，但是点越多往往精度越高)

5.点击"Start Calculation"开始求解

求解成功后，可以看见两个相机的相对位置已经更新到OpenGL可视化窗口，因为相机坐标系可能和实际摆放的坐标系存在偏差，所以可能需要手动移动更换视角来实现正常的显示。

### 开始三角化

在校准成功之后，可以开始连续的三角化。

点击"Triangulate"进行单次的三角化，或者"Start Triangulating"进行连续的三角化，若帧率足够，可以实现较为流畅的空间定位。

### 本工程的问题

1.关于相机同步的问题：本工程没有对相机做任何的同步，目前对于ESP32做同步还是较为困难的，本人也在处理中，若有相关想法欢迎联系讨论：3161554058@qq.com

2.关于相机校准的问题：本工程没有提供相机内参校准的嵌入，给用户带来不便，在此表示抱歉，正在找一个相比于opencv标定函数更为精确的标定方法，现有的标定方式可以使用Matlab或者其他开源项目。

3.关于Python上位机的问题：本人水平不太行，这个上位机仅仅是勉强能用，bug有点多...，若有什么问题或者改进欢迎批评指正。

