# 相机标定

## 一、依赖与安装

依赖：`OpenCV`

生成：```./setup.bat```

## 二、介绍

相机标定本质上是设计R<sup>3</sup>中已知尺寸的平面图案，使得其投影到图像后特征点具有显著性，容易被算法检测。接着通过构造三维空间和图像中的3D-2D匹配点对，使用张正友标定法计算相机内参和畸变参数。

常见的相机标定板有四种，分别是棋盘格，对称圆点标定板、非对称圆点标定板和ChArUco板。对应特征点检测算法分别检测角点(鞍点)、投影椭球中心、投影椭球中心、棋盘格与ArUco的角点。

<p align="center"><img src="./resources/calib_board.png" width=50%></p>

<h6 align="center">从外到内分别是对称圆点、非对称圆点、棋盘格和ChArUco标定板</h6>

OpenCV支持四种标定板的标定(Matlab2021之前的版本只支持棋盘格)，本仓库代码是C++ OpenCV对应官方标定例子的总结。

## 三、参考资料

[1] [OpenCV对称圆点标定](https://blog.csdn.net/weixin_51229250/article/details/120009716)

[2] [CSDN-ChArUco](https://blog.csdn.net/zhy29563/article/details/119039163)

