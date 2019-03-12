# Monocular-LRF_calibrate
单线激光雷达与单目相机的联合标定  
原项目参考https://github.com/TurtleZhong/camera_lidar_calibration，https://github.com/TurtleZhong/camera_lidar_calibration_v2  
在其基础上扩展了广角镜头的应用，使用方法改为Rviz获取标记点在激光雷达坐标系下的坐标，利用https://github.com/whu-hk/usb_cam_opencv 打开摄像头，通过简单的鼠标点击并获取标记点在畸变校正后图像上的坐标，这样将点对一一记录到/data/data/.txt，即可开始calibrate(利用ceres构建最小二乘问题求取R,T)
