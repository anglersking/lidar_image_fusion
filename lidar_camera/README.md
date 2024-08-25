# lidarcamera_calib

------
lidar 与 camera 在标定场内的自动标定

------
## 依赖
- 相机消息
- velodyne_multi_scans 消息
- velodyne驱动 
- whoami包

------
## 标定
- 在config目录下的camera.yaml文件中设置相机消息名称与内参json文件名称
- 在json目录下放置相机内参文件
- 在线运行：launch目录下的run.launch文件
- 离线运行：launch目录下的run.launch文件
- 最终生成的标定文件存放在config目录下

------
## 验证
- 在线：运行launch目录下的view.launch文件
- 离线：运行launch目录下的view.launch文件

------
## 可视化
- 打开 rqt_image_view 订阅 project 消息
- 标定时 project消息显示标定使用的角点（红为lidar投影点，黑为apriltag检测点）
- 验证时 project消息显示32线lidar所有点到图像的投影

------
## 评价标准
- topic 名称：“/lidar_calib_calibroom/evaluation_camera”
- 2d评价标准 投影到图像上的lidar角点与图像角点在像素坐标系x、y轴的坐标差
- 3d评价标准 映射到3d空间的图像角点与lidar角点在相机坐标系z轴的坐标差的百分比
