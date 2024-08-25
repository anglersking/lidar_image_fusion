// 标定结果评测

#include "evaluation.h"

Evaluate::Evaluate() {}
Evaluate::~Evaluate() {}

/*
** \brief 点云投影到图像
** @lidar_data LIDAR点云
** @camera_para 相机参数（内参、畸变、标定结果矩阵）
** @if_undistortpoints 是否需要去畸变
** @images 待处理图像
** @return 点云所对应的像素
*/
pcl::PointCloud<pcl::PointXY>::Ptr Evaluate::all_cloudproject(
    const pcl::PointCloud<XYZTIRA>::Ptr &lidar_data, 
    bool if_undistortpoints,
    const vector<Para> &camera_para, 
    vector<cv::Mat> &images) {
  pcl::PointCloud<pcl::PointXY>::Ptr pixels;
  std::unordered_map<int, typename std::shared_ptr<Projector> >
      _camProjectors;  // project

  for (int i = 0; i < images.size(); i++) {
    Eigen::Matrix3d _cameraMatrix;
    cv2eigen(camera_para[i].cameraMatrix, _cameraMatrix);
    vector<double> _distortion = IO::mat2ector(camera_para[i].distCoeffs);
    Eigen::Matrix4d eigen_transform = camera_para[i].extrinsic.cast<double>();
    _camProjectors[i] = std::make_shared<Projector>(
        eigen_transform, _cameraMatrix, _distortion);  // project
    paintCloud2Image<XYZTIRA>(images[i], lidar_data, pixels, _camProjectors[i],
                              if_undistortpoints);
  }

  return pixels;
}

/*
** \brief 点云投影到图像
** @lidar_data LIDAR点云
** @camera_para 相机参数（内参、畸变、标定结果矩阵）
** @if_undistortpoints 是否需要去畸变
** @images 待处理图像
** @return 点云所对应的像素
*/

pcl::PointCloud<pcl::PointXY>::Ptr Evaluate::cloudproject(
    const vector<cv::Point3f> &vPt3D, bool if_undistortpoints,
    const Para &camera_para, cv::Mat &image) {
  pcl::PointCloud<pcl::PointXY>::Ptr pixel;

  std::unordered_map<int, typename std::shared_ptr<Projector> >
      _camProjectors;  // project

  pcl::PointCloud<XYZTIRA>::Ptr lidar_data(new pcl::PointCloud<XYZTIRA>);

  for (int i = 0; i < vPt3D.size(); i++) {
    XYZTIRA p;
    p.x = vPt3D[i].x;
    p.y = vPt3D[i].y;
    p.z = vPt3D[i].z;
    p.intensity = 0;
    lidar_data->points.push_back(p);
  }

  Eigen::Matrix3d _cameraMatrix;
  cv2eigen(camera_para.cameraMatrix, _cameraMatrix);


  Eigen::Matrix4d eigen_transform = camera_para.extrinsic;//.cast<double>();
  vector<double> _distortion = IO::mat2ector(camera_para.distCoeffs);
  //vector<double> _distortion(5, 0);  //畸变参数为0
  _camProjectors[0] = std::make_shared<Projector>(
      eigen_transform, _cameraMatrix, _distortion);  //加畸变
  paintCloud2Image<XYZTIRA>(image, lidar_data, pixel, _camProjectors[0],
                            if_undistortpoints);

  return pixel;
}

/*
** \brief 2d点绘制到图片
** @iif_undistortpoints 是否需要去畸变
** @camera_para 相机参数（内参、畸变、标定结果矩阵）
** @images 绘制的图片
** @cloud_pixels 图片像素
*/
bool Evaluate::draw_imagepoints(const bool if_undistortpoints,
                                const Para camera_para,
                                const vector<cv::Point2f> &cloud_pixels,
                                cv::Mat &image) {
  if (if_undistortpoints) {
    pcl::PointCloud<pcl::PointXY>::Ptr pixels(
        new pcl::PointCloud<pcl::PointXY>);
    for (int i = 0; i < cloud_pixels.size(); i++) {
      pcl::PointXY p;
      p.x = cloud_pixels[i].x;
      p.y = cloud_pixels[i].y;
      pixels->points.push_back(p);
    }
    Eigen::Matrix3d _cameraMatrix;
    cv2eigen(camera_para.cameraMatrix, _cameraMatrix);
    vector<double> _distortion = IO::mat2ector(camera_para.distCoeffs);

    undistortpoints(_cameraMatrix, _distortion, pixels);
  }

  for (int i = 0; i < cloud_pixels.size(); i++) {
    auto p = cloud_pixels[i];
    cv::circle(image, Point(p.x, p.y), 3, Scalar(0, 0, 0));
  }
  return true;
}

/*
** \brief 计算重投影误差
** @cloud_2d 点云投影到图像的投影点
** @pixels 识别出的2D角点
** @evalu 参数评价标准
** @inliers 过滤数组
** @is_filter 是否需要过滤
** @return 误差是否计算成功
*/
bool Evaluate::calu_reprojecterr(
    const pcl::PointCloud<pcl::PointXY>::Ptr &cloud_2d,
    const vector<cv::Point2f> pixels, Evalu &evalu, const vector<int> &inliers,
    const bool is_filter) {
  int total_num;
  if (cloud_2d->points.size() != pixels.size()) {
    ROS_ERROR("cloud_2d and pixels has diff size.");
    return false;
  } else {
    total_num = cloud_2d->points.size();
    if (total_num == 0) {
      ROS_ERROR("cloud_2d and pixels is empty.");
      return false;
    }
  }

  double total_horizontal_reprojecterr = 0;
  double total_vertical_reprojecterr = 0;
  evalu.max_horizontal_reprojecterr = -1;
  evalu.max_vertical_reprojecterr = -1;

  if (is_filter) {
    for (int t = 0; t < inliers.size(); t++) {
      int i = inliers[t];
      double horizontal_dis = fabs(cloud_2d->points[i].x - pixels[i].x);
      double vertical_dis = fabs(cloud_2d->points[i].y - pixels[i].y);
      evalu.max_horizontal_reprojecterr =
          evalu.max_horizontal_reprojecterr < horizontal_dis
              ? horizontal_dis
              : evalu.max_horizontal_reprojecterr;
      evalu.max_vertical_reprojecterr =
          evalu.max_vertical_reprojecterr < vertical_dis
              ? vertical_dis
              : evalu.max_vertical_reprojecterr;
      total_horizontal_reprojecterr += horizontal_dis;
      total_vertical_reprojecterr += vertical_dis;
    }
    evalu.ave_horizontal_reprojecterr =
        total_horizontal_reprojecterr / inliers.size();
    evalu.ave_vertical_reprojecterr =
        total_vertical_reprojecterr / inliers.size();
  } else {
    for (int i = 0; i < total_num; i++) {
      double horizontal_dis = fabs(cloud_2d->points[i].x - pixels[i].x);
      double vertical_dis = fabs(cloud_2d->points[i].y - pixels[i].y);
      evalu.max_horizontal_reprojecterr =
          evalu.max_horizontal_reprojecterr < horizontal_dis
              ? horizontal_dis
              : evalu.max_horizontal_reprojecterr;
      evalu.max_vertical_reprojecterr =
          evalu.max_vertical_reprojecterr < vertical_dis
              ? vertical_dis
              : evalu.max_vertical_reprojecterr;
      total_horizontal_reprojecterr += horizontal_dis;
      total_vertical_reprojecterr += vertical_dis;
    }
    evalu.ave_horizontal_reprojecterr =
        total_horizontal_reprojecterr / total_num;
    evalu.ave_vertical_reprojecterr = total_vertical_reprojecterr / total_num;
  }

  return true;
}

/*
** \brief 计算2d到3d的距离误差
** @pixel3d 图像坐标系下apriltag的中心点
** @lidardata lidar在图像坐标系的3d点
** @evalu 参数评价标准
** @inliers 过滤数组
** @is_filter 是否需要过滤
** @return 误差是否计算成功
*/
bool Evaluate::calu_distanceerr(
    const pcl::PointCloud<pcl::PointXYZINormal>::Ptr pixel3d,
    const pcl::PointCloud<pcl::PointXYZINormal>::Ptr lidardata, Evalu &evalu,
    const vector<int> &inliers, const bool is_filter) {
  int total_num;
  if (pixel3d->points.size() != lidardata->points.size()) {
    ROS_ERROR("pixel3d and lidardata has diff size.");
    return false;
  } else {
    total_num = pixel3d->points.size();
    if (total_num == 0) {
      ROS_ERROR("cloud_2d and pixels is empty.");
      return false;
    } else if (total_num % 4 != 0) {
      ROS_ERROR("lidardata not an integer multiple of '4'.");
      return false;
    }
  }

  // 根据四个角点求重心
  vector<double> cloudcenters_x;
  vector<double> cloudcenters_y;
  vector<double> cloudcenters_z;
  for (int i = 0; i < total_num / 4; i++) {
    double total_x = 0;
    double total_y = 0;
    double total_z = 0;
    for (int j = 4 * i; j < 4 * i + 4; j++) {
      total_x += lidardata->points[j].x;
      total_y += lidardata->points[j].y;
      total_z += lidardata->points[j].z;
    }
    for (int j = 0; j < 4; j++) {
      cloudcenters_x.push_back(0.25 * total_x);
      cloudcenters_y.push_back(0.25 * total_y);
      cloudcenters_z.push_back(0.25 * total_z);
    }
  }

  evalu.max_deviation_x = -1;
  evalu.max_deviation_y = -1;
  evalu.max_deviation_z = -1;
  double total_disx = 0;
  double total_disy = 0;
  double total_disz = 0;

  int actual_number = 0;
  if (is_filter) {
    for (int t = 0; t < inliers.size(); t++) {
      int i = inliers[t];
      if (fabs(pixel3d->points[i].normal_x) *
              fabs(pixel3d->points[i].normal_y) *
              fabs(pixel3d->points[i].normal_z) ==
          0) {
        continue;
      }
      double dis_x, dis_y, dis_z;
      dis_x = fabs(cloudcenters_x[i] - pixel3d->points[i].normal_x) /
              fabs(pixel3d->points[i].normal_x);
      dis_y = fabs(cloudcenters_y[i] - pixel3d->points[i].normal_y) /
              fabs(pixel3d->points[i].normal_y);
      dis_z = fabs(cloudcenters_z[i] - pixel3d->points[i].normal_z) /
              fabs(pixel3d->points[i].normal_z);

      //只处理墙上的大tag
      int tag_id = pixel3d->points[i].intensity;
      if (tag_id < tagnum[0] || tag_id > tagnum[1]) {
        continue;
      }
      evalu.max_deviation_x =
          dis_x > evalu.max_deviation_x ? dis_x : evalu.max_deviation_x;
      evalu.max_deviation_y =
          dis_y > evalu.max_deviation_y ? dis_y : evalu.max_deviation_y;
      evalu.max_deviation_z =
          dis_z > evalu.max_deviation_z ? dis_z : evalu.max_deviation_z;

      total_disx += dis_x;
      total_disy += dis_y;
      total_disz += dis_z;
      actual_number++;
    }
    if (actual_number == 0) {
      return false;
    }
    evalu.ave_deviation_x = total_disx / actual_number;
    evalu.ave_deviation_y = total_disy / actual_number;
    evalu.ave_deviation_z = total_disz / actual_number;
  } else {
    for (int i = 0; i < total_num; i++) {
      double dis_x, dis_y, dis_z;
      if (fabs(pixel3d->points[i].normal_x) *
              fabs(pixel3d->points[i].normal_y) *
              fabs(pixel3d->points[i].normal_z) ==
          0) {
        continue;
      }
      dis_x = fabs(cloudcenters_x[i] - pixel3d->points[i].normal_x) /
              fabs(pixel3d->points[i].normal_x);
      dis_y = fabs(cloudcenters_y[i] - pixel3d->points[i].normal_y) /
              fabs(pixel3d->points[i].normal_y);
      dis_z = fabs(cloudcenters_z[i] - pixel3d->points[i].normal_z) /
              fabs(pixel3d->points[i].normal_z);
      //只处理墙上的大tag
      int tag_id = pixel3d->points[i].intensity;
      if (tag_id < tagnum[0] || tag_id > tagnum[1]) {
        continue;
      }
      evalu.max_deviation_x =
          dis_x > evalu.max_deviation_x ? dis_x : evalu.max_deviation_x;
      evalu.max_deviation_y =
          dis_y > evalu.max_deviation_y ? dis_y : evalu.max_deviation_y;
      evalu.max_deviation_z =
          dis_z > evalu.max_deviation_z ? dis_z : evalu.max_deviation_z;

      total_disx += dis_x;
      total_disy += dis_y;
      total_disz += dis_z;
      actual_number++;
    }

    if (actual_number == 0) {
      return false;
    }
    evalu.ave_deviation_x = total_disx / actual_number;
    evalu.ave_deviation_y = total_disy / actual_number;
    evalu.ave_deviation_z = total_disz / actual_number;
  }

  return true;
}
