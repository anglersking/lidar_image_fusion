#ifndef EVALUATION_H
#define EVALUATION_H
#include <ros/package.h>  // 查找packet路径
#include <ros/ros.h>
#include "io.h"
#include "projector_tools.h"
#include "projector.h"

struct Evalu {                                               //评测指标
  double max_deviation_x, max_deviation_y, max_deviation_z;  //最大偏差百分比
  double ave_deviation_x, ave_deviation_y, ave_deviation_z;  //平均偏差百分比

  double max_horizontal_reprojecterr;
  double max_vertical_reprojecterr;  //最大重投影误差
  double ave_horizontal_reprojecterr;
  double ave_vertical_reprojecterr;  //平均重投影误差
};

class Evaluate {
 public:
  Evaluate();
  ~Evaluate();

 public:
 public:
  pcl::PointCloud<pcl::PointXY>::Ptr all_cloudproject(
      const pcl::PointCloud<XYZTIRA>::Ptr &lidar_data, bool if_undistortpoints,
      const vector<Para> &camera_para, vector<cv::Mat> &images);

  pcl::PointCloud<pcl::PointXY>::Ptr cloudproject(
      const vector<cv::Point3f> &vPt3D, bool if_undistortpoints,
      const Para &camera_para, cv::Mat &image);

  bool draw_imagepoints(const bool if_undistortpoints, const Para camera_para,
                        const vector<cv::Point2f> &cloud_pixels,
                        cv::Mat &image);

  bool calu_reprojecterr(const pcl::PointCloud<pcl::PointXY>::Ptr &cloud_2d,
                         const vector<cv::Point2f> pixels, Evalu &evalu,
                         const vector<int> &inliers, const bool is_filter);

  bool calu_distanceerr(
      const pcl::PointCloud<pcl::PointXYZINormal>::Ptr pixel3d,
      const pcl::PointCloud<pcl::PointXYZINormal>::Ptr lidardata, Evalu &evalu,
      const vector<int> &inliers, const bool is_filter);
};
#endif
