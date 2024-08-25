//
// Created by hdmap on 18-11-15.
//

#ifndef LIDAR_FUSION_PROJECTOR_TOOLS_H
#define LIDAR_FUSION_PROJECTOR_TOOLS_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Eigen>
#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <vector>

#include "projector.h"
#include "io.h"
template <typename PointT>
inline float calsulateProjScore(pcl::PointCloud<pcl::PointXY>::Ptr cloud,
                                PointT pt_left_up, PointT pt_right_down) {
  int num_pts = cloud->points.size();
  int count_inside = 0;
  for (unsigned i = 0; i < cloud->points.size(); i++) {
    pcl::PointXY pt_ = cloud->points[i];
    if (pt_.x > pt_left_up.x && pt_.y > pt_left_up.y &&
        pt_.x < pt_right_down.x && pt_.y < pt_right_down.y)
      count_inside++;
  }
  return count_inside * 1.0 / num_pts;
}

template <typename PointT>

inline std::vector<float> calsulateProjScore2(
    pcl::PointCloud<pcl::PointXY>::Ptr cloud, const PointT &pt_left_up,
    const PointT &pt_right_down, std::shared_ptr<Projector> proj) {
  // putput
  std::vector<float> score(2);

  int num_pts = cloud->points.size();
  int count_inside = 0;
  int count_vali = 0;
  pcl::PointXY pt_lidar_lef_up, pt_lidar_right_down;
  pt_lidar_lef_up.x = 10000.0f;
  pt_lidar_lef_up.y = 10000.0f;
  pt_lidar_right_down.x = -10000.0f;
  pt_lidar_right_down.y = -10000.0f;

  for (unsigned i = 0; i < cloud->points.size(); i++) {
    pcl::PointXY pt_ = cloud->points[i];

    if (pt_.x != pt_.x || pt_.y != pt_.y) continue;
    if (pt_.x < 0 || pt_.x > image_width || pt_.y < 0 || pt_.y > image_height) continue;

    // lidar pt inside vision rect
    if (pt_.x > pt_left_up.x && pt_.y > pt_left_up.y &&
        pt_.x < pt_right_down.x && pt_.y < pt_right_down.y)
      count_inside++;

    // lidar_rect
    if (pt_.x < pt_lidar_lef_up.x) pt_lidar_lef_up.x = pt_.x;
    if (pt_.x > pt_lidar_right_down.x) pt_lidar_right_down.x = pt_.x;
    if (pt_.y < pt_lidar_lef_up.y) pt_lidar_lef_up.y = pt_.y;
    if (pt_.y > pt_lidar_right_down.y) pt_lidar_right_down.y = pt_.y;

    // pt inisde camera frame
    count_vali++;
  }

  if (count_vali < 5) {
    score[0] = 0.0f;
    score[1] = 0.0f;

    // this cluster is not inside this frame
    return score;
  } else {
    // judge this cluster if inside rect
    score[1] = count_inside * 1.0f / num_pts;

    // judge this cluster match rect // cross rect
    float x_min = std::max(pt_lidar_lef_up.x, pt_left_up.x);
    float x_max = std::min(pt_lidar_right_down.x, pt_right_down.x);
    float y_min = std::max(pt_lidar_lef_up.y, pt_left_up.y);
    float y_max = std::min(pt_lidar_right_down.y, pt_lidar_right_down.y);

    if (x_max < x_min || y_max < y_min)
      score[0] = 0.0;
    else if ((pt_right_down.x - pt_left_up.x) < 0.1 ||
             (pt_right_down.y - pt_left_up.y) < 0.1 ||
             (pt_right_down.x - pt_left_up.x) < 0.1 ||
             (pt_right_down.y - pt_left_up.y) < 0.1) {
      score[0] = 0.0;
    } else {
      float score0 = (x_max - x_min) * (y_max - y_min) /
                     ((pt_lidar_right_down.x - pt_lidar_lef_up.x) *
                      (pt_lidar_right_down.y - pt_lidar_lef_up.y));

      float score1 =
          (x_max - x_min) * (y_max - y_min) /
          ((pt_right_down.x - pt_left_up.x) * (pt_right_down.y - pt_left_up.y));

      score[0] = (score0 + score1) / 2.0f;
    }

    return score;
  }
}

inline void getColor_gray(uint8_t intensity, uint8_t &R, uint8_t &G,
                          uint8_t &B) {
  R = intensity;
  G = intensity;
  B = intensity;
  return;
}

inline void getColor_pseudo(uint8_t intensity, uint8_t &R, uint8_t &G,
                            uint8_t &B) {
  R = abs(255 - intensity);
  G = abs(127 - intensity);
  B = abs(0 - intensity);
}

inline void getColor_rainbow(uint8_t intensity, uint8_t &R, uint8_t &G,
                             uint8_t &B) {
  if (intensity <= 51) {
    B = 255;
    G = intensity * 5;
    R = 0;
  } else if (intensity <= 102) {
    intensity -= 51;
    B = 255 - intensity * 5;
    G = 255;
    R = 0;
  } else if (intensity <= 153) {
    intensity -= 102;
    B = 0;
    G = 255;
    R = intensity * 5;
  } else if (intensity <= 204) {
    intensity -= 153;
    B = 0;
    G = 255 - uchar(128.0 * intensity / 51.0 + 0.5);
    R = 255;
  } else {
    intensity -= 204;
    B = 0;
    G = 127 - uchar(127.0 * intensity / 51.0 + 0.5);
    R = 255;
  }
  return;
}

inline cv::Scalar getColor_random(int intensity) {
  cv::Scalar color;
  uint8_t R, G, B;
  if (intensity == -3) {
    R = 0;
    G = 0;
    B = 0;
  } else if (intensity == -2) {
    R = 100;
    G = 100;
    B = 100;
  } else if (intensity == -1) {
    R = 200;
    G = 200;
    B = 200;
  } else if (intensity == 0) {
    R = 0;
    G = 0;
    B = 150;
  }

  else
    getColor_rainbow((intensity * 11) % 255, R, G, B);

  //    R = (uint8_t)((intensity % 3)  * 255 / 8  % 220) + 30;
  //    G = (uint8_t)((intensity % 7)  * 255 / 7  % 220) + 30;
  //    B = (uint8_t)((intensity % 11) * 255 / 11 % 220) + 30;

  color[0] = B;
  color[1] = G;
  color[2] = R;

  return color;
}

inline cv::Scalar getColor_rainbow(int intensity) {
  cv::Scalar color;
  uint8_t R, G, B;
  intensity = intensity * 1.5;
  if (intensity <= 51) {
    B = 255;
    G = intensity * 5;
    R = 0;
  } else if (intensity <= 102) {
    intensity -= 51;
    B = 255 - intensity * 5;
    G = 255;
    R = 0;
  } else if (intensity <= 153) {
    intensity -= 102;
    B = 0;
    G = 255;
    R = intensity * 5;
  } else if (intensity <= 204) {
    intensity -= 153;
    B = 0;
    G = 255 - uchar(128.0 * intensity / 51.0 + 0.5);
    R = 255;
  } else {
    intensity -= 204;
    B = 0;
    G = 127 - uchar(127.0 * intensity / 51.0 + 0.5);
    R = 255;
  }

  color[0] = R;
  color[1] = G;
  color[2] = B;

  return color;
};

// 单个像素点去畸变
inline void undistortpoints(const Eigen::Matrix3d &cameraMatrix,
                            const std::vector<double> &distortion,
                            pcl::PointCloud<pcl::PointXY>::Ptr &pixels) {
  cv::Mat cv_cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
  cv_cameraMatrix.at<double>(0, 0) = cameraMatrix(0, 0);
  cv_cameraMatrix.at<double>(0, 1) = cameraMatrix(0, 1);
  cv_cameraMatrix.at<double>(0, 2) = cameraMatrix(0, 2);
  cv_cameraMatrix.at<double>(1, 1) = cameraMatrix(1, 1);
  cv_cameraMatrix.at<double>(1, 2) = cameraMatrix(1, 2);

  cv::Mat distCoeffs = cv::Mat::zeros(14, 1, CV_64F);
  for (unsigned j = 0; j < distortion.size(); j++) {
    distCoeffs.at<double>(j, 0) = distortion[j];
  }

  std::vector<cv::Point2f> inputDistortedPoints;  //相机原始像素点（带畸变）
  for (int i = 0; i < pixels->points.size(); i++) {
    Point2f p(pixels->points[i].x, pixels->points[i].y);
    inputDistortedPoints.push_back(p);
  }

  std::vector<cv::Point2f> outputUndistortedPoints;  //处理后的像素点（去畸变）
  cv::undistortPoints(inputDistortedPoints, outputUndistortedPoints,
                      cv_cameraMatrix, distCoeffs, cv::noArray(),
                      cv_cameraMatrix);

  pixels->points.clear();
  for (int i = 0; i < outputUndistortedPoints.size(); i++) {
    pcl::PointXY p;
    p.x = outputUndistortedPoints[i].x;
    p.y = outputUndistortedPoints[i].y;
    pixels->points.push_back(p);
  }
  return;
}

template <typename PointT>
inline void paintCloud2Image(cv::Mat image,
                             typename pcl::PointCloud<PointT>::Ptr cloud,
                             pcl::PointCloud<pcl::PointXY>::Ptr &pixels,
                             std::shared_ptr<Projector> proj_ptr,
                             bool if_undistortpoints, int size = 2) {
  pixels = proj_ptr->projCloud_3Dto2D<PointT>(cloud);  //加畸变的点?

  if (if_undistortpoints) {
    undistortpoints(proj_ptr->_cameraMatrix, proj_ptr->_distortion, pixels);
  }
  int num_points = pixels->points.size();
  for (unsigned i = 0; i < num_points; i++) {
    if (pixels->points[i].x != pixels->points[i].x ||
        pixels->points[i].y != pixels->points[i].y)
      continue;

    cv::Point cv_pt(pixels->points[i].x, pixels->points[i].y);
    cv::Scalar color = getColor_rainbow(cloud->points[i].intensity);

    cv::circle(image, cv_pt, size, color, -1);
  }
}

template <typename PointT>
inline void paint_Rect(cv::Mat image, std::pair<PointT, PointT> rect,
                       cv::Scalar color = cv::Scalar(255, 255, 255)) {
  cv::Point2d left_up, right_down;
  left_up.x = rect.first.x;
  left_up.y = rect.first.y;
  right_down.x = rect.second.x;
  right_down.y = rect.second.y;

  cv::rectangle(image, left_up, right_down, color, 5, 8);
}

inline void paint_Cluster(cv::Mat image,
                          pcl::PointCloud<pcl::PointXY>::Ptr pixels,
                          cv::Scalar color = cv::Scalar(255, 255, 255)) {
  for (unsigned i = 0; i < pixels->points.size(); i++) {
    cv::Point cv_pt(pixels->points[i].x, pixels->points[i].y);
    cv::circle(image, cv_pt, 4, color, -1);
  }
}

#endif  // LIDAR_FUSION_PROJECTOR_TOOLS_H
