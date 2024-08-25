//
// Created by hdmap on 18-11-14.
//

#ifndef LIDAR_FUSION_PROJECTOR_H
#define LIDAR_FUSION_PROJECTOR_H

#include <Eigen/Eigen>
#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/detail/distortion_model.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <vector>
#include "io.h"
using namespace std;
class Projector {
public:
  Projector() : _dis_image_cols(image_width), _dis_image_rows(image_height){};
  Projector(Eigen::Matrix4d &Rt, Eigen::Matrix3d &cameraMatrix,
            std::vector<double> &distortion)
      : _image_cols(image_width), _image_rows(image_height),
        _dis_image_cols(image_width * 1.5),
        _dis_image_rows(image_height * 1.5) {
    /*
   Projector() : _dis_image_cols(1280), _dis_image_rows(720){};
   Projector(Eigen::Matrix4d &Rt, Eigen::Matrix3d &cameraMatrix,
             std::vector<double> &distortion)
       : _image_cols(1280),
         _image_rows(720),
         _dis_image_cols(1960),
         _dis_image_rows(1080) {*/
    _half_delt_x = (_dis_image_cols - _image_cols) / 2;
    _half_delt_y = (_dis_image_rows - _image_rows) / 2;

    _Rt = Rt;
    _cameraMatrix = cameraMatrix;
    _distortion = distortion;

    //        cv::Mat cv_cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
    //        cv_cameraMatrix.at<double>(0, 0) = _cameraMatrix(0, 0);
    //        cv_cameraMatrix.at<double>(0, 1) = _cameraMatrix(0, 1);
    //        cv_cameraMatrix.at<double>(0, 2) = _cameraMatrix(0, 2);
    //        cv_cameraMatrix.at<double>(1, 1) = _cameraMatrix(1, 1);
    //        cv_cameraMatrix.at<double>(1, 2) = _cameraMatrix(1, 2);
    //
    //        cv::Mat distCoeffs = cv::Mat::zeros(5, 1, CV_64F);
    //        for(unsigned j = 0 ; j < 5 ; j++)
    //        {
    //            distCoeffs.at<double>(j, 0) = distortion[j];
    //        }

    // cv::Size imageSize(1280,720);
    cv::Size imageSize_raw(_image_cols, _image_rows);
    cv::Size imageSize(_dis_image_cols, _dis_image_rows);
    iniUndistortMap(_cameraMatrix, _distortion, imageSize_raw, imageSize,
                    _map_x, _map_y);
    //        cv::initUndistortRectifyMap(cv_cameraMatrix, distCoeffs,
    //        cv::Mat(), cv_cameraMatrix,
    //                                imageSize, CV_32FC1, _map_x, _map_y);
  };
  ~Projector(){};

private:
  void iniUndistortMap(Eigen::Matrix3d cameraMatrix,
                       std::vector<double> distortion, cv::Size raw_size,
                       cv::Size undis_size, cv::Mat &map1, cv::Mat &map2) {
    map1 = cv::Mat(undis_size, CV_32FC1);
    map2 = cv::Mat(undis_size, CV_32FC1);

    int delt_x, delt_y;
    delt_x = (undis_size.width - raw_size.width) / 2;
    delt_y = (undis_size.height - raw_size.height) / 2;

    _half_delt_x = delt_x;
    _half_delt_y = delt_y;

    double k1 = distortion[0];
    double k2 = distortion[1];
    double p1 = distortion[2];
    double p2 = distortion[3];
    double k3 = distortion.size() >= 5 ? distortion[4] : 0.;
    double k4 = distortion.size() >= 8 ? distortion[5] : 0.;
    double k5 = distortion.size() >= 8 ? distortion[6] : 0.;
    double k6 = distortion.size() >= 8 ? distortion[7] : 0.;
    double s1 = distortion.size() >= 12 ? distortion[8] : 0.;
    double s2 = distortion.size() >= 12 ? distortion[9] : 0.;
    double s3 = distortion.size() >= 12 ? distortion[10] : 0.;
    double s4 = distortion.size() >= 12 ? distortion[11] : 0.;
    double tauX = distortion.size() >= 14 ? distortion[12] : 0.;
    double tauY = distortion.size() >= 14 ? distortion[13] : 0.;
    cv::Matx33d matTilt = cv::Matx33d::eye();
    cv::detail::computeTiltProjectionMatrix(tauX, tauY, &matTilt);
    cv::Mat_<double> K_;
    eigen2cv(cameraMatrix, K_);
    cv::Mat_<double> K_i = K_.inv();

    /////////////////////////////////////////////
    for (int r = 0; r < undis_size.height; r++) {
      for (int c = 0; c < undis_size.width; c++) {
        cv::Point out_p;
        cv::Point p;

        p.x = c - delt_x;
        p.y = r - delt_y;

        // 1.get point in camera coordinate
        cv::Point2d uv_undistort = p;
        double _x =
            K_i(0, 0) * uv_undistort.x + K_i(0, 1) * uv_undistort.y + K_i(0, 2);
        double _y =
            K_i(1, 0) * uv_undistort.x + K_i(1, 1) * uv_undistort.y + K_i(1, 2);
        double _w =
            K_i(2, 0) * uv_undistort.x + K_i(2, 1) * uv_undistort.y + K_i(2, 2);
        double w = 1. / _w, x = _x * w, y = _y * w;

        // 2.distort
        double x2 = x * x, y2 = y * y;
        double r2 = x2 + y2, _2xy = 2 * x * y;
        double kr = (1 + ((k3 * r2 + k2) * r2 + k1) * r2) /
                    (1 + ((k6 * r2 + k5) * r2 + k4) * r2);
        double xd =
            (x * kr + p1 * _2xy + p2 * (r2 + 2 * x2) + s1 * r2 + s2 * r2 * r2);
        double yd =
            (y * kr + p1 * (r2 + 2 * y2) + p2 * _2xy + s3 * r2 + s4 * r2 * r2);

        // 3.project
        cv::Vec3d vecTilt = matTilt * cv::Vec3d(xd, yd, 1);
        double invProj = vecTilt(2) ? 1. / vecTilt(2) : 1;

        double u =
            cameraMatrix(0, 0) * invProj * vecTilt(0) + cameraMatrix(0, 2);
        double v =
            cameraMatrix(1, 1) * invProj * vecTilt(1) + cameraMatrix(1, 2);

        out_p.x = u;
        out_p.y = v;

        map1.at<float>(r, c) = out_p.x;
        map2.at<float>(r, c) = out_p.y;
      }
    }
  }

public:
  template <typename PointT>
  pcl::PointCloud<pcl::PointXY>::Ptr
  projCloud_3Dto2D(typename pcl::PointCloud<PointT>::Ptr cloud) {
    pcl::PointCloud<pcl::PointXY>::Ptr cloud_trans(
        new pcl::PointCloud<pcl::PointXY>);

    for (unsigned i = 0; i < cloud->points.size(); i++) {
      cloud_trans->points.push_back(projPt_3Dto2D_Dis(cloud->points[i]));
    }

    cloud_trans->width = cloud_trans->points.size();
    cloud_trans->height = 1;

    return cloud_trans;
  }

  template <typename PointT> pcl::PointXY projPt_3Dto2D_Dis(PointT &p) {
    return undistorPt(projPt_3Dto2D_Space(p));
  }

  // private:
  template <typename PointT>
  void transPointwithMatrix(const Eigen::Matrix4d &trans, const PointT &ptIn,
                            PointT &ptOut) {
    ptOut.x = ptIn.x * trans(0, 0) + ptIn.y * trans(0, 1) +
              ptIn.z * trans(0, 2) + trans(0, 3);
    ptOut.y = ptIn.x * trans(1, 0) + ptIn.y * trans(1, 1) +
              ptIn.z * trans(1, 2) + trans(1, 3);
    ptOut.z = ptIn.x * trans(2, 0) + ptIn.y * trans(2, 1) +
              ptIn.z * trans(2, 2) + trans(2, 3);
    ptOut.intensity = ptIn.intensity;
  }

  template <typename PointT> pcl::PointXY projPt_3Dto2D_Space(PointT &p) {
    pcl::PointXY pixel;
    pixel.x = NAN;
    pixel.y = NAN;

    PointT p_trans;

    transPointwithMatrix<PointT>(_Rt, p, p_trans);

    // 2、过滤不再图像范围的点
    if (p_trans.z < 0)
      return pixel;

    // 3、左乘内参矩阵
    //        float tmp_x = _cameraMatrix(0,0) * p_trans.x + _cameraMatrix(0,2)
    //        * p_trans.z; float tmp_y = _cameraMatrix(1,1) * p_trans.y +
    //        _cameraMatrix(1,2) * p_trans.z;
    float tmp_x =
        _cameraMatrix(0, 0) * p_trans.x + _cameraMatrix(0, 2) * p_trans.z;
    float tmp_y =
        _cameraMatrix(1, 1) * p_trans.y + _cameraMatrix(1, 2) * p_trans.z;
    p_trans.x = tmp_x;
    p_trans.y = tmp_y;

    // 4、x,y的值除z取整得到最终的像素坐标
    pixel.x = int(p_trans.x / p_trans.z);
    pixel.y = int(p_trans.y / p_trans.z);

    return pixel;
  }

  template <typename PointT> PointT undistorPt(PointT pt_in) {
    PointT pt = pt_in;
    PointT pt_trans;

    pt.x += _half_delt_x;
    pt.y += _half_delt_y;

    if (pt.x != pt.x || pt.y != pt.y || pt.x < 0 || pt.x >= _dis_image_cols ||
        pt.y < 0 || pt.y >= _dis_image_rows) {
      pt_trans.x = NAN;
      pt_trans.y = NAN;

      return pt_trans;
    }

    pt_trans.x = int(_map_x.at<float>(pt.y, pt.x));
    pt_trans.y = int(_map_y.at<float>(pt.y, pt.x));

    return pt_trans;
  }

public:
  std::vector<double> _distortion;
  Eigen::Matrix3d _cameraMatrix;
  Eigen::Matrix4d _Rt;

private:
  cv::Mat _map_x, _map_y;

  int _dis_image_cols;
  int _dis_image_rows;
  int _image_cols;
  int _image_rows;
  int _half_delt_x;
  int _half_delt_y;
};

#endif // LIDAR_FUSION_PROJECTOR_H
