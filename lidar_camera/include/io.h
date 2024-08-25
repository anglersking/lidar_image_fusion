
#ifndef IO_H
#define IO_H

#include <stdio.h>
#include <stdlib.h>

#include <ros/ros.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

#include <algorithm>
#include <boost/algorithm/string.hpp> //字符串分割
#include <boost/thread/mutex.hpp>     //读写锁
#include <iostream>
#include <jsoncpp/json/json.h>

#include <pcl/io/pcd_io.h>   // PCL的PCD格式文件的输入输出头文件
#include <pcl/point_types.h> // PCL对各种格式的点的支持头文件

using namespace std;
using namespace cv;

// velodyne 驱动中消息格式的定义
struct XYZTIRA {
  PCL_ADD_POINT4D;
  double timestamp;
  uint8_t intensity;
  uint16_t ring;
  float azimuth;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(
    XYZTIRA,
    (float, x, x)(float, y, y)(float, z, z)(double, timestamp, timestamp)(
        uint8_t, intensity, intensity)(uint16_t, ring, ring)(float, azimuth,
                                                             azimuth))

const int tag_num = 10000;         //二维码的标号的数量上限
const double tagsize = 0.64;       //墙上二维码的的尺寸
const int tagnum[] = {1000, 2000}; //墙上二维码对应的序号

//相机名称
// const vector<string> camera_names = {"cameraF30","cameraF50", "cameraF100","cameraLF100", "cameraRF100",
//                                      "cameraR50", "cameraLR100", "cameraRR100"};
const vector<string> camera_names = {"cameraR50"//,"cameraR50","cameraF50","cameraF100","cameraRF100","cameraLF100"
//  , "cameraF100","cameraLF100", "cameraRF100","cameraR50", "cameraLR100", "cameraRR100"
};
const int camera_size = camera_names.size();

//配置文件名称及路径
const string para_name = "camera.yaml";

//标定结果文件名及保存的路径
const string out_paraname = "tf_cam2lidarcar.json";

//图像尺寸
const int image_height = 1200;//原来1080
const int image_width = 1920;
const Mat whitemask(image_height, image_width, CV_8UC3, Scalar(255, 255, 255));
const Mat blackmask(image_height, image_width, CV_8UC3, Scalar(0, 0, 0));

// 标定结果json文件的保护锁
static boost::mutex m_mutexoutjsonfile_;

struct Para {
  string ros_topic;
  Mat cameraMatrix;
  Mat distCoeffs;
  Eigen::Matrix4d extrinsic;
  Mat cameraExtrinsic;
};

struct tf_para{
  double x;
  double y;
  double z;
  double rotation_x;
  double rotation_y;
  double rotation_z;
  double rotation_w;
};

class IO {
public:
  IO();
  ~IO();

  static Para get_param_single(const string jsonpath);
  static bool get_param(const string para_path, 
                        const string json_path,
                        const string camera_name, 
                        Para &camera_para);
  static bool write_param(const string para_path,
                          const vector<string> camera_names,
                          const vector<Para> &camera_paras);

  static bool Write_Json(const string &json_path,
                         const vector<string> camera_name_list,
                         const vector<Para> &camera_paras);
  static bool Read_outparam(const string json_path, 
                            const string tf_yaml ,
                            const string camera_name,
                            Para &camera_para);
  static vector<string> getfilepath(string root_path, string suffix);
  static vector<double> ReadJson(string path, string name);
  static string ReadJson_String(string path, string name);
  static double ReadJson_Double(string path, string name);
  static Eigen::MatrixXf vector2Matrix(const vector<double> &vec);
  static vector<double> Matrix2vector(const Eigen::MatrixXd matrix);
  static vector<double> mat2ector(const Mat &_t1f);
  static Eigen::Matrix4d tf_transform( string addr);

private:
};

#endif