#include <ros/package.h> // 查找packet路径
#include <ros/publisher.h>
#include <ros/ros.h>
#include <ros/subscriber.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/SVD>
#include <fstream>
#include <string>
#include "io.h"

vector<double> ReadJson(string path, string name) {
  vector<double> res;
  std::vector<std::string> sv;
  boost::algorithm::split(sv, name, boost::is_any_of("-")); // name为
  Json::Reader reader;
  Json::Value root;
  std::ifstream is;
  is.open(path.c_str());
  if (reader.parse(is, root, false)) {
    if (sv.size() == 1) { //单层json结构
      Json::Value array = root[sv[0]];
      for (int i = 0; i < array.size(); i++) {
        res.push_back(array[i].asDouble());
      }
    } else if (sv.size() == 2) { //双层json结构
      Json::Value array = root[sv[0]][sv[1]];
      for (int i = 0; i < array.size(); i++) {
        res.push_back(array[i].asDouble());
      }
    }
  }
  return res;
}
double ReadJson_single(string path, string name) {
  double temp;
  cv::FileStorage fs(path,cv::FileStorage::READ);
  if(!fs.isOpened()) exit (0);
  fs[name] >> temp;
  return temp;
}

string ReadJson_single_string(string path, string name) {
  std::string temp;
  cv::FileStorage fs(path,cv::FileStorage::READ);
  if(!fs.isOpened()) exit (0);
  fs[name] >> temp;
  return temp;
}

int main(int argc, char **argv)
{
    string filename1 = argv[1];
    string filename2 = argv[2];
    vector<double> rotation_matrix,
                   lidar2car;
    Eigen::Matrix4d transform_w2c;
    Eigen::Matrix4d transform_l2w;   
    Eigen::Matrix4d transform_l2c;  
    Eigen::Matrix3d temp;
    Eigen::Matrix<double, 3,1> temp1;
    transform_l2w.inverse();
    double cam_to_front,
           cam_to_left,
           cam_to_right,
           camera_height,
           camera_2_car_x;
    std::string      cam_direction;
    if(filename2 == "" | filename2 == "-")
    {
        transform_l2w=Eigen::Isometry3d::Identity().matrix();
    }
    else
    {
        fstream fin(filename2);
        if(!fin) std::cout << "input lidar2car.txt err" <<std::endl;
        while (!fin.eof())
        {
            double data;
            fin >>data;
            lidar2car.push_back(data);
        }
        fin.close();
        for (size_t i = 0; i < 4; ++i)
        {
            for (size_t j = 0; j < 4; ++j)
            {
            transform_l2w(i,j)=lidar2car[i*4+j];
            }
            
        }
    }
    rotation_matrix = ReadJson(filename1, "rotation_matrix");
    cam_to_front = ReadJson_single(filename1,"cam_to_front");
    cam_to_left = ReadJson_single(filename1,"cam_to_left");
    cam_to_right = ReadJson_single(filename1,"cam_to_right");
    camera_height = ReadJson_single(filename1,"camera_height");
    cam_direction = ReadJson_single_string(filename1,"cam_direction");
    if (rotation_matrix.size() != 9) {
        ROS_ERROR_STREAM("vec_camera_matrix size error :" );
        return false;
    }
    std::cout << "2222"<< std::endl;
    cv::Mat k(3, 3, CV_64F);
    cv::Mat j(4, 4, CV_64F);
    for (size_t i = 0; i < 3; ++i)
        {
            for (size_t j = 0; j < 3; ++j)
            {
            temp(i,j)=rotation_matrix[i*3+j];
            }
        }
    //std::cout << "#################rotation#########" << std::endl << temp << std::endl;
    temp=temp.inverse();
    //std::cout << "#################rotation-1#########" << std::endl << temp << std::endl;
    //std::cout << "####################################"<< std::endl;
    temp1(0,0)=-cam_to_front;
    std::cout << temp1(0,0) << std::endl;
    temp1(1,0)=(cam_to_left+cam_to_right)/2-cam_to_left;
    std::cout << temp1(1,0) << std::endl;
    temp1(2,0)=camera_height;
    std::cout << temp1(2,0) << std::endl;
    //std::cout << "#################T#########" << std::endl << temp1 << std::endl;
    temp1=-temp*temp1;
    //std::cout << "#################T-1#########" << std::endl << temp1 << std::endl;
    //std::cout << "#############################"<< std::endl;
    transform_w2c(0,0)=temp(0,0);
    transform_w2c(0,1)=temp(0,1);
    transform_w2c(0,2)=temp(0,2);
    transform_w2c(0,3)=temp1(0,0);
    transform_w2c(1,0)=temp(1,0);
    transform_w2c(1,1)=temp(1,1);
    transform_w2c(1,2)=temp(1,2);
    transform_w2c(1,3)=temp1(1,0);
    transform_w2c(2,0)=temp(2,0);
    transform_w2c(2,1)=temp(2,1);
    transform_w2c(2,2)=temp(2,2);
    transform_w2c(2,3)=temp1(2,0);
    transform_w2c(3,0)=0;
    transform_w2c(3,1)=0;
    transform_w2c(3,2)=0;
    transform_w2c(3,3)=1;

    transform_l2c = transform_w2c * transform_l2w;
    std::cout << "***************      " << cam_direction <<"       ***************"<< std::endl;
    std::cout << "#################transform_l2c###############" << std::endl << transform_l2c << std::endl;
    std::cout << "#################transform_w2c###############" << std::endl << transform_w2c << std::endl;
    std::cout << "#################transform_l2w###############" << std::endl << transform_l2w << std::endl;
    std::cout << "**********************************************"<< std::endl;
    return 0;
}