#include <ros/package.h> // 查找packet路径
#include <ros/publisher.h>
#include <ros/ros.h>
#include <ros/subscriber.h>
#include <yaml-cpp/yaml.h>
#include <opencv2/opencv.hpp>

#include <iostream>
#include "io.h"

#include <iostream>
#include <fstream>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>

#define CAMERAMAT "fx"
#define DISTCOEFF "fy"
Mat R;
Mat T;
Mat camMat;
Mat distCoeff;
using namespace std;
using namespace Eigen;
using namespace cv;
IO::IO() {}
IO::~IO() {}

/*
** \brief 将Eigen::Matrix多维矩阵转为一维数组
** @matrix 矩阵
** @return 一维数组
*/
vector<double> IO::Matrix2vector(const Eigen::MatrixXd matrix) {
  vector<double> res;
  for (int i = 0; i < matrix.rows(); i++) {
    for (int j = 0; j < matrix.cols(); j++) {
      res.push_back(matrix(i, j));
    }
  }
  return res;
}


int readPara(string filename)
{


    YAML::Node config = YAML::LoadFile(filename);
    cv::FileStorage fs(filename,cv::FileStorage::READ);
    if(!fs.isOpened())
    {
        ROS_ERROR_STREAM("failed to open file :" << filename);
        return false;
    }
    double demo2;

    std::cout<<filename<<std::endl;


    //config["fx"]>>demo1;
    fs[DISTCOEFF]>>demo2;
    //std::cout<<demo2<<std::endl;
    //std
    //std::cout<<config["fx"].as<string>()<<std::endl;
    fs.release();

    ROS_INFO("Read camera_config ok");
    //std::cout<<camMat<<std::endl;
    
}
/*
** \brief 将一维数组转为多维（2，3，4）矩阵
** @vec 一维数组（长度为4\9\16）
** @return MatrixXd的矩阵
*/
Eigen::MatrixXf IO::vector2Matrix(const vector<double> &vec) {
  Eigen::MatrixXf res;
  int n = sqrt(vec.size());     //矩阵维数
  if (vec.size() - n * n > 0) { // vector的size并非4,9,16
    ROS_ERROR_STREAM("Invalid parameter in vector2Matrix !");
    return res;
  }

  //初始化矩阵
  if (n == 2) { // 2×2矩阵
    res = Eigen::Matrix2f::Zero();
  } else if (n == 3) { // 3×3矩阵
    res = Eigen::Matrix3f::Zero();
  } else if (n == 4) { // 4×4矩阵
    res = Eigen::Matrix4f::Zero();
  } else if (n == 0) { // 4×4矩阵
    // ROS_ERROR_STREAM("vector size is null vector2Matrix !");
    return res;
  } else {
    ROS_ERROR_STREAM("Illegal parameter in vector2Matrix !");
    return res;
  }

  for (int i = 0; i < vec.size(); i++) {
    res(i / n, i % n) = vec[i];
  }

  return res;
}

/*
** \brief 读取json文件获取参数
** @path json 文件的路径
** @name 参数名称
** @return 参数的一维数组
*/
vector<double> IO::ReadJson(string path, string name) {
  // m_mutexoutjsonfile_.lock();
  vector<double> res;

  std::vector<std::string> sv;
  boost::algorithm::split(sv, name, boost::is_any_of("-")); // name为
  // cout<<name<<"sv.size"<<sv.size()<<endl;
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
  // m_mutexoutjsonfile_.unlock();
  return res;
}

double IO::ReadJson_Double(string path, string name){
  double stamp;
  cv::FileStorage fs(path,cv::FileStorage::READ);
  if(!fs.isOpened()) exit(0);
  fs[name] >> stamp;
}

string IO::ReadJson_String(string path, string name){
  string stamp;
  cv::FileStorage fs(path,cv::FileStorage::READ);
  if(!fs.isOpened()) exit(0);
  fs[name] >> stamp;
}

/*
** \brief 从文件夹中读取固定后缀的文件
** @root_path 文件夹根目录
** @suffix 制定的文件后缀
** @return 文件路径数组
*/
vector<string> IO::getfilepath(string root_path, string suffix) {
  vector<string> paths;
  set<string> sort_paths;
  struct dirent *dirp;
  DIR *dir = opendir(root_path.c_str());

  while ((dirp = readdir(dir)) != 0) {
    if (dirp->d_type == DT_REG) { // 文件
      string filename(dirp->d_name);
      if (strcmp(suffix.c_str(), filename
                                     .substr(filename.size() - suffix.size(),
                                             filename.size() - 1)
                                     .c_str()) == 0) {
        sort_paths.insert(root_path + "/" + filename);
      }
    }
  }
  set<string>::iterator it; //定义前向迭代器
  //中序遍历集合中的所有元素
  for (it = sort_paths.begin(); it != sort_paths.end(); it++) {
    paths.push_back(*it);
  }

  closedir(dir);
  return paths;
}

/*
** \brief 将标定结果写入json文件
** @json_path json 文件输出路径
** @camera_name_list 相机名称列表
** @camera_paras 相机参数数组
*/
bool IO::Write_Json(const string &json_path,
                    const vector<string> camera_name_list,
                    const vector<Para> &camera_paras) {
  // m_mutexoutjsonfile_.lock();
  std::fstream ff(json_path, std::ios::out);
  Json::Value root_value;

  for (unsigned i = 0; i < camera_name_list.size(); i++) {
    Json::Value sub_value;
    //外参
    for (int r = 0; r < 4; r++) {
      for (int c = 0; c < 4; c++) {
        sub_value["lidar2camera"].append(camera_paras[i].extrinsic(r, c));
      }
    }
    //内参
    for (int r = 0; r < 3; r++) {
      for (int c = 0; c < 3; c++) {
        sub_value["camera_matrix"].append(
            camera_paras[i].cameraMatrix.at<double>(r, c));
      }
    }

    //畸变参数
    Eigen::MatrixXd eigen_discoeff;
    cv2eigen(camera_paras[i].distCoeffs, eigen_discoeff);
    vector<double> tmp_discoeff = Matrix2vector(eigen_discoeff);
    for (int j = 0; j < tmp_discoeff.size(); j++) {
      sub_value["distortion_coefficients"].append(tmp_discoeff[j]);
    }

    root_value[camera_name_list[i]] = sub_value;
  }
  ff << root_value;
  ff.close();
  // m_mutexoutjsonfile_.unlock();
  return true;
}

/*
** \brief 读取输出的标定文件
** @json_path 标定文件路径
** @camera_name 每个相机的具体名称（FC、LF、RF）
** @camera_para 返回相机所需的全部参数
** @return 判断所需要读取的文件是否存在
*/
bool IO::Read_outparam(const string json_path, 
                       const string tf_yaml ,
                       const string camera_name,
                       Para &camera_para) {
  vector<double> vec_extrinsic;
  vec_extrinsic =ReadJson(json_path, camera_name + "-lidar2camera"); //读取二级目录 /slovepnp算
  // cv2eigen(vector2Matrix(vec_extrinsic),camera_para.extrinsic);
  Eigen::Matrix4f temp;
  Eigen::MatrixXf mat = vector2Matrix(vec_extrinsic);
  if (mat.size() == 0) {
    return false;
  }
  //camera_para.extrinsic = mat;
  //temp = mat * IO::tf_transform(tf_yaml);
  //camera_para.extrinsic = temp;
  // std::cout << "###################MAT####################" << std::endl;
  // std::cout << mat << std::endl;
  // std::cout << "###################END####################" << std::endl;
  // std::cout << temp << std::endl;
  // std::cout << "###################YAML####################" << std::endl;
  
  return true;
}

/*
** \brief 读取单个json文件的内参
** @jsonpath json文件的内参
** @return 内参
*/
Para IO::get_param_single(const string jsonpath) {
  Para camera_para;

  vector<double> vec_camera_matrix, tmp_distortion_coefficients;
  vec_camera_matrix = ReadJson(jsonpath, "camera_matrix");
  tmp_distortion_coefficients = ReadJson(jsonpath, "distortion_coefficients");
  vector<double> vec_distortion_coefficients(8, 0);
  for (int i = 0; i < min(tmp_distortion_coefficients.size(),
                          vec_distortion_coefficients.size());
       i++) {
    vec_distortion_coefficients[i] = tmp_distortion_coefficients[i];
  }

  if (vec_camera_matrix.size() != 9) {
    ROS_ERROR_STREAM("vec_camera_matrix size error :" << jsonpath);
    return camera_para;
  }
  // cv::Mat k {vec_camera_matrix};
  cv::Mat k(3, 3, CV_64F);

  for (int i = 0; i < vec_camera_matrix.size(); i++) {
    k.at<double>(i / 3, i % 3) = vec_camera_matrix[i];
  }

  cv::Mat d{vec_distortion_coefficients};

  camera_para.cameraMatrix = k.clone();
  camera_para.distCoeffs = d.clone();

  return camera_para;
}

/*
** \brief 根据参数文件路径读取相机的内参数和相机消息名称
** @para_path 配置文件路径
** @camera_name 每个相机的具体名称（FC、LF、RF）
** @json_path json文件夹路径
** @camera_para 返回相机所需的全部参数
** @return 判断所需要读取的文件是否存在
*/
bool IO::get_param(const string para_path, 
                   const string json_path,
                   const string camera_name, 
                   Para &camera_para) {
  
  cv::FileStorage camera_yaml(para_path, cv::FileStorage::READ);
  if (!camera_yaml.isOpened()) {
    ROS_ERROR_STREAM("failed to open file :" << para_path);
    return false;
  }
  string jsonfile;
  string yamlfile;
  camera_yaml["camera_topic_" + camera_name] >> camera_para.ros_topic;
  camera_yaml["intrinsic_file_" + camera_name] >> jsonfile;
  camera_yaml["yaml_file_" + camera_name] >> yamlfile;
  camera_yaml.release();
  string jsonpath = json_path + "/"+jsonfile; // json内参文件的路径
  string yamlpath = json_path + "/"+yamlfile;

  //std::cout<<jsonpath<<std::endl;

  std::cout<<yamlpath<<std::endl;

  ROS_INFO("Read camera_config ok");


  /* 编译能通过运行会报错
    cv::FileStorage fs_intric(jsonpath, cv::FileStorage::READ);
    if(!fs_intric.isOpened()){
                  ROS_ERROR_STREAM("failed to open file :"<<jsonpath);
                  return false;
          }
    cout<<"success open "<<jsonpath<<endl;

    fs_intric["camera_matrix"] >> camera_para.cameraMatrix;
    fs_intric["distortion_coefficients"] >> camera_para.distCoeffs;

    fs_intric.release();
  */
  // vector<double> vec_camera_matrix, vec_distortion_coefficients;
  vector<double> vec_camera_matrix, 
                 tmp_distortion_coefficients ,
                 rotation_matrix;
  double         cam_to_front ,
                 cam_to_left,
                 cam_to_right,
                 camera_height;
  Eigen::Matrix4d transform;
  Eigen::Matrix4d temp;
  Eigen::Matrix<double, 3,1> camera_translation;
  Eigen::Matrix3d camera_rotation;
 // vec_camera_matrix = ReadJson(jsonpath, "camera_matrix");//yaml对应处已找到内参相机内参

 // std::cout<<"$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$"<<std::endl;
  //std::cout<<""<<vec_camera_matrix[0]<<std::endl;
  YAML::Node config = YAML::LoadFile(yamlpath); 
  std::cout<<config["fx"].as<double>()<<std::endl;
  std::cout<<config["fy"].as<double>()<<std::endl;
  std::cout<<config["cx"].as<double>()<<std::endl;
  std::cout<<config["cy"].as<double>()<<std::endl;

  vec_camera_matrix.push_back(config["fx"].as<double>());
  vec_camera_matrix.push_back(0.0);
  vec_camera_matrix.push_back(config["cx"].as<double>());
  vec_camera_matrix.push_back(0.0);
  vec_camera_matrix.push_back(config["fy"].as<double>());
  vec_camera_matrix.push_back(config["cy"].as<double>());
  vec_camera_matrix.push_back(0.0);
  vec_camera_matrix.push_back(0.0);
  vec_camera_matrix.push_back(1.0);


  
/*
fx: 1944.778126  # 内参-焦距-fx, 单位 像素
fy: 1944.778126  # 内参-焦距-fy, 单位 像素
cx: 997.53677  # 内参-主点-cx, 单位 像素
cy: 610.1907179999999  # 内参-主点-cy, 单位 像素 
*/
    

  

  //vec_camera_matrix[0]=1944.778126;
  //ROS_INFO(vec_camera_matrix);
  //tmp_distortion_coefficients = ReadJson(jsonpath, "distortion_coefficients");//畸变系数找到yaml有

  std::cout << "^^^^^^^^^^^^^^^^Kc^^^^^^^^^^^^^^^"<< std::endl;
/*
  std::cout << tmp_distortion_coefficients[0] << std::endl;
  tmp_distortion_coefficients[0]=-0.164557;

  tmp_distortion_coefficients[1]=-0.075049;
  tmp_distortion_coefficients[4]=0;*/

  tmp_distortion_coefficients.push_back(config["kc2"].as<double>());
  tmp_distortion_coefficients.push_back(config["kc3"].as<double>());
  tmp_distortion_coefficients.push_back(config["kc4"].as<double>());
  tmp_distortion_coefficients.push_back(config["kc5"].as<double>());
  tmp_distortion_coefficients.push_back(0.0);
  std::cout << tmp_distortion_coefficients[0] << std::endl;
  std::cout << tmp_distortion_coefficients[1] << std::endl;
  std::cout << tmp_distortion_coefficients[2] << std::endl;
  std::cout << tmp_distortion_coefficients[3] << std::endl;
  std::cout << tmp_distortion_coefficients[4] << std::endl;



/*

kc2: -0.164557  # 内参-畸变系数-kc2
kc3: -0.075049  # 内参-畸变系数-kc3
kc4: 0  # 内参-畸变系数-kc4
kc5: 0  # 内参-畸变系数-kc5

   -0.49625882506370544,
    0.26961573958396912,
    0.0,
    0.0,
    -0.099497996270656586
*/


  std::cout <<"^^^^^^^^^^^^^^^^^^^^^rotation_matrix^^^^^^^^^^^^^^^^^^^^"<< std::endl;
  rotation_matrix = ReadJson(jsonpath, "rotation_matrix");//旋转向量未找到opencv Rodrigue
 // std::cout<<rotation_matrix[0]<<std::endl;


  //std::cout<<config["r_s2b"][0]<<std::endl;


	//旋转向量  模代表旋转角度
/*

r_s2b:  # 传感器到车体系的旋转, i.e. p_b = R(r_s2b)*p_s, p_b表示车体系下的点, R(.)表示把旋转向量转换成旋转矩阵的函数, p_s表示传感器系的点
  [-1.239343, -1.195186, 1.202236]



*/
	//转化成旋转矩阵
	//cv::Rodrigues(src, dst);


  Eigen::Vector3d rvec (config["r_s2b"][0].as<double>(),config["r_s2b"][1].as<double>(),config["r_s2b"][2].as<double>());   

  double n_norm = rvec.norm();
  Eigen::AngleAxisd rotation_vector (n_norm, rvec/n_norm);
  Eigen::Matrix3d rotm;
  rotm = rotation_vector.toRotationMatrix();
  //cout <<"asdsadadada"<< rotm << endl;
    //rotation_matrix=rotm;





    
  
   /*


    Eigen::AngleAxisd rotation_vector(alpha,Vector3d(x,y,z))
    Eigen::Matrix3d rotation_matrix;
   
    rotation_matrix=rotation_vector.matrix();
     //cout << "Rotation_matrix2"  rotation_matrix << endl;

*/

  
	//cv::Mat result = dst * vec;
	//std::cout << result << std::endl;

  //cam_to_front = ReadJson_Double(jsonpath,"cam_to_front");//暂未找到
 // cam_to_left = ReadJson_Double(jsonpath,"cam_to_left");//暂未找到
  //cam_to_right = ReadJson_Double(jsonpath,"cam_to_right");//暂未找到
  //camera_height = ReadJson_Double(jsonpath,"camera_height");//暂未找到 T是旋转r是平移
  vector<double> vec_distortion_coefficients(8, 0);
  for (int i = 0; i < min(tmp_distortion_coefficients.size(),
                          vec_distortion_coefficients.size());
       i++) {
    vec_distortion_coefficients[i] = tmp_distortion_coefficients[i];
  }


  // Eigen::MatrixXd eigen_camera_matrix=vector2Matrix(vec_camera_matrix);
  // Eigen::MatrixXd
  // eigen_distortion_coefficients=vector2Matrix(vec_distortion_coefficients);

  if (vec_camera_matrix.size() != 9) {
    ROS_ERROR_STREAM("vec_camera_matrix size error :" << jsonpath);
    return false;
  }
  ROS_INFO("Read Camera_Jeson ok");
  // cv::Mat k {vec_camera_matrix};
  cv::Mat k(3, 3, CV_64F);
  //cv::Mat camera_rotation(3, 3, CV_64F);
  cv::Mat j(3, 1, CV_64F);
  cv::Mat a(4, 4, CV_64F);
 
 /*
  if (rotation_matrix.size() != 9) {
        ROS_ERROR_STREAM("vec_camera_matrix size error :" );
        return false;
    }

  for (int i = 0; i < rotation_matrix.size(); i++) {
    camera_rotation(i / 3, i % 3) = rotation_matrix[i];
  }
  */
   for (int i = 0; i < vec_camera_matrix.size(); i++) {
    k.at<double>(i / 3, i % 3) = vec_camera_matrix[i];
  }
  std::cout <<"^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^"<< std::endl;

  camera_rotation=rotm.transpose();

  //const string yaparapath = "/tmp/calib_/calibration/camera_rear_mid_down.yaml";
  //readPara(yaparapath);
  std::cout <<"^^^^^^^^^^^^^^^^camera_rotation^^^^^^^^^^^^^^^"<< std::endl;
  std::cout << camera_rotation << std::endl;

   //cout << rotm.transpose()<< endl;
  /*
  camera_translation(0,0)= -cam_to_front+4;
  camera_translation(1,0)= (cam_to_left+3+cam_to_right+3)/2-cam_to_left-3;
  camera_translation(2,0)= camera_height+1;
  */
/*
  camera_translation(0,0)= config["t_s2b"][0];
  camera_translation(1,0)= config["t_s2b"][1];
  camera_translation(2,0)= config["t_s2b"][2];
*/
 camera_translation(0,0)= config["t_s2b"][0].as<double>();
  camera_translation(1,0)= config["t_s2b"][1].as<double>();
  camera_translation(2,0)= config["t_s2b"][2].as<double>();
  

/*
t_s2b:  # 传感器到车体的平移, 单位 m
  [-0.241181, 0.03795765, 1.723368]
  camera_translation(0,0)= -cam_to_front;
  camera_translation(1,0)= (cam_to_left+cam_to_right)/2-cam_to_left;
  camera_translation(2,0)= camera_height;

 */
  
/*
  auto _rotation_vector = reader.get<std::vector<float>>("rotation_vector", false, std::vector<float>(3, 0.f));
  auto _translation = reader.get<std::vector<float>>("translation", false, std::vector<float>(3, 0.f));
  auto output_channel = reader.get<std::size_t>("output_channel");

  Eigen::Vector3f rotation_vector{_rotation_vector.data()};
  Eigen::Vector3f translation{_translation.data()};
  auto transform = Eigen::Isometry3f::Identity();
  transform.translate(translation).rotate(Eigen::AngleAxisf(rotation_vector.norm(),rotation_vector.normalized()));

          if (!lidar_process::transformation::register_transformation(
                  lidar_manager_, input_channel, transform.matrix(),
                  output_channel)) {

*/
  /*


  camera_translation(0,0)= -0.953459;
  camera_translation(1,0)= 0.03578284;
  camera_translation(2,0)= 0.6572103;
*/

/*
  camera_translation(0,0)= cam_to_front;
  camera_translation(1,0)= (cam_to_left+cam_to_right)/2-cam_to_left;
  camera_translation(2,0)= camera_height;*/

  std::cout <<"^^^^^^^^^^^^^^^^camera_translation^^^^^^^^^^^^^^^"<<std::endl;
  std::cout << camera_translation << std::endl;
  std::cout <<"^^^^^^^^^^^^^^^^camera_rotation^^^^^^^^^^^^^^^"<<std::endl;
  std::cout << camera_rotation << std::endl;
   std::cout <<"^^^^^^^^^^^^^^^^camera_translation^^^^^^^^^^^^^^^"<<std::endl;
  camera_translation = -camera_rotation *camera_translation;

/*
  camera_translation(0,0)= -cam_to_front;
  camera_translation(1,0)= (cam_to_left+cam_to_right)/2-cam_to_left;
  camera_translation(2,0)= camera_height;
*/

  
  std::cout << camera_translation << std::endl;
  transform(0,0)=camera_rotation(0,0);
  transform(0,1)=camera_rotation(0,1);
  transform(0,2)=camera_rotation(0,2);
  transform(0,3)= camera_translation(0,0);
  transform(1,0)=camera_rotation(1,0);
  transform(1,1)=camera_rotation(1,1);
  transform(1,2)=camera_rotation(1,2);
  transform(1,3)=camera_translation(1,0);
  transform(2,0)=camera_rotation(2,0);
  transform(2,1)=camera_rotation(2,1);
  transform(2,2)=camera_rotation(2,2);
  transform(2,3)=camera_translation(2,0);
  transform(3,0)=0;
  transform(3,1)=0;
  transform(3,2)=0;
  transform(3,3)=1;
  std::cout << "^^^^^^^^^^^^^^^^transform^^^^^^^^^^^^^^^"<< std::endl;
  std::cout << transform << std::endl;
  std::cout <<"^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^"<< std::endl;
  //a = k * a;
  cv::Mat d{vec_distortion_coefficients};
  
  
  //std::cout << tf_transform(tf_yaml) << std::endl;
  camera_para.extrinsic = transform;
  camera_para.cameraMatrix = k.clone();//走yaml相机内参
  camera_para.distCoeffs = d.clone();//走畸变系数
  return true;
}

/*
** \brief 根据参数文件路径和相机消息名称将外参写入到json中
** @para_path 外参保存路径
** @camera_names 每个相机的具体名称（FC、LF、RF）
** @camera_paras 返回相机所需的全部参数
** @return 判断标定文件是否成功写入
*/
bool IO::write_param(const string para_path, const vector<string> camera_names,
                     const vector<Para> &camera_paras) {
  cv::FileStorage fs_extrinsic(para_path, cv::FileStorage::WRITE);
  if (!fs_extrinsic.isOpened()) {
    ROS_ERROR_STREAM("failed to write file :" << para_path);
    return false;
  }

  for (int i = 0; i < camera_names.size(); i++) {
    fs_extrinsic << camera_names[i] << "{:";
    Mat mat_extrinsic;
    eigen2cv(camera_paras[i].extrinsic, mat_extrinsic);
    fs_extrinsic << "lidar2camera" << mat_extrinsic;
    fs_extrinsic << "camera_matrix" << camera_paras[i].cameraMatrix;
    fs_extrinsic << "distortion_coefficients" << camera_paras[i].distCoeffs;
    fs_extrinsic << "}";
  }

  fs_extrinsic.release();
  return true;
}

/*
** \brief Mat转1维vector
** @Mat 输入的mat
** @return vector数组
*/
vector<double> IO::mat2ector(const Mat &_t1f) {
  Mat t1f;
  _t1f.convertTo(t1f, CV_64F);
  return (vector<double>)(t1f.reshape(1, 1));
}


Eigen::Matrix4d IO::tf_transform( string addr){
  ROS_INFO("tf_transfim!!!!!");
  YAML::Node config = YAML::LoadFile(addr);
  ROS_INFO("tf_transfim######");
  double x,y,z,rotation_x,rotation_y,rotation_z,rotation_w;
  Eigen::Matrix3d rotation_matrix;
  Eigen::Matrix4d transform;
  for (std::size_t i = 0; i < config.size(); ++i) {
    if(config[i]["frame_id"].as<std::string>() == "car"){
      x = config[i]["args"][0].as<double>();
      y = config[i]["args"][1].as<double>();
      z = config[i]["args"][2].as<double>();
      rotation_x = config[i]["args"][3].as<double>();
      rotation_y = config[i]["args"][4].as<double>();
      rotation_z = config[i]["args"][5].as<double>();
      rotation_w = config[i]["args"][6].as<double>();
      /*std::cout << "tf_yaml=======" << std::endl 
      << x << std::endl
      << y << std::endl
      << z << std::endl
      << rotation_x << std::endl
      << rotation_y << std::endl
      << rotation_z << std::endl
      << rotation_w << std::endl;*/
    }
    /*std::cout << "i======" << i << std::endl;*/
  }
  Eigen::Quaterniond quaternion(rotation_w,rotation_x,rotation_y,rotation_z);
  //std::cout << "$$$$$$$$$$$$$$$$$$$quaternion$$$$$$$$$$$$$$$$$$" << std::endl;
  //std::cout << quaternion << std::endl;
  rotation_matrix=quaternion.matrix();
  std::cout << "$$$$$$$$$$$$$$$$$$$rotation_matrix$$$$$$$$$$$$$$$$$$" << std::endl;
  std::cout << rotation_matrix << std::endl;
  /*std::cout << rotation_matrix << std::endl;*/
  transform(0,0)=rotation_matrix(0,0);
  transform(0,1)=rotation_matrix(0,1);
  transform(0,2)=rotation_matrix(0,2);
  transform(0,3)=x;
  transform(1,0)=rotation_matrix(1,0);
  transform(1,1)=rotation_matrix(1,1);
  transform(1,2)=rotation_matrix(1,2);
  transform(1,3)=y;
  transform(2,0)=rotation_matrix(2,0);
  transform(2,1)=rotation_matrix(2,1);
  transform(2,2)=rotation_matrix(2,2);
  transform(2,3)=z;
  transform(3,0)=0;
  transform(3,1)=0;
  transform(3,2)=0;
  transform(3,3)=1;
  std::cout << "$$$$$$$$$$$$$$$$$$$transform$$$$$$$$$$$$$$$$$$" << std::endl;
  std::cout << transform << std::endl;
  return transform;

}
