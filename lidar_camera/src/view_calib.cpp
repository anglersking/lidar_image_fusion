
//订阅图像和点云消息，调用tf参数实现投影,对标定结果定性验证
#include <ros/package.h> // 查找packet路径
#include <ros/publisher.h>
#include <ros/ros.h>

#include <ros/subscriber.h>
#include <yaml-cpp/yaml.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>

#include <pcl/registration/transformation_estimation.h>

#include <pcl/io/pcd_io.h>   // PCL的PCD格式文件的输入输出头文件
#include <pcl/point_types.h> // PCL对各种格式的点的支持头文件
#include <pcl_conversions/pcl_conversions.h> // pcl::fromROSMsg
#include <sensor_msgs/PointCloud2.h>
#include <sensor_interface_msgs/Pointcloud2.h>

#include <stdlib.h>
#include "io.h"
#include "evaluation.h"

#include <iostream>
#include "plot.h"
//#include "projector_tools.h"

using namespace std;
using namespace cv;
const string frame_id = "car";

std::string jeson_pa;



typedef XYZTIRA PointT;
typedef pcl::PointCloud<PointT> CloudT;
typedef CloudT::Ptr CloudPtr;

vector<Para> camera_para(camera_size);                    //参数
vector<image_transport::Publisher> ans_pubs(camera_size); // topic 句柄

/*
** \brief 从点云消息中初始化点云
** @cloud_msg 点云消息
** @center_points lidar点云
** @return 判断所需要读取的文件是否存在
*/
bool init_pointcloud(const sensor_msgs::PointCloud2::ConstPtr &cloud_msg,
                     CloudPtr &center_points) {
  CloudPtr cloud(new CloudT());
  pcl::fromROSMsg(*cloud_msg, *cloud);
  const int center_lidar_lines = 128; // lidar线束

  if (center_lidar_lines > cloud->height) {
    ROS_ERROR("lidar lines inadequate!");
    return false;
  }

  for (int i = 0; i < center_lidar_lines; i++) { //只读取前32条线
    for (int j = 0; j < cloud->width; j++) {
      PointT p;
      int pointid = i * cloud->width + j;
      p.x = cloud->points[pointid].x;
      p.y = cloud->points[pointid].y;
      p.z = cloud->points[pointid].z;
      p.intensity = cloud->points[pointid].intensity;
      center_points->points.push_back(p);
    }
  }
  //std::cout<<"lidar init over"<<std::endl;
  return true;
}



/*
** \brief 临时的参数读取文件，参数管理系统过渡时期使用，返回参数文件夹地址。/param 优先级高于 /param_aggregator
** @nh ros NodeHandle
** @return 参数文件夹路径
*/
std::string wait_param(ros::NodeHandle &nh) {
  const vector<std::string> paramnames_old={ "/param_path" , "/whoami" , "/what_date" };
  const vector<std::string> paramnames_new={ "/CAM_CALIB_DIR" ,"/WHICH_CAR" };
  vector<std::string> paramvalue_old(paramnames_old.size());
  vector<std::string> paramvalue_new(paramnames_new.size());

  std::string ans;
  bool is_waiting = false;

  while (ros::ok()) {
    if(is_waiting) {
      ROS_WARN("waiting for params '/param' or '/param_aggregator' ... ");
    }

    int paramnums_old=0;
    int paramnums_new=0;

    for(int i=0;i<paramnames_old.size();i++) {
      nh.param(paramnames_old[i], paramvalue_old[i], std::string(""));
      if (!paramvalue_old[i].empty()) {
        paramnums_old++;
      }
      if(paramnums_old == paramvalue_old.size()) {
        ROS_INFO("resoluting /whoami ...");
        ans = paramvalue_old[0] + "/cars" + "/" + paramvalue_old[1] + '/' + paramvalue_old[2];
        return ans;
      }
    }

    for(int i=0;i<paramnames_new.size();i++) {
      nh.param(paramnames_new[i], paramvalue_new[i], std::string(""));
      if (!paramvalue_new[i].empty()) {
        paramnums_new++;
      }
      if(paramnums_new == paramvalue_new.size()) {
        ROS_INFO("resoluting /param_aggregator ...");
        ans = "/tmp" + paramvalue_new[0] + "/" + paramvalue_new[1] + "/";;
        return ans;
      }
    }

    is_waiting = true;
    usleep(1e5); //100ms
  }
  return ans;
}



//回调函数
void calibCallback(const sensor_msgs::CompressedImageConstPtr &image0_msg,
                  //  const sensor_msgs::CompressedImageConstPtr &image1_msg,
                  //  const sensor_msgs::CompressedImageConstPtr &image2_msg,
                  //  const sensor_msgs::CompressedImageConstPtr &image3_msg,
                  //  const sensor_msgs::CompressedImageConstPtr &image4_msg,
                  //  const sensor_msgs::CompressedImageConstPtr &image5_msg,
                  //  const sensor_msgs::CompressedImageConstPtr &image6_msg,
                  //  const sensor_msgs::CompressedImageConstPtr &image7_msg,
                   const sensor_msgs::PointCloud2::ConstPtr &lidar_msg) {
 // std::cout<<"callback in"<<std::endl;
  //图片数据
   std::vector<cv::Mat> images; //原始图片
  // vector<cv::Mat> image_undistorts(camera_size);  //去畸变图片
  try {
    
    images.push_back(cv::imdecode(cv::Mat(image0_msg->data), 1));
   // std::cout<<"image decode over"<<std::endl;
    // images.push_back(cv::imdecode(cv::Mat(image1_msg->data), 1));
    // images.push_back(cv::imdecode(cv::Mat(image2_msg->data), 1));
    // images.push_back(cv::imdecode(cv::Mat(image3_msg->data), 1));
    // images.push_back(cv::imdecode(cv::Mat(image4_msg->data), 1));
    // images.push_back(cv::imdecode(cv::Mat(image5_msg->data), 1));
    // images.push_back(cv::imdecode(cv::Mat(image6_msg->data), 1));
    // images.push_back(cv::imdecode(cv::Mat(image7_msg->data), 1));
  } catch (cv_bridge::Exception &e) {
    ROS_ERROR("Could not convert to image!");
    return;
  }
  // 32lidar数据
  CloudPtr lidar_data(new CloudT());
  if (!init_pointcloud(lidar_msg, lidar_data)) {
    return;
  }
  // 更新标定文件
 
  // 开始投影
//std::cout<<"image ash "<<std::endl;
  Evaluate evalute;
  evalute.all_cloudproject(lidar_data, false, camera_para,images);
  // cv::namedWindow("test",cv::WINDOW_NORMAL);
  // cv::imshow("test",merge_image(1280,720,images));
  // cv::waitKey(1);
  // msg 发布
  for (int i = 0; i < camera_size; i++) {

    auto ans_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", images[i])
                       .toImageMsg(); // project
    ans_pubs[i].publish(ans_msg);
    ROS_INFO("publish");
  }

  return;
}

int main(int argc, char **argv) {

  for(int i=0;i<argc;i++)
	{
		std::cout<<"argument["<<i<<"] is: "<<argv[i]<<std::endl;
	}

  ros::init(argc, argv, "viewcalib");
  ros::NodeHandle node;


  string json_floder,jsonpath;
  //const string parapath = ros::package::getPath("lidar_camera") +
   //                       "/config/camera.yaml";

  const string parapath = "/config/camera.yaml";
                          
  //std::string vehicle_config = getenv("WHICH_CAR");
  //std::cout << "vehicle_config ====" << vehicle_config << std::endl;

  //jsonpath = "/calib/"+vehicle_config;//online

  jsonpath = "/tmp/calib_";//+vehicle_config;//online
  std::cout<<"parapath "<<parapath<<std::endl;
  std::cout<<"yamlpath "<<jsonpath<<std::endl;
  YAML::Node config = YAML::LoadFile(parapath); 

  string cam_topic_name=argv[1];

  
  std::cout<<config["camera_topic_"+cam_topic_name].as<string>()<<std::endl;
  std::cout << "lidar_topic_name:/sensor/lidar/point_cloud_no_compensate" << std::endl;

  //ROS_INFO("START");

  for (int i = 0; i < camera_size; i++) {
    // std::cout << "camera_names" <<camera_names[i] << std::endl;
    //  std::cout << "camera_para" <<camera_para[i] << std::endl;
    if (!IO::get_param(parapath,jsonpath, argv[1], camera_para[i])) {
      ROS_ERROR_STREAM("Failed to open param files!");
      return 0;
    } else {
      
    }
  }
 
  //读取标定结果，更新标定结果，只有标定完成后才有json文件
  ROS_INFO("Read para over");
  

  image_transport::ImageTransport it0(node);
  //image_transport::ImageTransport it1(node);
  
  //image_transport::ImageTransport it2(node);
  
  //image_transport::ImageTransport it3(node);
  //image_transport::ImageTransport it4(node);
  //image_transport::ImageTransport it5(node);
  // image_transport::ImageTransport it6(node);
  // image_transport::ImageTransport it7(node);
                                                                                                                                                                                                                    
  // ans_pubs[0] = it0.advertise(
  //     "/lidar_calib_calibroom/" + camera_names[0] + "_project", 1);

  ans_pubs[0] = it0.advertise(
      "/lidar_calib_calibroom/lidar_image_fit_project", 1);
  // ans_pubs[1] = it1.advertise(
  //     "/lidar_calib_calibroom/camera_rear_mid/_project", 1);
  
  // ans_pubs[2] = it2.advertise(
  //     "/lidar_calib_calibroom/" + camera_names[2] + "_project", 1);

  
  // ans_pubs[3] = it3.advertise(
  //     "/lidar_calib_calibroom/" + camera_names[3] + "_project", 1);
  // ans_pubs[4] = it4.advertise(
  //     "/lidar_calib_calibroom/" + camera_names[4] + "_project", 1);
  // ans_pubs[5] = it5.advertise(
  //     "/lidar_calib_calibroom/" + camera_names[5] + "_project", 1);
  // ans_pubs[6] = it6.advertise(
  //     "/lidar_calib_calibroom/" + camera_names[6] + "_project", 1);
  // ans_pubs[7] = it7.advertise(
  //     "/lidar_calib_calibroom/" + camera_names[7] + "_project", 1);


 // std::cout << "topic_name" <<camera_names[0] << std::endl;
  // std::cout << "topic_name" <<camera_names[1] << std::endl;
  // std::cout << "topic_name" <<camera_names[2] << std::endl;

  // std::cout << "topic_name" <<camera_names[3] << std::endl;
  // std::cout << "topic_name" <<camera_names[4] << std::endl;
   

 
 
  message_filters::Subscriber<sensor_msgs::CompressedImage> image_sub0(
      node, config["camera_topic_"+cam_topic_name].as<string>(), 1);
  // message_filters::Subscriber<sensor_msgs::CompressedImage> image_sub1(
  //     node, "/sensor/camera_rear_mid/image_raw/compressed", 1);
  // message_filters::Subscriber<sensor_msgs::CompressedImage> image_sub2(
  //      node,  "/sensor/camera_rear_mid_down/image_raw/compressed", 1);
  // message_filters::Subscriber<sensor_msgs::CompressedImage> image_sub3(
  //      node, "/sensor/camera_front_wide/image_raw/compressed", 1);
  // message_filters::Subscriber<sensor_msgs::CompressedImage> image_sub4(
  //      node,"/sensor/camera_right_front/image_raw/compressed", 1);
  // message_filters::Subscriber<sensor_msgs::CompressedImage> image_sub5(
  //      node, "/sensor/camera_left_front/image_raw/compressed", 1);
  // message_filters::Subscriber<sensor_msgs::CompressedImage> image_sub6(
  //     node, "/sensor/cameraLR100/image_raw/compressed", 1);
  // message_filters::Subscriber<sensor_msgs::CompressedImage> image_sub7(
  //     node,  "/sensor/cameraRR100/image_raw/compressed", 1);
  
  //std::cout<<"msgfilter image over"<<std::endl;

  message_filters::Subscriber<sensor_msgs::PointCloud2> lidar_sub(
      node, "/sensor/lidar/point_cloud_no_compensate", 1); // lidar 消息
 // std::cout<<"msgfilter lidar over"<<std::endl;
  typedef message_filters::sync_policies::ApproximateTime<
      sensor_msgs::CompressedImage,// sensor_msgs::CompressedImage,
      // sensor_msgs::CompressedImage, sensor_msgs::CompressedImage,
      // sensor_msgs::CompressedImage, sensor_msgs::CompressedImage,
      // sensor_msgs::CompressedImage, sensor_msgs::CompressedImage,
      sensor_msgs::PointCloud2>
      MySyncPolicy;
  //std::cout<<"msgfilter time together over"<<std::endl;   
  message_filters::Synchronizer<MySyncPolicy> sync(
      MySyncPolicy(50), image_sub0, //image_sub1,image_sub2,image_sub3,image_sub4, image_sub5,
      // image_sub2, image_sub3,image_sub4, image_sub5, image_sub6 ,image_sub7, 
      lidar_sub);

  sync.registerCallback(
      boost::bind(&calibCallback, _1, _2//, _3, _4,_5,_6,_7
      // , _4, _5, _6, _7, _8 ,_9
      )); // 回调

  
  ros::spin();
}
