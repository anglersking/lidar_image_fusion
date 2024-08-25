#include "robosense_pointcloud/transform_extractor.h"
#include <boost/filesystem.hpp>
#include <boost/foreach.hpp>
#include <fstream>
#include <iostream>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <string>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/static_transform_broadcaster.h>

#define TF_STATIC_INDENTITY_MATRIX
#define OUTPUT_PCD_FILE_COMPRESSED
#define foreach BOOST_FOREACH

//"frame_id"与"child_frame_id"和标定文件tf.yaml对应,不要修改
const std::string frame_id = "car";
const std::string child_frame_id = "velodyne_center";

void process(std::string bagname, std::string outputfolder)
{
  rosbag::Bag bag;
  ROS_INFO_STREAM("bagname: " << bagname);
  ROS_INFO_STREAM("outputfolder: " << outputfolder);
  bool tf_init = false; // lidar 到 car 变换参数初始化成功的标志位
  bag.open(bagname, rosbag::bagmode::Read);
  geometry_msgs::TransformStamped tf;

#ifndef TF_STATIC_INDENTITY_MATRIX
  std::vector<std::string> wanted_topics = {"/tf_static",
                                            "/sensor/lidar/multi_scan"};
#else
  tf.header.frame_id = frame_id;
  tf.child_frame_id = child_frame_id;
  tf.transform.translation.x = 0;
  tf.transform.translation.y = 0;
  tf.transform.translation.z = 0;
  tf.transform.rotation.x = 0;
  tf.transform.rotation.y = 0;
  tf.transform.rotation.z = 0;
  tf.transform.rotation.w = 1;
  tf_init = true;
  ROS_INFO_STREAM("TF init ok, indentity matrix! "
                  << "translation [x,y,z]:["
                  << tf.transform.translation.x << ","
                  << tf.transform.translation.y << ","
                  << tf.transform.translation.z << "]; "
                  << "rotation [x,y,z,w]:["
                  << tf.transform.rotation.x << ","
                  << tf.transform.rotation.y << ","
                  << tf.transform.rotation.z << ","
                  << tf.transform.rotation.w << "]");
  std::vector<std::string> wanted_topics = {"/sensor/lidar/multi_scan"};
#endif

  rosbag::View view(bag, rosbag::TopicQuery(wanted_topics));
  robosense_pointcloud::Transform_Extractor cvt;
  foreach (rosbag::MessageInstance const m, view)
  {
#ifndef TF_STATIC_INDENTITY_MATRIX
    tf2_msgs::TFMessage::ConstPtr tf_ptr = m.instantiate<tf2_msgs::TFMessage>();
    if (tf_ptr != NULL)
    {
      if (tf_init)
      {
        continue;
      }
      for (int i = 0; i < tf_ptr->transforms.size(); i++)
      {
        if (tf_ptr->transforms[i].header.frame_id == frame_id &&
            tf_ptr->transforms[i].child_frame_id == child_frame_id)
        {
          tf = tf_ptr->transforms[i];
          tf_init = true;
          ROS_INFO_STREAM("TF init ok ! "
                          << "trans [x,y,z] : ["
                          << tf.transform.translation.x << ","
                          << tf.transform.translation.y << ","
                          << tf.transform.translation.z << "] "
                          << "rota [x,y,z,w] : ["
                          << tf.transform.rotation.x << ","
                          << tf.transform.rotation.y << ","
                          << tf.transform.rotation.z << ","
                          << tf.transform.rotation.w << "]");
        }
      }
    }
#endif

    sensor_interface_msgs::LidarScan::ConstPtr scan_ptr =
        m.instantiate<sensor_interface_msgs::LidarScan>();
    if (scan_ptr != NULL)
    {
      if (!tf_init)
      {
        ROS_WARN_STREAM("Wait transform from " << child_frame_id << " to "
                                               << frame_id);
        continue;
      }
      cvt.extractorScan(scan_ptr, tf);
      std::string filename = outputfolder + "/" +
                             std::to_string(cvt.outMsg->header.stamp) + ".pcd";
      ROS_INFO_STREAM("Save pcd: " << filename);

#ifdef OUTPUT_PCD_FILE_COMPRESSED
      pcl::io::savePCDFileBinaryCompressed(filename, *(cvt.outMsg));
#else
      pcl::io::savePCDFileASCII(filename, *(cvt.outMsg));
#endif
    }
  }

  bag.close();
  return;
}

// 如果参数个数为1,则为bag路径
// 如果参数个数为2,则为生成pcd的目标保存目录
int main(int argc, char *argv[])
{
  std::string bagname, outputfolder;
  if (argc <= 1)
  {
    std::cerr << "Too lettle params !" << std::endl;
    return 0;
  }
  else if (argc > 3)
  {
    std::cerr << "Too many params !" << std::endl;
    return 0;
  }
  else
  {
    std::string bagfile = argv[1];
    boost::filesystem::path bag_path(bagfile);
    std::string extensionname = bag_path.extension().string();
    bagname = bagfile;
    if (extensionname == ".bag")
    {
      bagname = bagfile;
    }
    else
    {
      std::cerr << "The first parameter should be the bag path !" << std::endl;
      return 0;
    }
    if (argc == 3)
    {
      outputfolder = argv[2];
    }
    else
    {
      outputfolder = bag_path.parent_path().string();
      if (!boost::filesystem::exists(outputfolder.c_str()))
      {
        boost::filesystem::create_directories(outputfolder);
      }
    }
  }

  process(bagname, outputfolder);
  return 0;
}
