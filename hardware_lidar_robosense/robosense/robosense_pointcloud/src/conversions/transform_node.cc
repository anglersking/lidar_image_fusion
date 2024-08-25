/** \file

    This ROS node transforms raw Robosense LIDAR packets to PointCloud2
    in the /enu frame of reference.

*/

#include "robosense_pointcloud/transform.h"
//#include "robosense_pointcloud/robosense_pointcloud_log.h"
#include <ros/package.h> // 查找packet路径
#include <ros/ros.h>
/** Main node entry point. */
int main(int argc, char **argv) {
  ros::init(argc, argv, "transform_node");
  robosense_pointcloud::Transform transform(ros::NodeHandle(),
                                           ros::NodeHandle("~"));

  // handle callbacks until shut down
  ros::spin();

  return 0;
}