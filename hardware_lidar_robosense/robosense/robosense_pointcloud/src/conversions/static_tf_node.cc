#include <ros/ros.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <vector>
#include <yaml-cpp/yaml.h>
#include <Eigen/Dense>
using namespace std;

#if 0
/*
** \brief 读取某个rosparam变量值，若不存在则等待
** @nh ros NodeHandle
** @param_name 某个rosparam变量名，可通过 rosparam list 查看
** @return 变量值
*/
std::string wait_param(ros::NodeHandle &nh, const std::string &param_name) {
  std::string ans;
  bool is_waiting = false;
  while (ans.empty() && ros::ok()) {
    if (is_waiting) {
      ROS_WARN("waiting for ${%s}", param_name.c_str());
    }
    nh.param(param_name, ans, std::string(""));
    usleep(1e5); // 100ms
    is_waiting = true;
  }
  return ans;
}

/*
** \brief
*临时的参数读取文件，参数管理系统过渡时期使用，返回参数文件夹地址。/param
*优先级高于 /param_aggregator
** @nh ros NodeHandle
** @return 参数文件夹路径
*/
std::string wait_param(ros::NodeHandle &nh) {
  const vector<std::string> paramnames_old = {"/param_path", "/whoami",
                                              "/what_date"};
  const vector<std::string> paramnames_new = {"/CALIB_FOLDER", "/WHICH_CAR"};
  vector<std::string> paramvalue_old(paramnames_old.size());
  vector<std::string> paramvalue_new(paramnames_new.size());

  std::string ans;
  bool is_waiting = false;

  while (ros::ok()) {
    if (is_waiting) {
      ROS_WARN("static_tf_node waiting for params '/param' or "
               "'/param_aggregator' ... ");
    }

    unsigned int paramnums_old = 0;
    unsigned int paramnums_new = 0;

    for (unsigned int i = 0; i < paramnames_new.size(); i++) {
      nh.param(paramnames_new[i], paramvalue_new[i], std::string(""));
      if (!paramvalue_new[i].empty()) {
        paramnums_new++;
      }
      if (paramnums_new == paramvalue_new.size()) {
        ROS_DEBUG("resoluting /param_aggregator ...");
        ans = paramvalue_new[0] + "/" + paramvalue_new[1] + "/";
        return ans;
      }
    }

    for (unsigned int i = 0; i < paramnames_old.size(); i++) {
      nh.param(paramnames_old[i], paramvalue_old[i], std::string(""));
      if (!paramvalue_old[i].empty()) {
        paramnums_old++;
      }
      if (paramnums_old == paramvalue_old.size()) {
        ROS_DEBUG("resoluting /whoami ...");
        ans = paramvalue_old[0] + "/cars" + "/" + paramvalue_old[1] + '/' +
              paramvalue_old[2];
        return ans;
      }
    }

    is_waiting = true;
    usleep(1e5); // 100ms
  }
  return ans;
}
#endif

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "static_tf_node");
  ros::NodeHandle nh;

  // std::string dir = "/tmp" + wait_param(nh, "/CAM_CALIB_DIR")+ "/" +
  // wait_param(nh, "/WHICH_CAR"); //add by gyf
  // std::string dir = wait_param(nh);

  std::string which_car = getenv("WHICH_CAR");
  if (which_car.empty()) {
      ROS_ERROR("can't find env WHICH_CAR!");
      exit(-1);
  }
  std::string dir = "/calib/"+ which_car + "/";

  nh.setParam("/robosense_paramdir", dir);
  // YAML::Node config = YAML::LoadFile(dir + "/tf.yaml");

  tf2_ros::StaticTransformBroadcaster tf_broadcaster;
  geometry_msgs::TransformStamped msg;
  // std::cout << "read tf from: " << dir + "/tf.yaml\n" << std::endl;

  bool use_tf_yaml_;
  nh.param("use_tf_yaml", use_tf_yaml_, true);
  if (use_tf_yaml_) {
    ROS_ERROR_STREAM("try to find" + dir + "tf.yaml!");

    try {
      YAML::Node config = YAML::LoadFile(dir + "tf.yaml");
      ROS_ERROR_STREAM("read tf from: " << dir + "tf.yaml");
      for (int i = config.size() - 1; i >= 0; i--) {
        msg.transform.translation.x = config[i]["args"][0].as<double>();
        msg.transform.translation.y = config[i]["args"][1].as<double>();
        msg.transform.translation.z = config[i]["args"][2].as<double>();
        msg.transform.rotation.x = config[i]["args"][3].as<double>();
        msg.transform.rotation.y = config[i]["args"][4].as<double>();
        msg.transform.rotation.z = config[i]["args"][5].as<double>();
        msg.transform.rotation.w = config[i]["args"][6].as<double>();
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = config[i]["frame_id"].as<std::string>();
        msg.child_frame_id = config[i]["child_frame_id"].as<std::string>();
        if (msg.header.frame_id == "velodyne_center" &&
            msg.transform.translation.z != 0) {
          while (ros::ok) {
            ROS_ERROR_STREAM("Calibration file does not match driver");
            usleep(1e5); // 100ms
          }
        }

        tf_broadcaster.sendTransform(msg);
      }
    }
    catch(const YAML::Exception& e) {
      ROS_ERROR_STREAM("!!!Can't find " << dir + "tf.yaml");
      exit(-1);
    }
  }
  else {
    ROS_ERROR_STREAM("try to find" + dir + "calibration/lidar.yaml!");

    try {
      YAML::Node config = YAML::LoadFile(dir + "calibration/lidar.yaml");
      ROS_ERROR_STREAM("read tf from: " << dir + "calibration/lidar.yaml");
      msg.transform.translation.x = config["t_s2b"][0].as<double>();
      msg.transform.translation.y = config["t_s2b"][1].as<double>();
      msg.transform.translation.z = config["t_s2b"][2].as<double>();

      Eigen::Vector3d to_r_vector(config["r_s2b"][0].as<double>(),config["r_s2b"][1].as<double>(),config["r_s2b"][2].as<double>());
      Eigen::AngleAxisd r_vector(to_r_vector.norm(),Eigen::Vector3d(to_r_vector/to_r_vector.norm()));
      Eigen::Quaterniond q_data = Eigen::Quaterniond(r_vector);
      msg.transform.rotation.x = q_data.x();
      msg.transform.rotation.y = q_data.y();
      msg.transform.rotation.z = q_data.z();
      msg.transform.rotation.w = q_data.w();
      msg.header.stamp = ros::Time::now();
      msg.header.frame_id = "car";
      msg.child_frame_id = "velodyne_center";
      tf_broadcaster.sendTransform(msg);
    }
    catch(const YAML::Exception& e) {
      ROS_ERROR_STREAM("!!!Can't find " + dir + "calibration/lidar.yaml!");
      exit(-1);
    }
  }

  ROS_ERROR_STREAM("TF init ok, indentity matrix! "
                  << "translation [x,y,z]:["
                  << msg.transform.translation.x << ","
                  << msg.transform.translation.y << ","
                  << msg.transform.translation.z << "]; "
                  << "rotation [x,y,z,w]:["
                  << msg.transform.rotation.x << ","
                  << msg.transform.rotation.y << ","
                  << msg.transform.rotation.z << ","
                  << msg.transform.rotation.w << "]");

  ros::spin();
  return 0;
}
