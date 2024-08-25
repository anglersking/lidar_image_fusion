#include <cmath>
#include <fstream>
#include <iostream>
#include <limits>
#include <string>
#include <yaml-cpp/yaml.h>

#ifdef HAVE_NEW_YAMLCPP
namespace YAML {

// The >> operator disappeared in yaml-cpp 0.5, so this function is
// added to provide support for code written under the yaml-cpp 0.3 API.
template <typename T> void operator>>(const YAML::Node &node, T &i) {
  i = node.as<T>();
}
} // namespace YAML
#endif // HAVE_NEW_YAMLCPP

#include <ros/ros.h>
#include <robosense_pointcloud/calibration.h>

namespace robosense_pointcloud {

const std::string NUM_LASERS = "num_lasers";
const std::string DISTANCE_RESOLUTION = "distance_resolution";
const std::string DSR_TOFFET = "dsr_toffet";
const std::string FIRING_FREQUENCY = "firing_frequency";
const std::string RX = "rx";
const std::string RY = "ry";
const std::string RZ = "rz";
const std::string MIN_DISTANCE = "min_distance";
const std::string MAX_DISTANCE = "max_distance";
const std::string ECHO_MODE = "echo_mode";
const std::string LASERS = "lasers";
const std::string LASER_ID = "laser_id";
const std::string VERT_ANGLE = "vert_angle";
const std::string HORI_ANGLE = "hori_angle";

/** Read calibration for a single laser. */
void operator>>(const YAML::Node &node,
                std::pair<int, LaserAngle> &laser_angle) {
  node[LASER_ID] >> laser_angle.first;
  node[VERT_ANGLE] >> laser_angle.second.vert_angle;
  node[HORI_ANGLE] >> laser_angle.second.hori_angle;
  node[LASER_ID] >> laser_angle.second.laser_ring;
}

/** Read entire calibration file. */
void operator>>(const YAML::Node &node, Calibration &calibration) {
  int num_lasers;
  node[NUM_LASERS] >> num_lasers;

  calibration.num_lasers = num_lasers;
  node[DISTANCE_RESOLUTION] >> calibration.distance_resolution_m;
  node[DSR_TOFFET] >> calibration.dsr_toffet;
  node[FIRING_FREQUENCY] >> calibration.firing_frequency;
  node[RX] >> calibration.rx;
  node[RY] >> calibration.ry;
  node[RZ] >> calibration.rz;
  node[MIN_DISTANCE] >> calibration.min_distance;
  node[MAX_DISTANCE] >> calibration.max_distance;
  node[ECHO_MODE] >> calibration.echo_mode;

  const YAML::Node &lasers = node[LASERS];
  calibration.laser_angles.clear();
  calibration.laser_angles.resize(num_lasers);
  for (int i = 0; i < num_lasers; i++) {
    std::pair<int, LaserAngle> laser_angle;
    lasers[i] >> laser_angle;
    const unsigned int index = laser_angle.first;
    if (index >= calibration.laser_angles.size()) {
      calibration.laser_angles.resize(index + 1);
    }
    calibration.laser_angles[index] = (laser_angle.second);
    calibration.laser_angle_map.insert(laser_angle);
  }
}

YAML::Emitter &operator<<(YAML::Emitter &out,
                          const std::pair<int, LaserAngle> laser_angle) {
  out << YAML::BeginMap;
  out << YAML::Key << LASER_ID << YAML::Value << laser_angle.first;
  out << YAML::Key << VERT_ANGLE << YAML::Value
      << laser_angle.second.vert_angle;
  out << YAML::Key << HORI_ANGLE << YAML::Value
      << laser_angle.second.hori_angle;
  out << YAML::EndMap;
  return out;
}

YAML::Emitter &operator<<(YAML::Emitter &out, const Calibration &calibration) {
  out << YAML::BeginMap;
  out << YAML::Key << NUM_LASERS << YAML::Value
      << calibration.laser_angles.size();
  out << YAML::Key << DISTANCE_RESOLUTION << YAML::Value
      << calibration.distance_resolution_m;
  out << YAML::Key << DSR_TOFFET << YAML::Value
      << calibration.dsr_toffet;
  out << YAML::Key << FIRING_FREQUENCY << YAML::Value
      << calibration.firing_frequency;
  out << YAML::Key << RX << YAML::Value
      << calibration.rx;
  out << YAML::Key << RY << YAML::Value
      << calibration.ry;
  out << YAML::Key << RZ << YAML::Value
      << calibration.rz;
  out << YAML::Key << MIN_DISTANCE << YAML::Value
      << calibration.min_distance;
  out << YAML::Key << MAX_DISTANCE << YAML::Value
      << calibration.max_distance;
  out << YAML::Key << ECHO_MODE << YAML::Value
      << calibration.echo_mode;
  out << YAML::Key << LASERS << YAML::Value << YAML::BeginSeq;
  for (std::map<int, LaserAngle>::const_iterator it =
           calibration.laser_angle_map.begin();
       it != calibration.laser_angle_map.end(); it++) {
    out << *it;
  }
  out << YAML::EndSeq;
  out << YAML::EndMap;
  return out;
}

void Calibration::read(const std::string &calibration_file) {
  std::ifstream fin(calibration_file.c_str());
  if (!fin.is_open()) {
    initialized = false;
    return;
  }
  initialized = true;
  try {
    YAML::Node doc;
#ifdef HAVE_NEW_YAMLCPP
    fin.close();
    doc = YAML::LoadFile(calibration_file);
#else
    YAML::Parser parser(fin);
    parser.GetNextDocument(doc);
#endif
    doc >> *this;
  } catch (YAML::Exception &e) {
    std::cerr << "YAML Exception: " << e.what() << std::endl;
    initialized = false;
  }
  fin.close();
}

void Calibration::write(const std::string &calibration_file) {
  std::ofstream fout(calibration_file.c_str());
  YAML::Emitter out;
  out << *this;
  fout << out.c_str();
  fout.close();
}

} // namespace robosense_pointcloud
