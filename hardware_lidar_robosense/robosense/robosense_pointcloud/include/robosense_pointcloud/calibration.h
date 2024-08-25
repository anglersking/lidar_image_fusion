// Copyright (C) 2012, 2019 Austin Robot Technology, Piyush Khandelwal, Joshua
// Whitley All rights reserved.
//
// Software License Agreement (BSD License 2.0)
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above
//    copyright notice, this list of conditions and the following
//    disclaimer in the documentation and/or other materials provided
//    with the distribution.
//  * Neither the name of {copyright_holder} nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#ifndef ROBOSENSE_POINTCLOUD_CALIBRATION_H
#define ROBOSENSE_POINTCLOUD_CALIBRATION_H

#include <map>
#include <string>
#include <vector>
using namespace std;
namespace robosense_pointcloud {

/** \brief Correction information for a single laser. */
struct LaserAngle {
  /** parameters in db.xml */
  int vert_angle;
  int hori_angle;
  int laser_ring; ///< ring number for this laser
};

/** \brief Calibration information for the entire device. */
class Calibration {
public:
  float distance_resolution_m;
  float dsr_toffet;
  float firing_frequency;
  float rx;
  float ry;
  float rz;
  float min_distance;
  float max_distance;
  int echo_mode;
  std::map<int, LaserAngle> laser_angle_map;
  std::vector<LaserAngle> laser_angles;
  int num_lasers;
  bool initialized;
  bool ros_info;

public:
  explicit Calibration(bool info = true)
      : distance_resolution_m(0.0f), dsr_toffet(0.0f), firing_frequency(0.0f), rx(0.0f), ry(0.0f), rz(0.0f), 
        min_distance(0.0f), max_distance(0.0f), echo_mode(0), num_lasers(0), initialized(false), ros_info(info) {}
  explicit Calibration(const std::string &calibration_file, bool info = true)
      : distance_resolution_m(0.0f), dsr_toffet(0.0f), firing_frequency(0.0f), rx(0.0f), ry(0.0f), rz(0.0f), 
        min_distance(0.0f), max_distance(0.0f), echo_mode(0), num_lasers(0), initialized(false), ros_info(info) {
    read(calibration_file);
  }

  void read(const std::string &calibration_file);
  void write(const std::string &calibration_file);
};

} // namespace robosense_pointcloud

#endif // ROBOSENSE_POINTCLOUD_CALIBRATION_H
