// Copyright (C) 2007, 2009, 2010, 2012, 2019 Yaxin Liu, Patrick Beeson, Jack
// O'Quin, Joshua Whitley All rights reserved.
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

/** @file
 *
 *  @brief Interfaces for interpreting raw packets from the Robosense 3D LIDAR.
 */

#ifndef ROBOSENSE_POINTCLOUD_RAWDATA_H
#define ROBOSENSE_POINTCLOUD_RAWDATA_H

#include <boost/format.hpp>
#include <errno.h>
#include <math.h>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <stdint.h>
#include <string>
#include <sys/time.h>
#include <tf_conversions/tf_eigen.h>
#include <vector>
#include <sensor_interface_msgs/LidarScan.h>
#include <robosense_pointcloud/calibration.h>
#include <robosense_pointcloud/datacontainerbase.h>
#include <robosense_pointcloud/point_types.h>

using namespace std;
namespace robosense_rawdata {
// Shorthand typedefs for point cloud representations
typedef robosense_pointcloud::PointXYZIT VPoint;
typedef pcl::PointCloud<VPoint> VPointCloud;

/**
 * Raw Robosense packet constants and structures.
 */
#define RS_SWAP_SHORT(x) ((((x)&0xFF) << 8) | (((x)&0xFF00) >> 8))
#define RS_SWAP_LONG(x) ((((x)&0xFF) << 24) | (((x)&0xFF00) << 8) | (((x)&0xFF0000) >> 8) | (((x)&0xFF000000) >> 24))
#define RS_TO_RADS(x) ((x) * (M_PI) / 180)

static const int SIZE_BLOCK = 100;
static const int RAW_SCAN_SIZE = 3;
static const int SCANS_PER_BLOCK = 128;
static const int BLOCK_DATA_SIZE = (SCANS_PER_BLOCK * RAW_SCAN_SIZE);

static const float ROTATION_RESOLUTION = 0.01f;    // [deg]
static const uint16_t ROTATION_MAX_UNITS = 36000u; // [deg/100]

static const int RS_ONE_ROUND = 36000;

static const uint8_t BLOCK_HEADER_RS_RUBY = 0xFE;
static const int PACKET_RATE = 6000;
static const int PACKET_SIZE = 1168;
static const int BLOCKS_PER_PACKET = 3;
static const int PACKET_STATUS_SIZE = 4;
static const int SCANS_PER_PACKET = (SCANS_PER_BLOCK * BLOCKS_PER_PACKET);

/* Echo mode definition */
enum RSEchoMode
{
  ECHO_LAST = 1,
  ECHO_STRONGEST,
  ECHO_DUAL
};

#pragma pack(push, 1)
/** \brief Raw Robosense data block.
 */
typedef struct
{
  uint16_t distance;
  uint8_t intensity;
} RSChannel;

typedef struct raw_block {
  uint8_t id;   ///< fix 0xFE
  uint8_t ret_id;   ///< return id, 01-03
  uint16_t azimuth; ///< 0-35999, divide by 100 to get degrees
  RSChannel channels[SCANS_PER_BLOCK];
} raw_block_t;

/** \brief Raw Robosense packet.
 */
typedef struct raw_packet {
  raw_block_t blocks[BLOCKS_PER_PACKET];
  uint8_t status[PACKET_STATUS_SIZE];
} raw_packet_t;

#pragma pack(pop)

/** \brief Robosense data conversion class */
class RawData {
public:
  RawData();
  ~RawData() {}

  /** \brief Set up for data processing.
   *
   *  Perform initializations needed before data processing can
   *  begin:
   *
   *    - read device-specific angles calibration
   *
   *  @param private_nh private node handle for ROS parameters
   *  @returns 0 if successful;
   *           errno value for failure
   */
  int setup(ros::NodeHandle private_nh);
  int setupOffline(int pointcloud_width,
                   int pointcloud_height,
                   int pointcloud_rpm,
                   float start_angle);

  inline long long get_timenow() {
    long long timenow;
    struct timeval ce;
    gettimeofday(&ce, NULL);
    timenow = ce.tv_sec * 1000000 + ce.tv_usec;
    return timenow;
  }

  void unpack_rsruby128(const sensor_interface_msgs::RobosensePacket &pkt, VPointCloud &pc,
                     const Eigen::Isometry3d &is3d);

  void setParameters(double view_direction, double view_width);

  inline void reset_startingangle() {
    config_.starting_angle = static_cast<uint16_t>(starting_angle * 100);
  }

private:
  /** configuration parameters */
  typedef struct {
    std::string calibrationFile; ///< calibration file name
    int min_angle;               ///< minimum angle to publish
    int max_angle;               ///< maximum angle to publish
    uint16_t starting_angle;

    double tmp_min_angle;
    double tmp_max_angle;
  } Config;
  Config config_;

  /**
   * Calibration file
   */
  robosense_pointcloud::Calibration calibration_;
  float sin_rot_table_[ROTATION_MAX_UNITS];
  float cos_rot_table_[ROTATION_MAX_UNITS];

  // return column between [0, 1875]
  int inline get_column(int rotation) {
    int column = static_cast<int>(rotation * 0.052);  ///< rotation / 36000 * 1875
    if (column < 0) {
      column += width_;
    } else if (column >= width_) {
      column -= width_;
    }
    return column;
  }

  /* we want to process a whole 360 degree of points, instead of using constant
   * packets count. So we need to store previous packet incase it contains
   * some block that belongs to next frame */
  float starting_angle = 0.f;
  int columnIndex_ = 0;
  int width_ = -1;
  int height_ = -1;

  int rpm_ = -1;
  float fov_time_jump_diff_ = 0.f;
  float time_duration_between_blocks_ = 0.f;
  float azi_diff_between_block_theoretical_ = 0.f;
  float current_temperature_ = 0.f;
};

} // namespace robosense_rawdata

#endif // ROBOSENSE_POINTCLOUD_RAWDATA_H
