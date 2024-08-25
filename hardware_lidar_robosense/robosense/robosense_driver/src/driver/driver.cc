// Copyright (C) 2007, 2009-2012 Austin Robot Technology, Patrick Beeson, Jack
// O'Quin All rights reserved.
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

/** \file
 *
 *  ROS driver implementation for the Robosense 3D LIDARs
 */

#include <cmath>
#include <string>

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <sensor_interface_msgs/LidarScan.h>

#include "robosense_driver/driver.h"
#include "robosense_driver/status.h"
#include "robosense_driver/robosense_status.h"

using namespace msquare;
using namespace hardware;

namespace robosense_driver {

RobosenseDriver::RobosenseDriver(ros::NodeHandle node,
                               ros::NodeHandle private_nh) {
  // use private node handle to get parameters
  private_nh.param("frame_id", config_.frame_id, std::string("robosense"));
  std::string tf_prefix = tf::getPrefixParam(private_nh);
  ROS_DEBUG_STREAM("tf_prefix: " << tf_prefix);
  config_.frame_id = tf::resolve(tf_prefix, config_.frame_id);
  // get model name, validate string, determine packet rate
  private_nh.param("model", config_.model, std::string("RS-RUBY"));
  double packet_rate; // packet frequency (Hz)
  std::string model_full_name;
  if (config_.model == "RS-RUBY") {
    packet_rate = 6000.0; // robosense128 packet rate
    model_full_name = "RS-RUBY";
  }
  else {
    ROS_ERROR_STREAM("unknown Robosense LIDAR model: " << config_.model);
    packet_rate = 6000.0;
  }
  std::string deviceName(std::string("Robosense ") + model_full_name);

  private_nh.param("rpm", config_.rpm, 600.0);
  ROS_INFO_STREAM(deviceName << " rotating at " << config_.rpm << " RPM");
  double frequency = (config_.rpm / 60.0); // expected Hz rate

  // default number of packets for each scan is a single revolution
  // (fractions rounded up)
  config_.npackets = (int)ceil(packet_rate / frequency);
  private_nh.getParam("npackets", config_.npackets);
  ROS_INFO_STREAM("publishing " << config_.npackets << " packets per scan");

  std::string dump_file;
  private_nh.param("pcap", dump_file, std::string(""));

  double cut_angle;
  private_nh.param("cut_angle", cut_angle, -0.01);
  if (cut_angle < 0.0) {
    ROS_INFO_STREAM("Cut at specific angle feature deactivated.");
  } else if (cut_angle < (2 * M_PI)) {
    ROS_INFO_STREAM("Cut at specific angle feature activated. "
                    "Cutting robosense points always at "
                    << cut_angle << " rad.");
  } else {
    ROS_ERROR_STREAM("cut_angle parameter is out of range. Allowed range is "
                     << "between 0.0 and 2*PI or negative values to deactivate "
                        "this feature.");
    cut_angle = -0.01;
  }

  // Convert cut_angle from radian to one-hundredth degree,
  // which is used in robosense packets
  config_.cut_angle = int((cut_angle * 360 / (2 * M_PI)) * 100);
  int udp_port;
  private_nh.param("port", udp_port, (int)DATA_PORT_NUMBER);

  // Initialize dynamic reconfigure
  srv_ = boost::make_shared<
      dynamic_reconfigure::Server<robosense_driver::RobosenseNodeConfig>>(
      private_nh);
  dynamic_reconfigure::Server<robosense_driver::RobosenseNodeConfig>::CallbackType
      f;
  f = boost::bind(&RobosenseDriver::callback, this, _1, _2);
  srv_->setCallback(f); // Set callback function und call initially

  // initialize diagnostics
  diagnostics_.setHardwareID(deviceName);
  const double diag_freq = packet_rate / config_.npackets;
  diag_max_freq_ = diag_freq;
  diag_min_freq_ = diag_freq;
  ROS_INFO("expected frequency: %.3f (Hz)", diag_freq);

  using namespace diagnostic_updater;
  diag_topic_.reset(new TopicDiagnostic(
      "robosense_multi_scans", diagnostics_,
      FrequencyStatusParam(&diag_min_freq_, &diag_max_freq_, 0.1, 10),
      TimeStampStatusParam()));
  diag_timer_ = private_nh.createTimer(
      ros::Duration(0.2), &RobosenseDriver::diagTimerCallback, this);


  config_.enabled = true;

  // open Robosense input device or file
  if (dump_file != "") // have PCAP file?
  {
    // read data from packet capture file
    input_.reset(new robosense_driver::InputPCAP(private_nh, udp_port,
                                                packet_rate, dump_file));
  } else {
    // read data from live socket
    input_.reset(new robosense_driver::InputSocket(private_nh, udp_port));
  }

  // raw packet output topic
  output_ =
      node.advertise<sensor_interface_msgs::LidarScan>("/sensor/lidar/multi_scan", 10);

  // raw single packet output topic
  packet_output_ =
      node.advertise<sensor_interface_msgs::RobosensePacket>("/sensor/lidar/packet", 100);

  last_azimuth_ = -1;
}

/** poll the device
 *
 *  @returns true unless end of file reached
 */
bool RobosenseDriver::poll(void) {
  if (!config_.enabled) {
    // If we are not enabled exit once a second to let the caller handle
    // anything it might need to, such as if it needs to exit.
    ros::Duration(1).sleep();
    return true;
  }
  int packet_id = 0;
  // Allocate a new shared pointer for zero-copy sharing with other nodelets.
  sensor_interface_msgs::LidarScanPtr scan(new sensor_interface_msgs::LidarScan);
  scan->available = scan->LIDAR_TYPE_ROBOSENSE;
  // ROS_ERROR_STREAM("driver init scan "<<scan<<" "<<config_.enabled);
  if (config_.cut_angle >= 0) // Cut at specific angle feature enabled
  {
    scan->robosense_packets.reserve(config_.npackets);
    while (true) {
      sensor_interface_msgs::RobosensePacketPtr tmp_packet(
          new sensor_interface_msgs::RobosensePacket);
      while (true) {
        int rc = input_->getPacket(&(*tmp_packet), config_.time_offset);
        // int rc = input_->getPacket(&tmp_packet, config_.time_offset);
        if (rc == 0)
          break; // got a full packet?
        if (rc < 0)
          return false; // end of file reached?
      }
      scan->robosense_packets.push_back(*tmp_packet);
      packet_id++;
      tmp_packet->id = packet_id;
      //ROS_ERROR_STREAM("before driver pub "<<tmp_packet->id<<" ");
      packet_output_.publish(tmp_packet);
      //ROS_ERROR_STREAM("after driver pub "<<tmp_packet->id<<" ");

      // Extract base rotation of first block in packet
      int azimuth = ((int)(tmp_packet->data[2]) << 8) + tmp_packet->data[3];

      // if first packet in scan, there is no "valid" last_azimuth_
      if (last_azimuth_ == -1) {
        last_azimuth_ = azimuth;
        continue;
      }
      if ((last_azimuth_ < config_.cut_angle && config_.cut_angle <= azimuth) ||
          (config_.cut_angle <= azimuth && azimuth < last_azimuth_) ||
          (azimuth < last_azimuth_ && last_azimuth_ < config_.cut_angle)) {
        last_azimuth_ = azimuth;
        break; // Cut angle passed, one full revolution collected
      }
      last_azimuth_ = azimuth;
    }
  } else // standard behaviour
  {
    // Since the robosense delivers data at a very high rate, keep
    // reading and publishing scans as fast as possible.
    scan->robosense_packets.resize(config_.npackets);
    for (int i = 0; i < config_.npackets; ++i) {
      while (true) {
        // keep reading until full packet received
        int rc = input_->getPacket(&scan->robosense_packets[i], config_.time_offset);
        if (rc == 0)
          break; // got a full packet?
        if (rc < 0)
          return false; // end of file reached?
      }
    }
  }

  // publish message using time of last packet read
  ROS_DEBUG("Publishing a full Robosense scan.");
  scan->header.stamp.fromNSec(scan->robosense_packets.back().stamp);
  scan->header.frame_id = config_.frame_id;
  scan->available = scan->LIDAR_TYPE_ROBOSENSE;
  ROS_DEBUG_STREAM("publish(scan) " << scan->robosense_packets.back().stamp);
  output_.publish(scan);
  // ROS_ERROR_STREAM("driver pub scan "<<scan);

  // notify diagnostics that a message has been published, updating
  // its status
  diag_topic_->tick(scan->header.stamp);
  diagnostics_.update();

  return true;
}

void RobosenseDriver::callback(robosense_driver::RobosenseNodeConfig &config,
                              uint32_t level) {
  ROS_INFO("Reconfigure Request");
  if (level & 1) {
    config_.time_offset = config.time_offset;
  }
  if (level & 2) {
    config_.enabled = config.enabled;
  }
}

void RobosenseDriver::diagTimerCallback(const ros::TimerEvent &event) {
  (void)event;
  // Call necessary to provide an error when no robosense packets are received
  diagnostics_.update();
}

} // namespace robosense_driver
