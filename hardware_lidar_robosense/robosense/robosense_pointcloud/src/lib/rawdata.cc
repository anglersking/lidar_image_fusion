/**
 *  @file
 *
 *  Robosense 3D LIDAR data accessor class implementation.
 *
 *  Class for unpacking raw Robosense LIDAR packets into useful
 *  formats.
 *
 *  Derived classes accept raw Robosense data for either single packets
 *  or entire rotations, and provide it in various formats for either
 *  on-line or off-line processing.
 */

#include <fstream>
#include <math.h>

#include <angles/angles.h>
#include <ros/package.h>
#include <ros/ros.h>

#include <robosense_pointcloud/rawdata.h>

namespace robosense_rawdata {
////////////////////////////////////////////////////////////////////////
//
// RawData base class implementation
//
////////////////////////////////////////////////////////////////////////

RawData::RawData() {}

/** Update parameters: conversions and update */
void RawData::setParameters(double view_direction, double view_width) {
  // converting angle parameters into the robosense reference (rad)
  config_.tmp_min_angle = view_direction + view_width / 2;
  config_.tmp_max_angle = view_direction - view_width / 2;

  // computing positive modulo to keep theses angles into [0;2*M_PI]
  config_.tmp_min_angle =
      fmod(fmod(config_.tmp_min_angle, 2 * M_PI) + 2 * M_PI, 2 * M_PI);
  config_.tmp_max_angle =
      fmod(fmod(config_.tmp_max_angle, 2 * M_PI) + 2 * M_PI, 2 * M_PI);

  // converting into the hardware robosense ref (negative yaml and degrees)
  // adding 0.5 perfomrs a centered double to int conversion
  config_.min_angle =
      100 * (2 * M_PI - config_.tmp_min_angle) * 180 / M_PI + 0.5;
  config_.max_angle =
      100 * (2 * M_PI - config_.tmp_max_angle) * 180 / M_PI + 0.5;
  if (config_.min_angle == config_.max_angle) {
    // avoid returning empty cloud if min_angle = max_angle
    config_.min_angle = 0;
    config_.max_angle = 36000;
  }
}

/** Set up for on-line operation. */
int RawData::setup(ros::NodeHandle private_nh) {
  private_nh.param("starting_angle", starting_angle, 0.f);
  if (starting_angle >= 360 || starting_angle < 0) {
    ROS_ERROR("bad starting angle: %f. Should fall in [0,360)", starting_angle);
    starting_angle = 0.f;
  } else {
    reset_startingangle();
  }

  private_nh.param("pointcloud_width", width_, 0);
  private_nh.param("pointcloud_height", height_, 0);
  private_nh.param("rpm", rpm_, 600);

  // get path to angles.config file for this device
  if (!private_nh.getParam("calibration", config_.calibrationFile)) {
    ROS_ERROR_STREAM("No calibration angles specified! Using test values!");

    // have to use something: grab unit test version as a default
    std::string pkgPath = ros::package::getPath("robosense_pointcloud");
    config_.calibrationFile = pkgPath + "/params/robosense-ruby128.yaml";
  }

  ROS_INFO_STREAM("correction angles: " << config_.calibrationFile);

  calibration_.read(config_.calibrationFile);
  if (!calibration_.initialized) {
    ROS_ERROR_STREAM(
        "Unable to open calibration file: " << config_.calibrationFile);
    return -1;
  }

  ROS_INFO_STREAM("Number of lasers: " << calibration_.num_lasers << ".");

  time_duration_between_blocks_ = 1 / ((float)PACKET_RATE * BLOCKS_PER_PACKET);
  int blocks_per_round = (PACKET_RATE / (rpm_ / 60)) * BLOCKS_PER_PACKET;
  fov_time_jump_diff_ = time_duration_between_blocks_ * (float)blocks_per_round;
  azi_diff_between_block_theoretical_ = RS_ONE_ROUND / (float)blocks_per_round;

#if 0
  ROS_ERROR(">>>>>>>> start/width/height/rpm: %f/%d/%d/%d", starting_angle, width_, height_, rpm_);
  ROS_ERROR("++++++ num/dist/dsr/fir/rx/ry/rz: %d/%f/%f/%f/%f/%f/%f",
            calibration_.num_lasers, calibration_.distance_resolution_m, calibration_.dsr_toffet,
            calibration_.firing_frequency, calibration_.rx, calibration_.ry, calibration_.rz);

  for (uint16_t i = 0; i < calibration_.num_lasers; i++)
  {
      ROS_ERROR(">>>>>>>> laser: %d, ring:%d, vert/hori: %d/%d", i, calibration_.laser_angles[i].laser_ring,
                calibration_.laser_angles[i].vert_angle, calibration_.laser_angles[i].hori_angle);
  }

  ROS_ERROR(">>>>>>>> time/block/fov/azi: %f/%d/%f/%f",
            time_duration_between_blocks_, blocks_per_round, fov_time_jump_diff_, azi_diff_between_block_theoretical_);
#endif

  // Set up cached values for sin and cos of all the possible headings
  for (uint16_t rot_index = 0; rot_index < ROTATION_MAX_UNITS; ++rot_index) {
    float rotation = angles::from_degrees(ROTATION_RESOLUTION * rot_index);
    cos_rot_table_[rot_index] = cosf(rotation);
    sin_rot_table_[rot_index] = sinf(rotation);
  }

  return 0;
}

/** Set up for off-line operation. */
int RawData::setupOffline(int pointcloud_width,
                          int pointcloud_height,
                          int pointcloud_rpm,
                          float start_angle) {
  starting_angle = start_angle;
  if (starting_angle >= 360 || starting_angle < 0) {
    ROS_ERROR("bad starting angle: %f. Should fall in [0,360)", starting_angle);
    starting_angle = 0.f;
  } else {
    reset_startingangle();
  }

  width_ = pointcloud_width;
  height_ = pointcloud_height;
  rpm_ = pointcloud_rpm;

  // have to use something: grab unit test version as a default
  std::string pkgPath = ros::package::getPath("robosense_pointcloud");
  config_.calibrationFile = pkgPath + "/params/robosense-ruby128.yaml";

  ROS_INFO_STREAM("correction angles: " << config_.calibrationFile);

  calibration_.read(config_.calibrationFile);
  if (!calibration_.initialized) {
    ROS_ERROR_STREAM(
        "Unable to open calibration file: " << config_.calibrationFile);
    return -1;
  }

  ROS_INFO_STREAM("Number of lasers: " << calibration_.num_lasers << ".");

  time_duration_between_blocks_ = 1 / ((float)PACKET_RATE * BLOCKS_PER_PACKET);
  int blocks_per_round = (PACKET_RATE / (rpm_ / 60)) * BLOCKS_PER_PACKET;
  fov_time_jump_diff_ = time_duration_between_blocks_ * (float)blocks_per_round;
  azi_diff_between_block_theoretical_ = RS_ONE_ROUND / (float)blocks_per_round;

#if 0
  ROS_ERROR(">>>>>>>> start/width/height/rpm: %f/%d/%d/%d", starting_angle, width_, height_, rpm_);
  ROS_ERROR("++++++ num/dist/dsr/fir/rx/ry/rz: %d/%f/%f/%f/%f/%f/%f",
            calibration_.num_lasers, calibration_.distance_resolution_m, calibration_.dsr_toffet,
            calibration_.firing_frequency, calibration_.rx, calibration_.ry, calibration_.rz);

  for (uint16_t i = 0; i < calibration_.num_lasers; i++)
  {
      ROS_ERROR(">>>>>>>> laser: %d, ring:%d, vert/hori: %d/%d", i, calibration_.laser_angles[i].laser_ring,
                calibration_.laser_angles[i].vert_angle, calibration_.laser_angles[i].hori_angle);
  }

  ROS_ERROR(">>>>>>>> time/block/fov/azi: %f/%d/%f/%f",
            time_duration_between_blocks_, blocks_per_round, fov_time_jump_diff_, azi_diff_between_block_theoretical_);
#endif

  // Set up cached values for sin and cos of all the possible headings
  for (uint16_t rot_index = 0; rot_index < ROTATION_MAX_UNITS; ++rot_index) {
    float rotation = angles::from_degrees(ROTATION_RESOLUTION * rot_index);
    cos_rot_table_[rot_index] = cosf(rotation);
    sin_rot_table_[rot_index] = sinf(rotation);
  }

  return 0;
}

void RawData::unpack_rsruby128(const sensor_interface_msgs::RobosensePacket &pkt,
                            VPointCloud &pc, const Eigen::Isometry3d &is3d) {
  Eigen::Matrix3f R;
  Eigen::Vector3f t;
  Eigen::Quaterniond quat(is3d.rotation());
  quat.normalize();
  Eigen::Vector3d pos(is3d.translation());
  R = quat.toRotationMatrix().cast<float>();
  t = pos.cast<float>();

  double block_timestamp = pkt.stamp * 1e-9; //.toSec();
  const raw_packet_t *raw = (const raw_packet_t *)&pkt.data[0];
  float azi_diff = 0;

  for (int blk_idx = 0; blk_idx < BLOCKS_PER_PACKET; blk_idx++) {
    if (raw->blocks[blk_idx].id != BLOCK_HEADER_RS_RUBY)
      break;

  /* block_timestamp, set as packet time, or add offset */
#if 0
    int cur_azi = RS_SWAP_SHORT(raw->blocks[blk_idx].azimuth);
    if (calibration_.echo_mode == ECHO_DUAL) {
      azi_diff = (float)((RS_ONE_ROUND + RS_SWAP_SHORT(raw->blocks[2].azimuth) - RS_SWAP_SHORT(raw->blocks[0].azimuth)) % RS_ONE_ROUND);
      if (RS_SWAP_SHORT(raw->blocks[0].azimuth) == RS_SWAP_SHORT(raw->blocks[1].azimuth)) {   ///< AAB
        if (blk_idx == 2)
          block_timestamp = (azi_diff > 100) ? (block_timestamp + fov_time_jump_diff_) : (block_timestamp + time_duration_between_blocks_);
      }
      else {    ///< ABB
        if (blk_idx == 1)
          block_timestamp = (azi_diff > 100) ? (block_timestamp + fov_time_jump_diff_) : (block_timestamp + time_duration_between_blocks_);
      }
    }
    else {
      if (blk_idx == 0)
        azi_diff = (float)((RS_ONE_ROUND + RS_SWAP_SHORT(raw->blocks[blk_idx + 1].azimuth) - cur_azi) % RS_ONE_ROUND);
      else {
        azi_diff = (float)((RS_ONE_ROUND + cur_azi - RS_SWAP_SHORT(raw->blocks[blk_idx - 1].azimuth)) % RS_ONE_ROUND);
        block_timestamp = (azi_diff > 100) ? (block_timestamp + fov_time_jump_diff_) : (block_timestamp + time_duration_between_blocks_);
      }
    }
#else
    int cur_azi = RS_SWAP_SHORT(raw->blocks[blk_idx].azimuth);
    if (calibration_.echo_mode == ECHO_DUAL) {
      azi_diff = (float)((RS_ONE_ROUND + RS_SWAP_SHORT(raw->blocks[2].azimuth) - RS_SWAP_SHORT(raw->blocks[0].azimuth)) % RS_ONE_ROUND);
    }
    else {
      if (blk_idx == 0)
        azi_diff = (float)((RS_ONE_ROUND + RS_SWAP_SHORT(raw->blocks[blk_idx + 1].azimuth) - cur_azi) % RS_ONE_ROUND);
      else
        azi_diff = (float)((RS_ONE_ROUND + cur_azi - RS_SWAP_SHORT(raw->blocks[blk_idx - 1].azimuth)) % RS_ONE_ROUND);
    }
#endif

    azi_diff = (azi_diff > 100) ? azi_diff_between_block_theoretical_ : azi_diff;
    for (size_t channel_idx = 0; channel_idx < SCANS_PER_BLOCK; channel_idx++) {
      /* 1, check distance in range */
      float distance = RS_SWAP_SHORT(raw->blocks[blk_idx].channels[channel_idx].distance) * calibration_.distance_resolution_m;
      if (distance > calibration_.max_distance || distance < calibration_.min_distance)
        continue;

      /* 2, calc angle of horizon and vertical */
      robosense_pointcloud::LaserAngle &laser_angle = calibration_.laser_angles[channel_idx];

      int dsr_temp = (channel_idx / 4) % 16;
      float azi_channel_ori = cur_azi + (azi_diff * float(dsr_temp) * calibration_.dsr_toffet * calibration_.firing_frequency);
      int azi_channel_final = ((int)azi_channel_ori + laser_angle.hori_angle + RS_ONE_ROUND) % RS_ONE_ROUND;

      int angle_horiz = (int)(azi_channel_ori + RS_ONE_ROUND) % RS_ONE_ROUND;
      int angle_vert = (laser_angle.vert_angle + RS_ONE_ROUND) % RS_ONE_ROUND;

      /* 3, get coordinate and intensity of point3D */
      float x_coord =  distance * cos_rot_table_[angle_vert] * cos_rot_table_[azi_channel_final] + calibration_.rx * cos_rot_table_[angle_horiz];
      float y_coord = -distance * cos_rot_table_[angle_vert] * sin_rot_table_[azi_channel_final] - calibration_.rx * sin_rot_table_[angle_horiz];
      float z_coord =  distance * sin_rot_table_[angle_vert] + calibration_.rz;
      float intensity = raw->blocks[blk_idx].channels[channel_idx].intensity;

      /* 4, store point info */
      VPoint point;
      point.timestamp = block_timestamp;
      point.ring = laser_angle.laser_ring;
      point.azimuth = azi_channel_final * 0.01f;
      point.intensity = intensity;
      Eigen::Vector3f pt = R * Eigen::Vector3f(x_coord, y_coord, z_coord) + t;
      point.x = pt[0];
      point.y = pt[1];
      point.z = pt[2];

      uint16_t rotation = (azi_channel_final < config_.starting_angle)
                         ? azi_channel_final + RS_ONE_ROUND - config_.starting_angle
                         : azi_channel_final - config_.starting_angle;
      columnIndex_ = get_column(rotation);
      // point.intensity = columnIndex_ * 47 % 255;   ///< line and column alignment test, this will make intensity change along columnIndex
      pc.points[columnIndex_ + pc.width * laser_angle.laser_ring] = point;
    }
  }
}

} // namespace robosense_rawdata
