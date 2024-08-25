// Copyright (C) 2019 Matthew Pitropov, Joshua Whitley
// All rights reserved.
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

#include "robosense_driver/time_conversion.hpp"
#include <gtest/gtest.h>
#include <ros/time.h>

TEST(TimeConversion, BytesToTimestamp) {
  ros::Time::init();
  ros::Time ros_stamp = ros::Time::now();

  uint8_t native_format[10] = {0};
  native_format[2] = 0xFF & (((uint32_t)ros_stamp.sec) >> 24);
  native_format[3] = 0xFF & (((uint32_t)ros_stamp.sec) >> 16);
  native_format[4] = 0xFF & (((uint32_t)ros_stamp.sec) >> 8);
  native_format[5] = 0xFF & ros_stamp.nsec;
  native_format[6] = 0xFF & (((uint32_t)ros_stamp.nsec) >> 24);
  native_format[7] = 0xFF & (((uint32_t)ros_stamp.nsec) >> 16);
  native_format[8] = 0xFF & (((uint32_t)ros_stamp.nsec) >> 8);
  native_format[9] = 0xFF & ros_stamp.nsec;

  ros::Time ros_stamp_converted = rosTimeFromGpsTimestamp(native_format);

  ASSERT_EQ(ros_stamp_converted.sec, ros_stamp.sec);
  ASSERT_NEAR(ros_stamp_converted.nsec, ros_stamp.nsec, 1000);
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  int ret = RUN_ALL_TESTS();
  return ret;
}
