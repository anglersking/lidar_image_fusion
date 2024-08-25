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

#ifndef ROBOSENSE_DRIVER_TIME_CONVERSION_HPP
#define ROBOSENSE_DRIVER_TIME_CONVERSION_HPP

#include <ros/ros.h>
#include <ros/time.h>

ros::Time rosTimeFromGpsTimestamp(const uint8_t *const data) {
  // time for each packet is a 10 byte uint
  // It is the UTC timestamp, nanoseconds
  uint32_t sec =
      (uint32_t)(((uint32_t)data[2]) << 24 | ((uint32_t)data[3]) << 16 |
                 ((uint32_t)data[4]) << 8 | ((uint32_t)data[5]));

  uint32_t nsec =
      (uint32_t)(((uint32_t)data[6]) << 24 | ((uint32_t)data[7]) << 16 |
                 ((uint32_t)data[8]) << 8 | ((uint32_t)data[9])) * 1000;

  ros::Time stamp = ros::Time(sec, nsec);
  return stamp;
}

#endif // ROBOSENSE_DRIVER_TIME_CONVERSION_HPP
