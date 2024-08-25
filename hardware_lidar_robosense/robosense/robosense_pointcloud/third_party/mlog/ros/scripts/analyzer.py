#!/usr/bin/env python

import rospy
import yaml
import json 
import time
import numpy
import math
import json
import os
from std_msgs.msg import String
from mlog.msg import Logs

def logs_callback(msg, meta):
  print msg

def main():
  rospy.init_node('car_pseudo', anonymous=True)
  sub_mlog_topic = rospy.get_param("~mlog_topic", "/mlog/logs")

  meta = {}
  rospy.Subscriber(sub_mlog_topic, Logs, logs_callback, meta)
  rospy.spin()

if __name__ == '__main__':
  main()


