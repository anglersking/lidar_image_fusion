#!/usr/bin/env python2
# Author: YuanJingYang <YuanJingYang@wanshannnt.ai>

import sys
reload(sys)
sys.setdefaultencoding('utf8')
import os
import re
import threading
import time
import json

import rospy
import rostopic
import roslib
from std_msgs.msg import Header

# argc topics
topics = []

# time_history: [us]
time_history = {}

# env: value
env = {}


def get_string_with_space(string):
  global env

  topic_max_len = env.get("topic_max_len", -1)
  if topic_max_len == -1:
    print "topic_max_len was not define!"
    exit(-1)

  string_len = len(string)
  if string_len > topic_max_len:
    print "%s(%d) is too long!" % (string, string_len)
    exit(-1)

  return string + " " * (topic_max_len - string_len)

def write_to_file(file_name):
  global time_history

  while True:
    time.sleep(2)
    with open(file_name,"w") as f:
      f.write(json.dumps(time_history))

def display_time_info():
  global env
  global topics
  global time_history

  while True:
    time.sleep(1)
    os.system("clear")
    
    print "%s\t%s\t%s\t%s\t%s\n" % (get_string_with_space("Topic Name"),
                                "Min Delay",
                                "Max Delay",
                                "Average Delay",
                                "times")


    for topic in topics:
      try:
        history_time = time_history[topic]
        history_len = len(history_time)
        average_t = sum(history_time) / history_len
        min_t = min(history_time)
        max_t = max(history_time)
        print "%s\t%.3fms\t%.3fms\t%.3fms\t%d" % (get_string_with_space(topic), 
                                                  min_t/1000.0,
                                                  max_t/1000.0,
                                                  average_t/1000.0,
                                                  len(time_history[topic]))
      except Exception as e:
        # print e
        pass
      
    
def delay_test(data, *args):
  global time_history

  topic = args[0]
  try:
    time_stamp = rospy.get_rostime() - data.header.stamp
    us = time_stamp.secs * 1000000 + time_stamp.nsecs / 1000
    time_history.setdefault(topic, []).append(us)
    
  except Exception as e:
    # print e
    pass

  # display_time_info()


if __name__ == '__main__':
  rospy.init_node('topic_delay_test_node', anonymous=True)
  topics_all = sorted(list(map(lambda x:x[0],rospy.client.get_published_topics())))
  
  if len(sys.argv) == 1:
    topics = topics_all
    topics.remove("/rosout")
    topics.remove("/rosout_agg")
  else:
    topics_re = sys.argv[1:]
    for topic in topics_all:
      for topic_re in topics_re:
        if re.match(topic_re,topic) != None:
          topics.append(topic)
          break

  for topic in topics:
    try:
      data_type = rostopic.get_topic_type(topic, blocking=False)[0]
      if data_type:
        data_class = roslib.message.get_message_class(data_type)
        rospy.Subscriber(topic, data_class, delay_test, tcp_nodelay=True, callback_args=(topic))
    except Exception as e:
      # print e
      pass
  
  if topics != []:
    file_name = os.path.expanduser("~") + "/Downloads/topic_" + time.strftime("%Y-%m-%d_%H%M%S", time.localtime()) + ".log"
    tasks = [
      threading.Thread(target=display_time_info),
      threading.Thread(target=write_to_file,args=[file_name])
    ]
    for t in tasks:
      t.daemon = True
      t.start()

    env["topic_max_len"] = max([max(map(lambda x: len(x), topics)), 20])
    rospy.spin()



