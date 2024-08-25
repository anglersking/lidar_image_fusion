#! /usr/bin/env python2
import json
import random
import monotonic
import collections
import time


class MessageEvent:
  def __init__(self, msg_id, tag, timestamp_ns):
    self.msg_id = msg_id
    self.tag = tag
    self.timestamp_ns = timestamp_ns

class MessageRelation:
  def __init__(self, current_msg_id, relevant_msg_id):
    self.current_msg_id = current_msg_id 
    self.relevant_msg_id = relevant_msg_id


class Analyzer:
  class Node:
    def __init__(self, msg_id):
      self.precursor = []
      self.event_time_dict = {}
      self.msg_id = msg_id
      self.created_time_sec = monotonic.time.time()
  
    def update_event(self, event):
      self.event_time_dict[event.tag] = event.timestamp_ns

    def connect_precursor(self, node):
      self.precursor.append(node)

    def traversal_tree(self, result):
      result["id"] = self.msg_id
      result["event"] = self.event_time_dict
      result["child"] = []
      for node in self.precursor:
        ret = node.traversal_tree({})
        result["child"].append(ret) 
      return result

    def traversal_flat(self, result):
      for tag, time in self.event_time_dict.items():
        if (tag not in result): result[tag] = []
        result[tag].append(time)
      for node in self.precursor:
        node.traversal_flat(result)
      return result

  class StatisticMachine:
    def __init__(self, root):
      self.root = root


  def __init__(self, garbage_collection_sec = 10, statistic_delay_sec = -1):
    self.nodes = {}
    self.all_msgs = collections.deque()
    self.tracking_msgs = collections.deque() 
    self.garbage_collection_sec = garbage_collection_sec 
    self.statistic_delay_sec = statistic_delay_sec

  def get_nodes(self, msg_id):
    if (not msg_id in self.nodes): 
      self.nodes[msg_id] = self.Node(msg_id)
      self.all_msgs.append(msg_id)
      self.tracking_msgs.append(msg_id)
    return self.nodes[msg_id]

  def garbage_collection(self):
    now = monotonic.time.time() 
    while (len(self.all_msgs) > 0):
      msg_id = self.all_msgs[0] 
      timestamp_sec = self.nodes[msg_id].created_time_sec
      if (now - timestamp_sec > self.garbage_collection_sec):
        del self.nodes[msg_id]
        self.all_msgs.popleft()
      else:
        break

  def analyze(self, callback, user_data):
    now = monotonic.time.time() 
    while (len(self.tracking_msgs) > 0):
      msg_id = self.tracking_msgs[0] 
      timestamp_sec = self.nodes[msg_id].created_time_sec
      if (now - timestamp_sec > self.statistic_delay_sec):
        node = self.nodes[msg_id] 
        callback(node, user_data)  
        self.tracking_msgs.popleft()
      else:
        break

  def update_event(self, event):
    self.get_nodes(event.msg_id).update_event(event)
  
  def update_relation(self, relation):
    current_node = self.get_nodes(relation.current_msg_id) 
    relevant_node = self.get_nodes(relation.relevant_msg_id) 
    current_node.connect_precursor(relevant_node)






