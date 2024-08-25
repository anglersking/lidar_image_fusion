#! /usr/bin/env python2
from analyzer import MessageEvent 
from analyzer import MessageRelation
from analyzer import Analyzer

import json
import random

if __name__ == "__main__":

  events = [
    MessageEvent("cam_LF_1", "camera_recv", 10),
    MessageEvent("cam_RF_1", "camera_recv", 11),
    MessageEvent("cam_LF_2", "camera_recv", 21),
    MessageEvent("cam_RF_2", "camera_recv", 20),
  
    MessageEvent("cam_LF_1", "camera_send", 15),
    MessageEvent("cam_RF_1", "camera_send", 17),
    MessageEvent("cam_LF_2", "camera_send", 27),
    MessageEvent("cam_RF_2", "camera_send", 22),
  
    MessageEvent("cam_LF_1", "mpa_cam_recv", 31),
    MessageEvent("cam_RF_1", "mpa_cam_recv", 32),
    MessageEvent("cam_LF_2", "mpa_cam_recv", 43),
    MessageEvent("cam_RF_2", "mpa_cam_recv", 42),
  
    MessageEvent("mpa_1", "mpa_pred_send", 40),
    MessageEvent("mpa_2", "mpa_pred_send", 50),
  ]
  
  relations = [
    MessageRelation("mpa_1", "cam_LF_1"),
    MessageRelation("mpa_1", "cam_RF_1"),
    MessageRelation("mpa_2", "cam_LF_2"),
    MessageRelation("mpa_2", "cam_RF_2"),
  ]

  config = """
  {
    "garbage_collection_sec": 10,
    "statistic_delay_sec": -1,
    "statistic_rules": [
       {"hint": "total_delay", "start_event": "camera_recv", "end_event": "mpa_pred_send"},
       {"hint": "camera_delay", "start_event": "camera_recv", "end_event": "camera_send"}
    ]
  }
  """
  statistic_rules = json.loads(config)["statistic_rules"]

  analyzer = Analyzer()
  data = events + relations
  random.shuffle(data)
  
  for entry in data:
    if isinstance(entry, MessageEvent): 
      analyzer.update_event(entry)
    if isinstance(entry, MessageRelation): 
      analyzer.update_relation(entry)
    analyzer.garbage_collection()

  def analyzer_callback(node, statistic_rules):
    flat = node.traversal_flat({})
    print flat
    for rule in statistic_rules:
      start, end = rule["start_event"], rule["end_event"] 
      if end in node.event_time_dict and start in flat:
        print rule["hint"], node.event_time_dict[end] - min(flat[start]) 
    
  analyzer.analyze(analyzer_callback, statistic_rules)
