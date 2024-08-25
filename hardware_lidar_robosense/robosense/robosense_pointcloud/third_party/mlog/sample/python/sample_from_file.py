#! /usr/bin/env python2
from analyzer import MessageEvent 
from analyzer import MessageRelation
from analyzer import Analyzer

import json
import argparse
import random

config = """
{
  "garbage_collection_sec": 10,
  "statistic_delay_sec": -1,
  "statistic_rules": [
     {"hint": "total_delay", "start_event": "send", "end_event": "output"}
  ]
}
"""

if __name__ == "__main__":
  parser = argparse.ArgumentParser() 
  parser.add_argument("--input", "-i", help="input log file")
  args = parser.parse_args()
  with open(args.input) as f:
    content = f.readlines()

  data = []
  for line in content:
    if (line.startswith("MSG_TIMESTAMP ")):
      ele = line.strip().split(" ")
      data.append(MessageEvent(ele[1], ele[2], int(ele[3])))
    if (line.startswith("MSG_RELATION ")):
      ele = line.strip().split(" ")
      data.append(MessageRelation(ele[1], ele[2]))

  statistic_rules = json.loads(config)["statistic_rules"]
  analyzer = Analyzer()
  
  for entry in data:
    if isinstance(entry, MessageEvent): 
      analyzer.update_event(entry)
    if isinstance(entry, MessageRelation): 
      analyzer.update_relation(entry)
    analyzer.garbage_collection()

  def analyzer_callback(node, statistic_rules):
    flat = node.traversal_flat({})
    #print "flat entry:", flat
    for rule in statistic_rules:
      start, end = rule["start_event"], rule["end_event"] 
      if end in node.event_time_dict and start in flat:
        print rule["hint"], node.event_time_dict[end] - min(flat[start]) 
    
  analyzer.analyze(analyzer_callback, statistic_rules)
