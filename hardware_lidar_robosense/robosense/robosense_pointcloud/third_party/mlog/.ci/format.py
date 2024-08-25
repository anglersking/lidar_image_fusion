#!/usr/bin/env python2 

import os
import subprocess 
import argparse

def error(msg=""):  
  print "error: %s" % msg
  exit(1)

def get_root_path(anchor=".clang-format"):
  path = os.path.abspath(__file__)
  while True:
    path = os.path.dirname(path)
    if (os.path.exists(path + "/" + anchor)):
      return path
    if (path == "/"): 
      error("%s not found" % anchor)

def run():
  file_types = ["*.cpp", "*.c", "*.hpp", "*.h"]
  sub_folders = ["core/src", "core/include", "sample"]
  format_file = ".clang-format"
  root = get_root_path(format_file)
  print "format in [%s] with [%s]" % (", ".join(sub_folders), ", ".join(file_types)) 
  for folder in sub_folders:
    for file_type in file_types:
      cmd = "find %s/%s -name %s | xargs clang-format -i 2>/dev/null" %(
              root, folder, file_type) 
      os.system(cmd)

def check():
  file_types = ["*.cpp", "*.c", "*.hpp", "*.h"]
  sub_folders = ["core/src", "core/include", "sample"]
  format_file = ".clang-format"
  root = get_root_path(format_file)
  print "check in [%s] with [%s]" % (", ".join(sub_folders), ", ".join(file_types)) 
  for folder in sub_folders:
    for file_type in file_types:
      try:
         cmd = "find %s/%s -name %s | xargs clang-format -output-replacements-xml | grep -c '<replacement '" %(
              root, folder, file_type) 
         result = subprocess.check_output(cmd, shell=True)
         error("not all %s in %s/%s is formatted" % (file_type, root, folder))
      except Exception, e:
         continue

if __name__ == "__main__":
  parser = argparse.ArgumentParser()
  parser.add_argument("action", choices=["check", "run"], nargs="?", default="run", help="The actions")
  args = parser.parse_args()
  if (args.action == "run"):
      run()
  elif (args.action == "check"):
      check()
  exit(0)
