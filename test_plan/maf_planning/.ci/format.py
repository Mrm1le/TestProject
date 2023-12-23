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


def _run_cpp(sub_folders,
             file_types=["*.cpp", "*.cc", "*.c", "*.hpp", "*.h"],
             format_file=".clang-format"):
  assert isinstance(file_types, list)
  assert format_file.strip()
  root = get_root_path(format_file)
  print "check in [%s] with [%s]" % (", ".join(sub_folders),
                                     ", ".join(file_types))
  for folder in sub_folders:
    for file_type in file_types:
      cmd = "find %s/%s -name %s | xargs clang-format -i 2>/dev/null" % (
        root, folder, file_type)
      os.system(cmd)


def _run_py(sub_folders, file_types=["*.py"], format_file=".style.cfg"):
  assert isinstance(file_types, list)
  assert format_file.strip()
  root = get_root_path(format_file)
  print "check in [%s] with [%s]" % (", ".join(sub_folders),
                                     ", ".join(file_types))
  for folder in sub_folders:
    for file_type in file_types:
      cmd = "yapf --style %s/%s -i -r %s/%s %s" % (root, format_file, root,
                                                   folder, file_type)
      os.system(cmd)


def run():
  sub_folders = [
    "src", "include/msd", "sample", "adapter/mfr", "adapter/ap",
    "adapter/ros/src"
  ]
  _run_cpp(sub_folders)
  sub_folders = [".ci", "pnc_offline_debug_scripts"]
  _run_py(sub_folders)


def _check_cpp(sub_folders,
               file_types=["*.cpp", "*.cc", "*.c", "*.hpp", "*.h"],
               format_file=".clang-format"):
  assert isinstance(file_types, list)
  assert format_file.strip()
  root = get_root_path(format_file)
  print "check in [%s] with [%s]" % (", ".join(sub_folders),
                                     ", ".join(file_types))
  for folder in sub_folders:
    for file_type in file_types:
      try:
        cmd = "find %s/%s -name %s | xargs clang-format -output-replacements-xml | grep -c '<replacement '" % (
          root, folder, file_type)
        result = subprocess.check_output(cmd, shell=True)
        error("not all %s in %s/%s is formatted" % (file_type, root, folder))
      except Exception as e:
        continue


def _check_py(sub_folders, file_types=["*.py"], format_file=".style.cfg"):
  assert isinstance(file_types, list)
  assert format_file.strip()
  root = get_root_path(format_file)
  print "check in [%s] with [%s]" % (", ".join(sub_folders),
                                     ", ".join(file_types))
  for folder in sub_folders:
    for file_type in file_types:
      try:
        cmd = "yapf --style %s/%s -d -r %s/%s '%s'" % (root, format_file, root,
                                                       folder, file_type)
        result = subprocess.check_output(cmd, shell=True)
      except subprocess.CalledProcessError as e:
        error("not all %s in %s/%s is formatted" % (file_type, root, folder))


def check():
  sub_folders = [
    "src", "include/msd", "sample", "adapter/mfr", "adapter/ap",
    "adapter/ros/src"
  ]
  _check_cpp(sub_folders)
  sub_folders = [".ci"]
  _check_py(sub_folders)


if __name__ == "__main__":
  parser = argparse.ArgumentParser()
  parser.add_argument("action",
                      choices=["check", "run"],
                      nargs="?",
                      default="run",
                      help="The actions")
  args = parser.parse_args()
  if (args.action == "run"):
    run()
  elif (args.action == "check"):
    check()
  exit(0)
