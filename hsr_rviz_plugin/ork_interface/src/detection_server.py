#!/usr/bin/env python
# -*- coding:UTF-8 -*-

"""
This script launches several detection pipelines defined in a configuration file to detect several objects in a
common snapshot of data
"""

import threading
import rospy
from object_recognition_core.pipelines.plasm import create_plasm
from object_recognition_core.utils.training_detection_args import create_parser, read_arguments
from ecto.opts import scheduler_options, run_plasm
import argparse

from ork_interface.srv import *
import os

import ctypes
import time
import inspect
import signal

import time

t_detection = None
pid = 0

def stop_thread(thread):
    os.kill(pid, signal.SIGINT)

def execute_detection(cmd,path):
    parser = create_parser()

    # add ecto options
    scheduler_options(parser)
    pid = os.getpid()
    print("pid = ", pid)
    args = parser.parse_args(['-c', cmd])
    args = parser.parse_args(['--config_file', path])

    # create the plasm that will run the detection
    # args = parser.parse_args()
    ork_params, _args = read_arguments(args)
    plasm = create_plasm(ork_params)

    # run the detection plasm
    run_plasm(args, plasm)
    print "Exit Finished"

def handles_detection(req):
    global t_detection
    if req.isDetected:
        print "to be stop detecting."
        stop_thread(t_detection)
    else:
        t_detection= threading.Thread(target=execute_detection,args=(req.cmd,req.path))
        t_detection.setDaemon(True)
        t_detection.start()
        print "to be detecting."

    return detection_srvResponse()

def detection_server():
    # 初始化节点 用节点 发布服务数据
    rospy.init_node('detection_server', argv = None)
    # 声明了一个 'detection'的服务， detection_srv 调用handles_training回调函数
    s = rospy.Service('detection', detection_srv, handles_detection)
    rate = rospy.Rate(10.0)
    global pid
    pid = os.getpid()
    print "Ready to detect object."
    rospy.spin()

if __name__ == "__main__":
    sys.argv[1] = "nothing"
    detection_server()
