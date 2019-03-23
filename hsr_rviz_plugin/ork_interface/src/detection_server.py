#!/usr/bin/env python
# -*- coding:UTF-8 -*-

#
# Software License Agreement (BSD License)
#
# Copyright (c) 2019, Foshan Huashu Robotics Co.,Ltd
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#     * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#      * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#      * Neither the name of the Southwest Research Institute, nor the names
#      of its contributors may be used to endorse or promote products derived
#      from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#

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
    # 声明了一个 'detection'的服务， detection_srv 调用handles_detection回调函数
    s = rospy.Service('detection', detection_srv, handles_detection)
    rate = rospy.Rate(10.0)
    global pid
    pid = os.getpid()
    print "Ready to detect object."
    rospy.spin()

if __name__ == "__main__":
    sys.argv[1] = "nothing"
    detection_server()
