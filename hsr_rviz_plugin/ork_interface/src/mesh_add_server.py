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

# Server that adds a mesh to an object in the database

import rospy

from object_recognition_core.db import models
from object_recognition_core.db.tools import args_to_db_params
import object_recognition_core.db.tools as dbtools
import couchdb

from ork_interface.srv import *

DEFAULT_DB_ROOT = 'http://localhost:5984'

'''
def handles_mesh_add(req):
    parser = argparse.ArgumentParser(description='Add a mesh to an object.', fromfile_prefix_chars='@')
    parser.add_argument('object_id', metavar='OBJECT_ID', type=str, help='ID of the object to add a mesh to.')
    parser.add_argument('mesh_original', metavar='MESH_ORIGINAL', type=str, help='Original mesh of the object.')
    dbtools.add_db_arguments(parser)

    args = parser.parse_args()

    return args
'''

def handles_mesh_add(req):
    #args = parse_args()

    couch = couchdb.Server(DEFAULT_DB_ROOT)
    db = dbtools.init_object_databases(couch)
    # dbtools.upload_mesh(db, args.object_id, args.mesh_original, cloud_path=None, mesh_path=None)
    # dbtools.upload_mesh(db, req.id, req.path, cloud_path=None, mesh_path=None)
    dbtools.upload_mesh(db, req.id, req.path, cloud_path=None, mesh_path=None)
    print('Stored mesh for object id :', req.id)
    print('Use the --commit option to commit the proposed change.')
    return meshAddResponse()

def mesh_add_server():
    # 初始化节点 用节点 发布服务数据
    rospy.init_node('mesh_add_server')
    # 声明了一个 'mesh_add'的服务， 数据类型为meshAdd， 调用handles_mesh_add回调函数
    s = rospy.Service('mesh_add', meshAdd, handles_mesh_add)
    print "Ready to add mesh."
    rospy.spin()
		
if __name__ == "__main__":
    mesh_add_server()
