#!/usr/bin/env python
# -*- coding:UTF-8 -*-

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
