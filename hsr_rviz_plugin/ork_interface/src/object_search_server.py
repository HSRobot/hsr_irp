#!/usr/bin/env python
# -*- coding:UTF-8 -*-

import rospy

import object_recognition_core
from object_recognition_core.db import tools as dbtools, models
import couchdb

from ork_interface.srv import *

DEFAULT_DB_ROOT = 'http://localhost:5984'

def handles_object_search(req):
    couch = couchdb.Server(DEFAULT_DB_ROOT)
    db = dbtools.init_object_databases(couch)
    results = models.Object.by_object_name(db)
    name_list = []
    id_list = []
    description_list = []
    num = len(results)
    for obj in results:
        print "******************************"
        print "Object Name:", obj.object_name
        print "Object ID:", obj.id
        name_list.append(obj.object_name)
        id_list.append(obj.id)
        description_list.append(obj.description)
    return objSearchResponse(name_list,id_list,description_list,num)
		
def object_search_server():
    # 初始化节点 用节点 发布服务数据
    rospy.init_node('object_search_server')
    # 声明了一个 'object_search'的服务， 数据类型为objSearch， 调用handles_object_search回调函数
    s = rospy.Service('object_search', objSearch, handles_object_search)
    print "Ready to search object."
    rospy.spin()
		
if __name__ == "__main__":
    object_search_server()
