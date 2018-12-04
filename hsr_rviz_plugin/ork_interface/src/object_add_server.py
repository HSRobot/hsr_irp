#!/usr/bin/env python
# -*- coding:UTF-8 -*-

# Server that adds an object to the database

import rospy

from object_recognition_core.db import models
from object_recognition_core.db.tools import args_to_db_params
import object_recognition_core.db.tools as dbtools
import couchdb

from ork_interface.srv import *

DEFAULT_DB_ROOT = 'http://localhost:5984'

def handles_object_add(req):
    print "Object Name:", req.name
    obj = models.Object(object_name=req.name,description=req.description)
    couch = couchdb.Server(DEFAULT_DB_ROOT)
    db = dbtools.init_object_databases(couch)
    objects = db
    existing = models.Object.by_object_name(objects, key=obj.object_name)
    store_new = True
    print('*****************test****************************.')	
    if len(existing) > 0:
        print('It appears that there are %d object(s) with the same name.' % len(existing))
        for x in existing:
            print(x)
            print('Use the object id above? [y,n]')
            use_it = raw_input('')
            if 'y' in use_it.lower():
                store_new = False
                obj = x
                break
    else:
        store_new = True
    if store_new:
        obj.store(objects)
        print('Stored new object with id:', obj.id)
        print('Use the --commit option to commit the proposed change.')
    newObj_ID = obj.id 		
    return objAddResponse(newObj_ID)
			
def object_add_server():
    # 初始化节点 用节点 发布服务数据
    rospy.init_node('object_add_server')
    # 声明了一个 'object_add'的服务， 数据类型为objAdd， 调用handles_object_add回调函数
    s = rospy.Service('object_add', objAdd, handles_object_add)
    print "Ready to add a object."
    rospy.spin()
		
if __name__ == "__main__":
    object_add_server()
