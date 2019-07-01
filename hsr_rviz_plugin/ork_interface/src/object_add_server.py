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
