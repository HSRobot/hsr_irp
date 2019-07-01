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
