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

# Script that adds an object to the database

import rospy

import object_recognition_core.db.tools as dbtools
import couchdb
import argparse

from ork_interface.srv import *

DEFAULT_DB_ROOT = 'http://localhost:5984'

'''
def parse_args():
    parser = argparse.ArgumentParser(description='Delete objects from the database.')
    parser.add_argument('objects', metavar='OBJECTS', type=str, nargs='+',
                   help='Object ids to delete.')
    dbtools.add_db_arguments(parser)
    args = parser.parse_args()
    return args
'''

def deletion_view(db):
    view = couchdb.design.ViewDefinition('object_recognition', 'refer_object_id', '''function(doc) {
         if (doc.object_id)
             emit(doc.object_id, doc);
    }''')
    view.sync(db)

'''
def delete_object(db, object_ids, commit):
    for object_id in object_ids:
        for row in db.view('object_recognition/refer_object_id', key=object_id):
            print('Deleting doc: %s of type "%s" referring to object: %s' % (row.id, row.value['Type'], row.key))
            if commit:
                del db[row.id]
        if not db.get(object_id):
             print("No object by id", object_id, "found!")
        elif commit:
            print("Deleting object:", object_id)
            del db[object_id]
'''
def handles_object_delete(req):
    couch = couchdb.Server(DEFAULT_DB_ROOT)
    db = dbtools.init_object_databases(couch)
    deletion_view(db)
    #delete_object(db, req.id, req.commit)
    del db[req.id]
    print('just kidding. --commit to actually do it.')
    return objDeleteResponse()

def object_delete_server():
    # 初始化节点 用节点 发布服务数据
    rospy.init_node('object_delete_server')
    # 声明了一个 'detection'的服务， objDelete 调用handles_training回调函数
    s = rospy.Service('object_delete', objDelete, handles_object_delete)
    print "Ready to delete object."
    rospy.spin()

if __name__ == "__main__":
    object_delete_server()
