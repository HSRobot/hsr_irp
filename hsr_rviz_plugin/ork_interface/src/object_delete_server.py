#!/usr/bin/env python
# -*- coding:UTF-8 -*-

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
