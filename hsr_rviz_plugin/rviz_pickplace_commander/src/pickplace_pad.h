/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2019, Foshan Huashu Robotics Co.,Ltd
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *      * Redistributions of source code must retain the above copyright
 *      notice, this list of conditions and the following disclaimer.
 *      * Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in the
 *      documentation and/or other materials provided with the distribution.
 *      * Neither the name of the Southwest Research Institute, nor the names
 *      of its contributors may be used to endorse or promote products derived
 *      from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef CONTROL_PAD_H
#define CONTROL_PAD_H

/* ros头文件 */
#include <ros/ros.h>
#include <ros/console.h>
#include <rviz/panel.h>                                     /* plugin基类的头文件 */

#include "hsr_gripper_driver/serial_open_srv.h"
#include "hsr_gripper_driver/close_srv.h"
#include "hsr_gripper_driver/open_srv.h"
#include "hsr_gripper_driver/stop_srv.h"
#include "hsr_gripper_driver/open_size_srv.h"
#include "hsr_gripper_driver/read_open_size_srv.h"

#include <visualization_msgs/InteractiveMarkerFeedback.h>   /* 从模型读取坐标的头文件 */
#include <geometry_msgs/Pose.h>                             /* 读取笛卡尔姿态头文件 */ 
#include <geometry_msgs/PoseStamped.h>
//#include <tf>
#include <tf/transform_listener.h>
#include <object_recognition_msgs/RecognizedObjectArray.h>

#include "serial/serial.h"

#include "ork_interface/objSearch.h"
#include "ork_interface/objAdd.h"
#include "ork_interface/objDelete.h"
#include "ork_interface/meshAdd.h"
#include "ork_interface/training_srv.h"
#include "ork_interface/detection_srv.h"

#include "hsr_pick/pickPlace.h"

/* qt头文件*/
#include<QVBoxLayout>

#include <QTimer>

#include <QFileDialog>
#include <QFileInfo>
#include <QTextCodec>

#include "SerialWdg.h"
#include "GripperWdg.h"
#include "OrkWdg.h"
#include "PickPlaceWdg.h"

/* 数学库 */
#include <cmath>

/* 其他 */
#include <glob.h>
#include <stdio.h>
#include <string.h>
#include <vector>

/* 宏定义及全局变量 */
#define PI 3.1415926

typedef struct
{
   std::string name;
   std::string id;
   std::string description;
}TRAINED_MODEL;

typedef struct
{
   float X;
   float Y;
   float Z;
   float eulerR;
   float eulerP;
   float eulerY;
   float quaterX;
   float quaterY;
   float quaterZ;
   float quaterW;
}T_POSE_FORM_CAMERA;

namespace rviz_pickplace_commander
{
    /* 所有的plugin都必须是rviz::Panel的子类 */
class PickPlacePanel: public rviz::Panel
{
    Q_OBJECT

public:
    PickPlacePanel( QWidget* parent = 0 );	
	~PickPlacePanel();
	
	/* 可识别物体列表 初始化 */
	void canRecognizedModelListInit();
	
	/*命令行执行命令并输出结果*/
	void executeCMD(const char *cmd, char *result);
 
    /* 公共槽 */
public Q_SLOTS:
   
protected Q_SLOTS:

protected:
	/* Qt相关 */
    SerialWdg *m_serialWdg;                        /* 串口模块 */
    GripperWdg *m_gripperWdg;                      /* 夹爪模块 */  
	OrkWdg *m_orkWdg;                              /* 识别与定位模块 */
	PickPlaceWdg *m_pickPlaceWdg;                  /* 抓取与放置模块 */
	
	QVBoxLayout *mainLayout;                       /* 整体布局 */
	
	QTimer *getPickPoseFromORK_Timer;              /* 从ORK模块获取抓取物坐标 实时刷新定时器 */
	QTimer *syncGetSerialDevNo_Timer;              /* 获取串口设备号 实时刷新定时器 */
    //QTimer *showPoseFromCamera_Timer;              /* 从Camera获取位姿 实时刷新定时器 */

	/* ROS相关 */
	ros::NodeHandle n_pickPlace;                   /* 节点句柄 */
	ros::NodeHandle n_runPickPlace;                /* 节点句柄 */
	ros::ServiceClient client_serialOpen;          /* 串口打开客户端 */
    ros::ServiceClient client_gripperOpenSize;     /* 夹爪开口大小客户端 */
	ros::ServiceClient client_gripperOpen;         /* 夹爪打开客户端 */
	ros::ServiceClient client_gripperClose;        /* 夹爪关闭客户端 */
	ros::ServiceClient client_gripperStop;         /* 夹爪关闭客户端 */
	
	ros::ServiceClient client_objDelete;           /* 删除识别物体客户端 */
	ros::ServiceClient client_objAdd;              /* 添加识别物体客户端 */
	ros::ServiceClient client_meshAdd;             /* 添加物体mesh客户端 */
	ros::ServiceClient client_objSearch;           /* 查找识别物体客户端 */
	ros::ServiceClient client_training;            /* 训练模型客户端 */
	ros::ServiceClient client_detection;           /* 训练模型客户端 */
	
	ros::ServiceClient client_pickPlace;           /* 抓取与放置客户端 */
	
	ros::Subscriber sub_poseFrom3d;                /* 订阅读取位姿话题，从三维环境中读取位姿信息 */
	ros::Subscriber sub_poseFromCamera;            /* 订阅从相机读取位姿话题 */
	
private:
    int openSizeMax;
	int openSizeMin;
	int gripperSpeed;
	int gripperForce;
	
	TRAINED_MODEL *m_trainingMdoel;
	T_POSE_FORM_CAMERA *m_poseFromCamera;
    T_POSE_FORM_CAMERA *m_poseFrom3ds;
	
	QString newAddModel_ID;
	
	//std::vector<geometry_msgs::PoseStamped> detectPoseFromCamera;
	//std::vector<geometry_msgs::PoseStamped> base_detectPoseFromCamera;
	geometry_msgs::PoseStamped *detectPoseFromCamera;
	geometry_msgs::PoseStamped *base_detectPoseFromCamera;
    geometry_msgs::PoseStamped *detectPoseFrom3d;
    geometry_msgs::PoseStamped *base_detectPoseFrom3d;

    int num_detectedObj;          /*本次监测到的识别物体值*/
    int pro_num_detectedObj;      /*上一次监测到的识别物体值*/

    std::string path_detect_config;  /* 识别配置文件路径 */
	
private Q_SLOTS:
	
	void slotSerialNoComoBoxClicked();                     /* 点击串口设备选择下拉框 槽函数 */
	
	void slotSerialConnectBtn();                           /* 串口连接按钮槽函数 */
	void slotGripperParmSetComboBox(int index);            /* 夹爪参数设置项切换槽函数 */
	void slotGripperSetBtn();                              /* 夹爪设置按钮槽函数 */
	void slotGripperActRunBtn();                           /* 夹爪动作执行按钮槽函数 */
	
	void slotAddOrDeleteComboBox(int index);               /* 新增或删除模型选择下拉框 切换槽函数 */
	void slotToSelectDeleteComoBox(int index);             /* 待删除模型选择下拉框 切换槽函数 */
	
	//void slotCanRecognModelComboBox(int index);          /* 可识别模型选择下拉框 切换槽函数 */
	void slotNewAddOrDeleteOkBtn();                        /* 新增加或删除模型确认按钮槽函数 */
	void slotModelPathOpenBtn();                           /* 打开待训练模型路径按钮槽函数 */
	void slotAddMeshOkBtn();                               /* 新增模型Mesh确认按钮槽函数 */
	void slotToTrainBtn();                                 /* 开始训练按钮槽函数*/
	void slotToRecognBtn();                                /* 开始识别按钮槽函数 */

   // void slotShowPoseFromCameraTimer();                    /* 从Camera获取位姿 实时刷新定时器 槽函数  */
	
	void slotToSelectDetectComoBox(int index);             /* 待选择识别物选择下拉框 切换槽函数 */
	void slotGetPickPoseFromOrk();                         /* 从ORK模块获取抓取物坐标 槽函数 */
	
	void slotGetPickPoseMeansComoBox(int index);           /* 获取抓取目标点坐标方式选择下拉框 切换槽函数 */
	void slotGetPlacePoseMeansComoBox(int index);          /* 获取放置目标点坐标方式选择下拉框 切换槽函数 */
	void slotGetPlacePoseOkBtn();                          /* 获取放置位姿确认按钮 槽函数*/
	
	void poseFromCamera_callback(const object_recognition_msgs::RecognizedObjectArray::ConstPtr& msg);        /* 从三维环境获取坐标 回调函数 */
	void poseFrom3d_callback(const visualization_msgs::InteractiveMarkerFeedback::ConstPtr& msg);            /* 从三维环境获取坐标 回调函数 */
	
	void slotRunPickPlaceBtn();                            /* 执行抓取与放置 槽函数 */
	
};
 
} // end namespace rviz_control_commander

#endif
