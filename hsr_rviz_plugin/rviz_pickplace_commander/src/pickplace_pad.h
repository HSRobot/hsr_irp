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

/* rosͷ�ļ� */
#include <ros/ros.h>
#include <ros/console.h>
#include <rviz/panel.h>                                     /* plugin�����ͷ�ļ� */

#include "hsr_gripper_driver/serial_open_srv.h"
#include "hsr_gripper_driver/close_srv.h"
#include "hsr_gripper_driver/open_srv.h"
#include "hsr_gripper_driver/stop_srv.h"
#include "hsr_gripper_driver/open_size_srv.h"
#include "hsr_gripper_driver/read_open_size_srv.h"

#include <visualization_msgs/InteractiveMarkerFeedback.h>   /* ��ģ�Ͷ�ȡ�����ͷ�ļ� */
#include <geometry_msgs/Pose.h>                             /* ��ȡ�ѿ�����̬ͷ�ļ� */ 
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

/* qtͷ�ļ�*/
#include<QVBoxLayout>

#include <QTimer>

#include <QFileDialog>
#include <QFileInfo>
#include <QTextCodec>

#include "SerialWdg.h"
#include "GripperWdg.h"
#include "OrkWdg.h"
#include "PickPlaceWdg.h"

/* ��ѧ�� */
#include <cmath>

/* ���� */
#include <glob.h>
#include <stdio.h>
#include <string.h>
#include <vector>

/* �궨�弰ȫ�ֱ��� */
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
    /* ���е�plugin��������rviz::Panel������ */
class PickPlacePanel: public rviz::Panel
{
    Q_OBJECT

public:
    PickPlacePanel( QWidget* parent = 0 );	
	~PickPlacePanel();
	
	/* ��ʶ�������б� ��ʼ�� */
	void canRecognizedModelListInit();
	
	/*������ִ�����������*/
	void executeCMD(const char *cmd, char *result);
 
    /* ������ */
public Q_SLOTS:
   
protected Q_SLOTS:

protected:
	/* Qt��� */
    SerialWdg *m_serialWdg;                        /* ����ģ�� */
    GripperWdg *m_gripperWdg;                      /* ��צģ�� */  
	OrkWdg *m_orkWdg;                              /* ʶ���붨λģ�� */
	PickPlaceWdg *m_pickPlaceWdg;                  /* ץȡ�����ģ�� */
	
	QVBoxLayout *mainLayout;                       /* ���岼�� */
	
	QTimer *getPickPoseFromORK_Timer;              /* ��ORKģ���ȡץȡ������ ʵʱˢ�¶�ʱ�� */
	QTimer *syncGetSerialDevNo_Timer;              /* ��ȡ�����豸�� ʵʱˢ�¶�ʱ�� */
    //QTimer *showPoseFromCamera_Timer;              /* ��Camera��ȡλ�� ʵʱˢ�¶�ʱ�� */

	/* ROS��� */
	ros::NodeHandle n_pickPlace;                   /* �ڵ��� */
	ros::NodeHandle n_runPickPlace;                /* �ڵ��� */
	ros::ServiceClient client_serialOpen;          /* ���ڴ򿪿ͻ��� */
    ros::ServiceClient client_gripperOpenSize;     /* ��צ���ڴ�С�ͻ��� */
	ros::ServiceClient client_gripperOpen;         /* ��צ�򿪿ͻ��� */
	ros::ServiceClient client_gripperClose;        /* ��צ�رտͻ��� */
	ros::ServiceClient client_gripperStop;         /* ��צ�رտͻ��� */
	
	ros::ServiceClient client_objDelete;           /* ɾ��ʶ������ͻ��� */
	ros::ServiceClient client_objAdd;              /* ���ʶ������ͻ��� */
	ros::ServiceClient client_meshAdd;             /* �������mesh�ͻ��� */
	ros::ServiceClient client_objSearch;           /* ����ʶ������ͻ��� */
	ros::ServiceClient client_training;            /* ѵ��ģ�Ϳͻ��� */
	ros::ServiceClient client_detection;           /* ѵ��ģ�Ϳͻ��� */
	
	ros::ServiceClient client_pickPlace;           /* ץȡ����ÿͻ��� */
	
	ros::Subscriber sub_poseFrom3d;                /* ���Ķ�ȡλ�˻��⣬����ά�����ж�ȡλ����Ϣ */
	ros::Subscriber sub_poseFromCamera;            /* ���Ĵ������ȡλ�˻��� */
	
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

    int num_detectedObj;          /*���μ�⵽��ʶ������ֵ*/
    int pro_num_detectedObj;      /*��һ�μ�⵽��ʶ������ֵ*/

    std::string path_detect_config;  /* ʶ�������ļ�·�� */
	
private Q_SLOTS:
	
	void slotSerialNoComoBoxClicked();                     /* ��������豸ѡ�������� �ۺ��� */
	
	void slotSerialConnectBtn();                           /* �������Ӱ�ť�ۺ��� */
	void slotGripperParmSetComboBox(int index);            /* ��צ�����������л��ۺ��� */
	void slotGripperSetBtn();                              /* ��צ���ð�ť�ۺ��� */
	void slotGripperActRunBtn();                           /* ��צ����ִ�а�ť�ۺ��� */
	
	void slotAddOrDeleteComboBox(int index);               /* ������ɾ��ģ��ѡ�������� �л��ۺ��� */
	void slotToSelectDeleteComoBox(int index);             /* ��ɾ��ģ��ѡ�������� �л��ۺ��� */
	
	//void slotCanRecognModelComboBox(int index);          /* ��ʶ��ģ��ѡ�������� �л��ۺ��� */
	void slotNewAddOrDeleteOkBtn();                        /* �����ӻ�ɾ��ģ��ȷ�ϰ�ť�ۺ��� */
	void slotModelPathOpenBtn();                           /* �򿪴�ѵ��ģ��·����ť�ۺ��� */
	void slotAddMeshOkBtn();                               /* ����ģ��Meshȷ�ϰ�ť�ۺ��� */
	void slotToTrainBtn();                                 /* ��ʼѵ����ť�ۺ���*/
	void slotToRecognBtn();                                /* ��ʼʶ��ť�ۺ��� */

   // void slotShowPoseFromCameraTimer();                    /* ��Camera��ȡλ�� ʵʱˢ�¶�ʱ�� �ۺ���  */
	
	void slotToSelectDetectComoBox(int index);             /* ��ѡ��ʶ����ѡ�������� �л��ۺ��� */
	void slotGetPickPoseFromOrk();                         /* ��ORKģ���ȡץȡ������ �ۺ��� */
	
	void slotGetPickPoseMeansComoBox(int index);           /* ��ȡץȡĿ������귽ʽѡ�������� �л��ۺ��� */
	void slotGetPlacePoseMeansComoBox(int index);          /* ��ȡ����Ŀ������귽ʽѡ�������� �л��ۺ��� */
	void slotGetPlacePoseOkBtn();                          /* ��ȡ����λ��ȷ�ϰ�ť �ۺ���*/
	
	void poseFromCamera_callback(const object_recognition_msgs::RecognizedObjectArray::ConstPtr& msg);        /* ����ά������ȡ���� �ص����� */
	void poseFrom3d_callback(const visualization_msgs::InteractiveMarkerFeedback::ConstPtr& msg);            /* ����ά������ȡ���� �ص����� */
	
	void slotRunPickPlaceBtn();                            /* ִ��ץȡ����� �ۺ��� */
	
};
 
} // end namespace rviz_control_commander

#endif
