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

#include <ros/ros.h>
#include <ros/console.h>
#include <rviz/panel.h>                                    

#include "hsr_gripper_driver/serial_open_srv.h"
#include "hsr_gripper_driver/close_srv.h"
#include "hsr_gripper_driver/open_srv.h"
#include "hsr_gripper_driver/stop_srv.h"
#include "hsr_gripper_driver/open_size_srv.h"
#include "hsr_gripper_driver/read_open_size_srv.h"

#include <visualization_msgs/InteractiveMarkerFeedback.h>   
#include <geometry_msgs/Pose.h>                             
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

#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h> 
#include <moveit_msgs/DisplayTrajectory.h> 
#include <moveit_msgs/AttachedCollisionObject.h> 
#include <moveit_msgs/CollisionObject.h> 

#include <rviz_pickplace_commander/voicePickCoke.h>
#include <rviz_pickplace_commander/voicePickDiamond.h>

#include "hsr_pick/pickPlace.h"

#include<QVBoxLayout>

#include <QTimer>

#include <QFileDialog>
#include <QFileInfo>
#include <QTextCodec>

#include "SerialWdg.h"
#include "GripperWdg.h"
#include "OrkWdg.h"
#include "PickPlaceWdg.h"


#include <cmath>

#include <glob.h>
#include <stdio.h>
#include <string.h>
#include <vector>
#include <sstream>

#define PI 3.1415926
#define COKE_ID "c74677f2dac7ae441e6fb2901d00285a"
#define DIAMOND_ID "c74677f2dac7ae441e6fb2901d00285a"

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
  
class PickPlacePanel: public rviz::Panel
{
    Q_OBJECT

public:
    PickPlacePanel( QWidget* parent = 0 );	
	~PickPlacePanel();
	
	
	void canRecognizedModelListInit();
	
	void executeCMD(const char *cmd, char *result);
 
public Q_SLOTS:
   
protected Q_SLOTS:

protected:
        SerialWdg *m_serialWdg;                       
        GripperWdg *m_gripperWdg;                     
	OrkWdg *m_orkWdg;                            
	PickPlaceWdg *m_pickPlaceWdg;                 
	
	QVBoxLayout *mainLayout;                     
	
	QTimer *getPickPoseFromORK_Timer;             
	QTimer *syncGetSerialDevNo_Timer;             
    //QTimer *showPoseFromCamera_Timer;             

	
	ros::NodeHandle n_pickPlace;                 
	ros::NodeHandle n_runPickPlace;               
	ros::ServiceClient client_serialOpen;        
        ros::ServiceClient client_gripperOpenSize;     
	ros::ServiceClient client_gripperOpen;        
	ros::ServiceClient client_gripperClose;       
	ros::ServiceClient client_gripperStop;        
	
	ros::ServiceClient client_objDelete;           
	ros::ServiceClient client_objAdd;            
	ros::ServiceClient client_meshAdd;          
	ros::ServiceClient client_objSearch;      
	ros::ServiceClient client_training;         
	ros::ServiceClient client_detection;         
	
	ros::ServiceClient client_pickPlace;

        ros::ServiceServer voice_pick_coke;         
	ros::ServiceServer voice_pick_diamond;

	ros::Subscriber sub_poseFrom3d;              
	ros::Subscriber sub_poseFromCamera;
        moveit::planning_interface::PlanningSceneInterface planning_scene_interface;     
	
private:
        int openSizeMax;
	int openSizeMin;
	int gripperSpeed;
	int gripperForce;
	
	TRAINED_MODEL *m_trainingMdoel;
	T_POSE_FORM_CAMERA *m_poseFromCamera;
        T_POSE_FORM_CAMERA *m_poseFrom3ds;
	
	QString newAddModel_ID;
        QString detectionModel_ID;
	
	//std::vector<geometry_msgs::PoseStamped> detectPoseFromCamera;
	//std::vector<geometry_msgs::PoseStamped> base_detectPoseFromCamera;
	geometry_msgs::PoseStamped *detectPoseFromCamera;
	geometry_msgs::PoseStamped *base_detectPoseFromCamera;
        geometry_msgs::PoseStamped *detectPoseFrom3d;
        geometry_msgs::PoseStamped *base_detectPoseFrom3d;

        int num_detectedObj;         
        int pro_num_detectedObj;    
        std::string detection_ID[10];
        std::string path_detect_config;
        std::stringstream os;  
  
        moveit_msgs::CollisionObject collision_object;
    
        shape_msgs::SolidPrimitive primitive;
        geometry_msgs::Pose box_pose;    
        std::vector<moveit_msgs::CollisionObject> collision_objects;
        std::vector<std::string> object_ids;
	
private Q_SLOTS:
	
	void slotSerialNoComoBoxClicked();                  
	
	void slotSerialConnectBtn();                        
	void slotGripperParmSetComboBox(int index);        
	void slotGripperSetBtn();                          
	void slotGripperActRunBtn();                        
	
	void slotAddOrDeleteComboBox(int index);            
	void slotToSelectDeleteComoBox(int index);           
	
	//void slotCanRecognModelComboBox(int index);        
	void slotNewAddOrDeleteOkBtn();                     
	void slotModelPathOpenBtn();                         
	void slotAddMeshOkBtn();                            
	void slotToTrainBtn();                               
	void slotToRecognBtn();                              

   // void slotShowPoseFromCameraTimer();                   
	
	void slotToSelectDetectComoBox(int index);            
	void slotGetPickPoseFromOrk();                       
	
	void slotGetPickPoseMeansComoBox(int index);           
	void slotGetPlacePoseMeansComoBox(int index);        
	void slotGetPlacePoseOkBtn();                       
	
	void poseFromCamera_callback(const object_recognition_msgs::RecognizedObjectArray::ConstPtr& msg);      
	void poseFrom3d_callback(const visualization_msgs::InteractiveMarkerFeedback::ConstPtr& msg);           
	
	void slotRunPickPlaceBtn();  
        void slotVoicePickPlaceBtn();  
       
        void addObjectBoxdefines(std::string detection_id,int num,geometry_msgs::Pose box_pose); 
        void addObjectBox();   
        void removeObjectBox();    
        void voicePickServer();
        bool voicePickCoke_callback(rviz_pickplace_commander::voicePickCoke::Request  &req,
                                      rviz_pickplace_commander::voicePickCoke::Response &res);
        bool voicePickDiamond_callback(rviz_pickplace_commander::voicePickDiamond::Request  &req,
                                         rviz_pickplace_commander::voicePickDiamond::Response &res);                    
	
};
 
} // end namespace rviz_control_commander

#endif
