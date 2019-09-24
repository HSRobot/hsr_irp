/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2019, Foshan Huashu Robotics Co.,Ltd
 * All rights reserved.
 * 
 * Author: Kunlin Xu <1125290220@qq.com>
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

#include <moveit/move_group_interface/move_group_interface.h>
#include <hsr_msgs/Joints.h>
#include <iostream>
#define DEBUG
moveit::planning_interface::MoveGroupInterface *group;

/* --------------直角空间运动接口回调函数-----------------*/
void poseCallback(const geometry_msgs::Pose::ConstPtr& msg){
#ifdef DEBUG
	std::cout << "x: " << msg->orientation.x << std::endl;
	std::cout << "y: " << msg->orientation.y << std::endl;
	std::cout << "z: " << msg->orientation.z << std::endl;
	std::cout << "w: " << msg->orientation.w << std::endl;

	std::cout << "x: " << msg->position.x << std::endl;
	std::cout << "y: " << msg->position.y << std::endl;
	std::cout << "z: " << msg->position.z << std::endl;
#endif
	group->setPoseTarget(*msg);
}

/* --------------关节空间运动接口回调函数-----------------*/
void jointCallback(const hsr_msgs::Joints::ConstPtr& msg){

#ifdef DEBUG
	std::cout << "joint1: " << msg->joint1 << std::endl;
	std::cout << "joint2: " << msg->joint2 << std::endl;
	std::cout << "joint3: " << msg->joint3 << std::endl;
	std::cout << "joint4: " << msg->joint4 << std::endl;
	std::cout << "joint5: " << msg->joint5 << std::endl;
	std::cout << "joint6: " << msg->joint6 << std::endl;
#endif 
	std::vector<double> joint_pos(6);

	joint_pos[0] = msg->joint1;
	joint_pos[1] = msg->joint2;
	joint_pos[2] = msg->joint3;
	joint_pos[3] = msg->joint4;
	joint_pos[4] = msg->joint5;
	joint_pos[5] = msg->joint6;	

	group->setJointValueTarget(joint_pos);
	group->move();
}

int main(int argc, char **argv){

	ros::init(argc, argv, "hsr_moveit_api");
	ros::NodeHandle n;

	std::string groupName;

	// 启动消息循环
	ros::AsyncSpinner spinner(1);
  	spinner.start();

	// 发布 robot_state的接口
	ros::Publisher pose_pub = n.advertise<geometry_msgs::PoseStamped>("robot_pose", 1000);
	
	// 订阅 target_pose话题
	ros::Subscriber pose_sub = n.subscribe("target_pose", 1000, poseCallback);

	// 订阅 target_joints话题
	ros::Subscriber joint_sub = n.subscribe("target_joint", 1000, jointCallback);

	// 获取轴组名称
	n.param<std::string>("move_group_name", groupName, "arm");

	// 初始化movit的接口
	//group = new move_group_interface::MoveGroup(groupName);//ros_indigo
        group = new moveit::planning_interface::MoveGroupInterface(groupName);

	// 发布频率
	ros::Rate loop_rate(20);
	
	std::vector<double> rpy;

	while(ros::ok()){

		// 保存Pose信息的变量
		geometry_msgs::PoseStamped now_pose;	

		// 获取当前笛卡尔坐标
		now_pose = group->getCurrentPose();

		// 获取当前RPY
		//rpy = group->getCurrentRPY();

		// 发布当前笛卡尔坐标
		pose_pub.publish(now_pose);
                 //group->setRandomTarget();
                 //std::cout<< group->move()<<std::endl;

		// 休眠
		loop_rate.sleep();
		
		//std::cout << "rpy = " << rpy[0] << " || " << rpy[1] << " || " << rpy[2] << std::endl; 
	
	}
}
