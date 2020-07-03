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

#include <algorithm>
#include "joint_trajectory_full_interface.h"
#include "simple_message/joint_traj_pt.h"
#include "simple_message/joint_traj_pt_full.h"
#include "industrial_utils/param_utils.h"



namespace industrial_robot_client
{
namespace joint_trajectory_interface
{
    JointTrajectoryFullInterface::JointTrajectoryFullInterface() : mode(FULL_PT) {}

    void JointTrajectoryFullInterface::jointTrajectoryCB(const trajectory_msgs::JointTrajectoryConstPtr &msg){
    ROS_ERROR("Receiving joint trajectory message");
	ROS_ERROR_STREAM("msg->points size "<<msg->points.size());
	// check for STOP command
	//<param name="execution_duration_monitoring" value="false"/>


	//首先注释了这段停止操作
	if (msg->points.empty())
	{
        ROS_ERROR("Empty trajectory received, canceling current trajectory");
        trajectoryStop();
		return;
  	}

	if(mode != FULL_PT){
           ROS_ERROR("jointTrajectoryCB set Mode Error, Please switch the continue mode ....");
           return;
	}


	// convert trajectory into robot-format
	std::vector<JointTrajPtFullMessage> robot_msgs;
	if (!trajectory_to_msgs(msg, &robot_msgs)){
		ROS_ERROR("trajectory_to_msgs ERROR ");
    	return;
	}

	// send command messages to robot
	send_to_robot(robot_msgs);
}


bool JointTrajectoryFullInterface::trajectory_to_msgs(const trajectory_msgs::JointTrajectoryConstPtr& traj, std::vector<JointTrajPtFullMessage>* msgs){

	msgs->clear();
	if (!is_valid(*traj))
		return false;

	int valid_fields = industrial::joint_traj_pt_full::ValidFieldTypes::TIME;
	valid_fields |= industrial::joint_traj_pt_full::ValidFieldTypes::POSITION;
	valid_fields |= industrial::joint_traj_pt_full::ValidFieldTypes::VELOCITY;
	valid_fields |= industrial::joint_traj_pt_full::ValidFieldTypes::ACCELERATION;
	

	for (size_t i=0; i<traj->points.size(); ++i)
  	{
		trajectory_msgs::JointTrajectoryPoint rbt_pt;

		rbt_pt = traj->points[i];		// 获取第一个点位
		// rbt_pt.duration.toNSec()
    	JointTrajPtFullMessage msg = create_message(0, i, valid_fields, rbt_pt.time_from_start.toSec(), rbt_pt.positions, \
			rbt_pt.velocities, rbt_pt.accelerations);		// 构建消息

    	msgs->push_back(msg);
  	}

  	return true;	
}


JointTrajPtFullMessage JointTrajectoryFullInterface::create_message(int robot_id, int seq, int valid_fields, float time, std::vector<double> joint_pos, \
		std::vector<double> velocity, std::vector<double> accelerations){
	
  	industrial::joint_data::JointData pos, vel, acc;
  	ROS_ASSERT(joint_pos.size() <= (unsigned int)pos.getMaxNumJoints());


  	for (size_t i=0; i<joint_pos.size(); ++i){
    	pos.setJoint(i, joint_pos[i]);
		vel.setJoint(i, velocity[i]);
		acc.setJoint(i, accelerations[i]);
	}

    ROS_DEBUG("time=%f \n", time);

  	industrial::joint_traj_pt_full::JointTrajPtFull pt;
  	pt.init(robot_id, seq, valid_fields, time, pos, vel, acc);

  	JointTrajPtFullMessage msg;
  	msg.init(pt);

  	return msg;
}

JointTrajPtMessage JointTrajectoryFullInterface::create_message( const std::vector<double> &joint_pos,int seq) {
    JointTrajPtMessage ptMessage;
    industrial::joint_traj_pt::JointTrajPt pt;
    industrial::joint_data::JointData pos;
    for(int i = 0; i < 6; i++){
        pos.setJoint(i, joint_pos.at(i));
    }
    ROS_ERROR_STREAM("Pos: "<<joint_pos.at(0)<<" "<<joint_pos.at(1)<<" "<<joint_pos.at(2)<<" "
                                    <<joint_pos.at(3)<<" "<<joint_pos.at(4) <<" "<<joint_pos.at(5));
    industrial::shared_types::shared_real vel,acc ;
    pt.init(seq,pos, vel, acc);
    ptMessage.init(pt);
    return ptMessage;
}


bool JointTrajectoryFullInterface::send_to_robot(const std::vector<JointTrajPtMessage>& messages)
{
    return false;
}

bool JointTrajectoryFullInterface::send_to_robot(int messg_type)
{
    RobotSysMessage msg;
    SimpleMessage simsg;
    msg.init(messg_type);

    if(!this->connection_->isConnected())
    {
       this->connection_->makeConnect();
    }

    msg.toTopic(simsg);

    bool ret = this->connection_->sendMsg(simsg);

    return ret;
}

bool JointTrajectoryFullInterface::setRobotEnableCB(hsr_rosi_device::SetEnableSrv::Request &req, hsr_rosi_device::SetEnableSrv::Response &res)
{
    if(req.enable)
    {
        res.finsh = send_to_robot(2600);
    }
    else
    {
        res.finsh = send_to_robot(2601);
    }
    return res.finsh;
}

bool JointTrajectoryFullInterface::stopRobotMovingCB(hsr_rosi_device::StopMoveSrv::Request &req, hsr_rosi_device::StopMoveSrv::Response &res)
{
    res.finsh = send_to_robot(2602);
    return res.finsh;
}

bool JointTrajectoryFullInterface::clearRobotFaultCB(hsr_rosi_device::ClearFaultSrv::Request &req, hsr_rosi_device::ClearFaultSrv::Response &res)
{
    res.finsh = send_to_robot(2603);
    return res.finsh;
}

bool JointTrajectoryFullInterface::service_start(ros::NodeHandle &n)
{
    n_rosi = n;
    set_enable_srv = n_rosi.advertiseService("set_robot_enable",&JointTrajectoryFullInterface::setRobotEnableCB,this);
    stop_move_srv = n_rosi.advertiseService("stop_robot_moving",&JointTrajectoryFullInterface::stopRobotMovingCB,this);
    clear_fault_srv = n_rosi.advertiseService("clear_robot_fault",&JointTrajectoryFullInterface::clearRobotFaultCB,this);
    set_mode_srv = n_rosi.advertiseService("set_mode_srv",&JointTrajectoryFullInterface::setRobotModelCB,this);
    get_mode_srv = n_rosi.advertiseService("get_mode_srv",&JointTrajectoryFullInterface::getRobotModelCB,this);
}

/**
* @brief impedanceCB 力矩控制反馈
* @param msg
*/
void JointTrajectoryFullInterface::impedanceCB(const sensor_msgs::JointState::ConstPtr &msg) {
//        send_to_robot(ptm);
}


bool JointTrajectoryFullInterface::setRobotModelCB(hsr_rosi_device::setModeSrv::Request &req,
                                                   hsr_rosi_device::setModeSrv::Response &res) {
    mode = req.mode;
    res.finsh = true;
    return true;
}

bool JointTrajectoryFullInterface::getRobotModelCB(hsr_rosi_device::getModeSrv::Request &req,
                                                   hsr_rosi_device::getModeSrv::Response &res) {

    res.mode = mode;
    return true;
}



}
}
