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

#ifndef JOINT_TRAJECTORY_FULL_INTERFACE_H
#define JOINT_TRAJECTORY_FULL_INTERFACE_H


#include <map>
#include <vector>
#include <string>


#include <industrial_robot_client/joint_trajectory_interface.h>
#include "simple_message/messages/joint_traj_pt_full_message.h"
#include "simple_message/messages/joint_traj_pt_message.h"
#include "robot_sys_message.h"
#include "hsr_rosi_device/SetEnableSrv.h"
#include "hsr_rosi_device/ClearFaultSrv.h"
#include "hsr_rosi_device/StopMoveSrv.h"
#include "hsr_rosi_device/setModeSrv.h"
#include "hsr_rosi_device/getModeSrv.h"
#define  FULL_PT 0
#define  ONCE_PT 1

namespace industrial_robot_client
{
namespace joint_trajectory_interface
{
	using industrial::joint_traj_pt_full_message::JointTrajPtFullMessage;
    using industrial::joint_traj_pt_message::JointTrajPtMessage;
    using industrial::robot_sys_message::RobotSysMessage;
    using industrial::simple_message::SimpleMessage;
	using industrial_robot_client::joint_trajectory_interface::JointTrajectoryInterface;
	namespace StandardSocketPorts = industrial::simple_socket::StandardSocketPorts;

 /**
 * @brief The JointTrajectoryFullInterface class 华数机器人的Ros轨迹点下发的功能类
 */
class JointTrajectoryFullInterface : public JointTrajectoryInterface{

public:
    JointTrajectoryFullInterface();

    void jointTrajectoryCB(const trajectory_msgs::JointTrajectoryConstPtr &msg);	// 重写接收函数
    virtual bool send_to_robot(const std::vector<JointTrajPtMessage>& messages);			// 纯虚函数实现
	virtual bool send_to_robot(const std::vector<JointTrajPtFullMessage>& messages) = 0;		// c虚函数实现
	

    bool service_start(ros::NodeHandle& n);

    bool send_to_robot(int messg_type);
    bool setRobotEnableCB(hsr_rosi_device::SetEnableSrv::Request &req,hsr_rosi_device::SetEnableSrv::Response &res);
    bool clearRobotFaultCB(hsr_rosi_device::ClearFaultSrv::Request &req,hsr_rosi_device::ClearFaultSrv::Response &res);
    bool setRobotModelCB(hsr_rosi_device::setModeSrv::Request &req,hsr_rosi_device::setModeSrv::Response &res);
    bool stopRobotMovingCB(hsr_rosi_device::StopMoveSrv::Request &req, hsr_rosi_device::StopMoveSrv::Response &res);
    bool getRobotModelCB(hsr_rosi_device::getModeSrv::Request &req, hsr_rosi_device::getModeSrv::Response &res);

    void impedanceCB(const sensor_msgs::JointState::ConstPtr &msg);

private:
	static JointTrajPtFullMessage create_message(int robot_id, int seq, int valid_fields, float time, std::vector<double> joint_pos, \
		std::vector<double> velocity, std::vector<double> accelerations);

protected:
    static JointTrajPtMessage create_message(const std::vector<double>& joint_pos,int seq);

	bool trajectory_to_msgs(const trajectory_msgs::JointTrajectoryConstPtr& traj, std::vector<JointTrajPtFullMessage>* msgs);

private:
    ros::ServiceServer set_enable_srv;
    ros::ServiceServer stop_move_srv;
    ros::ServiceServer clear_fault_srv;
    ros::ServiceServer set_mode_srv,get_mode_srv;
protected:
    ros::NodeHandle n_rosi;
    ros::Subscriber imperrSub;

    int mode ; // 0 点位全下发 1 逐一下发随动
    // 插补偏移值
    std::vector<double> impedanceErr;
};

}
}

#endif 
